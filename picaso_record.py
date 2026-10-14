#!/usr/bin/env python3
"""
picaso_record.py  —  PiCASo continuous data recorder

Uploads a waveform once, then acquires continuously until Ctrl+C.
Each acquisition writes a paired .bin + .jpg file in a timestamped folder.

Usage
-----
    python picaso_record.py --mode A --events 20 --waveform pulse.csv
    python picaso_record.py --mode P --events 10
    python picaso_record.py --mode A --events 20 --waveform pulse.csv --port COM5
    python picaso_record.py --mode A --events 20 --waveform pulse.csv --camera 1

Arguments
---------
    --mode      A = Active (upload waveform + capture), P = Passive (capture only)
                Default: A
    --events    GPDMA nodes per acquisition (1–32, each node = 16 384 bytes)
                Default: 20
    --waveform  Path to CSV file with DAC sample values (integers 0–4095).
                Required for mode A.  May contain one value per line or
                multiple comma-separated values per line.
    --port      Serial port.  Auto-detected by USB VID/PID if omitted.
    --camera    Webcam device index for paired image capture (default: 0).
                Pass -1 to disable camera.

Output
------
    <YYYY_MM_DD_HHMMSS>[_<tag>]/capture_0001.bin   PDM ring-buffer data
    <YYYY_MM_DD_HHMMSS>[_<tag>]/capture_0001.jpg   scene photo at trigger time
    <YYYY_MM_DD_HHMMSS>[_<tag>]/timestamps.csv     one row per capture
    ...

TIME SYNC
---------
`timestamps.csv` is what lets a capture be lined up with lidar, camera or pose
data recorded by another process.  One row per capture:

    t_cmd_unix    CLOCK_REALTIME as the A/P command leaves the host
    t_done_unix   CLOCK_REALTIME as the last byte of the capture lands
    t_cmd_mono    CLOCK_MONOTONIC, same instant as t_cmd_unix
    t_done_mono   CLOCK_MONOTONIC, same instant as t_done_unix
    t_cmd_utc     t_cmd_unix again, ISO 8601, for reading by eye

Use the unix columns to join against other logs; they are the same clock a
ROS 2 message header stamp uses.  Use the monotonic columns for durations,
since they cannot jump when NTP steps the system clock.

Run this on the same machine as the other sensors and the clock is shared, so
no further setup is needed.  Across two machines, run NTP or chrony between
them first, or the join is only as good as their offset.

The acoustic emission happens somewhere between t_cmd_unix and t_done_unix,
close to t_cmd_unix: the firmware polls for commands in its main loop, so
there are a few unlogged milliseconds between the write and the real pulse.
That residual is the floor on host-side sync.  The capture itself helps here,
because the transmit crosstalk is visible in the data at a fixed offset from
the start of the capture window and anchors the acoustic zero inside the file.
Removing the residual entirely would need a GPIO trigger pulse on emit that
the host can timestamp, which is firmware work.

Each .bin file is raw little-endian uint16 samples from the ring buffer:
    low  byte of each uint16: GPIOB[7:0] at TIM8_CH1 rising  edge
    high byte of each uint16: GPIOB[7:0] at TIM8_CH1 falling edge
"""

import sys
import csv
import math
import time
import argparse
import datetime
from pathlib import Path

import serial
import serial.tools.list_ports

try:
    import cv2          # pip install opencv-python
except ImportError:
    cv2 = None          # type: ignore[assignment]

# Capture backend: DirectShow is Windows-only, AVFoundation is the macOS one.
if cv2 is None:
    CAM_BACKEND = None
elif sys.platform == "win32":
    CAM_BACKEND = cv2.CAP_DSHOW
elif sys.platform == "darwin":
    CAM_BACKEND = cv2.CAP_AVFOUNDATION
else:
    CAM_BACKEND = cv2.CAP_ANY

# ── Device identity (matches usb_descriptors.c) ───────────────────────────────
TARGET_VID = 0x0483
TARGET_PID = 0x5740

# ── Capture geometry (must match throughput_test.c / pulse.h) ─────────────────
CHUNK_SAMPLES = 8192
CHUNK_BYTES   = CHUNK_SAMPLES * 2   # uint16 per sample → 16 384 bytes per node
PULSE_SAMPLES = 1024                # DAC waveform length
DAC_MID       = 2048                # mid-scale = silence

# ── Range <-> events ──────────────────────────────────────────────────────────
# One event node records CHUNK_SAMPLES at FS_RAW; range = t*c/2 (round trip).
FS_RAW = 4.44444e6                  # PDM sample rate [Hz]
C_AIR  = 343.0                      # speed of sound [m/s]
RANGE_PER_EVENT = CHUNK_SAMPLES / FS_RAW * C_AIR / 2   # ≈ 0.316 m per event

# ── Camera ────────────────────────────────────────────────────────────────────
CAM_WARMUP_FRAMES = 10   # discard to let auto-exposure settle


# ── Port helpers ──────────────────────────────────────────────────────────────

def find_port(vid: int, pid: int) -> str | None:
    for info in serial.tools.list_ports.comports():
        if info.vid == vid and info.pid == pid:
            return info.device
    return None


def list_ports() -> None:
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("  (no serial ports found)")
        return
    for p in ports:
        vid = f"{p.vid:#06x}" if p.vid else "  n/a "
        pid = f"{p.pid:#06x}" if p.pid else "  n/a "
        print(f"  {p.device:<12}  VID={vid}  PID={pid}  {p.description}")


# ── Camera helpers ────────────────────────────────────────────────────────────

def open_camera(index: int) -> object | None:
    """Open webcam device, run warmup frames, return capture object or None."""
    cam = cv2.VideoCapture(index, CAM_BACKEND)
    if not cam.isOpened():
        cam.release()
        return None
    # Warmup: consume buffered frames so auto-exposure/white-balance settles
    for _ in range(CAM_WARMUP_FRAMES):
        cam.read()
    return cam


def enumerate_cameras(max_index: int = 6) -> list[int]:
    """Return list of openable device indices."""
    found = []
    for i in range(max_index):
        cam = cv2.VideoCapture(i, CAM_BACKEND)
        if cam.isOpened():
            found.append(i)
        cam.release()
    return found


def print_cameras(indices: list[int]) -> None:
    if not indices:
        print("  (no cameras found)")
        return
    for i in indices:
        cam = cv2.VideoCapture(i, CAM_BACKEND)
        w   = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
        h   = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = cam.get(cv2.CAP_PROP_FPS)
        cam.release()
        print(f"  [{i}]  {w}×{h}  {fps:.0f} fps")


def select_camera() -> int:
    """List available cameras and prompt the user to pick one. Returns index or -1."""
    print("Scanning cameras ...")
    indices = enumerate_cameras()
    if not indices:
        print("  No cameras found — photos disabled.")
        return -1
    print_cameras(indices)
    while True:
        raw = input(f"Select camera [{indices[0]}–{indices[-1]}] (Enter to disable): ").strip()
        if not raw:
            return -1
        try:
            idx = int(raw)
        except ValueError:
            print("  Enter a number or press Enter to skip.")
            continue
        if idx in indices:
            return idx
        print(f"  Not in list: {indices}")


def grab_frame(cam: object) -> object | None:
    """Return latest camera frame, or None on failure."""
    # Flush the internal buffer by reading until no more queued frames.
    # On a 30 fps webcam the buffer can hold ~3 frames; we want the newest one.
    ret, frame = cam.read()
    if not ret:
        return None
    # One extra grab to get a fresher frame if the driver buffers two
    ret2, frame2 = cam.read()
    return frame2 if ret2 else frame


# ── Waveform ──────────────────────────────────────────────────────────────────

def load_waveform(path: str) -> list[int]:
    """Read CSV of DAC values (0–4095) and return up to PULSE_SAMPLES entries."""
    samples: list[int] = []
    with open(path, newline='') as f:
        for row in csv.reader(f):
            for cell in row:
                s = cell.strip()
                if not s:
                    continue
                v = int(s)
                if not (0 <= v <= 4095):
                    raise ValueError(f"Sample value {v} out of range 0-4095 in '{path}'")
                samples.append(v)
    return samples[:PULSE_SAMPLES]


def upload_waveform(ser: serial.Serial, samples: list[int]) -> None:
    """Upload all D commands in one write to avoid per-call latency."""
    # The firmware replays the whole PULSE_SAMPLES DAC buffer, so a waveform
    # shorter than that must overwrite the rest with silence — otherwise the
    # previous upload's tail still gets played after ours.
    pad = PULSE_SAMPLES - len(samples)
    padded = samples + [DAC_MID] * pad
    print(f"  Uploading {len(samples)} samples"
          + (f" + {pad} silence" if pad else "") + " ... ", end='', flush=True)
    payload = ''.join(f"D,{i},{v}\n" for i, v in enumerate(padded))
    ser.write(payload.encode('ascii'))
    time.sleep(0.3)   # firmware processes commands inside its main loop
    print("done.")


# ── Acquisition ───────────────────────────────────────────────────────────────

def acquire(ser: serial.Serial, mode: str, n_events: int
            ) -> tuple[bytes, tuple[float, float], tuple[float, float]]:
    """
    Send one A/P command and read exactly n_events × CHUNK_BYTES bytes.
    Raises TimeoutError if the port goes silent before all bytes arrive.

    Returns (raw, t_cmd, t_done).  Each timestamp is a (unix, monotonic) pair:
    t_cmd is taken as the command leaves the host, t_done as the last byte
    lands.  The acoustic event sits just after t_cmd — see TIME SYNC below.
    """
    total = n_events * CHUNK_BYTES

    # Taken as late as possible before the write so the gap to the real
    # emission is as small as we can make it from the host side.
    t_cmd = (time.time(), time.monotonic())
    ser.write(f"{mode},{n_events},0\n".encode('ascii'))

    raw = b''
    while len(raw) < total:
        chunk = ser.read(total - len(raw))   # blocks up to ser.timeout for first byte
        if not chunk:
            raise TimeoutError(
                f"Serial timeout — received {len(raw):,} of {total:,} bytes.\n"
                f"  Check firmware is running and TIMEOUT_S covers capture + transfer."
            )
        raw += chunk
    t_done = (time.time(), time.monotonic())
    return raw, t_cmd, t_done


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description="PiCASo continuous recorder — Ctrl+C to stop.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--mode",     choices=["A", "P"], default="A",
                        help="A=Active (pulse + capture), P=Passive (default: A)")
    parser.add_argument("--events",   type=int, default=20, metavar="N",
                        help="GPDMA nodes per acquisition (default: 20)")
    parser.add_argument("--max-range", type=float, default=None, metavar="M",
                        help="Limit capture depth to M metres (overrides --events; "
                             f"each event ≈ {RANGE_PER_EVENT:.3f} m).")
    parser.add_argument("--shots",    type=int, default=None, metavar="N",
                        help="Stop after N captures (default: run until Ctrl+C)")
    parser.add_argument("--tag",      metavar="TAG",
                        help="Output folder tag (default: ask interactively)")
    parser.add_argument("--waveform", metavar="FILE",
                        help="CSV of DAC values 0-4095 (required for mode A)")
    parser.add_argument("--port",     metavar="PORT",
                        help="Serial port (default: auto-detect by VID/PID)")
    parser.add_argument("--camera",   type=int, default=None, metavar="N",
                        help="Webcam device index for paired photos. "
                             "Omit to select interactively; pass -1 to disable.")
    args = parser.parse_args()

    if args.mode == "A" and args.waveform is None:
        parser.error("--waveform FILE is required for mode A")

    # Range limit: derive the smallest #events that still covers max_range.
    if args.max_range is not None:
        args.events = max(1, min(32, math.ceil(args.max_range / RANGE_PER_EVENT)))
        print(f"max-range {args.max_range:.2f} m -> {args.events} events "
              f"(capture depth {args.events * RANGE_PER_EVENT:.2f} m)")

    # ── Timestamp + folder ────────────────────────────────────────────────────
    now       = datetime.datetime.now()
    timestamp = now.strftime("%Y_%m_%d_%H%M%S")

    tag = (args.tag if args.tag is not None
           else input("Measurement tag (Enter to skip): ")).strip()
    folder_name = f"{timestamp}_{tag}" if tag else timestamp
    folder = Path(folder_name)
    folder.mkdir(parents=True, exist_ok=True)
    print(f"Output folder : {folder.resolve()}")

    # ── Camera ────────────────────────────────────────────────────────────────
    cam: object | None = None
    if cv2 is None:
        print("Camera        : disabled (opencv-python not installed)")
    else:
        cam_index = args.camera if args.camera is not None else select_camera()
        if cam_index >= 0:
            cam = open_camera(cam_index)
            if cam is None:
                print(f"Warning: could not open camera {cam_index} — photos disabled.")
                print_cameras(enumerate_cameras())
            else:
                w   = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
                h   = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))
                fps = cam.get(cv2.CAP_PROP_FPS)
                print(f"Camera        : device {cam_index}  {w}×{h}  {fps:.0f} fps")

    # ── Port ──────────────────────────────────────────────────────────────────
    port = args.port or find_port(TARGET_VID, TARGET_PID)
    if port is None:
        print(f"Device not found (VID={TARGET_VID:#06x} PID={TARGET_PID:#06x}).")
        print("Available ports:")
        list_ports()
        if cam:
            cam.release()
        sys.exit(1)
    print(f"Serial port   : {port}")

    # Timeout must cover: capture window + USB transfer
    # capture  = N_EVENTS × 2.048 ms   ≈ 41 ms for 20 events
    # transfer = N_EVENTS × 16 384 B / ~1 MB/s ≈ 328 ms for 20 events
    timeout_s = max(10, args.events * CHUNK_BYTES / 500_000 + 5)

    ser = serial.Serial(port, baudrate=115200,
                        timeout=timeout_s, write_timeout=5)
    ser.reset_input_buffer()
    print(f"Read timeout  : {timeout_s:.1f} s")

    # ── Waveform (uploaded once, persists across acquisitions) ────────────────
    if args.mode == "A":
        samples = load_waveform(args.waveform)
        print(f"Waveform      : {len(samples)} samples from '{args.waveform}'")
        upload_waveform(ser, samples)

    # ── Acquisition loop ──────────────────────────────────────────────────────
    total_bytes = args.events * CHUNK_BYTES
    print(f"\nMode={args.mode}  events/capture={args.events}  "
          f"bytes/capture={total_bytes:,}  ({total_bytes/1024:.0f} KiB)"
          + ("  camera=on" if cam else "  camera=off"))
    print(f"Shots         : {args.shots}" if args.shots else "Ctrl+C to stop.")
    print()

    ts_path   = folder / "timestamps.csv"
    ts_file   = open(ts_path, "w", newline="")
    ts_writer = csv.writer(ts_file)
    ts_writer.writerow([
        "capture", "file", "mode", "events", "bytes",
        "t_cmd_unix", "t_done_unix", "t_cmd_mono", "t_done_mono", "t_cmd_utc",
    ])
    ts_file.flush()
    print(f"Time log      : {ts_path.name}")

    capture_n = 0
    try:
        while args.shots is None or capture_n < args.shots:
            capture_n += 1
            stem     = f"capture_{capture_n:04d}"
            bin_path = folder / f"{stem}.bin"
            jpg_path = folder / f"{stem}.jpg"

            # Grab camera frame right before triggering — documents scene at fire time.
            # cam.read() returns a buffered frame and is fast (<1 ms after warmup).
            frame = grab_frame(cam) if cam else None

            raw, t_cmd, t_done = acquire(ser, args.mode, args.events)
            elapsed = t_done[1] - t_cmd[1]

            bin_path.write_bytes(raw)

            # Flush every row: a cut power or a crash must not cost the sync
            # information for the captures already on disk.
            ts_writer.writerow([
                capture_n, bin_path.name, args.mode, args.events, len(raw),
                f"{t_cmd[0]:.6f}", f"{t_done[0]:.6f}",
                f"{t_cmd[1]:.6f}", f"{t_done[1]:.6f}",
                datetime.datetime.fromtimestamp(
                    t_cmd[0], datetime.timezone.utc).isoformat(),
            ])
            ts_file.flush()

            cam_tag = ""
            if frame is not None:
                cv2.imwrite(str(jpg_path), frame)
                cam_tag = "  +jpg"
            elif cam:
                cam_tag = "  cam?"   # camera open but grab failed

            hms = datetime.datetime.now().strftime("%H:%M:%S")
            print(f"[{hms}]  #{capture_n:04d}  {len(raw):,} B  "
                  f"{elapsed*1e3:.0f} ms  {len(raw)/elapsed/1e6:.2f} MB/s"
                  f"  →  {stem}{cam_tag}")

        if args.shots:
            print(f"\nCampaign complete.  {capture_n} capture(s) recorded.")

    except KeyboardInterrupt:
        print(f"\nStopped.  {capture_n} capture(s) recorded.")
    except TimeoutError as exc:
        print(f"\nError: {exc}")
    finally:
        ts_file.close()
        ser.close()
        if cam:
            cam.release()
        print(f"Port closed.  Data: {folder.resolve()}")


if __name__ == "__main__":
    main()
