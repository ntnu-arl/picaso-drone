#!/usr/bin/env python3
"""
beamform_frtis.py — Python-Übersetzung von beamformFRTIS.m

So treu wie möglich Zeile-für-Zeile portiert. Die EINE Sache, die nicht 1:1
kopierbar ist, ist die geschlossene CLAIT-Toolbox
(clait.calculateAcousticImage / clait.plotPolarEnergyscape).  Deren
DMAS3+CF-Beamformer ist hier als Standard-DMAS (Delay-Multiply-and-Sum) mit
Coherence-Factor nachgebaut — funktional gleichwertig, aber NICHT bit-identisch.

Ablauf (== beamformFRTIS.m):
  1. Grundwerte (16 Mikros, c=343, fs)
  2. sweep.csv laden  -> Matched-Filter-Referenz
  3. Mikrofon-Ringkoordinaten (R=8.25mm, "swap"-Mapping)
  4. Beamformer-Einstellungen (Azimut -90..90, MF 20-80kHz, DMAS+CF)
  5. Alle capture_*.bin in einem Ordner durchgehen
  6. pro Datei: de2bi -> butter(6,100k) -> dezimieren x10 -> blank 1800
                -> DMAS+CF-Energyscape
  7. Energyscape + Kamerafoto nebeneinander -> Frame ins Video schreiben
  8. Alle Energyscapes als .mat speichern

Aufruf:
  python beamform_frtis.py 2026_07_02_134624_first_shot
  python beamform_frtis.py <ordner> --sweep sweeps/sweep_512_taper30.csv --method dmas --cf
  python beamform_frtis.py <ordner> --method das --limit 20      # schneller Test
"""
import argparse
import re
from pathlib import Path

import numpy as np
from scipy.signal import butter, lfilter, hilbert, correlate
from scipy.io import savemat
import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ── 1. Grundwerte  (== "Variables" in beamformFRTIS.m) ────────────────────────
N_MICS = 16
C      = 343.0                 # Schallgeschwindigkeit [m/s]
FS_RAW = 4.44444e6             # Roh-Abtastrate der Mikros (fs im Loop)
DECIM  = 10                    # dataFiltDec = dataFiltered(1:10:end)
FS_DEC = FS_RAW / DECIM        # 444.444 kHz  (== structSensor.sampleRate)
R_MIC  = 0.00825              # Ringradius [m]
BLANK  = 1800                  # dataFiltDec(1:1800,:) = 0
# Beamformer-Einstellungen (== structImage in der .m)
AZIMUTH_DEG   = np.arange(-90, 91, 1)      # directionsAzimuth = -90:1:90
MF_BAND       = (20e3, 80e3)               # matchedFilterFreq
ENV_LOWPASS   = 2.5e3                       # lowpassFreq (Hüllkurven-Glättung)


# ── 3. Mikrofon-Ringkoordinaten  (== "Calculate Microphone coordinates") ──────
def mic_positions():
    """pos_swap aus beamformFRTIS.m: Bits 0-7 -> gerade Mikros (2..16),
    Bits 8-15 -> ungerade (1..15). PCB in der Y-Z-Ebene, +X nach vorne."""
    mic_n = np.empty(N_MICS, dtype=int)
    mic_n[0:8]  = 2 * np.arange(8) + 2     # bits 0-7  -> 2,4,...,16
    mic_n[8:16] = 2 * np.arange(8) + 1     # bits 8-15 -> 1,3,...,15
    theta = np.deg2rad((mic_n - 1) * 22.5)
    return np.column_stack([np.zeros(N_MICS),
                            -R_MIC * np.sin(theta),
                             R_MIC * np.cos(theta)])          # (16,3)


# ── 2. Referenzsignal  (== readmatrix('sweep.csv')) ───────────────────────────
def load_reference(sweep_path):
    ref = np.loadtxt(sweep_path, delimiter=",").ravel()
    return (ref - 2048) / 2047.0            # == (refSignal-2048)/2047


def capture_list(folder):
    return sorted(folder.glob("capture_*.bin"),
                  key=lambda p: int(re.search(r"(\d+)", p.stem).group(1)))


# ── 6. Akustikdaten dekodieren  (== "Extract acoustic data") ──────────────────
def decode(path):
    """de2bi -> butter(6,100k) -> dezimieren x10 -> blank 1800.  (Ndec,16)"""
    raw  = np.fromfile(path, dtype=np.uint16)
    bits = ((raw[:, None] >> np.arange(N_MICS)) & 1).astype(float)   # de2bi
    b, a = butter(6, 100e3 / (FS_RAW / 2))
    filt = lfilter(b, a, bits - bits.mean(axis=0, keepdims=True), axis=0)
    dec  = filt[::DECIM, :]                                          # 1:10:end
    dec[:BLANK, :] = 0.0                                             # blank
    return dec


# ── 4./6. Beamformer  (== clait.calculateAcousticImage mit DMAS3+CF) ──────────
def matched_filter(dec, ref_bp):
    """Pro Mikro Matched Filter gegen den (gebandpassten) Sweep -> analytisch."""
    mf = np.empty_like(dec)
    for m in range(N_MICS):
        mf[:, m] = correlate(dec[:, m], ref_bp, mode="same")
    return mf


def beamform(dec, ref_bp, pos, az_rad, method, use_cf, tvg, rng):
    """DAS oder DMAS (+ optional Coherence Factor) über alle Azimut-Richtungen.
    Rückgabe: Energyscape-Bild (Ndec, n_az), Amplitude (Hüllkurve)."""
    n = dec.shape[0]
    mf = matched_filter(dec, ref_bp)                    # (n,16) reell, 20-80 kHz
    freqs = np.fft.fftfreq(n, d=1.0 / FS_DEC)
    F = np.fft.fft(mf, axis=0)                           # (n,16)
    u = np.column_stack([np.cos(az_rad), np.sin(az_rad), np.zeros_like(az_rad)])
    tau = (pos @ u.T) / C                                # (16, n_az) Laufzeiten
    b_env, a_env = butter(2, ENV_LOWPASS / (FS_DEC / 2))  # Hüllkurven-Glättung

    out = np.empty((n, len(az_rad)))
    for j in range(len(az_rad)):
        phase = np.exp(-2j * np.pi * freqs[:, None] * tau[:, j][None, :])
        aligned = np.fft.ifft(F * phase, axis=0).real    # (n,16) ausgerichtet
        if method == "dmas":
            # DMAS: Summe aller Paarprodukte  = ((Σŝ)^2 - Σŝ^2)/2 ,  ŝ=sign·√|s|
            sr = np.sign(aligned) * np.sqrt(np.abs(aligned))
            beam = 0.5 * (sr.sum(1) ** 2 - (sr ** 2).sum(1))
        else:                                            # das
            beam = aligned.sum(1)
        env = np.abs(hilbert(beam))
        env = lfilter(b_env, a_env, env)                 # doEnvelope + lowpass
        if use_cf:                                       # Coherence Factor
            num = np.abs(aligned.sum(1)) ** 2
            den = N_MICS * (aligned ** 2).sum(1) + 1e-12
            env *= num / den
        out[:, j] = env
    out[out < 0] = 0.0                                   # doThresholdAtZero
    if tvg:
        out *= (rng ** tvg)[:, None]                     # rangeCompensation
    return out


# ── 7. Ein Frame zeichnen: Energyscape + Kamerafoto  (== subplot 2x1) ─────────
def render_frame(E, rng, az_rad, jpg_path, idx, db, size):
    Z = E / (E.max() + 1e-12)
    if db:
        Z = 10 * np.log10(Z + 1e-12); vmin, vmax = -40, 0
    else:
        vmin, vmax = 0, 1
    Rg, Az = np.meshgrid(rng, az_rad, indexing="ij")
    fig = plt.figure(figsize=(size[0] / 100, size[1] / 100), dpi=100)
    ax0 = fig.add_subplot(2, 1, 1, projection="polar")
    ax0.set_theta_zero_location("N"); ax0.set_theta_direction(1)
    ax0.set_thetamin(-90); ax0.set_thetamax(90)
    ax0.pcolormesh(Az, Rg, Z, shading="auto", cmap="turbo", vmin=vmin, vmax=vmax)
    ax0.set_title(f"Energyscape Frame {idx}")
    ax1 = fig.add_subplot(2, 1, 2)
    if jpg_path.exists():
        ax1.imshow(plt.imread(str(jpg_path))); ax1.set_title("Optical Image of Scene")
    else:
        ax1.text(0.5, 0.5, "(kein Kamerafoto)", ha="center", va="center")
        ax1.set_title("Optical Image of Scene")
    ax1.axis("off")
    fig.tight_layout()
    fig.canvas.draw()
    buf = np.asarray(fig.canvas.buffer_rgba())[:, :, :3]
    plt.close(fig)
    return cv2.cvtColor(buf, cv2.COLOR_RGB2BGR)


# ── 5./8. Hauptschleife über den Ordner ───────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("folder")
    ap.add_argument("--sweep", default="sweeps/sweep_512_taper30.csv")
    ap.add_argument("--method", choices=["das", "dmas"], default="dmas")
    ap.add_argument("--cf", action="store_true", default=True,
                    help="Coherence Factor (Standard an; --no-cf zum Abschalten)")
    ap.add_argument("--no-cf", dest="cf", action="store_false")
    ap.add_argument("--tvg", type=float, default=2.0, help="rangeCompensation")
    ap.add_argument("--rmax", type=float, default=None,
                    help="Auswertung/Anzeige auf max. RMAX Meter beschränken")
    ap.add_argument("--db", action="store_true")
    ap.add_argument("--fps", type=int, default=10)
    ap.add_argument("--limit", type=int, default=None, help="nur erste N Frames")
    ap.add_argument("--out", default=None, help="Video-Datei (.avi)")
    args = ap.parse_args()

    folder = Path(args.folder)
    files = capture_list(folder)
    if not files:
        raise SystemExit(f"Keine capture_*.bin in {folder}")
    if args.limit:
        files = files[:args.limit]

    pos    = mic_positions()
    az_rad = np.deg2rad(AZIMUTH_DEG)
    ref    = load_reference(args.sweep)
    b_bp, a_bp = butter(4, list(MF_BAND), btype="band", fs=FS_DEC)
    ref_bp = lfilter(b_bp, a_bp, ref)
    ref_bp /= np.linalg.norm(ref_bp) + 1e-9

    n0  = np.fromfile(files[0], dtype=np.uint16).size // DECIM
    rng = np.arange(n0) / FS_DEC * C / 2                 # == rangeVector / maxRange
    keep = rng <= args.rmax if args.rmax else np.ones(n0, dtype=bool)
    rng_k = rng[keep]                                    # beschnittene Range-Achse
    print(f"{len(files)} Aufnahmen | {len(AZIMUTH_DEG)} Azimut | "
          f"maxRange {rng_k[-1]:.2f} m | Methode {args.method.upper()}"
          f"{'+CF' if args.cf else ''}")

    out_video = Path(args.out) if args.out else folder.parent / f"{folder.name}.avi"
    size = (900, 950)                                    # Frame-Größe (B,H)
    vw = cv2.VideoWriter(str(out_video), cv2.VideoWriter_fourcc(*"MJPG"),
                         args.fps, size)                 # == Motion JPEG AVI

    stack = []
    for i, f in enumerate(files, 1):
        E = beamform(decode(f), ref_bp, pos, az_rad, args.method, args.cf,
                     args.tvg, rng)[:len(rng), :][keep, :]
        stack.append(E.astype(np.float32))
        frame = render_frame(E, rng_k, az_rad, folder / f"{f.stem}.jpg",
                             i, args.db, size)
        vw.write(cv2.resize(frame, size))
        print(f"  [{i}/{len(files)}] {f.stem}", end="\r")
    print()
    vw.release()

    mat_path = folder.parent / f"{folder.name}.mat"
    savemat(mat_path, {"allBeamformedImages": np.stack(stack, axis=-1)})
    print(f"Video: {out_video}\nMAT  : {mat_path}  shape "
          f"{np.stack(stack, axis=-1).shape}")


if __name__ == "__main__":
    main()
