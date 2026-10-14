# PiCASo Sonar, Drone Integration

## ROS 2 driver

The `feat/ros2-driver` branch adds two Humble packages while retaining the original
recorder and offline tools:

- `picaso_msgs`: the `Capture` interface.
- `picaso_driver`: a serial acquisition worker and ROS node (`air_sonar`).

In the Unified Autonomy Stack this repository lives at
`workspaces/ws_air_sonar/src/picaso-drone`. From the **stack root**:

```bash
docker buildx bake ros2_air_sonar --load --allow=network.host
make build-air_sonar
sudo cp workspaces/ws_air_sonar/src/picaso-drone/99-picaso.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty --action=add
make launch DOCKER_COMPOSE_FILE=docker-compose.air_sonar.yml
```

Only the sonar service is launched. The dedicated workspace build uses its own
Compose project. For detailed Docker, USB and bringup instructions, see the stack's
`docs/air_sonar.md`. Host udev rules are necessary; installing them inside a
container does not create `/dev/picaso` on the host. The rule uses `dialout` and
mode `0660`, rather than making the sensor world-writable. Like VectorNav, setup
is simply copying a rule to the host and reloading udev; no installer script is
needed. The sensor can be disconnected during setup; `/dev/picaso` appears when
it is connected. Several STM32 devices
share the same VID/PID: qualify the rule by serial number when needed.

For a native ROS 2 installation, use `rosdep install --from-paths src --ignore-src -r -y`
and `colcon build --symlink-install` in the workspace. Then source `install/setup.bash`
and run `ros2 run picaso_driver picaso_driver`. The bundled sweep is installed with
the package; SciPy/matplotlib are not required by the driver.

### Topics and timestamps

`/air_sonar/capture` (`picaso_msgs/msg/Capture`) contains the complete measurement:
mode, event count, successful capture counter, original byte count, samples per
event, sample rate, physical microphone mapping, and the actual uploaded DAC
waveform. `pdm_words` is a **lossless `uint16[]` representation** of the little-endian
`.bin` data. These are packed PDM bits, not PCM amplitudes or estimated distances.
Bits 0..7 belong to physical microphones 2,4,..16; bits 8..15 to 1,3,..15.

All recorder clocks are retained as integer nanoseconds:

| Message field | Original CSV field | Clock |
| --- | --- | --- |
| `t_cmd_unix_ns` | `t_cmd_unix` | Host realtime before command write |
| `t_done_unix_ns` | `t_done_unix` | Host realtime after final response byte |
| `t_cmd_mono_ns` | `t_cmd_mono` | Host monotonic before command write |
| `t_done_mono_ns` | `t_done_mono` | Host monotonic after final response byte |
| `t_cmd_utc` | `t_cmd_utc` | ISO UTC text for command time |

Divide an integer clock field by `1e9` to recover seconds. `header.stamp` equals
`t_cmd_unix_ns` exactly, and `header.frame_id` is configured by `frame_id`. Physical
sensor stamps always use host realtime, independent of a simulation `/clock`.
There is **no device/acoustic emission timestamp** in the firmware protocol.
The successful capture ID can be used to export `capture_0001.bin` names; the live
driver does not create raw files automatically. A ROS bag records data and metadata
together. To reconstruct `.bin` bytes, encode `pdm_words` as **little-endian uint16**.

`/air_sonar/status` publishes standard diagnostics with connection state, counters
and errors. Missing hardware and truncated captures trigger bounded reconnects;
partial measurements are never published. Each new active connection uploads all
1024 DAC samples, padding the selected sweep with silence (2048). Because responses
have no header, sequence, checksum or acknowledgment, reconnect first drains until
the stream is quiet. Persistent misalignment may still require a sensor reset;
the protocol cannot prove synchronization or detect same-length corruption.

The optional **host webcam** publishes JPEG `sensor_msgs/msg/CompressedImage` on
`/air_sonar/image/compressed`. `has_image` and `image_stamp` in the capture associate
it with the pre-command image. Its stamp is host frame receipt, not a calibrated
exposure time. Camera acquisition is disabled by default (`camera_index: -1`).
The driver publishes no mounting TF; supply a measured extrinsic separately.

Acquisition parameters are read-only after startup. Change
`robot_bringup/config/ros2/air_sonar.yaml` and restart the service. Set `port: ""`
for VID/PID discovery, optionally filtered by `serial_number`. An explicit port
takes priority over discovery. `capture_period: 0.0` acquires back-to-back; positive
values specify the minimum period including transfer time.

### Export a ROS 2 bag to the original recorder format

With ROS 2 and the built sonar workspace sourced, run the script from this
repository. It reads the bag directly; no sensor, playback or running ROS nodes
are needed:

```bash
python3 picaso_export_bag.py /path/to/sonar_bag \
    --topic /air_sonar/capture --output /path/to/sonar_export
```

`bag` must be a ROS 2 bag **directory containing `metadata.yaml`**. `--topic`
must name a recorded `picaso_msgs/msg/Capture` topic. `--output` is optional;
it defaults to `<bag_name>_export` beside the bag. The output folder must not
already exist. The script works across split bag files. SQLite bags work in
the driver image; other storage formats require their ROS 2 storage plugin.
For native use, install `ros-${ROS_DISTRO}-rosbag2-py` if it is missing.

To run inside the existing driver image, from the **stack root** (paths in the
command refer to the container):

```bash
docker run --rm --network none \
  -v "$PWD/workspaces/ws_air_sonar:/workspace" \
  -w /workspace/src/picaso-drone unified_autonomy:ros2_air_sonar \
  python3 picaso_export_bag.py /workspace/recordings/sonar \
    --topic /air_sonar/capture --output /workspace/recordings/sonar_export
```

The folder contains `capture_0001.bin`, `capture_0002.bin`, etc., and
`timestamps.csv`. Binary files are byte-exact little-endian `uint16` PDM data.
The CSV's first ten columns match the original recorder: `capture`, `file`,
`mode`, `events`, `bytes`, `t_cmd_unix`, `t_done_unix`, `t_cmd_mono`,
`t_done_mono`, `t_cmd_utc`. The four numeric clocks use decimal seconds with
**nine fractional digits** to preserve the ROS message's nanoseconds without
float rounding. UTC text is copied unchanged.

Extra columns preserve the original `capture_id`, `header_stamp_ns`, `frame_id`,
`bag_stamp_ns` (the bag recording timestamp), and `image_stamp_ns` (blank when
no image was paired). File numbering and the `capture` column are sequential
in bag order, so IDs resetting after a driver restart cannot overwrite files.
This exporter does not extract JPEGs or waveform tables. If corrupt data or an
I/O error interrupts export, the partial output folder is retained for inspection;
choose a new output folder when retrying.

Test the exporter in the driver image with
`python3 -m pytest tests/test_export_bag.py -q` from the repository directory.

### Tests without hardware

After building, run the real serial pseudo-terminal and ROS 2 launch tests in
the driver image (from the stack root):

```bash
docker run --rm --network host -e ROS_DOMAIN_ID=224 \
  -v "$PWD/workspaces/ws_air_sonar:/workspace" \
  -v "$PWD/workspaces/robot_bringup:/workspace/src/robot_bringup:ro" \
  -w /workspace unified_autonomy:ros2_air_sonar \
  python3 -m pytest src/picaso-drone/picaso_driver/test -q -s
```

These tests launch only the sonar node. They verify active/passive modes, the
1024-sample upload, byte-exact PDM round trips, all clock fields, partial-response
rejection, reconnect/reupload, and ROS topic delivery. Physical USB behavior,
acoustic emission and webcam hardware still require sensor-connected tests.

---

In-air sonar sensor: 16 MEMS microphones and one ultrasonic emitter.
It is a plain USB-C serial device, so there is no SDK and no driver to install.
`picaso_record.py` sends short ASCII commands and stores the raw bytes it
gets back.

The measurement campaign itself is described in `MEASUREMENT_PLAN.md`.

## Install

```bash
python3 -m venv venv
venv/bin/pip install pyserial numpy
```

That is everything the acquisition needs. `scipy` and `matplotlib` are only
required if you also want to run the offline checks (see below).

Then install the udev rule so the sensor always shows up at the same path
instead of a shifting `/dev/ttyACM*`:

```bash
sudo cp 99-picaso.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## Record

Quick smoke test, 5 captures, to confirm the sensor answers:

```bash
venv/bin/python picaso_record.py --mode A --events 20 --camera -1 \
    --port /dev/picaso --waveform sweeps/sweep_512_taper30.csv \
    --shots 5 --tag smoketest
```

`--camera -1` disables the webcam capture, which the drone does not need.
OpenCV is then never imported. Ctrl+C stops a run, or use `--shots N` for a
fixed count.

**For the campaign itself, use `MEASUREMENT_PLAN.md`.** It has a ready to copy
and paste command for every test scenario, so nothing has to be assembled by
hand on the day.

## What it writes

One folder per run:

```
2026_10_14_143022_test/
    capture_0001.bin
    capture_0002.bin
    ...
    timestamps.csv
```

Each `.bin` is raw little endian `uint16`, about 327 KB for 20 events.
At roughly 3 captures per second that is 3.5 GB per hour, so write to NVMe
rather than an SD card.

The low byte of each sample is `GPIOB[7:0]` at the TIM8_CH1 rising edge and
the high byte is the same pins at the falling edge, which together carry the
16 channel PDM stream. The docstring in `picaso_record.py` has the full detail.

## Time sync

`timestamps.csv` is what lets a capture be lined up with your lidar, camera or
pose data. One row per capture, written and flushed as the capture lands:

| column | meaning |
|---|---|
| `t_cmd_unix` | CLOCK_REALTIME as the record command leaves the host |
| `t_done_unix` | CLOCK_REALTIME as the last byte arrives |
| `t_cmd_mono`, `t_done_mono` | the same two instants on CLOCK_MONOTONIC |
| `t_cmd_utc` | `t_cmd_unix` again in ISO 8601, for reading by eye |

Join on the unix columns. They are the same clock a ROS 2 header stamp uses,
so if this script runs on the same Jetson as the perception stack, the clock
is already shared and nothing else is needed. Use the monotonic columns for
durations, because they cannot jump if NTP steps the system clock.

If the recorder ends up on a different machine than the perception stack,
tell us, because then the two need NTP or chrony between them first.

The emission itself happens shortly after `t_cmd_unix`. The firmware polls for
commands in its main loop, so a few milliseconds there are not logged. That is
the floor on host-side sync. If it turns out to be too coarse for the flights,
the fix is a GPIO trigger pulse on emit, which is firmware work. We would
rather find that out during the integration test than after the campaign.

## The serial protocol

If you want to drive the sensor from your own code, this is all of it:

| Command | Meaning |
|---|---|
| `D,<index>,<value>` | Upload one DAC sample, 0 to 4095. Sent 1024 times to load the sweep, once per session. |
| `A,<events>,0` | Active: fire the sweep, then capture. |
| `P,<events>,0` | Passive: capture only. |

Every command ends with `\n`. After `A` or `P` the device streams exactly
`events * 16384` bytes. One event covers about 0.316 m of range and the
maximum is 32 events, so roughly 10 m.

Port settings: 115200 baud, VID `0x0483`, PID `0x5740`.

## Offline checks

These need `scipy` and `matplotlib` and produce a PNG you can open.

```bash
venv/bin/python sensorcheck.py <folder> --rmax 5   # transmit pulse and A-scan
venv/bin/python miccheck.py   <folder>             # level per microphone
```

Run `sensorcheck.py` before and after flying. Vibration can kill a
microphone without any other symptom, and the second run is what tells you
which captures are still trustworthy.

## Contact

Sonar: Vincent Katter. Drone: Albert G. Puigjaner.
