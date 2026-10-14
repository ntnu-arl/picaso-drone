# PiCASo Sonar, Drone Integration

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
```

Each `.bin` is raw little endian `uint16`, about 327 KB for 20 events.
At roughly 3 captures per second that is 3.5 GB per hour, so write to NVMe
rather than an SD card.

The low byte of each sample is `GPIOB[7:0]` at the TIM8_CH1 rising edge and
the high byte is the same pins at the falling edge, which together carry the
16 channel PDM stream. The docstring in `picaso_record.py` has the full detail.

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
