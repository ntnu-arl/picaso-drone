# Drone Measurement Campaign

Airborne sonar (PiCASo, 16 mics + 1 ultrasonic emitter) on the NTNU drone,
flown in a hall with the perception stack recording alongside.

Sonar: Vincent Katter. Drone: Albert G. Puigjaner.

## Setup

The sensor is a USB-C serial device. No SDK, no driver, just a Python script
that sends short ASCII commands and reads the raw bytes back.

```bash
python3 -m venv venv
venv/bin/pip install pyserial numpy
```

The camera is not needed, the drone has its own. Pass `--camera -1`.

Optional but handy: a udev rule so the sensor always shows up as
`/dev/picaso` instead of a shifting `/dev/ttyACM*`.

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", SYMLINK+="picaso", MODE="0666"
```

Transmit waveform: `sweeps/sweep_512_taper30.csv`, a hyperbolic downsweep
from 80 to 20 kHz, 1.15 ms long. The evaluation scripts expect this file,
so no extra flags are needed.

## What we need from the drone side

* Perception stack recording during every block, including the stationary ones.
* Shared clock. The recorder writes UTC and monotonic time per capture, so if
  it runs on the same Jetson as the perception stack we are already synced.
* Static transforms: sonar to lidar, sonar to camera, sonar to flight base.
* Air temperature per session. It shifts the speed of sound and biases range.

## Blocks

Blocks 1 to 3 run on the ground and should pass before the flight slot.
Drone blocks come before the rover so Albert can leave when he is done.

**1. Sensor health.** 20 active shots, then `sensorcheck.py` and `miccheck.py`.
Transmit pulse present, all 16 mics alive. Run this again after flying,
vibration can kill a mic without anyone noticing.

**2. Distance reference.** Motors off, emitter on, facing a flat wall at
1, 2, 3 and 5 m measured by tape. This proves the whole chain works and
gives us the amplitude yardstick for everything after.

**3. Self-noise.** Motors on, emitter off, props at 20 to 100 % throttle.
Rotor noise in the 20-80 kHz band has to stay below the Block 2 echo level.
This is the go/no-go for the campaign, and it also fixes the final sweep band.
Add one run with the sensor foam-mounted to see whether the noise travels
through the frame or through the air.

**4. Horizontal flights.** Head-on wall approach, lateral translation along
a wall, yaw sweep while hovering.

**5. Vertical flights.** Climb and descend facing a wall.

**6. Promo footage.** While the rig is still assembled.

## Commands

```bash
# active (blocks 1, 2, 4, 5)
venv/bin/python picaso_record.py --mode A --max-range 5 --camera -1 \
    --port /dev/picaso --waveform sweeps/sweep_512_taper30.csv --tag flight_h_01

# passive (block 3)
venv/bin/python picaso_record.py --mode P --events 20 --camera -1 \
    --port /dev/picaso --shots 20 --tag noise_60pct
```

Each run writes one folder of `capture_NNNN.bin` files. About 327 KB per
capture and roughly 3 captures per second, so budget 3.5 GB per hour and
write to NVMe rather than an SD card.

Keep a short log per run: folder, mode, sweep, throttle, target distance,
temperature, mount, notes.

## Open questions

* Does the perception stack run on the same Jetson as the recorder?
* Mount to flight base transform?
* USB-C throughput on the Jetson, which sets the real update rate?
* Hall size and wall material?
