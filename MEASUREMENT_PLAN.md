# Drone Measurement Campaign

Airborne sonar (PiCASo, 16 mics + 1 ultrasonic emitter) on the NTNU drone,
flown in a hall with the perception stack recording alongside.

Sonar: Vincent Katter. Drone: Albert G. Puigjaner.

Every command below is ready to copy and paste. The only thing that changes
between runs is the `--tag`, which becomes the output folder name.

## Before you start

Install once, see README.md for details:

```bash
python3 -m venv venv
venv/bin/pip install pyserial numpy
sudo cp 99-picaso.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Then confirm the sensor is found:

```bash
ls -l /dev/picaso
```

Start the perception stack recording before every block below, including the
ones where the drone stays on the ground.

## Block 1: sensor health

Run this before flying and again after flying. Motors off, sensor pointing at
anything a few metres away.

```bash
venv/bin/python picaso_record.py --mode A --events 20 --camera -1 \
  --port /dev/picaso --waveform sweeps/sweep_512_taper30.csv \
  --shots 20 --tag health_pre
```

After flying, the same thing with a different tag:

```bash
venv/bin/python picaso_record.py --mode A --events 20 --camera -1 \
  --port /dev/picaso --waveform sweeps/sweep_512_taper30.csv \
  --shots 20 --tag health_post
```

We check these offline. Vibration can kill a microphone without any other
symptom, and the second run is what tells us which captures are trustworthy.

## Block 2: distance reference

Motors off, emitter on, sensor facing a flat wall. Measure the distance with a
tape and run one command per distance.

```bash
venv/bin/python picaso_record.py --mode A --events 20 --camera -1 \
  --port /dev/picaso --waveform sweeps/sweep_512_taper30.csv \
  --shots 20 --tag wall_1m

venv/bin/python picaso_record.py --mode A --events 20 --camera -1 \
  --port /dev/picaso --waveform sweeps/sweep_512_taper30.csv \
  --shots 20 --tag wall_2m

venv/bin/python picaso_record.py --mode A --events 20 --camera -1 \
  --port /dev/picaso --waveform sweeps/sweep_512_taper30.csv \
  --shots 20 --tag wall_3m

venv/bin/python picaso_record.py --mode A --events 20 --camera -1 \
  --port /dev/picaso --waveform sweeps/sweep_512_taper30.csv \
  --shots 20 --tag wall_5m
```

Please note the air temperature here. It shifts the speed of sound and biases
the range.

This block proves the whole chain works and gives us the amplitude reference
for everything that follows.

## Block 3: self-noise

Motors on, emitter off, drone secured on the ground. One run per throttle step.
Note that there is no `--waveform` here, passive mode does not emit.

```bash
venv/bin/python picaso_record.py --mode P --events 20 --camera -1 \
  --port /dev/picaso --shots 20 --tag noise_20pct

venv/bin/python picaso_record.py --mode P --events 20 --camera -1 \
  --port /dev/picaso --shots 20 --tag noise_40pct

venv/bin/python picaso_record.py --mode P --events 20 --camera -1 \
  --port /dev/picaso --shots 20 --tag noise_60pct

venv/bin/python picaso_record.py --mode P --events 20 --camera -1 \
  --port /dev/picaso --shots 20 --tag noise_80pct

venv/bin/python picaso_record.py --mode P --events 20 --camera -1 \
  --port /dev/picaso --shots 20 --tag noise_100pct
```

If it is easy to do, one extra run with the sensor foam-mounted instead of
bolted rigidly, at one throttle setting:

```bash
venv/bin/python picaso_record.py --mode P --events 20 --camera -1 \
  --port /dev/picaso --shots 20 --tag noise_60pct_foam
```

This tells us whether the noise travels through the frame or through the air,
which decides whether a better mount fixes it.

This block is the go/no-go for the campaign. Rotor noise has to stay below the
Block 2 echo level, and it also decides the final sweep band.

## Block 4: horizontal flights

Motors on, emitter on. No `--shots` here, so it records until you press Ctrl+C.
Start the command, fly the manoeuvre, then stop it.

```bash
# head-on approach to a wall, slow and steady
venv/bin/python picaso_record.py --mode A --max-range 5 --camera -1 \
  --port /dev/picaso --waveform sweeps/sweep_512_taper30.csv \
  --tag flight_h_approach

# lateral translation along a wall
venv/bin/python picaso_record.py --mode A --max-range 5 --camera -1 \
  --port /dev/picaso --waveform sweeps/sweep_512_taper30.csv \
  --tag flight_h_lateral

# yaw sweep while hovering at a fixed stand-off
venv/bin/python picaso_record.py --mode A --max-range 5 --camera -1 \
  --port /dev/picaso --waveform sweeps/sweep_512_taper30.csv \
  --tag flight_h_yaw
```

## Block 5: vertical flights

```bash
# climb and descend facing a wall
venv/bin/python picaso_record.py --mode A --max-range 5 --camera -1 \
  --port /dev/picaso --waveform sweeps/sweep_512_taper30.csv \
  --tag flight_v_wall
```

## Block 6: promo footage

While the rig is still assembled and working.

## Order of the day

Blocks 1 to 3 run on the ground and should pass before the flight slot, so no
drone time is spent on a broken signal chain. The drone blocks run before the
rover, so Albert can leave when he is done.

## What we need from the drone side

* Perception stack recording during every block, including the stationary ones.
* Shared clock. The recorder writes a `timestamps.csv` into every run folder,
  with a unix and a monotonic timestamp per capture. If it runs on the same
  Jetson as the perception stack we are already synced and nothing else is
  needed. If it runs on a different machine, please tell us, because then the
  two clocks need NTP or chrony between them.
* Static transforms: sonar to lidar, sonar to camera, sonar to flight base.
* Air temperature per session.

## Data

Each run writes one folder of `capture_NNNN.bin` files plus a `timestamps.csv`,
named after the timestamp and the tag. About 327 KB per capture and roughly 3 captures per
second, so budget 3.5 GB per hour and write to NVMe rather than an SD card.

Keep a short log per run: folder, mode, throttle, target distance, temperature,
mount, notes.

## Open questions

* Does the perception stack run on the same Jetson as the recorder?
* Mount to flight base transform?
* USB-C throughput on the Jetson, which sets the real update rate?
* Hall size and wall material?
