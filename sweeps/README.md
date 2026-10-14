# Transmit sweep

Use **`sweep_512_taper30.csv`**. It is the default in every script, so no
`--sweep` flag is needed anywhere.

What it is:

* Hyperbolic FM downsweep, 80 kHz down to 20 kHz
* 512 samples at 444.444 kHz, so 1.15 ms long
* 30 % Hanning ramps at both ends
* Transmit ring-down has decayed by about 0.20 m, which is the near blind zone

Why these numbers: the ramps push range sidelobes down to about -35 dB, so a
strong echo from the mount does not mask weaker ones nearby. The short length
halves the blind zone compared to a 1024 sample sweep, and costs about 4.8 dB
of transmit energy, which we can afford.

The matched filter has to match what was actually emitted, so if you change
the sweep you must pass the same file to the evaluation scripts.

## Making a different one

`make_sweep_512_taper30.m` generates the CSV and documents every design
choice. It calls `fm_sweep.m`. Edit `fStart`, `fEnd` or `nSamples` and run it
in MATLAB.

One change we may make after the self-noise measurement: if rotor noise turns
out to dominate below 30 kHz, move the low edge up to 30 kHz. Rotor noise
falls off with frequency while air absorption rises, so the band placement is
a compromise and the noise measurement decides it.
