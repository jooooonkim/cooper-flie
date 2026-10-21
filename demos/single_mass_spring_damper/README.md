# Single mass-spring-damper (pitch control)

The drone is the mass. Its measured position and velocity along x drive a
virtual spring and damper, and the resulting acceleration is produced by
tilting:

```
F = -k (x - x0) - b vx  [+ k_hand (x_hand - x) + b_hand (v_hand - vx) while grabbed]
a = F / m
pitch = atan(a / g)  ≈ a / g  (clamped to max_tilt_deg)
```

Commands go out through `send_zdistance_setpoint(roll, pitch, yawrate, z)`:
height is held onboard, y is held by a PD loop on roll, and yaw by a
proportional loop on yaw rate. Since the spring and damper act on the
measured state, they work like a PD position controller with k and b as
the gains. The drone's real inertia, attitude lag and drag are inside the
loop, so the flight is the virtual system plus the drone's own physics.

Natural frequency `wn = sqrt(k/m)`, damping ratio `zeta = b / (2 sqrt(k m))`.
Defaults (m 1, k 3, b 0.4) give a 3.6 s period with zeta 0.12, and a 0.4 m
release peaks around 7 deg of tilt.

## Run order

1. `calibrate_signs.py` — flies three small steps and prints the pitch, roll
   and yaw-rate signs. Copy them into the settings block at the top of
   `msd_pitch.py` and set `SIGNS_VERIFIED = True`. Do this once per firmware
   version.
2. `msd_pitch.py --sim` — try it without hardware.
3. `msd_pitch.py` — fly it. Grab to drag the mass along x, open your hand to
   release it. `+`/`-` change k, `[`/`]` change b, `q` lands, `Esc` kills.
4. `plot_log.py` — plots the newest log against the ideal model, re-simulated
   from the drone's measured state at the start of every free segment.

Useful flags: `--offset 0.4` (start displaced), `--no-camera`,
`--headless --duration 15` (no window; for logging runs).

## What to expect

In simulation (drag 0.25 1/s, 60 ms tilt lag) the period comes out about
10 % longer than ideal and the decay is faster. The real drone should show
a similar gap, which is the interesting part to measure.
