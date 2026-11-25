# single mass spring damper

the drone is the mass. its measured x and vx (lighthouse) go into a virtual
spring and an energy damper, and the acceleration comes from tilting:

```
x'' = -wn^2 x - b (E - Ed) x'
E   = 1/2 m x'^2 + 1/2 k x^2
pitch = x'' / g   (small angle)
```

- E > Ed: the damper takes energy out
- E < Ed: it puts energy in
- E = Ed: plain spring

so it settles into a steady swing with amplitude A = sqrt(2 Ed / k) and holds
it, even with the drone's own drag and tilt lag.

commands go out with `send_zdistance_setpoint(roll, pitch, yawrate, z)`.
height is held onboard, y with roll, yaw with yaw rate.

```
calibrate_signs.py   measures the pitch / roll / yaw signs, run once before flying
msd_energy.py        the demo
msd_rose.py          same idea in the x-z plane: goes around a ring while wobbling, draws a flower
```

## run order

1. `calibrate_signs.py`: flies three small steps and prints the signs. they go
   in the `attitude_setpoint` call in `msd_energy.py`, and in `DEFAULT_SIGNS`
   in `common/drone.py` so the sim matches
2. `msd_energy.py --sim`: try it with no drone
3. `msd_energy.py`: takes off, moves to x0 + `AMPLITUDE`, lets go. runs until
   `q` (land) or `esc` (kill motors). also lands if it leaves the fence or
   loses position

## settings

top of `msd_energy.py`: `M`, `K`, `B` (energy gain), `X0` (rest position),
`AMPLITUDE` (target swing), `HEIGHT`, `MAX_TILT`, `FENCE`.

## notes

- in sim it settles at 0.496 m for a 0.5 m target. drag keeps taking energy
  out, so E sits a bit under Ed. bigger `B` closes the gap
- the period runs a bit longer than 2 pi / wn because of the tilt lag
- esc drops the drone from wherever it is, first try it close to the floor
