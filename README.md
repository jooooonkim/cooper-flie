# cooper-flie
Dynamic systems flown by Crazyflie Drones @ Cooper Union Dynamics and Controls Lab

Webcam hand tracking for interaction. Lighthouse positioning.

```
setup/         drone + cfclient + radio + lighthouse setup, start here
common/        shared code: drone wrapper (+ simulated drone), hand tracker, mapping, safety
test/          bring-up scripts, run in order
demos/         dynamic-system demos
2024_demos/    the original 2024 demos, kept as they were
```

## Setup

```
python -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt
```

The hand-tracking model (`models/hand_landmarker.task`, about 8 MB) downloads
on first use. For the drone, radio, cfclient and lighthouse side see
`setup/README.md`.

Each script keeps its own settings in a block at the top (radio URI, fence,
gains, hand thresholds), so different scripts can use different values.
Check `URI` and `FENCE` before flying. The URI can also be given on the
command line: `--uri radio://0/80/2M/E7E7E7E7E9`.

Every flying script takes `--sim` to run against a simulated drone first.

## Tests

| Script | What it checks |
|---|---|
| `test/01_keyboard_flight.py` | radio, positioning, flight. `w/a/s/d` front/left/back/right, `o/p` up/down, `q` land, `Esc` kill |
| `test/02_hand_tracking.py` | camera and grab detection, no drone. Use it to tune the grab thresholds, then copy them into the flying scripts |
| `test/03_hand_drone.py` | grab = move the drone, let go = it holds position |

## Demos

- `demos/single_mass_spring_damper/` — the drone acts as the mass, driven
  by pitch commands. See its README.

## Safety

- Streaming setpoints must keep arriving; if a script freezes, the firmware
  levels the drone after ~0.5 s and stops the motors after ~2 s.
- Every script lands on `q`, Ctrl+C, leaving the fence or losing the position
  estimate. `Esc` stops the motors immediately.
- Attitude-controlled demos refuse to fly until `SIGNS_VERIFIED = True` in that script.
