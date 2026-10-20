# cooper-flie
Dynamic systems flown by Crazyflie Drones @ Cooper Union Dynamics and Controls Lab

Webcam hand tracking for interaction. Lighthouse positioning.

```
common/        shared code: drone wrapper (+ simulated drone), hand tracker, mapping, safety
test/          bring-up scripts, run in order
demos/         dynamic-system demos
2024_demos/    the original 2024 demos, kept as they were
config.yaml    radio URIs, fence, camera, hand thresholds, demo parameters
```

## Setup

```
python -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt
```

The hand-tracking model (`models/hand_landmarker.task`, about 8 MB) downloads
on first use. Set the drone URI in `config.yaml` and check that `flight.fence`
fits your Lighthouse space.

Every flying script takes `--sim` to run against a simulated drone first.

## Tests

| Script | What it checks |
|---|---|
| `test/01_keyboard_flight.py` | radio, positioning, flight. `w/a/s/d` front/left/back/right, `o/p` up/down, `q` land, `Esc` kill |
| `test/02_hand_tracking.py` | camera and grab detection, no drone. Use it to tune `hand` and `mapping` in config |
| `test/03_hand_drone.py` | grab = move the drone, let go = it holds position |

## Demos

- `demos/single_mass_spring_damper/` — the drone acts as the mass, driven
  by pitch commands. See its README.

## Safety

- Streaming setpoints must keep arriving; if a script freezes, the firmware
  levels the drone after ~0.5 s and stops the motors after ~2 s.
- Every script lands on `q`, Ctrl+C, leaving the fence or losing the position
  estimate. `Esc` stops the motors immediately.
- Attitude-controlled demos refuse to fly until `attitude.verified: true`.
