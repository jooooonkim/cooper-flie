"""Single mass-spring-damper where the drone IS the mass.

The drone's measured position and velocity along x feed a virtual spring and
damper. The resulting force sets an acceleration, and the acceleration is
produced by tilting: a = g*tan(theta), roughly g*theta for small angles.

    F     = -k (x - x0) - b vx  [+ hand spring while grabbed]
    a     = F / m
    pitch = atan(a / g), clamped to max_tilt_deg

Height is held onboard by the zdistance setpoint, y by a PD loop on roll,
and yaw by a proportional loop on yaw rate. Since the drone's real inertia
and air drag are part of the loop, the motion you see is the virtual system
plus the drone's own physics, not a replay of a simulation.

Grab (close your hand) to attach a second virtual spring from your hand to
the mass and drag it along x. Open your hand to release it mid-swing; it
keeps its momentum.

Keys (window focused):
    + / -   stiffness k up / down       ] / [   damping b up / down
    q       land                        Esc     EMERGENCY stop motors

    python demos/single_mass_spring_damper/msd_pitch.py --sim
    python demos/single_mass_spring_damper/msd_pitch.py --sim --headless --duration 15
    python demos/single_mass_spring_damper/msd_pitch.py
"""

import argparse
import math
import sys
import time
from contextlib import nullcontext
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import cv2
import numpy as np

from common.config import load_config
from common.drone import G, open_drone
from common.hand import HandState, HandTracker, draw_hand, put_lines
from common.mapping import HandMapper
from common.safety import Fence
from common.util import CsvLog, Rate

LOG_FIELDS = ["t", "x", "y", "z", "vx", "vy", "yaw", "grabbed", "x_hand", "force", "ax_cmd",
              "pitch_cmd", "roll_cmd", "yawrate_cmd", "m", "k", "b", "x0"]


def tilt_deg(accel, limit):
    return float(np.clip(math.degrees(math.atan2(accel, G)), -limit, limit))


def draw_scene(img, mapper, p, x, x_hand, grabbed):
    """Wall, spring, damper and mass along the bottom of the image."""
    h, w = img.shape[:2]
    y = int(h * 0.82)
    to_px = lambda value: int(mapper.h_to_u(value) * w)
    wall = to_px(mapper.h_range[0])
    mass = to_px(x)
    rest = to_px(p["x0"])
    half = 22

    cv2.line(img, (wall, y - 45), (wall, y + 45), (200, 200, 200), 3)
    # Spring (zigzag) above the centerline, damper below.
    end = mass - half
    n = 12
    xs = np.linspace(wall, end, 2 * n + 1)
    ys = [y - 12 + (0 if i in (0, 2 * n) else (-9 if i % 2 else 9)) for i in range(2 * n + 1)]
    cv2.polylines(img, [np.array(list(zip(xs, ys)), np.int32)], False, (0, 220, 255), 2, cv2.LINE_AA)
    mid = (wall + end) // 2
    cv2.line(img, (wall, y + 12), (mid, y + 12), (255, 0, 255), 2)
    cv2.rectangle(img, (mid - 14, y + 4), (mid + 14, y + 20), (255, 0, 255), 2)
    cv2.line(img, (mid + 4, y + 12), (end, y + 12), (255, 0, 255), 2)

    cv2.line(img, (rest, y - 40), (rest, y + 40), (150, 150, 150), 1)
    cv2.rectangle(img, (mass - half, y - half), (mass + half, y + half), (40, 40, 230), -1)
    cv2.putText(img, "m", (mass - 7, y + 6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    if x_hand is not None:
        hx = to_px(x_hand)
        cv2.circle(img, (hx, y - 60), 9, (60, 220, 60), -1 if grabbed else 1, cv2.LINE_AA)
        if grabbed:
            cv2.line(img, (hx, y - 60), (mass, y - half), (60, 220, 60), 1, cv2.LINE_AA)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--sim", action="store_true", help="use the simulated drone")
    parser.add_argument("--uri", help="override the radio URI from config.yaml")
    parser.add_argument("--no-camera", action="store_true", help="no hand tracking; blank canvas")
    parser.add_argument("--headless", action="store_true", help="no camera and no window (needs --duration)")
    parser.add_argument("--duration", type=float, help="land after this many seconds")
    parser.add_argument("--offset", type=float, help="initial displacement from x0 in m (overrides config)")
    args = parser.parse_args()
    if args.headless and not args.duration:
        parser.error("--headless needs --duration")

    cfg = load_config()
    p = dict(cfg["single_msd"])
    signs = cfg["attitude"]
    if not args.sim and not signs["verified"]:
        sys.exit("Attitude signs are not verified. Run calibrate_signs.py, update config.yaml, "
                 "then set attitude.verified: true.")
    if cfg["mapping"]["horizontal"] != "x":
        sys.exit("This demo moves along x; set mapping.horizontal: x in config.yaml.")

    fence = Fence(cfg["flight"]["fence"])
    mapper = HandMapper(cfg["mapping"], fence)
    use_camera = not (args.no_camera or args.headless)
    timeout = cfg["flight"]["state_timeout"]
    offset = p["start_offset"] if args.offset is None else args.offset
    z = p["height"]

    tracker = HandTracker(cfg) if use_camera else nullcontext()
    with open_drone(cfg, sim=args.sim, uri=args.uri) as drone, tracker, CsvLog("single_msd", LOG_FIELDS) as log:
        print(f"Logging to {log.path}")
        drone.takeoff(z)
        start = fence.clamp([p["x0"] + offset, p["y0"], z], margin=0.1)
        drone.go_to(*start, 0.0, 3.0)

        rate = Rate(p["rate_hz"])
        t0 = time.monotonic()
        canvas = np.full((cfg["camera"]["height"], cfg["camera"]["width"], 3), 30, np.uint8)
        last_id = -1
        reason = "done"
        kill = False
        print("Running. q land, Esc kill, +/- stiffness, [/] damping.")

        while True:
            now = time.monotonic()
            s = drone.state
            if args.duration and now - t0 > args.duration:
                break
            if drone.state_age() > timeout:
                reason = "lost the position estimate"
                break
            if not fence.contains(s.pos, slack=0.05):
                reason = f"left the fence at ({s.x:+.2f}, {s.y:+.2f}, {s.z:.2f})"
                break
            if abs(s.yaw) > p["max_yaw_error"]:
                reason = f"yaw drifted to {s.yaw:+.0f} deg (check attitude.yaw sign)"
                break

            frame_id, frame, hand = tracker.latest() if use_camera else (0, None, HandState())
            grabbed = hand.detected and hand.grabbed

            # Virtual physics on the measured state.
            force = -p["k"] * (s.x - p["x0"]) - p["b"] * s.vx
            x_hand = None
            if hand.detected:
                x_hand = mapper.to_world(hand.u, hand.v)["x"]
            if grabbed:
                v_hand = mapper.velocity_to_world(hand.du, hand.dv)["x"]
                force += p["k_hand"] * (x_hand - s.x) + p["b_hand"] * (v_hand - s.vx)
            ax = force / p["m"]
            ay = -p["y_kp"] * (s.y - p["y0"]) - p["y_kd"] * s.vy

            # World-frame acceleration to body-frame tilt.
            c, sn = math.cos(math.radians(s.yaw)), math.sin(math.radians(s.yaw))
            pitch = tilt_deg(c * ax + sn * ay, p["max_tilt_deg"])
            roll = tilt_deg(-sn * ax + c * ay, p["max_tilt_deg"])
            yawrate = float(np.clip(-p["yaw_kp"] * s.yaw, -p["max_yawrate"], p["max_yawrate"]))
            drone.attitude_setpoint(signs["roll"] * roll, signs["pitch"] * pitch, signs["yaw"] * yawrate, z)

            log.write(t=round(now - t0, 4), x=s.x, y=s.y, z=s.z, vx=s.vx, vy=s.vy, yaw=s.yaw,
                      grabbed=int(grabbed), x_hand="" if x_hand is None else x_hand, force=force,
                      ax_cmd=ax, pitch_cmd=pitch, roll_cmd=roll, yawrate_cmd=yawrate,
                      m=p["m"], k=p["k"], b=p["b"], x0=p["x0"])

            if not args.headless:
                new_frame = frame is not None and frame_id != last_id
                if new_frame or not use_camera:
                    last_id = frame_id
                    img = frame.copy() if new_frame else canvas.copy()
                    draw_hand(img, hand)
                    draw_scene(img, mapper, p, s.x, x_hand, grabbed)
                    wn = math.sqrt(p["k"] / p["m"])
                    zeta = p["b"] / (2 * math.sqrt(p["k"] * p["m"]))
                    put_lines(img, [
                        f"k {p['k']:.1f}  b {p['b']:.2f}  m {p['m']:.1f}   wn {wn:.2f} rad/s  zeta {zeta:.2f}",
                        f"x {s.x:+.2f} m  vx {s.vx:+.2f} m/s  pitch {pitch:+.1f} deg",
                        ("GRABBED" if grabbed else "free") + ("   [SIM]" if args.sim else ""),
                    ])
                    cv2.imshow("single mass-spring-damper", img)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                if key == 27:
                    kill = True
                    break
                if key in (ord("+"), ord("=")):
                    p["k"] = round(p["k"] + 0.5, 2)
                elif key == ord("-"):
                    p["k"] = max(0.5, round(p["k"] - 0.5, 2))
                elif key == ord("]"):
                    p["b"] = round(p["b"] + 0.1, 2)
                elif key == ord("["):
                    p["b"] = max(0.0, round(p["b"] - 0.1, 2))
            rate.sleep()

        if kill:
            print("EMERGENCY STOP")
            drone.kill()
        else:
            print(f"Stopping ({reason}), landing.")
            drone.go_to(*fence.clamp(drone.state.pos, margin=0.1), 0.0, 1.5)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
