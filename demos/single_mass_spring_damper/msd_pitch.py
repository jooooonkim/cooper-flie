"""single msd, the drone is the mass

F = -k (x - x0) - b vx (+ hand spring when grabbed), a = F/m, pitch = atan(a/g) ~ a/g
height held onboard (zdistance), y held w/ roll, yaw held w/ yawrate.
uses the measured x, vx so the drone's real inertia/drag is in the loop too

grab to drag it along x, let go mid swing and it keeps going
keys: +/- k, ]/[ b, q land, esc kill

python demos/single_mass_spring_damper/msd_pitch.py --sim
python demos/single_mass_spring_damper/msd_pitch.py --sim --headless --duration 15
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

from common.drone import G, open_drone
from common.hand import HandState, HandTracker, draw_hand, put_lines
from common.mapping import HandMapper
from common.safety import Fence
from common.util import CsvLog, Rate

# settings
URI = "radio://0/80/2M/E7E7E7E7E9"

# wn = sqrt(K/M), zeta = B / (2 sqrt(K M))
M = 1.0                  # kg
K = 3.0                  # N/m
B = 0.4                  # N*s/m
X0 = 0.0                 # rest position, m
START_OFFSET = 0.0       # m from X0 at start (--offset overrides)
K_HAND = 8.0             # hand spring, N/m
B_HAND = 1.0             # N*s/m

HEIGHT = 0.7             # m
Y0 = 0.0                 # m
MAX_TILT_DEG = 12.0      # small angle stuff breaks past ~15 deg
Y_KP, Y_KD = 4.0, 2.5    # y hold
YAW_KP = 2.0             # deg/s per deg
MAX_YAWRATE = 60.0       # deg/s
RATE_HZ = 50

# from calibrate_signs.py, wont fly until verified
PITCH_SIGN = -1
ROLL_SIGN = -1
YAW_SIGN = 1
SIGNS_VERIFIED = False

FENCE = Fence(x=(-1.0, 1.0), y=(-1.0, 1.0), z=(0.2, 1.4))   # m
STATE_TIMEOUT = 0.25     # s with no position -> land
MAX_YAW_ERROR = 25.0     # deg -> land

CAMERA = 0
GRAB_BELOW = 1.30        # from 02_hand_tracking
RELEASE_ABOVE = 1.60
ACTIVE_U = (0.15, 0.85)

SIGNS = {"pitch": PITCH_SIGN, "roll": ROLL_SIGN, "yaw": YAW_SIGN}
LOG_FIELDS = ["t", "x", "y", "z", "vx", "vy", "yaw", "grabbed", "x_hand", "force", "ax_cmd",
              "pitch_cmd", "roll_cmd", "yawrate_cmd", "m", "k", "b", "x0"]


def tilt_deg(accel, limit):
    return float(np.clip(math.degrees(math.atan2(accel, G)), -limit, limit))


def draw_scene(img, mapper, x, x_hand, grabbed):
    h, w = img.shape[:2]
    y = int(h * 0.82)
    to_px = lambda value: int(mapper.h_to_u(value) * w)
    wall = to_px(mapper.h_range[0])
    mass = to_px(x)
    rest = to_px(X0)
    half = 22

    cv2.line(img, (wall, y - 45), (wall, y + 45), (200, 200, 200), 3)
    # spring on top, damper below
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
    parser.add_argument("--uri", default=URI, help=f"radio URI (default {URI})")
    parser.add_argument("--no-camera", action="store_true", help="no hand tracking; blank canvas")
    parser.add_argument("--headless", action="store_true", help="no camera and no window (needs --duration)")
    parser.add_argument("--duration", type=float, help="land after this many seconds")
    parser.add_argument("--offset", type=float, help="initial displacement from X0 in m")
    args = parser.parse_args()
    if args.headless and not args.duration:
        parser.error("--headless needs --duration")
    if not args.sim and not SIGNS_VERIFIED:
        sys.exit("Attitude signs are not verified. Run calibrate_signs.py, copy the printed "
                 "signs into this script, then set SIGNS_VERIFIED = True.")

    k, b = K, B  # can change these live
    mapper = HandMapper(FENCE, "x", "z", active_u=ACTIVE_U)
    use_camera = not (args.no_camera or args.headless)
    offset = START_OFFSET if args.offset is None else args.offset

    tracker = (HandTracker(camera=CAMERA, grab_below=GRAB_BELOW, release_above=RELEASE_ABOVE)
               if use_camera else nullcontext())
    with open_drone(args.uri, sim=args.sim, signs=SIGNS) as drone, tracker, \
            CsvLog("single_msd", LOG_FIELDS) as log:
        print(f"Logging to {log.path}")
        drone.takeoff(HEIGHT)
        start = FENCE.clamp([X0 + offset, Y0, HEIGHT], margin=0.1)
        drone.go_to(*start, 0.0, 3.0)

        rate = Rate(RATE_HZ)
        t0 = time.monotonic()
        canvas = np.full((480, 640, 3), 30, np.uint8)
        last_id = -1
        reason = "done"
        kill = False
        print("Running. q land, Esc kill, +/- stiffness, [/] damping.")

        while True:
            now = time.monotonic()
            s = drone.state
            if args.duration and now - t0 > args.duration:
                break
            if drone.state_age() > STATE_TIMEOUT:
                reason = "lost the position estimate"
                break
            if not FENCE.contains(s.pos, slack=0.05):
                reason = f"left the fence at ({s.x:+.2f}, {s.y:+.2f}, {s.z:.2f})"
                break
            if abs(s.yaw) > MAX_YAW_ERROR:
                reason = f"yaw drifted to {s.yaw:+.0f} deg (check YAW_SIGN)"
                break

            frame_id, frame, hand = tracker.latest() if use_camera else (0, None, HandState())
            grabbed = hand.detected and hand.grabbed

            # virtual spring/damper on measured state
            force = -k * (s.x - X0) - b * s.vx
            x_hand = None
            if hand.detected:
                x_hand = mapper.to_world(hand.u, hand.v)["x"]
            if grabbed:
                v_hand = mapper.velocity_to_world(hand.du, hand.dv)["x"]
                force += K_HAND * (x_hand - s.x) + B_HAND * (v_hand - s.vx)
            ax = force / M
            ay = -Y_KP * (s.y - Y0) - Y_KD * s.vy

            # world accel -> body tilt
            c, sn = math.cos(math.radians(s.yaw)), math.sin(math.radians(s.yaw))
            pitch = tilt_deg(c * ax + sn * ay, MAX_TILT_DEG)
            roll = tilt_deg(-sn * ax + c * ay, MAX_TILT_DEG)
            yawrate = float(np.clip(-YAW_KP * s.yaw, -MAX_YAWRATE, MAX_YAWRATE))
            drone.attitude_setpoint(ROLL_SIGN * roll, PITCH_SIGN * pitch, YAW_SIGN * yawrate, HEIGHT)

            log.write(t=round(now - t0, 4), x=s.x, y=s.y, z=s.z, vx=s.vx, vy=s.vy, yaw=s.yaw,
                      grabbed=int(grabbed), x_hand="" if x_hand is None else x_hand, force=force,
                      ax_cmd=ax, pitch_cmd=pitch, roll_cmd=roll, yawrate_cmd=yawrate,
                      m=M, k=k, b=b, x0=X0)

            if not args.headless:
                new_frame = frame is not None and frame_id != last_id
                if new_frame or not use_camera:
                    last_id = frame_id
                    img = frame.copy() if new_frame else canvas.copy()
                    draw_hand(img, hand)
                    draw_scene(img, mapper, s.x, x_hand, grabbed)
                    wn = math.sqrt(k / M)
                    zeta = b / (2 * math.sqrt(k * M))
                    put_lines(img, [
                        f"k {k:.1f}  b {b:.2f}  m {M:.1f}   wn {wn:.2f} rad/s  zeta {zeta:.2f}",
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
                    k = round(k + 0.5, 2)
                elif key == ord("-"):
                    k = max(0.5, round(k - 0.5, 2))
                elif key == ord("]"):
                    b = round(b + 0.1, 2)
                elif key == ord("["):
                    b = max(0.0, round(b - 0.1, 2))
            rate.sleep()

        if kill:
            print("EMERGENCY STOP")
            drone.kill()
        else:
            print(f"Stopping ({reason}), landing.")
            drone.go_to(*FENCE.clamp(drone.state.pos, margin=0.1), 0.0, 1.5)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
