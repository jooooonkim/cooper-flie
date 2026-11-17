"""measure pitch/roll/yaw signs with small +/- steps. q land, esc kill

python demos/single_mass_spring_damper/calibrate_signs.py [--sim]
"""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np

from common.drone import open_drone
from common.keys import Keys
from common.safety import Fence
from common.util import Rate

# settings
URI = "radio://0/50/2M/E7E7E7E7E1"
HEIGHT = 0.7             # m
FENCE = Fence(x=(-1.0, 1.0), y=(-1.0, 1.0), z=(0.2, 1.4))   # m

TILT = 3.0               # deg
TILT_TIME = 0.3          # s
YAWRATE = 20.0           # deg/s
YAW_TIME = 1.0           # s
RETURN_SPEED = 0.15      # m/s, back to center between steps
MAX_DIST = 0.25          # m from center -> stop and land


class Stopped(Exception):
    pass


def hold(drone, keys, center, seconds, rate):
    # ease back to center, then hold
    target = drone.state.pos
    target[2] = center[2]
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if keys.stop:
            raise Stopped("stopped early")
        step = center - target
        dist = np.linalg.norm(step)
        if dist > RETURN_SPEED * rate.dt:
            step *= RETURN_SPEED * rate.dt / dist
        target = target + step
        drone.position_setpoint(*target, 0.0)
        rate.sleep()


def step(drone, keys, center, seconds, rate, roll=0.0, pitch=0.0, yawrate=0.0):
    # state before, after
    start = drone.state
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if keys.stop:
            raise Stopped("stopped early")
        s = drone.state
        if np.linalg.norm(s.pos[:2] - center[:2]) > MAX_DIST or not FENCE.contains(s.pos):
            raise Stopped("moved too far")
        drone.attitude_setpoint(roll, pitch, yawrate, HEIGHT)
        rate.sleep()
    return start, drone.state


def pair(drone, keys, center, rate, axis):
    # + step then - step. difference cancels trim and most of the movement
    out = []
    for sign in (1, -1):
        hold(drone, keys, center, 2.5, rate)
        a, b = step(drone, keys, center, TILT_TIME, rate, **{axis: sign * TILT})
        v = "vx" if axis == "pitch" else "vy"
        out.append(getattr(b, v) - getattr(a, v))
        print(f"{axis} {sign * TILT:+.0f} deg: {v} changed {out[-1]:+.3f} m/s")
    return out[0] - out[1]


def sign_word(sign):
    return "1" if sign > 0 else "-1"


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--sim", action="store_true", help="use the simulated drone")
    parser.add_argument("--uri", default=URI, help=f"radio URI (default {URI})")
    args = parser.parse_args()

    rate = Rate(50)
    try:
        with open_drone(args.uri, sim=args.sim) as drone, Keys(drone) as keys:
            drone.takeoff(HEIGHT)
            s = drone.state
            center = FENCE.clamp([s.x, s.y, HEIGHT], margin=0.3)
            drone.go_to(*center, 0.0, 2.0)

            d_vx = pair(drone, keys, center, rate, "pitch")
            d_vy = pair(drone, keys, center, rate, "roll")

            hold(drone, keys, center, 2.5, rate)
            a, b = step(drone, keys, center, YAW_TIME, rate, yawrate=YAWRATE)
            d_yaw = (b.yaw - a.yaw + 180.0) % 360.0 - 180.0
            print(f"yawrate +{YAWRATE:.0f} deg/s: yaw changed {d_yaw:+.1f} deg")

            hold(drone, keys, center, 2.5, rate)
    except Stopped as e:
        sys.exit(f"{e}, no signs measured")

    weak = [name for name, value, floor in (("pitch", d_vx, 0.05), ("roll", d_vy, 0.05), ("yaw", d_yaw, 5.0))
            if abs(value) < floor]
    print("\nCopy into the settings block of msd_energy.py:")
    print(f"PITCH_SIGN = {sign_word(d_vx)}")
    print(f"ROLL_SIGN = {sign_word(d_vy)}")
    print(f"YAW_SIGN = {sign_word(d_yaw)}")
    if weak:
        print(f"\nThe response for {', '.join(weak)} was too small to trust. "
              "Run it again before setting SIGNS_VERIFIED = True.")
    else:
        print("SIGNS_VERIFIED = True")


if __name__ == "__main__":
    main()
