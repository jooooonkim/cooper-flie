"""figure out which way pitch / roll / yawrate commands actually move the drone

pitch sign on the cf is weird (inverted) so just measure it.
+4 deg pitch 0.4 s -> vx?, +4 deg roll 0.4 s -> vy?, +30 deg/s yaw 1 s -> yaw?
each step moves it ~10-20 cm, keep ~1 m clear. copy the printed signs into msd_pitch.py

python demos/single_mass_spring_damper/calibrate_signs.py [--sim]
"""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from common.drone import open_drone
from common.safety import Fence
from common.util import Rate

# settings
URI = "radio://0/80/2M/E7E7E7E7E9"
HEIGHT = 0.7             # m
FENCE = Fence(x=(-1.0, 1.0), y=(-1.0, 1.0), z=(0.2, 1.4))   # m

TILT = 4.0               # deg
TILT_TIME = 0.4          # s
YAWRATE = 30.0           # deg/s
YAW_TIME = 1.0           # s


def hold(drone, center, seconds, rate):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        drone.position_setpoint(*center, 0.0)
        rate.sleep()


def step(drone, seconds, rate, roll=0.0, pitch=0.0, yawrate=0.0):
    # returns state before and after
    start = drone.state
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if not FENCE.contains(drone.state.pos):
            raise RuntimeError("Left the fence during a step; aborting.")
        drone.attitude_setpoint(roll, pitch, yawrate, HEIGHT)
        rate.sleep()
    return start, drone.state


def sign_word(sign):
    return "1" if sign > 0 else "-1"


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--sim", action="store_true", help="use the simulated drone")
    parser.add_argument("--uri", default=URI, help=f"radio URI (default {URI})")
    args = parser.parse_args()

    rate = Rate(50)
    with open_drone(args.uri, sim=args.sim) as drone:
        drone.takeoff(HEIGHT)
        s = drone.state
        center = FENCE.clamp([s.x, s.y, HEIGHT], margin=0.3)
        drone.go_to(*center, 0.0, 2.0)

        hold(drone, center, 1.5, rate)
        a, b = step(drone, TILT_TIME, rate, pitch=TILT)
        d_vx = b.vx - a.vx
        print(f"pitch +{TILT:.0f} deg: vx changed {d_vx:+.3f} m/s")

        hold(drone, center, 2.5, rate)
        a, b = step(drone, TILT_TIME, rate, roll=TILT)
        d_vy = b.vy - a.vy
        print(f"roll  +{TILT:.0f} deg: vy changed {d_vy:+.3f} m/s")

        hold(drone, center, 2.5, rate)
        a, b = step(drone, YAW_TIME, rate, yawrate=YAWRATE)
        d_yaw = (b.yaw - a.yaw + 180.0) % 360.0 - 180.0
        print(f"yawrate +{YAWRATE:.0f} deg/s: yaw changed {d_yaw:+.1f} deg")

        hold(drone, center, 2.5, rate)

    weak = [name for name, value, floor in (("pitch", d_vx, 0.05), ("roll", d_vy, 0.05), ("yaw", d_yaw, 5.0))
            if abs(value) < floor]
    print("\nCopy into the settings block of msd_pitch.py:")
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
