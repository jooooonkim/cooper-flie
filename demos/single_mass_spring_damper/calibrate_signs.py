"""Measure which way pitch, roll and yaw-rate commands move the drone.

The zdistance setpoint passes roll/pitch/yawrate to the firmware unchanged,
and the firmware's sign conventions are not obvious (pitch in particular is
historically inverted). This script flies short, gentle steps and prints the
signs to copy into the settings block of msd_pitch.py (and any other
attitude-controlled demo).

Sequence, each step starting from a position hold at the takeoff point:
    +4 deg pitch for 0.4 s   -> does vx go up or down?
    +4 deg roll  for 0.4 s   -> does vy go up or down?
    +30 deg/s yaw rate, 1 s  -> does yaw go up or down?
Each step moves the drone roughly 10-20 cm. Leave about 1 m clear around it.

    Ctrl+C   land
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

# ---- settings -------------------------------------------------------------
URI = "radio://0/80/2M/E7E7E7E7E9"
HEIGHT = 0.7             # m
FENCE = Fence(x=(-1.0, 1.0), y=(-1.0, 1.0), z=(0.2, 1.4))   # m, Lighthouse frame

TILT = 4.0               # deg
TILT_TIME = 0.4          # s
YAWRATE = 30.0           # deg/s
YAW_TIME = 1.0           # s
# ---------------------------------------------------------------------------


def hold(drone, center, seconds, rate):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        drone.position_setpoint(*center, 0.0)
        rate.sleep()


def step(drone, seconds, rate, roll=0.0, pitch=0.0, yawrate=0.0):
    """Stream one attitude command; returns (start state, end state)."""
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
