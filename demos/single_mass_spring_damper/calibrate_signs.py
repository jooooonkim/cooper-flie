"""Measure which way pitch, roll and yaw-rate commands move the drone.

The zdistance setpoint passes roll/pitch/yawrate to the firmware unchanged,
and the firmware's sign conventions are not obvious (pitch in particular is
historically inverted). This script flies short, gentle steps and prints the
signs to put in the `attitude` section of config.yaml.

Sequence, each step starting from a position hold at the takeoff point:
    +4 deg pitch for 0.4 s   -> does vx go up or down?
    +4 deg roll  for 0.4 s   -> does vy go up or down?
    +30 deg/s yaw rate, 1 s  -> does yaw go up or down?
Each step moves the drone roughly 10-20 cm. Leave about 1 m clear around it.

    q / Ctrl+C   land
    python demos/single_mass_spring_damper/calibrate_signs.py [--sim]
"""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from common.config import load_config
from common.drone import open_drone
from common.safety import Fence
from common.util import Rate

TILT = 4.0  # deg
TILT_TIME = 0.4  # s
YAWRATE = 30.0  # deg/s
YAW_TIME = 1.0  # s


def hold(drone, center, seconds, rate):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        drone.position_setpoint(*center, 0.0)
        rate.sleep()


def step(drone, fence, z, seconds, rate, roll=0.0, pitch=0.0, yawrate=0.0):
    """Stream one attitude command; returns (start state, end state)."""
    start = drone.state
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if not fence.contains(drone.state.pos):
            raise RuntimeError("Left the fence during a step; aborting.")
        drone.attitude_setpoint(roll, pitch, yawrate, z)
        rate.sleep()
    return start, drone.state


def sign_word(sign):
    return "+1" if sign > 0 else "-1"


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--sim", action="store_true", help="use the simulated drone")
    parser.add_argument("--uri", help="override the radio URI from config.yaml")
    args = parser.parse_args()

    cfg = load_config()
    fence = Fence(cfg["flight"]["fence"])
    z = cfg["single_msd"]["height"]
    rate = Rate(50)

    with open_drone(cfg, sim=args.sim, uri=args.uri) as drone:
        drone.takeoff(z)
        s = drone.state
        center = fence.clamp([s.x, s.y, z], margin=0.3)
        drone.go_to(*center, 0.0, 2.0)

        hold(drone, center, 1.5, rate)
        a, b = step(drone, fence, z, TILT_TIME, rate, pitch=TILT)
        d_vx = b.vx - a.vx
        print(f"pitch +{TILT:.0f} deg: vx changed {d_vx:+.3f} m/s")

        hold(drone, center, 2.5, rate)
        a, b = step(drone, fence, z, TILT_TIME, rate, roll=TILT)
        d_vy = b.vy - a.vy
        print(f"roll  +{TILT:.0f} deg: vy changed {d_vy:+.3f} m/s")

        hold(drone, center, 2.5, rate)
        a, b = step(drone, fence, z, YAW_TIME, rate, yawrate=YAWRATE)
        d_yaw = (b.yaw - a.yaw + 180.0) % 360.0 - 180.0
        print(f"yawrate +{YAWRATE:.0f} deg/s: yaw changed {d_yaw:+.1f} deg")

        hold(drone, center, 2.5, rate)

    weak = [name for name, value, floor in (("pitch", d_vx, 0.05), ("roll", d_vy, 0.05), ("yaw", d_yaw, 5.0))
            if abs(value) < floor]
    print("\nPut these in config.yaml under `attitude`:")
    print(f"  pitch: {sign_word(d_vx)}")
    print(f"  roll: {sign_word(d_vy)}")
    print(f"  yaw: {sign_word(d_yaw)}")
    if weak:
        print(f"\nThe response for {', '.join(weak)} was too small to trust. Run it again before setting verified: true.")
    else:
        print("  verified: true")


if __name__ == "__main__":
    main()
