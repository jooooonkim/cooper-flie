"""msd with no hand. drone hovers at x0, moves to the offset, gets let go and oscillates

the drone is the mass: F = -k (x - x0) - b vx, a = F/m, pitch = atan(a/g)
x, vx come from lighthouse every loop. y held at Y0 with roll, yaw held at 0,
height held onboard (zdistance). ctrl+c lands

python demos/single_mass_spring_damper/msd_set.py --sim
"""

import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np

from common.drone import G, open_drone
from common.safety import Fence
from common.util import CsvLog, Rate

# settings
URI = "radio://0/80/2M/E7E7E7E7E9"

M = 1.0                  # kg
K = 3.0                  # N/m
B = 0.4                  # N*s/m
X0 = 0.0                 # rest position, m
OFFSET = 0.4             # m from X0, where it gets let go
DURATION = 15.0          # s of oscillation, then it lands

HEIGHT = 0.7             # m
Y0 = 0.0                 # m
MAX_TILT_DEG = 12.0      # deg
Y_KP, Y_KD = 4.0, 2.5    # y hold
YAW_KP = 2.0             # deg/s per deg
RATE_HZ = 50

# from calibrate_signs.py, wont fly until verified
PITCH_SIGN = -1
ROLL_SIGN = -1
YAW_SIGN = 1
SIGNS_VERIFIED = False

FENCE = Fence(x=(-1.0, 1.0), y=(-1.0, 1.0), z=(0.2, 1.4))   # m
STATE_TIMEOUT = 0.25     # s with no position -> land
MAX_YAW_ERROR = 25.0     # deg -> land

LOG_FIELDS = ["t", "x", "y", "z", "vx", "vy", "yaw", "pitch_cmd", "roll_cmd", "m", "k", "b", "x0"]


def tilt(accel):
    # m/s^2 -> deg, clamped
    return float(np.clip(math.degrees(math.atan2(accel, G)), -MAX_TILT_DEG, MAX_TILT_DEG))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--sim", action="store_true", help="use the simulated drone")
    parser.add_argument("--uri", default=URI, help=f"radio URI (default {URI})")
    args = parser.parse_args()
    if not args.sim and not SIGNS_VERIFIED:
        sys.exit("signs not verified. run calibrate_signs.py, copy the signs in, set SIGNS_VERIFIED = True")

    signs = {"pitch": PITCH_SIGN, "roll": ROLL_SIGN, "yaw": YAW_SIGN}
    with open_drone(args.uri, sim=args.sim, signs=signs) as drone, CsvLog("msd_set", LOG_FIELDS) as log:
        # 1. take off and hover at the rest position
        drone.takeoff(HEIGHT)
        drone.go_to(X0, Y0, HEIGHT, 0.0, 3.0)

        # 2. move out to the offset
        drone.go_to(X0 + OFFSET, Y0, HEIGHT, 0.0, 3.0)

        # 3. let go, from here its just the spring and damper
        print("let go")
        rate = Rate(RATE_HZ)
        t0 = time.monotonic()
        i = 0
        while time.monotonic() - t0 < DURATION:
            s = drone.state
            if drone.state_age() > STATE_TIMEOUT:
                print("lost position")
                break
            if not FENCE.contains(s.pos, slack=0.05):
                print("left the fence")
                break
            if abs(s.yaw) > MAX_YAW_ERROR:
                print("yaw drifted, check YAW_SIGN")
                break

            # spring + damper on where the drone actually is
            ax = (-K * (s.x - X0) - B * s.vx) / M

            # keep y and yaw still
            ay = -Y_KP * (s.y - Y0) - Y_KD * s.vy
            yawrate = -YAW_KP * s.yaw

            pitch = tilt(ax)
            roll = tilt(ay)
            drone.attitude_setpoint(ROLL_SIGN * roll, PITCH_SIGN * pitch, YAW_SIGN * yawrate, HEIGHT)

            t = time.monotonic() - t0
            log.write(t=round(t, 4), x=s.x, y=s.y, z=s.z, vx=s.vx, vy=s.vy, yaw=s.yaw,
                      pitch_cmd=pitch, roll_cmd=roll, m=M, k=K, b=B, x0=X0)
            if i % 25 == 0:
                print(f"t {t:5.1f}  x {s.x:+.2f}  vx {s.vx:+.2f}  pitch {pitch:+.1f}")
            i += 1
            rate.sleep()

        # 4. stop where it is, then the with block lands it
        print("landing")
        drone.go_to(*FENCE.clamp(drone.state.pos, margin=0.1), 0.0, 1.5)


if __name__ == "__main__":
    main()
