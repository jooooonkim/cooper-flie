"""msd with energy damping, holds a set amplitude. hover at x0, move to offset, let go. runs until q (land) or esc (kill)

x'' = -wn^2 x - b (E - Ed) x',  E = 1/2 m x'^2 + 1/2 k x^2

python3 demos/single_mass_spring_damper/msd_energy.py --sim
"""

import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np

from common.drone import G, open_drone
from common.keys import Keys
from common.safety import Fence
from common.util import CsvLog, Rate

# settings
URI = "radio://0/50/2M/E7E7E7E7E1"

M = 1.0                  # kg
K = 3.0                  # N/m
B = 10.0                 # energy gain, 1/(J*s)
X0 = 0.0                 # rest position, m
AMPLITUDE = 0.5          # target, m
OFFSET = 0.0             # release point, m
MAX_TIME = 300.0         # s, battery backstop

WN2 = K / M                          # wn^2
E_D = 0.5 * K * AMPLITUDE ** 2       # J

HEIGHT = 0.7             # m
Y0 = 0.0                 # m
MAX_TILT_DEG = 12.0      # deg
Y_KP, Y_KD = 4.0, 2.5    # y hold
YAW_KP = 2.0             # deg/s per deg
RATE_HZ = 50

# from calibrate_signs.py
PITCH_SIGN = -1
ROLL_SIGN = -1
YAW_SIGN = 1
SIGNS_VERIFIED = True

FENCE = Fence(x=(-1.0, 1.0), y=(-1.0, 1.0), z=(0.2, 1.4))   # m
STATE_TIMEOUT = 0.25     # s
MAX_YAW_ERROR = 25.0     # deg

LOG_FIELDS = ["t", "x", "y", "z", "vx", "vy", "yaw", "pitch_cmd", "roll_cmd",
              "m", "k", "b", "x0", "e", "e_d"]


def tilt(accel):
    return float(np.clip(math.degrees(math.atan2(accel, G)), -MAX_TILT_DEG, MAX_TILT_DEG))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--sim", action="store_true", help="use the simulated drone")
    parser.add_argument("--uri", default=URI, help=f"radio URI (default {URI})")
    args = parser.parse_args()
    if not args.sim and not SIGNS_VERIFIED:
        sys.exit("signs not verified. run calibrate_signs.py, copy the signs in, set SIGNS_VERIFIED = True")

    signs = {"pitch": PITCH_SIGN, "roll": ROLL_SIGN, "yaw": YAW_SIGN}
    with open_drone(args.uri, sim=args.sim, signs=signs) as drone, CsvLog("msd_energy", LOG_FIELDS) as log, \
            Keys(drone) as keys:
        # 1. hover at x0
        drone.takeoff(HEIGHT)
        drone.go_to(X0, Y0, HEIGHT, 0.0, 3.0)

        # 2. move to offset
        drone.go_to(X0 + OFFSET, Y0, HEIGHT, 0.0, 3.0)

        # 3. let go
        if not keys.stop:
            print(f"let go, Ed {E_D:.3f} J. q land, esc kill")
        rate = Rate(RATE_HZ)
        t0 = time.monotonic()
        i = 0
        # runnit baby
        while not keys.stop:
            if time.monotonic() - t0 > MAX_TIME:
                print("max time")
                break
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

            # energy
            x = s.x - X0
            e = 0.5 * M * s.vx ** 2 + 0.5 * K * x ** 2

            # spring + energy damper
            ax = -WN2 * x - B * (e - E_D) * s.vx

            # hold y, yaw
            ay = -Y_KP * (s.y - Y0) - Y_KD * s.vy
            yawrate = -YAW_KP * s.yaw

            pitch = tilt(ax)
            roll = tilt(ay)
            drone.attitude_setpoint(ROLL_SIGN * roll, PITCH_SIGN * pitch, YAW_SIGN * yawrate, HEIGHT)

            t = time.monotonic() - t0
            log.write(t=round(t, 4), x=s.x, y=s.y, z=s.z, vx=s.vx, vy=s.vy, yaw=s.yaw,
                      pitch_cmd=pitch, roll_cmd=roll, m=M, k=K, b=B, x0=X0, e=e, e_d=E_D)
            if i % 25 == 0:
                print(f"t {t:5.1f}  x {s.x:+.2f}  vx {s.vx:+.2f}  E {e:.3f}  pitch {pitch:+.1f}")
            i += 1
            rate.sleep()

        # 4. stop, land
        if not keys.kill:
            print("landing")
        drone.go_to(*FENCE.clamp(drone.state.pos, margin=0.1), 0.0, 1.5)


if __name__ == "__main__":
    main()
