"""msd with energy damping, holds a set amplitude. q land, esc kill

x'' = -wn^2 x - b (E - Ed) x',  E = 1/2 m x'^2 + 1/2 k x^2

python3 demos/single_mass_spring_damper/msd_energy.py --sim
"""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from common.drone import G, open_drone
from common.keys import Keys
from common.safety import Fence
from common.util import Rate

URI = "radio://0/50/2M/E7E7E7E7E1"

M = 1.0                  # kg
K = 3.0                  # N/m
B = 10.0                 # energy gain (? iono if thats the right word)
X0 = 0.0                 # initial position, m
AMPLITUDE = 0.5          # target, m
E_D = 0.5 * K * AMPLITUDE ** 2   # desired energy level, J

HEIGHT = 0.7             # m
MAX_TILT = 12.0          # deg
Y_KP, Y_KD = 4.0, 2.5    # y hold
YAW_KP = 2.0             # yaw hold
FENCE = Fence(x=(-1.5, 1.5), y=(-1.5, 1.5), z=(0.2, 1.4))   # m


def tilt(accel):
    # small angle, a = g theta
    return max(-MAX_TILT, min(MAX_TILT, math.degrees(accel / G)))


def main():
    sim = "--sim" in sys.argv
    with open_drone(URI, sim=sim) as drone, Keys(drone) as keys:
        drone.takeoff(HEIGHT)
        drone.go_to(X0 + AMPLITUDE, 0.0, HEIGHT, 0.0, 3.0)   # start at amplitude, E = Ed
        print("let go. q land, esc kill")

        rate = Rate(50)
        while not keys.stop:
            s = drone.state
            if drone.state_age() > 0.25 or not FENCE.contains(s.pos, slack=0.05):
                print("lost position or left fence")
                break

            x = s.x - X0
            e = 0.5 * M * s.vx ** 2 + 0.5 * K * x ** 2
            ax = -K / M * x - B * (e - E_D) * s.vx      # spring + energy damper
            ay = -Y_KP * s.y - Y_KD * s.vy              # hold y = 0

            pitch, roll = tilt(ax), tilt(ay)
            drone.attitude_setpoint(-1 * roll, -1 * pitch, -YAW_KP * s.yaw, HEIGHT)
            rate.sleep()

        if not keys.kill:
            print("landing")


if __name__ == "__main__":
    main()
