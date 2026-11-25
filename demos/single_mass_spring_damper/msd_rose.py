"""goes around a ring in the x-z plane while wobbling in and out of it like a rose. WOAH.

energy used to hold both the wobble size and the spin speed. q land, esc kill

radial:      rho'' = -k/m rho - b_r (Er - Erd) rho',  rho = r - r0
tangential:  vt'   = -b_t (Et - Etd) v,               Et = 1/2 m vt^2
petals per lap = sqrt(k/m) r0 / v. whole number closes, anything else keeps drifting

python3 demos/single_mass_spring_damper/msd_rose.py --sim
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
K = 3.0                  # N/m, wobble stiffness
XC, ZC = 0.0, 0.8        # ring center, m
R0 = 0.35                # ring radius, m
A = 0.1                  # wobble size, m
PETALS = 4.0             # per lap. 4 closes, 4.3 keeps drifting
V = math.sqrt(K / M) * R0 / PETALS   # spin speed, m/s
B_R = 100.0              # wobble energy gain
B_T = 500.0              # spin energy gain
E_RD = 0.5 * K * A ** 2  # J
E_TD = 0.5 * M * V ** 2  # J

MAX_TILT = 12.0          # deg
Z_WN = 4.0               # onboard height loop, rad/s (guess for real drone, exact in sim)
Y_KP, Y_KD = 4.0, 2.5    # y hold
YAW_KP = 2.0             # yaw hold
FENCE = Fence(x=(-1.5, 1.5), y=(-1.5, 1.5), z=(0.2, 1.4))   # m


def tilt(accel):
    # small angle, a = g theta
    return max(-MAX_TILT, min(MAX_TILT, math.degrees(accel / G)))


def height(z, vz, az):
    # zdistance only takes a height, so lead it to get az out of the onboard loop
    # onboard: z'' = wn^2 (z_cmd - z) - 2 wn z'
    z_cmd = z + (az + 2 * Z_WN * vz) / Z_WN ** 2
    return max(FENCE.lo[2], min(FENCE.hi[2], z_cmd))


def main():
    sim = "--sim" in sys.argv
    print(f"v {V:.3f} m/s, lap {2 * math.pi * R0 / V:.1f} s, {PETALS} petals per lap")
    with open_drone(URI, sim=sim) as drone, Keys(drone) as keys:
        drone.takeoff(ZC)
        drone.go_to(XC + R0 + A, 0.0, ZC, 0.0, 3.0)   # start at the top of a wobble
        print("let go. q land, esc kill")

        rate = Rate(50)
        while not keys.stop:
            s = drone.state
            if drone.state_age() > 0.25 or not FENCE.contains(s.pos, slack=0.05):
                print("\nlost position or left fence")
                break

            # polar around the center
            dx, dz = s.x - XC, s.z - ZC
            r = math.hypot(dx, dz)
            cr, sr = dx / r, dz / r                  # radial unit
            vr = s.vx * cr + s.vz * sr
            vt = -s.vx * sr + s.vz * cr              # counterclockwise
            rho = r - R0

            e_r = 0.5 * M * vr ** 2 + 0.5 * K * rho ** 2
            e_t = 0.5 * M * vt ** 2
            a_r = -K / M * rho - B_R * (e_r - E_RD) * vr - vt ** 2 / r   # spring + energy + centripetal
            a_t = -B_T * (e_t - E_TD) * V + vr * vt / r                   # energy holds spin speed

            ax = a_r * cr - a_t * sr
            az = a_r * sr + a_t * cr
            ay = -Y_KP * s.y - Y_KD * s.vy              # hold y = 0

            pitch, roll = tilt(ax), tilt(ay)
            drone.attitude_setpoint(-1 * roll, -1 * pitch, -YAW_KP * s.yaw, height(s.z, s.vz, az))
            print(f"x {s.x:+.3f}  z {s.z:+.3f}", end="\r")
            rate.sleep()

        if not keys.kill:
            print("\nlanding")


if __name__ == "__main__":
    main()
