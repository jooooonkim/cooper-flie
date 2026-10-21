"""keyboard flight test (radio, positioning, flight)

w/s fwd/back, a/d left/right, o/p up/down, q land, esc kill (motors off)
hold a key to move, let go and it holds. keys are global

python test/01_keyboard_flight.py [--sim]
"""

import argparse
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
from pynput import keyboard

from common.drone import open_drone
from common.safety import Fence
from common.util import Rate

# settings
URI = "radio://0/80/2M/E7E7E7E7E9"   # check in cfclient
TAKEOFF_HEIGHT = 0.5                 # m
SPEED = 0.5                          # m/s
FENCE = Fence(x=(-1.0, 1.0), y=(-1.0, 1.0), z=(0.2, 1.4))   # m
STATE_TIMEOUT = 0.25                 # s with no position -> land

DIRECTIONS = {
    "w": (1, 0, 0), "s": (-1, 0, 0),
    "a": (0, 1, 0), "d": (0, -1, 0),
    "o": (0, 0, 1), "p": (0, 0, -1),
}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--sim", action="store_true", help="use the simulated drone")
    parser.add_argument("--uri", default=URI, help=f"radio URI (default {URI})")
    args = parser.parse_args()

    held = set()
    quit_ = threading.Event()
    kill = threading.Event()

    def on_press(key):
        if key == keyboard.Key.esc:
            kill.set()
            quit_.set()
            return
        ch = getattr(key, "char", None)
        if ch is None:
            return
        ch = ch.lower()
        if ch == "q":
            quit_.set()
        elif ch in DIRECTIONS:
            held.add(ch)

    def on_release(key):
        ch = getattr(key, "char", None)
        if ch is not None:
            held.discard(ch.lower())

    with open_drone(args.uri, sim=args.sim) as drone, \
            keyboard.Listener(on_press=on_press, on_release=on_release):
        print("Taking off. w/a/s/d move, o/p up/down, q land, Esc kill.")
        drone.takeoff(TAKEOFF_HEIGHT)
        target = FENCE.clamp(drone.state.pos, margin=0.05)

        rate = Rate(50)
        last_print = 0.0
        while not quit_.is_set():
            if drone.state_age() > STATE_TIMEOUT:
                print("\nLost the position estimate, landing.")
                break

            direction = np.sum([DIRECTIONS[k] for k in list(held)] or [(0, 0, 0)], axis=0)
            if np.any(direction):
                direction = direction / np.linalg.norm(direction)
            target = FENCE.clamp(target + direction * SPEED * rate.dt, margin=0.05)
            drone.position_setpoint(*target, 0.0)

            now = time.monotonic()
            if now - last_print > 0.2:
                s = drone.state
                print(f"\rtarget ({target[0]:+.2f}, {target[1]:+.2f}, {target[2]:.2f})  "
                      f"actual ({s.x:+.2f}, {s.y:+.2f}, {s.z:.2f})  keys {''.join(sorted(held)) or '-':6}",
                      end="", flush=True)
                last_print = now
            rate.sleep()

        print()
        if kill.is_set():
            print("EMERGENCY STOP")
            drone.kill()
        else:
            print("Landing.")


if __name__ == "__main__":
    main()
