"""grab = drone follows hand, let go = hold. q land, esc kill

python test/03_hand_drone.py [--sim]
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import cv2
import numpy as np

from common.drone import open_drone
from common.hand import HandTracker, draw_hand, put_lines
from common.mapping import HandMapper
from common.safety import Fence
from common.util import Rate

# settings
URI = "radio://0/80/2M/E7E7E7E7E9"
TAKEOFF_HEIGHT = 0.5     # m
MAX_SPEED = 0.5          # m/s
FENCE = Fence(x=(-1.0, 1.0), y=(-1.0, 1.0), z=(0.2, 1.4))   # m
STATE_TIMEOUT = 0.25     # s

CAMERA = 0
GRAB_BELOW = 1.30        # from 02_hand_tracking
RELEASE_ABOVE = 1.60
HORIZONTAL, VERTICAL = "x", "z"   # image left-right / up-down
ACTIVE_U = (0.15, 0.85)
ACTIVE_V = (0.15, 0.85)

AXES = "xyz"


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--sim", action="store_true", help="use the simulated drone")
    parser.add_argument("--uri", default=URI, help=f"radio URI (default {URI})")
    args = parser.parse_args()

    mapper = HandMapper(FENCE, HORIZONTAL, VERTICAL, active_u=ACTIVE_U, active_v=ACTIVE_V)
    tracker = HandTracker(camera=CAMERA, grab_below=GRAB_BELOW, release_above=RELEASE_ABOVE)

    with open_drone(args.uri, sim=args.sim) as drone, tracker:
        drone.takeoff(TAKEOFF_HEIGHT)
        target = FENCE.clamp(drone.state.pos, margin=0.05)
        goal = target.copy()
        grabbing = False
        hand_anchor = drone_anchor = None

        rate = Rate(50)
        max_step = MAX_SPEED * rate.dt
        last_id = -1
        kill = False
        while True:
            if drone.state_age() > STATE_TIMEOUT:
                print("Lost the position estimate, landing.")
                break

            frame_id, frame, hand = tracker.latest()

            if hand.grabbed and not grabbing:
                grabbing = True
                hand_anchor = mapper.to_world(hand.u, hand.v)
                drone_anchor = target.copy()
            elif not hand.grabbed and grabbing:
                grabbing = False
                goal = target.copy()  # hold

            if grabbing:
                world = mapper.to_world(hand.u, hand.v)
                goal = drone_anchor.copy()
                for axis, value in world.items():
                    goal[AXES.index(axis)] += value - hand_anchor[axis]
                goal = FENCE.clamp(goal, margin=0.05)

            step = goal - target
            dist = np.linalg.norm(step)
            if dist > max_step:
                step *= max_step / dist
            target = target + step
            drone.position_setpoint(*target, 0.0)

            if frame is not None and frame_id != last_id:
                last_id = frame_id
                img = frame.copy()
                draw_hand(img, hand)
                s = drone.state
                put_lines(img, [
                    ("CONTROLLING" if grabbing else "HOLDING") + ("" if hand.detected else "  (no hand)"),
                    f"target ({target[0]:+.2f}, {target[1]:+.2f}, {target[2]:.2f})",
                    f"actual ({s.x:+.2f}, {s.y:+.2f}, {s.z:.2f})",
                    "q land   Esc kill" + ("   [SIM]" if args.sim else ""),
                ])
                cv2.imshow("03 hand drone", img)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == 27:
                kill = True
                break
            rate.sleep()

        if kill:
            print("EMERGENCY STOP")
            drone.kill()
        else:
            print("Landing.")
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
