"""Test 3: move the drone with your hand.

Close your hand (grab) to take control: the drone follows how far your hand
moves from where you grabbed, so grabbing never makes it jump. Open your
hand (let go) and the drone holds where it is. Losing the hand counts as
letting go. The drone's speed is capped at flight.max_speed.

Image left/right and up/down drive the world axes set in config.yaml
(mapping.horizontal / mapping.vertical, default x and z).

    q     land and quit (camera window must have focus)
    Esc   EMERGENCY: stop motors immediately

    python test/03_hand_drone.py [--sim]
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import cv2
import numpy as np

from common.config import load_config
from common.drone import open_drone
from common.hand import HandTracker, draw_hand, put_lines
from common.mapping import HandMapper
from common.safety import Fence
from common.util import Rate

AXES = "xyz"


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--sim", action="store_true", help="use the simulated drone")
    parser.add_argument("--uri", help="override the radio URI from config.yaml")
    args = parser.parse_args()

    cfg = load_config()
    fence = Fence(cfg["flight"]["fence"])
    mapper = HandMapper(cfg["mapping"], fence)
    max_step = cfg["flight"]["max_speed"]
    timeout = cfg["flight"]["state_timeout"]

    with open_drone(cfg, sim=args.sim, uri=args.uri) as drone, HandTracker(cfg) as tracker:
        drone.takeoff()
        target = fence.clamp(drone.state.pos, margin=0.05)
        goal = target.copy()
        grabbing = False
        hand_anchor = drone_anchor = None

        rate = Rate(50)
        max_step *= rate.dt
        last_id = -1
        kill = False
        while True:
            if drone.state_age() > timeout:
                print("Lost the position estimate, landing.")
                break

            frame_id, frame, hand = tracker.latest()

            if hand.grabbed and not grabbing:
                grabbing = True
                hand_anchor = mapper.to_world(hand.u, hand.v)
                drone_anchor = target.copy()
            elif not hand.grabbed and grabbing:
                grabbing = False
                goal = target.copy()  # let go: hold here

            if grabbing:
                world = mapper.to_world(hand.u, hand.v)
                goal = drone_anchor.copy()
                for axis, value in world.items():
                    goal[AXES.index(axis)] += value - hand_anchor[axis]
                goal = fence.clamp(goal, margin=0.05)

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
