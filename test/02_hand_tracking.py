"""Test 2: hand tracking and grab detection only. No drone.

Shows the palm point (outlined = open, filled = grabbed), the openness ratio
against the grab/release thresholds, and the world position the hand maps
to. Use it to tune the `hand` and `mapping` sections of config.yaml.

    q or Esc   quit
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import cv2

from common.config import load_config
from common.hand import HandTracker, draw_hand, put_lines
from common.mapping import HandMapper
from common.safety import Fence


def draw_openness_bar(img, hand, cfg):
    """Vertical bar: openness value with the two thresholds marked."""
    h, w = img.shape[:2]
    x0, y0, y1 = w - 40, 40, h - 40
    lo, hi = 0.8, 2.2

    def to_y(value):
        return int(y1 - (min(max(value, lo), hi) - lo) / (hi - lo) * (y1 - y0))

    cv2.rectangle(img, (x0, y0), (x0 + 16, y1), (80, 80, 80), 1)
    if hand.detected:
        color = (60, 220, 60) if hand.grabbed else (0, 200, 255)
        cv2.rectangle(img, (x0 + 2, to_y(hand.openness)), (x0 + 14, y1 - 1), color, -1)
    for value, label in ((cfg["grab_below"], "grab"), (cfg["release_above"], "open")):
        y = to_y(value)
        cv2.line(img, (x0 - 6, y), (x0 + 22, y), (255, 255, 255), 1)
        cv2.putText(img, label, (x0 - 44, y + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)


def main():
    cfg = load_config()
    mapper = HandMapper(cfg["mapping"], Fence(cfg["flight"]["fence"]))
    au, av = cfg["mapping"]["active_u"], cfg["mapping"]["active_v"]
    last_id = -1

    with HandTracker(cfg) as tracker:
        while True:
            frame_id, frame, hand = tracker.latest()
            if frame is None or frame_id == last_id:
                if cv2.waitKey(5) & 0xFF in (ord("q"), 27):
                    break
                continue
            last_id = frame_id
            img = frame.copy()
            h, w = img.shape[:2]

            cv2.rectangle(img, (int(au[0] * w), int(av[0] * h)), (int(au[1] * w), int(av[1] * h)),
                          (120, 120, 120), 1)
            draw_hand(img, hand)
            draw_openness_bar(img, hand, cfg["hand"])

            if hand.detected:
                world = mapper.to_world(hand.u, hand.v)
                status = "GRABBED" if hand.grabbed else "OPEN"
                lines = [
                    f"{status}   openness {hand.openness:.2f}",
                    f"palm u {hand.u:.2f}  v {hand.v:.2f}",
                    "maps to " + "  ".join(f"{a} {val:+.2f} m" for a, val in world.items()),
                ]
            else:
                lines = ["no hand"]
            put_lines(img, lines + [f"{hand.fps:.0f} fps"])

            cv2.imshow("02 hand tracking", img)
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
