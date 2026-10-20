"""Webcam hand tracking with grab detection, running in its own thread.

    with HandTracker(cfg) as tracker:
        frame_id, frame, hand = tracker.latest()

`frame` is the (mirrored) BGR image the landmarks were found in, and `hand`
is a HandState. Coordinates are normalized: u = 0..1 left to right,
v = 0..1 top to bottom.

Grab detection uses an "openness" ratio: the mean distance from the wrist to
each fingertip divided by the wrist-to-knuckle distance of the same finger.
An open hand is around 1.8-2.0, a fist around 1.0. Two thresholds with a
gap between them (hysteresis) plus a few frames of debounce keep the state
from flickering at the boundary.
"""

import threading
import time
import urllib.request
from dataclasses import dataclass, replace

import cv2
import numpy as np

from .config import resolve
from .util import OneEuroFilter

MODEL_URL = (
    "https://storage.googleapis.com/mediapipe-models/hand_landmarker/"
    "hand_landmarker/float16/latest/hand_landmarker.task"
)
WRIST = 0
PALM = 9  # middle-finger knuckle: stays put when the hand closes
TIPS = (8, 12, 16, 20)
KNUCKLES = (5, 9, 13, 17)


@dataclass
class HandState:
    t: float = 0.0  # time.monotonic() of the frame
    detected: bool = False
    grabbed: bool = False
    u: float = 0.5  # filtered palm position
    v: float = 0.5
    du: float = 0.0  # filtered palm velocity, per second
    dv: float = 0.0
    openness: float = 0.0
    landmarks: tuple = ()  # 21 raw (u, v) points, for drawing
    fps: float = 0.0


def ensure_model(path):
    if not path.exists():
        path.parent.mkdir(parents=True, exist_ok=True)
        print(f"Downloading hand model to {path} ...")
        urllib.request.urlretrieve(MODEL_URL, path)
    return path


def openness_ratio(pts, width, height):
    p = np.array(pts) * [width, height]  # pixels, so x and y share a scale
    wrist = p[WRIST]
    tips = np.linalg.norm(p[list(TIPS)] - wrist, axis=1)
    knuckles = np.linalg.norm(p[list(KNUCKLES)] - wrist, axis=1)
    return float(np.mean(tips / np.maximum(knuckles, 1e-6)))


class HandTracker:
    def __init__(self, cfg):
        self.cam_cfg = cfg["camera"]
        self.cfg = cfg["hand"]
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._frame = None
        self._frame_id = 0
        self._state = HandState()
        self._fu = OneEuroFilter(self.cfg["filter_min_cutoff"], self.cfg["filter_beta"])
        self._fv = OneEuroFilter(self.cfg["filter_min_cutoff"], self.cfg["filter_beta"])
        self._pending = None  # grab state waiting out the debounce
        self._pending_count = 0
        self._last_seen = 0.0

    def __enter__(self):
        from mediapipe.tasks.python import BaseOptions, vision

        model = ensure_model(resolve(self.cfg["model_path"]))
        options = vision.HandLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=str(model)),
            running_mode=vision.RunningMode.VIDEO,
            num_hands=1,
            min_hand_detection_confidence=self.cfg["min_detection_confidence"],
            min_hand_presence_confidence=self.cfg["min_detection_confidence"],
            min_tracking_confidence=self.cfg["min_tracking_confidence"],
        )
        self._landmarker = vision.HandLandmarker.create_from_options(options)

        self._cap = cv2.VideoCapture(self.cam_cfg["index"], cv2.CAP_DSHOW)
        if not self._cap.isOpened():
            self._cap = cv2.VideoCapture(self.cam_cfg["index"])
        if not self._cap.isOpened():
            raise RuntimeError(f"Could not open camera {self.cam_cfg['index']}")
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.cam_cfg["width"])
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.cam_cfg["height"])

        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return self

    def __exit__(self, *exc):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._cap.release()
        self._landmarker.close()

    def latest(self):
        """(frame_id, frame, HandState). frame is None until the first image arrives."""
        with self._lock:
            return self._frame_id, self._frame, replace(self._state)

    def _run(self):
        import mediapipe as mp

        last_ms = 0
        prev_t = time.monotonic()
        fps = 0.0
        while not self._stop.is_set():
            ok, frame = self._cap.read()
            if not ok:
                time.sleep(0.01)
                continue
            if self.cam_cfg.get("mirror", True):
                frame = cv2.flip(frame, 1)
            t = time.monotonic()
            fps = 0.9 * fps + 0.1 / max(t - prev_t, 1e-3)
            prev_t = t

            ts_ms = max(int(t * 1000), last_ms + 1)  # must strictly increase
            last_ms = ts_ms
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            result = self._landmarker.detect_for_video(
                mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb), ts_ms)

            state = self._update(result, t, frame.shape[1], frame.shape[0])
            state.fps = fps
            with self._lock:
                self._frame = frame
                self._frame_id += 1
                self._state = state

    def _update(self, result, t, width, height):
        prev = self._state
        if not result.hand_landmarks:
            if t - self._last_seen > self.cfg["lost_timeout"]:
                self._fu.reset()
                self._fv.reset()
                self._pending = None
                return HandState(t=t)
            return replace(prev, t=t, landmarks=())  # brief dropout: keep last state

        self._last_seen = t
        pts = tuple((lm.x, lm.y) for lm in result.hand_landmarks[0])
        openness = openness_ratio(pts, width, height)
        u = self._fu(pts[PALM][0], t)
        v = self._fv(pts[PALM][1], t)

        grabbed = prev.grabbed if prev.detected else False
        want = grabbed
        if openness < self.cfg["grab_below"]:
            want = True
        elif openness > self.cfg["release_above"]:
            want = False
        if want != grabbed:
            if want == self._pending:
                self._pending_count += 1
            else:
                self._pending, self._pending_count = want, 1
            if self._pending_count >= self.cfg["debounce_frames"]:
                grabbed = want
                self._pending = None
        else:
            self._pending = None

        return HandState(t=t, detected=True, grabbed=grabbed, u=u, v=v,
                         du=self._fu.dx, dv=self._fv.dx,
                         openness=openness, landmarks=pts)


def draw_hand(img, hand):
    """Draw the landmark skeleton and the palm point (filled when grabbed)."""
    from mediapipe.tasks.python.vision import HandLandmarksConnections

    h, w = img.shape[:2]
    if hand.landmarks:
        px = [(int(x * w), int(y * h)) for x, y in hand.landmarks]
        for c in HandLandmarksConnections.HAND_CONNECTIONS:
            cv2.line(img, px[c.start], px[c.end], (200, 200, 200), 1, cv2.LINE_AA)
        for p in px:
            cv2.circle(img, p, 2, (255, 255, 255), -1, cv2.LINE_AA)
    if hand.detected:
        center = (int(hand.u * w), int(hand.v * h))
        color = (60, 220, 60) if hand.grabbed else (0, 200, 255)
        cv2.circle(img, center, 14, color, -1 if hand.grabbed else 2, cv2.LINE_AA)


def put_lines(img, lines, origin=(10, 24), color=(255, 255, 255), scale=0.55):
    """Text with a dark outline so it reads on any background."""
    x, y = origin
    for line in lines:
        cv2.putText(img, line, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(img, line, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)
        y += int(26 * scale / 0.55)
