"""webcam hand tracking + grab detection in its own thread

u, v are normalized 0..1 (u left->right, v top->bottom)

grab uses openness = mean(wrist->tip / wrist->knuckle) over 4 fingers.
open hand ~1.8-2.0, fist ~1.0. two thresholds + debounce so it doesnt flicker
"""

import threading
import time
import urllib.request
from pathlib import Path
from dataclasses import dataclass, replace

import cv2
import numpy as np

from .paths import MODEL_PATH
from .util import OneEuroFilter

MODEL_URL = (
    "https://storage.googleapis.com/mediapipe-models/hand_landmarker/"
    "hand_landmarker/float16/latest/hand_landmarker.task"
)
WRIST = 0
PALM = 9  # middle knuckle, doesnt move much when you make a fist
TIPS = (8, 12, 16, 20)
KNUCKLES = (5, 9, 13, 17)


@dataclass
class HandState:
    t: float = 0.0
    detected: bool = False
    grabbed: bool = False
    u: float = 0.5  # filtered palm pos
    v: float = 0.5
    du: float = 0.0  # palm vel, 1/s
    dv: float = 0.0
    openness: float = 0.0
    landmarks: tuple = ()  # raw 21 pts for drawing
    fps: float = 0.0


def ensure_model(path):
    if not path.exists():
        path.parent.mkdir(parents=True, exist_ok=True)
        print(f"Downloading hand model to {path} ...")
        urllib.request.urlretrieve(MODEL_URL, path)
    return path


def openness_ratio(pts, width, height):
    p = np.array(pts) * [width, height]  # to pixels so x and y are same scale
    wrist = p[WRIST]
    tips = np.linalg.norm(p[list(TIPS)] - wrist, axis=1)
    knuckles = np.linalg.norm(p[list(KNUCKLES)] - wrist, axis=1)
    return float(np.mean(tips / np.maximum(knuckles, 1e-6)))


class HandTracker:
    # lost_timeout: s without a hand before it counts as let go
    # filter: lower min_cutoff = smoother, higher beta = less lag

    def __init__(self, camera=0, width=640, height=480, mirror=True,
                 grab_below=1.30, release_above=1.60, debounce_frames=3, lost_timeout=0.3,
                 min_detection_confidence=0.6, min_tracking_confidence=0.5,
                 filter_min_cutoff=1.0, filter_beta=0.05, model_path=MODEL_PATH):
        self.camera, self.width, self.height, self.mirror = camera, width, height, mirror
        self.grab_below, self.release_above = grab_below, release_above
        self.debounce_frames, self.lost_timeout = debounce_frames, lost_timeout
        self.min_detection_confidence = min_detection_confidence
        self.min_tracking_confidence = min_tracking_confidence
        self.model_path = Path(model_path)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._frame = None
        self._frame_id = 0
        self._state = HandState()
        self._fu = OneEuroFilter(filter_min_cutoff, filter_beta)
        self._fv = OneEuroFilter(filter_min_cutoff, filter_beta)
        self._pending = None  # grab state waiting on debounce
        self._pending_count = 0
        self._last_seen = 0.0

    def __enter__(self):
        from mediapipe.tasks.python import BaseOptions, vision

        model = ensure_model(self.model_path)
        options = vision.HandLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=str(model)),
            running_mode=vision.RunningMode.VIDEO,
            num_hands=1,
            min_hand_detection_confidence=self.min_detection_confidence,
            min_hand_presence_confidence=self.min_detection_confidence,
            min_tracking_confidence=self.min_tracking_confidence,
        )
        self._landmarker = vision.HandLandmarker.create_from_options(options)

        self._cap = cv2.VideoCapture(self.camera, cv2.CAP_DSHOW)
        if not self._cap.isOpened():
            self._cap = cv2.VideoCapture(self.camera)
        if not self._cap.isOpened():
            raise RuntimeError(f"Could not open camera {self.camera}")
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

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
        # frame is None until the first image comes in
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
            if self.mirror:
                frame = cv2.flip(frame, 1)
            t = time.monotonic()
            fps = 0.9 * fps + 0.1 / max(t - prev_t, 1e-3)
            prev_t = t

            ts_ms = max(int(t * 1000), last_ms + 1)  # has to keep increasing
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
            if t - self._last_seen > self.lost_timeout:
                self._fu.reset()
                self._fv.reset()
                self._pending = None
                return HandState(t=t)
            return replace(prev, t=t, landmarks=())  # short dropout, keep last state

        self._last_seen = t
        pts = tuple((lm.x, lm.y) for lm in result.hand_landmarks[0])
        openness = openness_ratio(pts, width, height)
        u = self._fu(pts[PALM][0], t)
        v = self._fv(pts[PALM][1], t)

        grabbed = prev.grabbed if prev.detected else False
        want = grabbed
        if openness < self.grab_below:
            want = True
        elif openness > self.release_above:
            want = False
        if want != grabbed:
            if want == self._pending:
                self._pending_count += 1
            else:
                self._pending, self._pending_count = want, 1
            if self._pending_count >= self.debounce_frames:
                grabbed = want
                self._pending = None
        else:
            self._pending = None

        return HandState(t=t, detected=True, grabbed=grabbed, u=u, v=v,
                         du=self._fu.dx, dv=self._fv.dx,
                         openness=openness, landmarks=pts)


def draw_hand(img, hand):
    # palm dot is filled when grabbed
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
    # outlined text so its readable on anything
    x, y = origin
    for line in lines:
        cv2.putText(img, line, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(img, line, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)
        y += int(26 * scale / 0.55)
