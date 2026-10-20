import numpy as np


class Fence:
    """Axis-aligned box in the world frame that the drone must stay inside."""

    def __init__(self, fence_cfg):
        self.lo = np.array([fence_cfg[a][0] for a in "xyz"], dtype=float)
        self.hi = np.array([fence_cfg[a][1] for a in "xyz"], dtype=float)

    def contains(self, p, slack=0.0):
        p = np.asarray(p, dtype=float)
        return bool(np.all(p >= self.lo - slack) and np.all(p <= self.hi + slack))

    def clamp(self, p, margin=0.0):
        return np.clip(np.asarray(p, dtype=float), self.lo + margin, self.hi - margin)

    def range(self, axis, margin=0.0):
        i = "xyz".index(axis)
        return self.lo[i] + margin, self.hi[i] - margin
