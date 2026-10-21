import numpy as np


class Fence:
    # box the drone has to stay in, world frame

    def __init__(self, x, y, z):
        self.lo = np.array([x[0], y[0], z[0]], dtype=float)
        self.hi = np.array([x[1], y[1], z[1]], dtype=float)

    def contains(self, p, slack=0.0):
        p = np.asarray(p, dtype=float)
        return bool(np.all(p >= self.lo - slack) and np.all(p <= self.hi + slack))

    def clamp(self, p, margin=0.0):
        return np.clip(np.asarray(p, dtype=float), self.lo + margin, self.hi - margin)

    def range(self, axis, margin=0.0):
        i = "xyz".index(axis)
        return self.lo[i] + margin, self.hi[i] - margin
