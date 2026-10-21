import csv
import math
import time
from datetime import datetime

from .paths import LOG_DIR


class Rate:
    """Fixed-rate loop timer that does not accumulate drift."""

    def __init__(self, hz):
        self.dt = 1.0 / hz
        self._next = time.perf_counter() + self.dt

    def sleep(self):
        delay = self._next - time.perf_counter()
        if delay > 0:
            time.sleep(delay)
            self._next += self.dt
        else:
            # Fell behind; restart the schedule instead of bursting to catch up.
            self._next = time.perf_counter() + self.dt


class OneEuroFilter:
    """Adaptive low-pass filter for noisy tracking signals.

    Smooths heavily when the signal is still and cuts lag when it moves fast.
    Casiez, Roussel & Vogel, "1 Euro Filter", CHI 2012.
    """

    def __init__(self, min_cutoff=1.0, beta=0.0, d_cutoff=1.0):
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff
        self.reset()

    def reset(self):
        self.x = None
        self.dx = 0.0
        self._t = None

    @staticmethod
    def _alpha(cutoff, dt):
        tau = 1.0 / (2 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)

    def __call__(self, x, t):
        if self.x is None or t <= self._t:
            self.x, self.dx, self._t = x, 0.0, t
            return x
        dt = t - self._t
        self._t = t
        a_d = self._alpha(self.d_cutoff, dt)
        self.dx = a_d * (x - self.x) / dt + (1 - a_d) * self.dx
        cutoff = self.min_cutoff + self.beta * abs(self.dx)
        a = self._alpha(cutoff, dt)
        self.x = a * x + (1 - a) * self.x
        return self.x


class CsvLog:
    """Append rows to logs/<name>_<timestamp>.csv."""

    def __init__(self, name, fields):
        LOG_DIR.mkdir(exist_ok=True)
        self.path = LOG_DIR / f"{name}_{datetime.now():%Y%m%d_%H%M%S}.csv"
        self.fields = fields
        self._f = open(self.path, "w", newline="", encoding="utf-8")
        self._w = csv.DictWriter(self._f, fieldnames=fields)
        self._w.writeheader()

    def write(self, **row):
        self._w.writerow(row)

    def close(self):
        self._f.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
