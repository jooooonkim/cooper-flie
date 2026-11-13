"""plot newest log vs ideal msd (or pass a csv)

python demos/single_mass_spring_damper/plot_log.py
"""

import csv
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import matplotlib.pyplot as plt
import numpy as np

from common.paths import LOG_DIR


def load(path):
    with open(path, encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    cols = {}
    for key in ("t", "x", "vx", "pitch_cmd", "m", "k", "b", "x0"):
        cols[key] = np.array([float(r[key]) for r in rows])
    cols["grabbed"] = np.array([float(r.get("grabbed") or 0) for r in rows])
    cols["x_hand"] = np.array([float(r["x_hand"]) if r.get("x_hand") else np.nan for r in rows])
    return cols


def ideal(t, x_start, v_start, m, k, b, x0):
    # rk4, m x'' = -k (x - x0) - b x'
    def f(state):
        x, v = state
        return np.array([v, (-k * (x - x0) - b * v) / m])

    out = np.empty(len(t))
    state = np.array([x_start, v_start])
    out[0] = x_start
    for i in range(1, len(t)):
        h = t[i] - t[i - 1]
        k1 = f(state)
        k2 = f(state + h / 2 * k1)
        k3 = f(state + h / 2 * k2)
        k4 = f(state + h * k3)
        state = state + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        out[i] = state[0]
    return out


def free_segments(d):
    # free stretches
    key = np.stack([d["grabbed"], d["m"], d["k"], d["b"], d["x0"]], axis=1)
    breaks = np.flatnonzero(np.any(np.diff(key, axis=0) != 0, axis=1)) + 1
    for seg in np.split(np.arange(len(d["t"])), breaks):
        if len(seg) > 5 and d["grabbed"][seg[0]] == 0:
            yield seg


def main():
    if len(sys.argv) > 1:
        path = Path(sys.argv[1])
    else:
        logs = sorted([*LOG_DIR.glob("single_msd_*.csv"), *LOG_DIR.glob("msd_set_*.csv")],
                      key=lambda p: p.stat().st_mtime)
        if not logs:
            sys.exit(f"No logs in {LOG_DIR}")
        path = logs[-1]
    d = load(path)
    t = d["t"]

    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(10, 6))
    ax1.plot(t, d["x"], label="drone x (measured)", color="tab:red")
    if not np.all(np.isnan(d["x_hand"])):
        ax1.plot(t, d["x_hand"], label="hand x", color="tab:green", alpha=0.5)
    first = True
    for seg in free_segments(d):
        i = seg[0]
        x_ideal = ideal(t[seg], d["x"][i], d["vx"][i], d["m"][i], d["k"][i], d["b"][i], d["x0"][i])
        ax1.plot(t[seg], x_ideal, "--", color="black", lw=1, label="ideal model" if first else None)
        first = False
    if d["grabbed"].any():
        ax1.fill_between(t, *ax1.get_ylim(), where=d["grabbed"] > 0, color="tab:green", alpha=0.1,
                         label="grabbed")
    ax1.set_ylabel("x (m)")
    ax1.legend(loc="upper right")
    ax1.set_title(path.name)

    ax2.plot(t, d["pitch_cmd"], color="tab:blue")
    ax2.set_ylabel("tilt command (deg)")
    ax2.set_xlabel("time (s)")
    fig.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
