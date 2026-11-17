"""plot newest msd_energy log vs the ideal model (or pass a csv)

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
    keys = ("t", "x", "vx", "pitch_cmd", "m", "k", "b", "x0", "e", "e_d")
    return {key: np.array([float(r[key]) for r in rows]) for key in keys}


def ideal(t, x_start, v_start, m, k, b, x0, e_d):
    # rk4, x'' = -k/m (x - x0) - b (E - Ed) x'
    def f(state):
        x, v = state
        e = 0.5 * m * v ** 2 + 0.5 * k * (x - x0) ** 2
        return np.array([v, -k / m * (x - x0) - b * (e - e_d) * v])

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


def main():
    if len(sys.argv) > 1:
        path = Path(sys.argv[1])
    else:
        logs = sorted(LOG_DIR.glob("msd_energy_*.csv"), key=lambda p: p.stat().st_mtime)
        if not logs:
            sys.exit(f"no logs in {LOG_DIR}")
        path = logs[-1]
    d = load(path)
    t = d["t"]
    m, k, b, x0, e_d = (d[key][0] for key in ("m", "k", "b", "x0", "e_d"))
    amp = np.sqrt(2 * e_d / k)

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True, figsize=(10, 8))

    ax1.plot(t, d["x"], color="tab:red", label="drone x (measured)")
    ax1.plot(t, ideal(t, d["x"][0], d["vx"][0], m, k, b, x0, e_d), "--", color="black", lw=1, label="ideal model")
    ax1.axhline(x0 + amp, color="gray", lw=0.8, ls=":", label="target amplitude")
    ax1.axhline(x0 - amp, color="gray", lw=0.8, ls=":")
    ax1.set_ylabel("x (m)")
    ax1.legend(loc="upper right")
    ax1.set_title(path.name)

    ax2.plot(t, d["e"], color="tab:red", label="E (measured)")
    ax2.plot(t, d["e_d"], color="gray", ls=":", label="Ed")
    ax2.set_ylabel("energy (J)")
    ax2.legend(loc="upper right")

    ax3.plot(t, d["pitch_cmd"], color="tab:blue")
    ax3.set_ylabel("tilt command (deg)")
    ax3.set_xlabel("time (s)")
    fig.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
