"""
Summary figure: filtered error as a percentage of raw error, for every
experiment in Weeks 1-5. Below 100% the filter helps; above 100% it
is worse than raw measurements. Log scale because the range is huge.

Numbers are the 200-seed means from the README tables (Week 1 is a
single demo run). Re-run monte_carlo.py / week5_demo.py to regenerate.
"""

import numpy as np
import matplotlib.pyplot as plt

# (label, filtered error, raw error)
rows = [
    ("Wk1  position-only filter (single run)", 10.6, 1.2),
    ("Wk2  constant-velocity filter",          1.19, 2.38),
    ("Wk3  two-sensor fusion (vs best sensor)", 0.62, 1.19),
    ("Wk4  straight path, q=0.01",             1.85, 3.74),
    ("Wk4  turning target, q=0.01 (mistuned)", 3.37, 3.74),
    ("Wk4  turning target, q=0.1 (tuned)",     2.40, 3.74),
    ("Wk5  radar, -20 dB SNR",                 0.158, 0.224),
    ("Wk5  radar, -26 dB, no gate",            106.917, 85.958),
    ("Wk5  radar, -26 dB, gated",              0.174, 85.958),
]

labels = [r[0] for r in rows]
pct = np.array([100 * f / raw for _, f, raw in rows])
colors = ["#2e7d32" if p < 100 else "#c62828" for p in pct]

fig, ax = plt.subplots(figsize=(10, 5.5))
y = np.arange(len(rows))[::-1]
ax.barh(y, pct - 100, left=100, color=colors, height=0.6)
ax.axvline(100, color="k", linewidth=1.2)
ax.set_xscale("log")
ax.set_yticks(y)
ax.set_yticklabels(labels)
for yi, p in zip(y, pct):
    ax.text(p * (1.12 if p >= 100 else 0.88), yi, f"{p:.1f}%",
            va="center", ha="left" if p >= 100 else "right", fontsize=9)
ax.set_xlim(0.05, 3000)
ax.set_xlabel("Filtered error as % of raw error  (log scale; <100% = filter helps, >100% = worse than raw)")
ax.set_title("Sensor Fusion Tracker: where the Kalman filter helps, and where it fails")
ax.grid(True, axis="x", alpha=0.3)
plt.tight_layout()
plt.savefig("../docs/summary.png", dpi=150)
plt.show()

for (name, f, raw), p in zip(rows, pct):
    print(f"{name:42s} {p:8.1f}%")