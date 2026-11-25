"""Flywheel speed vs distance chart for the portfolio's Launcher Physics panel (Dual Turrets slide).

Points: the six logged POLLEN trials. Curve: the SHIPPED fit, ShootingRegression.POLLEN_COEFFS
(c0 + c1 d + c2 d^2, d in inches, motor encoder ticks/s), drawn over POLLEN_RANGE. R^2 is computed
here from those coefficients on those six points - it is not copied from anywhere.

Sized for the slide slot (105.8 x 62 pt); rendered with a transparent background.
  python3 tools/portfolio/launcher/speed_vs_distance.py docs/portfolio/launcher_speed_vs_distance.png
"""
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm

PTS = [(30.5, 1150), (43.6, 1170), (51.5, 1170), (57.5, 1200), (62.2, 1250), (79.9, 1300)]
C = (1154.23, -1.56488, 0.0432406)          # ShootingRegression.POLLEN_COEFFS
RANGE = (30.5, 79.9)                         # ShootingRegression.POLLEN_RANGE

DARK, PT, GRID, FILL, PLOT = "#3F5A12", "#5E8A2A", "#C3D394", "#D9E5B4", "#EEF3DC"

def f(d):
    return C[0] + C[1] * d + C[2] * d * d

x = np.array([p[0] for p in PTS]); y = np.array([p[1] for p in PTS])
r2 = 1 - ((y - f(x)) ** 2).sum() / ((y - y.mean()) ** 2).sum()

for name in ("Avenir Next", "Helvetica Neue"):
    if any(name in fe.name for fe in fm.fontManager.ttflist):
        plt.rcParams["font.family"] = name
        break

W, H = 105.8 / 72, 62.0 / 72
fig = plt.figure(figsize=(W, H), dpi=1200)
ax = fig.add_axes([0.20, 0.25, 0.76, 0.56])
ax.set_facecolor(PLOT)
d = np.linspace(RANGE[0], RANGE[1], 200)          # the code clamps outside RANGE
ax.fill_between(d, 1120, f(d), color=FILL, lw=0, zorder=1)
ax.plot(d, f(d), color=DARK, lw=1.1, solid_capstyle="round", zorder=3)
ax.scatter(x, y, s=7, color=PT, edgecolor="white", linewidth=0.55, zorder=4)
ax.set_xlim(26, 84); ax.set_ylim(1120, 1325)
ax.set_xticks([30, 40, 50, 60, 70, 80]); ax.set_yticks([1150, 1200, 1250, 1300])
ax.grid(True, color=GRID, lw=0.3, ls=(0, (2, 2)))
for s in ("top", "right"):
    ax.spines[s].set_visible(False)
for s in ("left", "bottom"):
    ax.spines[s].set_color(DARK); ax.spines[s].set_linewidth(0.6)
ax.tick_params(colors=DARK, labelsize=4.2, length=1.5, width=0.4, pad=1)
for t in ax.get_xticklabels() + ax.get_yticklabels():
    t.set_fontweight("bold")
ax.set_xlabel("Distance (in)", fontsize=4.4, color=DARK, fontweight="bold", labelpad=1)
ax.set_ylabel("ticks/s", fontsize=4.4, color=DARK, fontweight="bold", labelpad=1.5)
eq = f"y = {C[2]:.4f}x² − {abs(C[1]):.3f}x + {C[0]:.1f}"
fig.text(0.04, 0.90, eq, fontsize=4.4, color=DARK, fontweight="bold", va="center")
fig.text(0.97, 0.90, f"R² = {r2:.3f}", fontsize=4.4, color=DARK, fontweight="bold", va="center", ha="right")
fig.savefig(sys.argv[1], transparent=True)
print(f"R2 = {r2:.4f}; {eq}")
