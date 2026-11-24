"""Transfer slide: mechanical-drawing style SECTION A-A of the sorting transfer (black linework; colour only on the
game pieces). Geometry from geom2d.npz (robot STL sections: x = 0 cut, x = -45 mm side rails / chamber).
  python3 drawing.py --font-dir DIR [--out FILE] [--dpi N]
"""
import argparse, json, os
from collections import defaultdict
import numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.collections import LineCollection
from matplotlib.patches import Circle, FancyArrowPatch, PathPatch, Rectangle
from matplotlib.path import Path

ap = argparse.ArgumentParser()
ap.add_argument("--font-dir"); ap.add_argument("--out", default="transfer_section.png"); ap.add_argument("--dpi", type=int, default=300)
a = ap.parse_args()
if a.font_dir:
    for f in os.listdir(a.font_dir):
        if f.startswith("Poppins-") and f.endswith(".ttf"):
            font_manager.fontManager.addfont(os.path.join(a.font_dir, f))
    plt.rcParams["font.family"] = "Poppins"
plt.rcParams["hatch.linewidth"] = 0.45

PAPER, INK, THIN = "#FFFFFF", "#111111", "#6B6B6B"
NECTAR, NECTAR_E, POLLEN, POLLEN_E = "#C2362B", "#7A1C14", "#F2C230", "#8A6800"

g = np.load("geom2d.npz")
S0, S45 = g["S0"], g["S45"]
VIEW = (-262.0, 250.0, 14.0, 300.0)
in_view = lambda S: S[((S[:, :, 0] > VIEW[0] - 5) & (S[:, :, 0] < VIEW[1] + 5) & (S[:, :, 1] > VIEW[2] - 5) & (S[:, :, 1] < VIEW[3] + 5)).all(1)]
S0, S45 = in_view(S0), in_view(S45)

def loops(S, tol=0.02):
    """Chain section segments into closed loops (the cut faces)."""
    key = lambda p: (round(p[0] / tol), round(p[1] / tol))
    adj = defaultdict(list)
    for i, (p, q) in enumerate(S):
        adj[key(p)].append((i, 1)); adj[key(q)].append((i, 0))
    used = np.zeros(len(S), bool); out = []
    for i in range(len(S)):
        if used[i]: continue
        used[i] = True; poly = [S[i][0], S[i][1]]; start = key(S[i][0]); cur = key(S[i][1])
        for _ in range(100000):
            if cur == start: break
            nxt = None
            for j, end in adj[cur]:
                if not used[j]: nxt = (j, end); break
            if nxt is None: break
            j, end = nxt; used[j] = True
            p = S[j][end]; poly.append(p); cur = key(p)
        if cur == start and len(poly) > 3:
            out.append(np.array(poly))
    return out

W_IN = 11.0
H_IN = W_IN * (VIEW[3] - VIEW[2]) / (VIEW[1] - VIEW[0])
fig = plt.figure(figsize=(W_IN, H_IN), dpi=a.dpi)
ax = fig.add_axes([0, 0, 1, 1]); ax.set_xlim(VIEW[0], VIEW[1]); ax.set_ylim(VIEW[2], VIEW[3]); ax.axis("off")
fig.patch.set_facecolor(PAPER)

# sheet border
border = Rectangle((VIEW[0] + 3, VIEW[2] + 3), VIEW[1] - VIEW[0] - 6, VIEW[3] - VIEW[2] - 6, fill=False, lw=1.4,
                   edgecolor=INK, zorder=20)
ax.add_patch(border)

# side features (x = -45: rails, chamber walls) thin; the centreline cut heavy + hatched solids
lc = ax.add_collection(LineCollection(S45, colors=THIN, linewidths=0.55, zorder=2)); lc.set_clip_path(border)
L = loops(S0)
verts, codes = [], []
for P in L:
    verts += list(P) + [P[0]]; codes += [Path.MOVETO] + [Path.LINETO] * (len(P) - 1) + [Path.CLOSEPOLY]
if verts:
    ax.add_patch(PathPatch(Path(np.array(verts), codes), facecolor="none", edgecolor=INK, hatch="//////", zorder=3,
                           linewidth=0))
lc = ax.add_collection(LineCollection(S0, colors=INK, linewidths=1.25, zorder=4)); lc.set_clip_path(border)

# game pieces (to scale) with centre marks and diameter callouts
balls = json.load(open("balls_final.json"))
for b in balls:
    r = b["d"] * 25.4 / 2
    fc, ec = (NECTAR, NECTAR_E) if b["kind"] == "NECTAR" else (POLLEN, POLLEN_E)
    ax.add_patch(Circle((b["y"], b["z"]), r, facecolor=fc, alpha=0.55, edgecolor="none", zorder=8))
    ax.add_patch(Circle((b["y"], b["z"]), r, facecolor="none", edgecolor=ec, linewidth=1.6, zorder=9))
    for dy, dz in ((1, 0), (0, 1)):
        ax.plot([b["y"] - dy * r * 0.25, b["y"] + dy * r * 0.25], [b["z"] - dz * r * 0.25, b["z"] + dz * r * 0.25],
                color=ec, lw=0.8, zorder=9)

def label(txt, xy, xytext, ha="left", va="center", color=INK, weight="medium", size=8.6):
    ax.annotate(txt, xy=xy, xytext=xytext, ha=ha, va=va, fontsize=size, color=color, fontweight=weight, zorder=12,
                arrowprops=dict(arrowstyle="-", lw=0.7, color=INK, shrinkA=2, shrinkB=0),
                bbox=dict(boxstyle="square,pad=0.15", fc=PAPER, ec="none"))

def path_arrow(pts, color, dashed=False):
    P = np.array(pts, float)
    t = np.linspace(0, 1, len(P)); tt = np.linspace(0, 1, 200)
    sy = np.interp(tt, t, P[:, 0]); sz = np.interp(tt, t, P[:, 1])
    k = 9; ky = np.convolve(np.pad(sy, k, mode="edge"), np.ones(2 * k + 1) / (2 * k + 1), "valid")
    kz = np.convolve(np.pad(sz, k, mode="edge"), np.ones(2 * k + 1) / (2 * k + 1), "valid")
    ax.plot(ky[:-6], kz[:-6], color=color, lw=2.6, ls=(0, (5, 3)) if dashed else "-", zorder=7, solid_capstyle="round")
    ax.add_patch(FancyArrowPatch((ky[-8], kz[-8]), (ky[-1], kz[-1]), arrowstyle="-|>,head_length=6,head_width=3.6",
                                 color=color, lw=2.6, zorder=7, mutation_scale=1.6))

# paths (only colour besides the pieces themselves), drawn behind the pieces
path_arrow([(0, 127), (35, 137), (60, 146), (78, 158), (84, 182), (68, 206), (32, 220), (-5, 216), (-40, 212),
            (-40, 252), (-40, 290)], NECTAR_E)
path_arrow([(100, 126), (128, 112), (152, 132), (168, 168), (162, 205), (158, 240), (158, 290)], POLLEN_E)

# legend (top-left, empty part of the section)
LX, LY = -248, 284
for i, (fc, ec, head, body) in enumerate((
        (NECTAR, NECTAR_E, "NECTAR  Ø 3.62 in", "Too wide to fall between the side rails: rides them up and is\nlifted into the front chamber by the spoke wheel."),
        (POLLEN, POLLEN_E, "POLLEN  Ø 2.80 in", "Narrow enough to slip between the rails onto the floor,\nthen climbs the rear tube."))):
    y = LY - i * 30
    ax.add_patch(Circle((LX + 4, y - 3), 4.2, facecolor=fc, edgecolor=ec, lw=1.0, zorder=12))
    ax.text(LX + 12, y, head, fontsize=8.4, fontweight="semibold", color=INK, va="center", zorder=12)
    ax.text(LX + 12, y - 6.5, body, fontsize=7.0, color=INK, va="top", zorder=12, linespacing=1.25)
ax.text(-40, 296, "TO NECTAR TURRET", ha="center", va="top", fontsize=7.6, color=INK, fontweight="semibold", zorder=12,
        bbox=dict(boxstyle="square,pad=0.15", fc=PAPER, ec="none"))
ax.text(158, 296, "TO POLLEN TURRET", ha="center", va="top", fontsize=7.6, color=INK, fontweight="semibold", zorder=12,
        bbox=dict(boxstyle="square,pad=0.15", fc=PAPER, ec="none"))

def balloon(n, xy, at):
    ax.annotate("", xy=xy, xytext=at, arrowprops=dict(arrowstyle="-", lw=0.8, color=INK, shrinkA=9, shrinkB=0), zorder=12)
    ax.add_patch(Circle(at, 8.5, facecolor=PAPER, edgecolor=INK, lw=1.1, zorder=13))
    ax.text(at[0], at[1] - 0.4, str(n), ha="center", va="center", fontsize=9, fontweight="semibold", color=INK, zorder=14)
balloon(1, (-44, 89), (-112, 58))      # intake exit: first rail standoff, where pieces leave the intake
balloon(2, (50, 108), (18, 44))        # transfer path: the split (rails vs floor)
balloon(3, (6, 252), (60, 284))        # turret inlets: both top-plate openings
ax.annotate("", xy=(102, 252), xytext=(60, 284), arrowprops=dict(arrowstyle="-", lw=0.8, color=INK, shrinkA=9, shrinkB=0), zorder=12)

# title block
tb = (VIEW[1] - 3 - 128, VIEW[2] + 3)
ax.add_patch(Rectangle(tb, 128, 24, facecolor=PAPER, edgecolor=INK, lw=1.0, zorder=11))
ax.plot([tb[0], tb[0] + 128], [tb[1] + 12, tb[1] + 12], color=INK, lw=0.6, zorder=12)
ax.text(tb[0] + 4, tb[1] + 18, "SECTION A–A · TRANSFER (CENTERLINE CUT)", fontsize=6.4, fontweight="semibold", va="center", zorder=12)
ax.text(tb[0] + 4, tb[1] + 6, "FRONT = LEFT   ·   GAME PIECES TO SCALE   ·   TEAM 14712", fontsize=6.0, va="center", zorder=12)

fig.savefig(a.out, facecolor=PAPER)
print("wrote", a.out, "loops", len(L))
