"""Transfer slide, v2 (operator style): cream sheet, soft robot silhouette, green linework; the RAMP alone in dark
green; only the transfer spoke wheel's surgical tubing shown; NECTAR / POLLEN as rendered wiffle balls (to scale).
Geometry: geom2d.npz (robot STL sections x = 0 and x = -45 mm + silhouette).
  python3 transfer_v2.py [--arrows] [--out FILE] [--dpi N]
"""
import argparse, json
from collections import defaultdict
import numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.patches import PathPatch, FancyArrowPatch
from matplotlib.path import Path
from scipy.ndimage import gaussian_filter

ap = argparse.ArgumentParser()
ap.add_argument("--arrows", action="store_true"); ap.add_argument("--out", default="transfer_v2.png")
ap.add_argument("--dpi", type=int, default=300)
a = ap.parse_args()

SURF = "#FBF8E6"
SIL_FILL, SIL_EDGE = "#E7EBD4", "#B9C49C"
SIDE, CUT = "#A9B585", "#6F7F50"            # other linework: x = -45 (light), x = 0 (mid)
RAMP = "#3F5A12"                            # the ramp only
TUBE_FILL, TUBE_EDGE = "#7E9A45", "#3F5A12"

g = np.load("geom2d.npz")
S0, S45, mask, (Y0, Y1, Z0, Z1) = g["S0"], g["S45"], g["mask"], g["extent"]
VIEW = (-255.0, 235.0, 30.0, 262.0)

def loops(S, tol=0.02):
    key = lambda p: (round(p[0] / tol), round(p[1] / tol))
    adj = defaultdict(list)
    for i, (p, q) in enumerate(S):
        adj[key(p)].append((i, 1)); adj[key(q)].append((i, 0))
    used = np.zeros(len(S), bool); out = []
    for i in range(len(S)):
        if used[i]: continue
        used[i] = True; idx = [i]; poly = [S[i][0], S[i][1]]; start = key(S[i][0]); cur = key(S[i][1])
        for _ in range(200000):
            if cur == start: break
            nxt = next(((j, e) for j, e in adj[cur] if not used[j]), None)
            if nxt is None: break
            j, e = nxt; used[j] = True; idx.append(j); poly.append(S[j][e]); cur = key(S[j][e])
        out.append((np.array(poly), idx, cur == start))
    return out

def dist_to_polyline(P, L):
    L = np.asarray(L, float); best = np.full(len(P), np.inf)
    for a_, b_ in zip(L[:-1], L[1:]):
        ab = b_ - a_; t = np.clip(((P - a_) @ ab) / (ab @ ab), 0, 1)
        best = np.minimum(best, np.linalg.norm(P - (a_ + t[:, None] * ab), axis=1))
    return best

# ---- classify linework
mid0 = S0.mean(1); mid45 = S45.mean(1)
INTAKE_STAR, SPOKE = np.array([-168.0, 140.0]), np.array([95.0, 198.0])
drop0 = (np.linalg.norm(mid0 - INTAKE_STAR, axis=1) < 80) & (np.linalg.norm(mid0 - np.array([-230.0, 100.0]), axis=1) > 40)   # keep the intake wheel                   # intake star tubing: hidden
FLOOR_J = [(-5, 66), (122, 95), (140, 105), (158, 122), (172, 143), (183, 168), (190, 195), (194, 225), (196, 250)]
ramp0 = (dist_to_polyline(mid0, FLOOR_J) < 6.5) & ~drop0
tube0 = (np.linalg.norm(mid0 - SPOKE, axis=1) < 72) & (mid0[:, 1] < 238) & ~ramp0

ramp45 = np.zeros(len(S45), bool)
RAIL = [(-50, 88), (120, 137)]
for poly, idx, closed in loops(S45):
    c = poly.mean(0); ext = poly.max(0) - poly.min(0)
    perim = np.sum(np.linalg.norm(np.diff(poly, axis=0), axis=1))
    is_rail = ext.max() < 14 and dist_to_polyline(c[None], RAIL)[0] < 7
    is_plate = perim > 220 and -40 < c[0] < 190 and 70 < c[1] < 240 and ext[1] > 100
    if is_rail or is_plate:
        ramp45[idx] = True

# ---- figure
W_IN = 10.0
fig = plt.figure(figsize=(W_IN, W_IN * (VIEW[3] - VIEW[2]) / (VIEW[1] - VIEW[0])), dpi=a.dpi)
ax = fig.add_axes([0, 0, 1, 1]); ax.set_xlim(VIEW[0], VIEW[1]); ax.set_ylim(VIEW[2], VIEW[3]); ax.axis("off")
fig.patch.set_facecolor(SURF)

# silhouette: smoothed mask, filled, with a soft outline
from scipy.ndimage import binary_closing, label
mk = binary_closing(mask, iterations=2)
lab, nl = label(~mk)
sizes = np.bincount(lab.ravel())
small = sizes < 400; small[0] = False
mk = mk | small[lab]
sm = gaussian_filter(mk.astype(float), 1.6)
yy = np.linspace(Y0, Y1, mask.shape[1]); zz = np.linspace(Z0, Z1, mask.shape[0])
ax.contourf(yy, zz, sm, levels=[0.5, 2], colors=[SIL_FILL], zorder=0, antialiased=True)
ax.contour(yy, zz, sm, levels=[0.5], colors=[SIL_EDGE], linewidths=0.7, zorder=0.5)

kw = dict(capstyle="round", joinstyle="round")
ax.add_collection(LineCollection(S45[~ramp45], colors=SIDE, linewidths=0.55, zorder=1, **kw))
ax.add_collection(LineCollection(S0[~(ramp0 | drop0 | tube0)], colors=CUT, linewidths=0.9, zorder=2, **kw))

# ramp: faint fill of its closed shapes + heavy outline
for S, m in ((S45, ramp45), (S0, ramp0)):
    for poly, idx, closed in loops(S[m]):
        if closed and len(poly) > 3:
            ax.add_patch(PathPatch(Path(np.vstack([poly, poly[:1]])), facecolor=RAMP, alpha=0.13, edgecolor="none", zorder=2.5))
    ax.add_collection(LineCollection(S[m], colors=RAMP, linewidths=1.9, zorder=3, **kw))

# surgical tubing on the transfer spoke wheel: rasterise the two walls of each tube, close the gap, fill solid
from scipy.ndimage import distance_transform_edt
TY0, TY1, TZ0, TZ1, TR = 15.0, 175.0, 125.0, 240.0, 0.2
tm = np.zeros((int((TZ1 - TZ0) / TR), int((TY1 - TY0) / TR)), bool)
for (p, q) in S0[tube0]:
    n = int(max(abs(q[0] - p[0]), abs(q[1] - p[1])) / TR) + 2
    ys = np.linspace(p[0], q[0], n); zs = np.linspace(p[1], q[1], n)
    iy = ((ys - TY0) / TR).astype(int); iz = ((zs - TZ0) / TR).astype(int)
    ok = (iy >= 0) & (iy < tm.shape[1]) & (iz >= 0) & (iz < tm.shape[0]); tm[iz[ok], iy[ok]] = True
close_r = 4.5 / TR
dil = distance_transform_edt(~tm) <= close_r
solid = distance_transform_edt(dil) > close_r
ty = TY0 + (np.arange(tm.shape[1]) + 0.5) * TR; tz = TZ0 + (np.arange(tm.shape[0]) + 0.5) * TR
ax.contourf(ty, tz, gaussian_filter(solid.astype(float), 1.0), levels=[0.5, 2], colors=[TUBE_FILL], zorder=3.5)
ax.add_collection(LineCollection(S0[tube0], colors=TUBE_EDGE, linewidths=1.0, zorder=3.6, **kw))

# ---- wiffle balls (per-pixel render)
def fib_dirs(n, seed):
    i = np.arange(n) + 0.5; phi = np.arccos(1 - 2 * i / n); th = np.pi * (1 + 5 ** 0.5) * i
    d = np.stack([np.cos(th) * np.sin(phi), np.sin(th) * np.sin(phi), np.cos(phi)], 1)
    rng = np.random.default_rng(seed); q = rng.normal(size=4); q /= np.linalg.norm(q)
    w, x, y, z = q
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    return d @ R.T, R

def hexrgb(h):
    h = h.lstrip("#"); return np.array([int(h[i:i + 2], 16) / 255 for i in (0, 2, 4)])

def wiffle(base_hex, shadow_hex, hole_hex, n_holes, hole_ang, seed, px=900):
    base = hexrgb(base_hex); shadow = hexrgb(shadow_hex); holec = hexrgb(hole_hex)
    s = np.linspace(-1, 1, px); X, Y = np.meshgrid(s, -s)
    r2 = X ** 2 + Y ** 2; inside = r2 <= 1
    Z = np.sqrt(np.clip(1 - r2, 0, 1)); N = np.stack([X, Y, Z], -1)
    L = np.array([-0.45, 0.6, 0.66]); L /= np.linalg.norm(L)
    lam = np.clip(N @ L, 0, 1)
    H = L + np.array([0, 0, 1]); H /= np.linalg.norm(H)
    spec = np.clip(N @ H, 0, 1) ** 60
    lt = np.clip(0.18 + 0.95 * lam, 0, 1)[..., None]
    col = shadow * (1 - lt) + base * lt + 0.42 * spec[..., None]
    rim = np.clip((r2 - 0.82) / 0.18, 0, 1)[..., None]           # darker towards the silhouette edge
    col = col * (1 - 0.12 * rim)
    dirs, R = fib_dirs(n_holes, seed)
    dots = np.tensordot(N, dirs.T, axes=1)                         # (px, px, n)
    best = dots.max(-1)
    c_hole = np.cos(hole_ang); c_lip = np.cos(hole_ang * 1.16)
    hole = (best > c_hole) & inside
    lip = (best > c_lip) & ~hole & inside
    # inside of the shell seen through a hole: darker, lit from the far side
    inner = holec * (0.80 + 0.35 * np.clip(-(N @ L), 0, 1)[..., None] + 0.20 * (1 - Z)[..., None])
    col = np.where(hole[..., None], inner, col)
    col = np.where(lip[..., None], col * 0.86, col)
    seam_n = R @ np.array([0.0, 0.0, 1.0])                        # molding seam: one great circle
    seam = (np.abs(N @ seam_n) < 0.010) & inside & ~hole
    col = np.where(seam[..., None], col * 0.82, col)
    rr = np.sqrt(r2); alpha = np.clip((1 - rr) * px / 2.0, 0, 1)   # 1-px antialiased edge
    edge = np.clip(1 - np.abs(rr - 0.995) * px / 3.0, 0, 1)[..., None]
    col = col * (1 - 0.30 * edge)
    return np.dstack([np.clip(col, 0, 1), alpha])

balls = json.load(open("balls_final.json"))
STYLE = {"NECTAR": ("#E8291E", "#8E120C", "#9A1810", 14, 0.235), "POLLEN": ("#FFD21A", "#C98A00", "#C38B00", 14, 0.255)}
paths = {"NECTAR": [], "POLLEN": []}
for k, b in enumerate(balls):
    r = b["d"] * 25.4 / 2
    col, sh, hc, n, ang = STYLE[b["kind"]]
    img = wiffle(col, sh, hc, n, ang, seed=11 + 7 * k)
    ax.imshow(img, extent=(b["y"] - r, b["y"] + r, b["z"] - r, b["z"] + r), zorder=8, interpolation="lanczos")

if a.arrows:
    def path_arrow(pts, color):
        P = np.array(pts, float); t = np.linspace(0, 1, len(P)); tt = np.linspace(0, 1, 240)
        sy, sz = np.interp(tt, t, P[:, 0]), np.interp(tt, t, P[:, 1]); k = 10
        sy = np.convolve(np.pad(sy, k, mode="edge"), np.ones(2 * k + 1) / (2 * k + 1), "valid")
        sz = np.convolve(np.pad(sz, k, mode="edge"), np.ones(2 * k + 1) / (2 * k + 1), "valid")
        ax.plot(sy[:-8], sz[:-8], color=color, lw=2.0, ls=(0, (4, 3)), zorder=6, solid_capstyle="round")
        ax.add_patch(FancyArrowPatch((sy[-10], sz[-10]), (sy[-1], sz[-1]), arrowstyle="-|>,head_length=5,head_width=3.2",
                                     color=color, lw=2.0, zorder=6, mutation_scale=1.5))
    path_arrow([(0, 127), (35, 137), (60, 146), (78, 158), (84, 182), (68, 206), (32, 220), (-5, 216), (-40, 212),
                (-40, 245), (-40, 258)], "#9E1A13")
    path_arrow([(100, 126), (128, 112), (152, 132), (168, 168), (162, 205), (158, 240), (158, 258)], "#A27A00")

fig.savefig(a.out, facecolor=SURF)
print("wrote", a.out, "ramp45", int(ramp45.sum()), "ramp0", int(ramp0.sum()), "tube", int(tube0.sum()), "dropped", int(drop0.sum()))
