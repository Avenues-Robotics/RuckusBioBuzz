"""Swerve speed-testing charts for the portfolio's Step 4 panel (Swerve Drive slide).

Data: the 45 full-stick straight runs recorded 2026-11-24 through TeleOp Drive Probe (the driver's code path)
on field tiles, tools/swervetune/runs/20261124-11*lane-*_line_b*.csv.gz (smoke runs excluded). FINDINGS.md
2026-11-24 "Full-speed cruise wobble". Battery 10.7-12.6 V under load (the recorder's volts column).

Speed = Pinpoint pose distance over a 0.2 s centred window. Cruise yaw = mean |heading rate| (0.2 s window)
while the stick is at full and speed > 50 in/s. Arms: steering floor 0.60 (lane-fh 'shipped' + fh06, n=15)
vs 0.80 (fh08, n=10), interleaved.

  python3 tools/portfolio/swerve/speed_testing.py docs/portfolio
"""
import sys, gzip, csv, glob, re, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
from scipy import stats

RUNS = os.path.join(os.path.dirname(__file__), "..", "..", "swervetune", "runs")
DARK, PT, GRID, FILL, LIME, ORANGE = "#3F5A12", "#5E8A2A", "#C3D394", "#D9E5B4", "#A8C93A", "#EFA94A"
for name in ("Avenir Next", "Helvetica Neue"):
    if any(name in fe.name for fe in fm.fontManager.ttflist):
        plt.rcParams["font.family"] = name
        break

def load(f):
    r = list(csv.DictReader(gzip.open(f, "rt").read().splitlines()[1:]))
    g = lambda k: np.array([float(x[k]) if x[k] not in ("", None) else np.nan for x in r])
    return g("t"), g("px"), g("py"), g("cf"), g("heading"), g("volts")

def deriv(t, y, h=0.1):
    out = np.full(len(t), np.nan)
    for i in range(len(t)):
        a = np.searchsorted(t, t[i] - h); b = min(np.searchsorted(t, t[i] + h), len(t) - 1)
        if b > a and t[b] - t[a] > h: out[i] = (y[b] - y[a]) / (t[b] - t[a])
    return out

runs = []
for f in sorted(glob.glob(os.path.join(RUNS, "20261124-11*lane-*_line_b*.csv.gz"))):
    if "smoke" in f: continue
    lab = re.search(r"lane-[^_]+_([a-z0-9]+)_line", f).group(1); grp = re.search(r"(lane-[^_]+)_", f).group(1)
    t, px, py, cf, hd, v = load(f)
    sp = np.hypot(deriv(t, px), deriv(t, py))
    on = np.where(np.abs(cf) > 0.5)[0]; t0 = t[on[0]]; toff = t[on[-1]]
    yr = deriv(t, np.degrees(np.unwrap(np.radians(hd))))
    cruise = (np.abs(cf) > 0.5) & (sp > 50)
    after = np.where((t > toff) & (sp < 2))[0]; ioff = np.searchsorted(t, toff)
    floor = 0.60 if (lab == "fh06" or (grp == "lane-fh-1124" and lab == "shipped")) else (0.80 if lab == "fh08" else None)
    runs.append(dict(t=t - t0, sp=sp, rel=toff - t0, peak=np.nanmax(sp), t60=t[np.argmax(sp > 60)] - t0,
                     tstop=t[after[0]] - toff, dstop=np.hypot(px[after[0]] - px[ioff], py[after[0]] - py[ioff]),
                     yaw=np.nanmean(np.abs(yr[cruise])), floor=floor))
peak = np.array([r["peak"] for r in runs]); t60 = np.array([r["t60"] for r in runs])
tstop = np.array([r["tstop"] for r in runs]); dstop = np.array([r["dstop"] for r in runs])
A = np.array([r["yaw"] for r in runs if r["floor"] == 0.60]); B = np.array([r["yaw"] for r in runs if r["floor"] == 0.80])
p = stats.ttest_ind(B, A, equal_var=False).pvalue
print(f"n={len(runs)} top {peak.mean():.1f}+/-{peak.std(ddof=1):.1f} in/s; 0-60 {t60.mean():.2f} s; stop {tstop.mean():.2f} s "
      f"{dstop.mean():.1f} in; yaw 0.60 {A.mean():.1f} (n{len(A)}) vs 0.80 {B.mean():.1f} (n{len(B)}) p={p:.4f}")
out = sys.argv[1]

# ---- chart 1: speed vs time ----
fig = plt.figure(figsize=(218 / 72, 100 / 72), dpi=1200)
ax = fig.add_axes([0.13, 0.24, 0.84, 0.70])
grid_t = np.linspace(-0.1, 1.45, 300)
curves = np.array([np.interp(grid_t, r["t"], np.nan_to_num(r["sp"]), left=np.nan, right=np.nan) for r in runs])
for c in curves: ax.plot(grid_t, c, color=LIME, lw=0.35, alpha=0.55, zorder=2)
typ = int(np.argmin(np.abs(peak - np.median(peak))))   # the run whose top speed is the median one
ax.plot(grid_t, curves[typ], color=DARK, lw=1.3, zorder=4, solid_capstyle="round")
ax.text(0.30, 50, "one typical run", fontsize=4.4, color=DARK, fontweight="bold", ha="right", va="bottom")
ax.text(0.62, 22, f"all {len(runs)} test runs", fontsize=4.4, color="#7FA02E", fontweight="bold", ha="left")
ax.axhline(peak.mean(), color=ORANGE, lw=0.6, ls=(0, (3, 2)), zorder=3)
ax.text(1.43, peak.mean() + 2.5, f"top speed {peak.mean():.0f} in/s", ha="right", va="bottom", fontsize=4.6, color=DARK, fontweight="bold")
rel = np.mean([r["rel"] for r in runs])
ax.axvspan(rel, 1.45, color=FILL, alpha=0.6, lw=0, zorder=1)
ax.text(rel + 0.02, 4, "driver\nlets go", fontsize=4.0, color=DARK, va="bottom", linespacing=1.1)
ax.set_xlim(-0.1, 1.45); ax.set_ylim(0, 88)
ax.set_xticks([0, 0.25, 0.5, 0.75, 1.0, 1.25]); ax.set_yticks([0, 20, 40, 60, 80])
ax.grid(True, color=GRID, lw=0.3, ls=(0, (2, 2)))
for s in ("top", "right"): ax.spines[s].set_visible(False)
for s in ("left", "bottom"): ax.spines[s].set_color(DARK); ax.spines[s].set_linewidth(0.6)
ax.tick_params(colors=DARK, labelsize=4.4, length=1.5, width=0.4, pad=1)
for tl in ax.get_xticklabels() + ax.get_yticklabels(): tl.set_fontweight("bold")
ax.set_xlabel("seconds after the driver pushes full speed", fontsize=4.6, color=DARK, fontweight="bold", labelpad=1)
ax.set_ylabel("speed (in/s)", fontsize=4.6, color=DARK, fontweight="bold", labelpad=1.5)
fig.savefig(os.path.join(out, "swerve_speed_vs_time.png"), transparent=True); plt.close(fig)

# ---- chart 2: iterate, plain before / after for judges ----
fig = plt.figure(figsize=(218 / 72, 50 / 72), dpi=1200)
ax = fig.add_axes([0.14, 0.30, 0.62, 0.66])
rng = np.random.default_rng(3)
for yv, arr, col, lab in ((1, A, "#B9CC86", "BEFORE"), (0, B, DARK, "AFTER")):
    ax.barh(yv, arr.mean(), height=0.62, color=col, zorder=2)
    ax.text(-0.02, yv, lab, transform=ax.get_yaxis_transform(), ha="right", va="center", fontsize=5.2, color=DARK, fontweight="bold")
    ax.text(arr.mean() + 0.3, yv, f"{arr.mean():.1f}\u00b0/s  ({len(arr)} runs)", ha="left", va="center", fontsize=5.0, color=DARK, fontweight="bold")
cut = (A.mean() - B.mean()) / A.mean()
ax.text(0.985, 0.62, f"{cut:.0%}\nless wobble", transform=fig.transFigure, ha="right", va="center", fontsize=7.0, color=ORANGE, fontweight="bold", linespacing=0.95)
ax.set_ylim(-0.6, 1.6); ax.set_yticks([]); ax.set_xlim(0, 16); ax.set_xticks([0, 5, 10, 15])
ax.grid(True, axis="x", color=GRID, lw=0.3, ls=(0, (2, 2))); ax.set_axisbelow(True)
for sp_ in ("top", "right", "left"): ax.spines[sp_].set_visible(False)
ax.spines["bottom"].set_color(DARK); ax.spines["bottom"].set_linewidth(0.6)
ax.tick_params(colors=DARK, labelsize=4.4, length=1.5, width=0.4, pad=1)
for tl in ax.get_xticklabels(): tl.set_fontweight("bold")
ax.set_xlabel("robot twisting off course at top speed (\u00b0/s, lower is better)", fontsize=4.4, color=DARK, fontweight="bold", labelpad=1)
fig.savefig(os.path.join(out, "swerve_floor_ab.png"), transparent=True); plt.close(fig)
print(f"cut {cut:.1%}; typical run peak {peak[typ]:.1f}")
