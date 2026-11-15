"""Pollen flywheel: velocity after a real shot, starting gains vs tuned gains, overlaid.

Reads Mechanism Tuner flight-recorder traces (one CSV per shot, `mechtune.py shots/ab --rec-each`),
lines every shot up on its shot event (the tuner's dip detector: first sample outside the at-speed
band, event code != 0), and plots velocity against time since that event.

Per shot it measures, from the trace itself:
  recovery_ms  time from the event to the first later sample back inside |v - T| <= frac * T
               (the shipped "at speed" test that shooter.java waits on before opening the gate)
  droop_pct    (T - min v) / T, the deepest point of the dip, in % of target
The tuner's own recoverMs (from trials.jsonl) uses the same band; both are written to the CSV.

usage:
  python plot_flywheel_recovery.py --arm "LABEL=GLOB" --arm "LABEL=GLOB" [--frac 0.05]
                                   [--title ...] [--note ...] [--out NAME]
Outputs NAME.png, NAME.svg and NAME_shots.csv next to this script.
"""
from __future__ import annotations

import argparse
import csv
import glob
import math
import os
import statistics

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))

# Reference categorical palette, slots 1-2 (validated: CVD dE 24.7, contrast >= 3:1 on #fcfcfb).
SERIES = ["#2a78d6", "#eb6834"]
SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK2 = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"
AXIS = "#c3c2b7"
BAND = "#f0efec"

PRE_S, POST_S = 0.10, 0.60


def load(path: str):
    with open(path, encoding="utf-8") as fh:
        rows = list(csv.DictReader(ln for ln in fh if not ln.startswith("#")))
    t = [float(r["t"]) for r in rows]
    v = [float(r["pVel"]) for r in rows]
    tgt = [float(r["pTgt"]) for r in rows]
    volts = [float(r["volts"]) for r in rows if r["volts"]]
    ev = [i for i, r in enumerate(rows) if r["event"] not in ("", "0")]
    return t, v, tgt, volts, ev


def shot(path: str, frac: float):
    t, v, tgt, volts, ev = load(path)
    if not ev:
        return None
    i0 = ev[0]
    T = tgt[i0]
    t0 = t[i0]
    after = [(t[k] - t0, v[k]) for k in range(i0, len(t)) if t[k] - t0 <= POST_S]
    # First dip only: walk forward from the event to the first sample back inside the band.
    # A second piece fed within the window (2026-11-14 ab1860_B_b3: two shots ~470 ms apart) is
    # its own shot, so the trace is also cut where it leaves the band again after recovering.
    rec, vmin, cut = None, after[0][1], POST_S
    for k, (dt, vv) in enumerate(after):
        if rec is None:
            vmin = min(vmin, vv)
            if k > 0 and abs(vv - T) <= frac * T:
                rec = dt
        elif vv < T * (1 - frac):
            cut = dt
            break
    win = [(t[k] - t0, v[k]) for k in range(len(t)) if -PRE_S <= t[k] - t0 < cut]
    return {
        "file": os.path.basename(path),
        "target": T,
        "droop_pct": 100.0 * (T - vmin) / T,
        "recovery_ms": None if rec is None else 1000.0 * rec,
        "volts": statistics.fmean(volts) if volts else float("nan"),
        "trace": win,
    }


def describe(xs):
    xs = [x for x in xs if x is not None]
    if not xs:
        return "n=0"
    m = statistics.fmean(xs)
    sd = statistics.stdev(xs) if len(xs) > 1 else float("nan")
    return f"{m:.0f} ± {sd:.0f} (n={len(xs)})" if m >= 10 else f"{m:.1f} ± {sd:.1f} (n={len(xs)})"


def median_trace(traces, step=0.005):
    """Median velocity across shots on a common time grid (linear interpolation)."""
    grid = [-PRE_S + k * step for k in range(int((PRE_S + POST_S) / step) + 1)]
    out = []
    for g in grid:
        vals = []
        for tr in traces:
            for (ta, va), (tb, vb) in zip(tr, tr[1:]):
                if ta <= g <= tb:
                    vals.append(va if tb == ta else va + (vb - va) * (g - ta) / (tb - ta))
                    break
        out.append(statistics.median(vals) if len(vals) >= max(2, len(traces) // 2) else math.nan)
    return grid, out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", action="append", required=True, help='"LABEL=GLOB" (repeat; 1-2 arms)')
    ap.add_argument("--frac", type=float, default=0.05, help="AT_SPEED_FRACTION")
    ap.add_argument("--title", default="Pollen flywheel: velocity after a real shot")
    ap.add_argument("--note", default="")
    ap.add_argument("--out", default="pollen_flywheel_recovery")
    a = ap.parse_args()

    arms = []
    for spec in a.arm:
        label, pattern = spec.split("=", 1)
        files = sorted(glob.glob(pattern))
        shots = [s for s in (shot(f, a.frac) for f in files) if s is not None]
        skipped = len(files) - len(shots)
        arms.append((label, shots, skipped))

    targets = {round(s["target"]) for _, shots, _ in arms for s in shots}
    if len(targets) != 1:
        raise SystemExit(f"arms were shot at different targets: {sorted(targets)}")
    T = targets.pop()

    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10, "svg.fonttype": "none"})
    fig, ax = plt.subplots(figsize=(9, 5.4), dpi=100)
    fig.patch.set_facecolor(SURFACE)
    ax.set_facecolor(SURFACE)

    band = ax.axhspan(T * (1 - a.frac), T * (1 + a.frac), color=BAND, zorder=0, lw=0,
                      label=f"at-speed band, target ±{a.frac * 100:.0f}%")
    (tline,) = ax.plot([-PRE_S * 1000, POST_S * 1000], [T, T], color=MUTED, lw=1, ls=(0, (4, 3)),
                       zorder=1, label=f"target {T} ticks/s")

    rows_out = []
    handles = []
    for k, (label, shots, skipped) in enumerate(arms):
        col = SERIES[k]
        for s in shots:
            xs = [1000 * x for x, _ in s["trace"]]
            ax.plot(xs, [y for _, y in s["trace"]], color=col, lw=1, alpha=0.30, zorder=2)
        g, m = median_trace([s["trace"] for s in shots])
        (h,) = ax.plot([1000 * x for x in g], m, color=col, lw=2.2, zorder=3,
                       label=(f"{label}\nrecovery {describe([s['recovery_ms'] for s in shots])} ms, "
                              f"droop {describe([s['droop_pct'] for s in shots])} %"))
        handles.append(h)
        for s in shots:
            rows_out.append({"arm": label, "file": s["file"], "target": s["target"],
                             "recovery_ms": "" if s["recovery_ms"] is None else round(s["recovery_ms"], 1),
                             "droop_pct": round(s["droop_pct"], 2), "volts": round(s["volts"], 2)})
        if skipped:
            print(f"{label}: {skipped} trace(s) without a shot event skipped")

    volts = [r["volts"] for r in rows_out]
    ax.set_xlim(-PRE_S * 1000, POST_S * 1000)
    lo = min(min(y for _, y in s["trace"]) for _, shots, _ in arms for s in shots)
    ax.set_ylim(lo - 0.04 * T, T * (1 + 2 * a.frac))
    ax.set_xlabel("time since shot detected (ms)", color=INK2)
    ax.set_ylabel("flywheel velocity (ticks/s)", color=INK2)
    ax.grid(True, color=GRID, lw=0.6)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(AXIS)
    ax.tick_params(colors=MUTED)
    ax.set_title(a.title, loc="left", color=INK, fontsize=13, pad=26)
    sub = (f"Each thin line is one shot, bold = median. Battery {min(volts):.2f}-{max(volts):.2f} V. "
           f"Recovery = back inside the band; mean ± sd.")
    ax.text(0, 1.02, sub, transform=ax.transAxes, color=INK2, fontsize=9)
    leg = ax.legend(handles=handles + [tline, band], loc="lower right", frameon=True, fontsize=9)
    leg.get_frame().set_edgecolor(GRID)
    leg.get_frame().set_facecolor(SURFACE)
    if a.note:
        import textwrap
        fig.text(0.01, 0.012, textwrap.fill(a.note, 135), color=MUTED, fontsize=8, va="bottom")
    fig.tight_layout(rect=(0, 0.06 if a.note else 0, 1, 1))

    base = os.path.join(HERE, a.out)
    fig.savefig(base + ".png", dpi=200, facecolor=SURFACE)
    fig.savefig(base + ".svg", facecolor=SURFACE)
    with open(base + "_shots.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows_out[0].keys()))
        w.writeheader()
        w.writerows(rows_out)
    for label, shots, _ in arms:
        print(f"{label}: recovery {describe([s['recovery_ms'] for s in shots])} ms, "
              f"droop {describe([s['droop_pct'] for s in shots])} %")
    print(f"wrote {base}.png, .svg, _shots.csv")


if __name__ == "__main__":
    main()
