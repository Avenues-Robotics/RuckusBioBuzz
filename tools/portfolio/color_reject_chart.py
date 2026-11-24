#!/usr/bin/env python3
"""Portfolio chart: the intake color check's recorded samples and the shipped thresholds (2026-11-24).

Reads TeamCode/.../diagnostics/color (1).csv (Mechanism Tuner samples, 2026-11-03: RED / BLUE / EMPTY) and draws
blue/red ratio (log) against distance, with the ColorCheck thresholds as they ship in SystemConstants.ColorCheck.
Sized for the TeleOp slide's big panel frame (235.5 x 221.2 pt), styled to the portfolio deck (Poppins, cream
panel, deck greens). Output: docs/portfolio/color_reject_separation.png

  python3 tools/portfolio/color_reject_chart.py [--font-dir DIR_WITH_Poppins-*.ttf]

Poppins (OFL, github.com/google/fonts ofl/poppins) is used when --font-dir holds it; otherwise Helvetica.
"""
import argparse
import csv
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
CSV = os.path.join(ROOT, "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/diagnostics/color (1).csv")
OUT = os.path.join(ROOT, "docs/portfolio/color_reject_separation.png")

# SystemConstants.ColorCheck as shipped
RED_OVER_BLUE, BLUE_OVER_RED, MAX_DIST_CM = 0.93, 1.62, 6.7

# deck tokens (read off the slide): cream panel, body ink, heading green
SURFACE, INK, INK2, HEAD, GRID = "#FBF8E6", "#1F2A12", "#55603F", "#3F5A12", "#E4E0C8"
RED, BLUE, OLIVE = "#C2362B", "#2F64C8", "#6E8B3D"   # validated: light mode on SURFACE, all six checks pass
CLASSES = [  # label, colour, marker, legend name
    ("RED", RED, "o", "Red NECTAR"),
    ("BLUE", BLUE, "s", "Blue NECTAR"),
    ("EMPTY", OLIVE, "^", "Empty intake"),
]
X_MIN, X_MAX, Y_MIN, Y_MAX = 0.38, 8.5, 0.8, 9.3


def called(r, b, d):
    if d >= MAX_DIST_CM:
        return "EMPTY"
    if r > RED_OVER_BLUE * b:
        return "RED"
    if b > BLUE_OVER_RED * r:
        return "BLUE"
    return "EMPTY"


def use_fonts(font_dir):
    if font_dir and os.path.isdir(font_dir):
        found = [f for f in os.listdir(font_dir) if f.startswith("Poppins-") and f.endswith(".ttf")]
        for f in found:
            font_manager.fontManager.addfont(os.path.join(font_dir, f))
        if found:
            return "Poppins"
    return "Helvetica"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--font-dir")
    ap.add_argument("--dpi", type=int, default=300)
    a = ap.parse_args()
    family = use_fonts(a.font_dir)

    rows = list(csv.DictReader(open(CSV)))
    pts = {c[0]: [] for c in CLASSES}
    wrong = 0
    for row in rows:
        r, b, d = float(row["r"]), float(row["b"]), float(row["dist"])
        pts[row["label"]].append((b / r, d))
        wrong += called(r, b, d) != row["label"]

    plt.rcParams.update({"font.family": family, "font.size": 16, "font.weight": "regular"})
    fig, ax = plt.subplots(figsize=(8.0, 8.0 * 221.2 / 235.5), dpi=a.dpi)
    fig.patch.set_facecolor(SURFACE)
    ax.set_facecolor(SURFACE)

    x_red, x_blue = 1 / RED_OVER_BLUE, BLUE_OVER_RED   # red if b/r < 1/0.93, blue if b/r > 1.62, both within 6.7 cm
    # decision regions, faint tints of each class colour
    ax.fill_between([X_MIN, x_red], Y_MIN, MAX_DIST_CM, color=RED, alpha=0.07, lw=0, zorder=0)
    ax.fill_between([x_blue, X_MAX], Y_MIN, MAX_DIST_CM, color=BLUE, alpha=0.07, lw=0, zorder=0)
    ax.fill_between([X_MIN, X_MAX], MAX_DIST_CM, Y_MAX, color=OLIVE, alpha=0.08, lw=0, zorder=0)

    dash = (0, (5, 4))
    ax.plot([X_MIN, X_MAX], [MAX_DIST_CM] * 2, color=INK2, lw=1.8, ls=dash, zorder=1)
    ax.plot([x_red] * 2, [Y_MIN, MAX_DIST_CM], color=RED, lw=1.8, ls=dash, zorder=1)
    ax.plot([x_blue] * 2, [Y_MIN, MAX_DIST_CM], color=BLUE, lw=1.8, ls=dash, zorder=1)

    for label, colour, marker, name in CLASSES:
        xs = [p[0] for p in pts[label]]
        ys = [p[1] for p in pts[label]]
        ax.scatter(xs, ys, s=110, marker=marker, color=colour, edgecolors=SURFACE, linewidths=1.5, zorder=3,
                   label=f"{name}  ·  {len(xs)}")

    ax.set_xscale("log")
    ax.set_xlim(X_MIN, X_MAX)
    ax.set_ylim(Y_MIN, Y_MAX)
    ax.set_xticks([0.5, 1, 2, 4, 8])
    ax.set_xticklabels(["0.5", "1", "2", "4", "8"])
    ax.set_xticks([], minor=True)
    ax.set_yticks([2, 4, 6, 8])
    ax.set_xlabel("blue ÷ red light", color=INK, fontsize=18, fontweight="medium", labelpad=8)
    ax.set_ylabel("distance to piece (cm)", color=INK, fontsize=18, fontweight="medium", labelpad=8)
    ax.tick_params(colors=INK2, length=0, labelsize=16.5, pad=6)
    ax.grid(True, which="major", color=GRID, lw=1, zorder=0)
    for s in ax.spines.values():
        s.set_visible(False)

    # region names (ink; colour is carried by the marks and tints) and threshold values
    ax.text(0.53, 6.2, "called RED", color=INK, fontsize=18, fontweight="semibold", ha="center", va="center")
    ax.text(4.9, 6.2, "called BLUE", color=INK, fontsize=18, fontweight="semibold", ha="center", va="center")
    ax.text(1.05, 8.8, "nothing in the intake", color=INK, fontsize=18, fontweight="semibold", ha="center",
            va="center")
    ax.text(X_MAX / 1.02, MAX_DIST_CM + 0.15, "6.7 cm", color=INK2, fontsize=15.5, ha="right", va="bottom")
    ax.text(x_red / 1.03, 3.0, "red ÷ blue > 0.93", color=INK2, fontsize=15, rotation=90, ha="right", va="center")
    ax.text(x_blue * 1.03, 3.0, "blue ÷ red > 1.62", color=INK2, fontsize=15, rotation=90, ha="left", va="center")

    leg = ax.legend(loc="upper right", bbox_to_anchor=(1.0, 0.985), frameon=False, fontsize=15.5, handletextpad=0.25,
                    labelspacing=0.45, labelcolor=INK)
    for h in leg.legend_handles:
        h.set_sizes([95])

    fig.text(0.035, 0.955, f"{len(rows)} recorded readings", color=HEAD, fontsize=23, fontweight="bold",
             ha="left", va="center")
    fig.text(0.965, 0.955, f"{wrong} misclassified", color=HEAD, fontsize=23, fontweight="bold", ha="right",
             va="center")

    fig.tight_layout(rect=(0, 0, 1, 0.94))
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    fig.savefig(OUT, facecolor=SURFACE)
    print(f"{OUT}  ({len(rows)} rows, {wrong} misclassified, font {family}, {a.dpi} dpi)")


if __name__ == "__main__":
    main()
