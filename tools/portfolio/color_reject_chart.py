#!/usr/bin/env python3
"""Portfolio chart: the intake color check's recorded samples and the shipped thresholds (2026-11-24).

Reads TeamCode/.../diagnostics/color (1).csv (Mechanism Tuner samples, 2026-11-03: RED / BLUE / EMPTY) and draws
blue/red ratio (log) against distance, with the ColorCheck thresholds as they ship in SystemConstants.ColorCheck.
Sized for the TeleOp slide's big panel frame (235.5 x 221.2 pt). Output: docs/portfolio/color_reject_separation.png

  python3 tools/portfolio/color_reject_chart.py
"""
import csv
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
CSV = os.path.join(ROOT, "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/diagnostics/color (1).csv")
OUT = os.path.join(ROOT, "docs/portfolio/color_reject_separation.png")

# SystemConstants.ColorCheck as shipped
RED_OVER_BLUE, BLUE_OVER_RED, MAX_DIST_CM = 0.93, 1.62, 6.7

SURFACE, INK, INK2, GRID = "#FBF8E6", "#1F2A12", "#4A5536", "#D9D5BC"
CLASSES = [  # label, colour (validated: light mode on SURFACE, all checks pass), marker, legend name
    ("RED", "#C2362B", "o", "Red NECTAR"),
    ("BLUE", "#2F64C8", "s", "Blue NECTAR"),
    ("EMPTY", "#6E8B3D", "^", "Empty intake"),
]


def called(r, b, d):
    if d >= MAX_DIST_CM:
        return "EMPTY"
    if r > RED_OVER_BLUE * b:
        return "RED"
    if b > BLUE_OVER_RED * r:
        return "BLUE"
    return "EMPTY"


def main():
    rows = list(csv.DictReader(open(CSV)))
    pts = {c[0]: [] for c in CLASSES}
    wrong = 0
    for row in rows:
        r, b, d = float(row["r"]), float(row["b"]), float(row["dist"])
        pts[row["label"]].append((b / r, d))
        wrong += called(r, b, d) != row["label"]

    plt.rcParams.update({"font.family": "Helvetica", "font.size": 17})
    fig, ax = plt.subplots(figsize=(8.0, 8.0 * 221.2 / 235.5), dpi=200)
    fig.patch.set_facecolor(SURFACE)
    ax.set_facecolor(SURFACE)

    # thresholds: red if b/r < 1/0.93, blue if b/r > 1.62, only when closer than 6.7 cm
    x_red, x_blue = 1 / RED_OVER_BLUE, BLUE_OVER_RED
    ax.axhline(MAX_DIST_CM, color=INK2, lw=2, ls=(0, (6, 4)), zorder=1)
    ax.plot([x_red, x_red], [0.8, MAX_DIST_CM], color="#C2362B", lw=2, ls=(0, (6, 4)), zorder=1)
    ax.plot([x_blue, x_blue], [0.8, MAX_DIST_CM], color="#2F64C8", lw=2, ls=(0, (6, 4)), zorder=1)

    for label, colour, marker, name in CLASSES:
        xs = [p[0] for p in pts[label]]
        ys = [p[1] for p in pts[label]]
        ax.scatter(xs, ys, s=95, marker=marker, color=colour, edgecolors=SURFACE, linewidths=1.6, zorder=3,
                   label=f"{name} ({len(xs)})")

    ax.set_xscale("log")
    ax.set_xlim(0.38, 8.5)
    ax.set_ylim(0.8, 9.2)
    ax.set_xticks([0.5, 1, 2, 4, 8])
    ax.set_xticklabels(["0.5", "1", "2", "4", "8"])
    ax.set_xlabel("blue ÷ red light", color=INK)
    ax.set_ylabel("distance to piece (cm)", color=INK)
    ax.tick_params(colors=INK2, length=0)
    ax.grid(True, which="major", color=GRID, lw=1)
    for s in ax.spines.values():
        s.set_visible(False)

    # direct labels on the decision regions + thresholds (text in ink, colour carried by the marks)
    ax.text(0.52, 6.15, "called RED", color=INK, fontsize=16, fontweight="bold", ha="center")
    ax.text(5.0, 6.15, "called BLUE", color=INK, fontsize=16, fontweight="bold", ha="center")
    ax.text(1.32, 8.55, "nothing in the intake", color=INK, fontsize=16, fontweight="bold", ha="center")
    ax.text(8.3, MAX_DIST_CM + 0.18, "6.7 cm", color=INK2, fontsize=14, ha="right", va="bottom")
    ax.text(x_red * 0.97, 4.6, "red ÷ blue > 0.93", color=INK2, fontsize=13, rotation=90, ha="right", va="center")
    ax.text(x_blue * 1.03, 4.6, "blue ÷ red > 1.62", color=INK2, fontsize=13, rotation=90, ha="left", va="center")

    leg = ax.legend(loc="upper right", bbox_to_anchor=(1.0, 0.97), frameon=False, fontsize=14, handletextpad=0.3,
                    labelcolor=INK)
    for h in leg.legend_handles:
        h.set_sizes([80])
    ax.set_title(f"{len(rows)} recorded readings · {wrong} misclassified", color=INK, fontsize=18,
                 fontweight="bold", loc="left", pad=12)

    fig.tight_layout()
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    fig.savefig(OUT, facecolor=SURFACE)
    print(f"{OUT}  ({len(rows)} rows, {wrong} misclassified)")


if __name__ == "__main__":
    main()
