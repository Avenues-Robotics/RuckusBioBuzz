"""Swerve Drive page, THE MATH panel as ONE image: header, labels, equations, captions (tool).

Why one image: the slide editor and the Slides API thumbnail lay out text boxes with
different line metrics, so captions placed as text boxes next to equation images drifted
in the editor (2026-11-25). Rendering the block whole fixes the spacing for every viewer.

Text is Poppins (repo copy from tools/pedro-visualizer/public/fonts), math is Computer
Modern. Labels sit on each equation's baseline; every vertical gap is a constant below.
Equations match the code (see math_equations.py for the sources).
Run: python3 tools/portfolio/swerve/math_panel.py  -> docs/portfolio/swerve_math_panel.png
"""
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patheffects as pe
from matplotlib.font_manager import FontProperties

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, "..", "..", ".."))
FONTS = os.path.join(ROOT, "tools", "pedro-visualizer", "public", "fonts")
OUT = os.path.join(ROOT, "docs", "portfolio", "swerve_math_panel.png")

REG = FontProperties(fname=os.path.join(FONTS, "Poppins-Regular.ttf"))
BOLD = FontProperties(fname=os.path.join(FONTS, "Poppins-SemiBold.ttf"))
plt.rcParams["mathtext.fontset"] = "cm"

INK, GREEN = "#1F2A12", "#4E7516"
W_PT = 154.0            # = the slide's math box width
PAD_L, PAD_T, PAD_B = 7.0, 5.0, 4.0
COL2 = 34.5             # equations + captions start here (labels at PAD_L)
EQ_PT, CAP_PT, LAB_PT, HEAD_PT = 8.5, 6.5, 6.25, 6.75
STROKE = 0.2            # gives CM hairlines body-text weight at 8.5 pt
EQ_TO_CAP = 6.0         # equation bottom -> first caption baseline
CAP_LEAD = 8.0          # caption line pitch
ROW_GAP = 5.0           # last caption baseline -> next equation top
HEAD_TO_ROW = 5.0       # header baseline -> first equation top
DPI = 1200

ROWS = [
    ("ADD", r"$v_i = v + \omega\, t_i$",
     [r"v = drive, $\omega$ = spin, $t_i \perp$ pod offset"]),
    ("ANGLE", r"$\theta_i = \tan^{-1}\!\left(\dfrac{y_i}{x_i}\right)$",
     [r"wheel points along $v_i = (x_i,\, y_i)$"]),
    ("SPEED", r"$s_i = \dfrac{|v_i|}{\mathrm{max}\,(1,\ \mathrm{max}_j\,|v_j|)}$",
     ["caps every wheel at full power"]),
    ("FLIP", r"$|\theta_i - \varphi_i| > 90^{\circ} \ \Rightarrow\ \mathrm{flip}$",
     [r"$\varphi_i$ = current pod angle ($\pm 10^{\circ}$ band)",
      r"flip: aim $\theta_i + 180^{\circ}$, reverse wheel"]),
]


def eq_extent(fig, tex):
    """(ascent, descent) of a mathtext line about its baseline, in pt."""
    t = fig.text(0, 0.5, tex, fontsize=EQ_PT, va="baseline")
    r = fig.canvas.get_renderer()
    bb = t.get_window_extent(r)
    base = 0.5 * fig.bbox.height
    t.remove()
    k = 72.0 / fig.dpi
    return (bb.y1 - base) * k, (base - bb.y0) * k


def layout(fig):
    y = PAD_T + HEAD_PT * 0.75          # header baseline (cap height ~0.75 em)
    items = [("head", PAD_L, y, "THE MATH")]
    y += HEAD_TO_ROW
    for label, tex, caps in ROWS:
        asc, desc = eq_extent(fig, tex)
        base = y + asc
        items.append(("label", PAD_L, base, label))
        items.append(("eq", COL2, base, tex))
        cb = base + desc + EQ_TO_CAP
        for c in caps:
            items.append(("cap", COL2, cb, c))
            cb += CAP_LEAD
        y = cb - CAP_LEAD + ROW_GAP
    height = y - ROW_GAP + 1.8 + PAD_B   # last caption descender + bottom pad
    return items, height


probe = plt.figure(figsize=(W_PT / 72, 2), dpi=DPI)
items, H_PT = layout(probe)
plt.close(probe)

fig = plt.figure(figsize=(W_PT / 72, H_PT / 72), dpi=DPI)
X = lambda x: x / W_PT
Y = lambda y: 1 - y / H_PT
for kind, x, y, s in items:
    if kind == "head":
        fig.text(X(x), Y(y), s, fontproperties=BOLD, fontsize=HEAD_PT, color=GREEN, va="baseline")
    elif kind == "label":
        fig.text(X(x), Y(y), s, fontproperties=BOLD, fontsize=LAB_PT, color=GREEN, va="baseline")
    elif kind == "eq":
        t = fig.text(X(x), Y(y), s, fontsize=EQ_PT, color=INK, va="baseline")
        t.set_path_effects([pe.withStroke(linewidth=STROKE, foreground=INK)])
    else:
        fig.text(X(x), Y(y), s, fontproperties=REG, fontsize=CAP_PT, color=INK, va="baseline")
r = fig.canvas.get_renderer()
for t in fig.texts:
    x1 = t.get_window_extent(r).x1 * 72 / DPI
    if x1 > W_PT - 4:
        raise SystemExit(f"overflows the box ({x1:.1f} pt > {W_PT - 4} pt): {t.get_text()}")
fig.savefig(OUT, dpi=DPI, transparent=True)
plt.close(fig)
print(f"{OUT}  {W_PT:.1f} x {H_PT:.1f} pt")
