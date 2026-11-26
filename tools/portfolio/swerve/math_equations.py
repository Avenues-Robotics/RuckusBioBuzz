"""Swerve Drive page, THE MATH panel: the four mixer equations as LaTeX-style PNGs (tool).

One font size and one DPI for all four, tight-cropped, so the slide can place them at
width_px / PX_PER_PT points with no stretching. Matches the code:
  ADD   - pod vector = drive + spin x (perpendicular to the pod offset)   Swerve.java mixer
  ANGLE - atan2 of the pod vector                                          Swerve.thetaOf
  SPEED - divide every pod by max(1, largest pod magnitude)                Swerve.java "maxMagnitude"
  FLIP  - |error| > 90 deg -> aim +180 and reverse the wheel, +-10 deg band CoaxialPod.decideFlip
Run: python3 tools/portfolio/swerve/math_equations.py
"""
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUT = os.path.join(os.path.dirname(__file__), "..", "..", "..", "docs", "portfolio")
FONT_PT = 8.0
DPI = 800                      # PX_PER_PT = DPI / 72
INK = "#1F2A12"
plt.rcParams["mathtext.fontset"] = "cm"

EQS = {
    "add":   r"$v_i = v + \omega\, t_i$",
    "angle": r"$\theta_i = \tan^{-1}\!\left(\dfrac{y_i}{x_i}\right)$",
    "speed": r"$s_i = \dfrac{|v_i|}{\mathrm{max}\,(1,\ \mathrm{max}_j\,|v_j|)}$",
    "flip":  r"$|\theta_i - \varphi_i| > 90^{\circ} \ \Rightarrow\ \mathrm{flip}$",
}

for name, tex in EQS.items():
    fig = plt.figure(figsize=(4, 1))
    fig.text(0, 0.5, tex, fontsize=FONT_PT, color=INK, va="center")
    path = os.path.normpath(os.path.join(OUT, f"swerve_math_{name}.png"))
    fig.savefig(path, dpi=DPI, transparent=True, bbox_inches="tight", pad_inches=0.01)
    plt.close(fig)
    from PIL import Image
    w, h = Image.open(path).size
    print(f"{name:6s} {w}x{h}px = {w*72/DPI:.2f} x {h*72/DPI:.2f} pt  {path}")
