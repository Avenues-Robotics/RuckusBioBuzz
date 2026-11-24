"""Precompute 2D geometry (mm, side view: Y = front(-)/back(+), Z = up) from the robot STL for the transfer drawing.

Saves geom2d.npz: sections at x = 0 and x = -45 mm (line segments) and a silhouette mask of the x <= 0 half.
"""
import numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection

B = np.load("tris_blender.npy") * 1000.0          # (n,3,3) blender coords in mm: X side, Y front/back, Z up

def section(xc):
    m = (B[:, :, 0].min(1) <= xc) & (B[:, :, 0].max(1) >= xc)
    T = B[m]
    d = T[:, :, 0] - xc
    segs = []
    for t, dd in zip(T, d):
        pts = []
        for i in range(3):
            j = (i + 1) % 3
            if dd[i] * dd[j] < 0:
                s = dd[i] / (dd[i] - dd[j]); p = t[i] + s * (t[j] - t[i]); pts.append((p[1], p[2]))
        if len(pts) == 2:
            segs.append(pts)
    return np.array(segs)

S0, S45 = section(0.0), section(-45.0)

# silhouette of the half the drawing keeps (x <= 0): rasterise projected triangles at 0.5 mm
half = B[B[:, :, 0].max(1) <= 0.5]
Y0, Y1, Z0, Z1, RES = -270.0, 250.0, -10.0, 370.0, 0.5
W, H = int((Y1 - Y0) / RES), int((Z1 - Z0) / RES)
fig = plt.figure(figsize=(W / 100, H / 100), dpi=100)
ax = fig.add_axes([0, 0, 1, 1]); ax.set_xlim(Y0, Y1); ax.set_ylim(Z0, Z1); ax.axis("off")
ax.add_collection(PolyCollection(half[:, :, 1:3], facecolors="k", edgecolors="k", linewidths=0.3, antialiased=False))
fig.canvas.draw()
img = np.asarray(fig.canvas.buffer_rgba())[:, :, 0]
mask = (img < 128)[::-1]                    # row 0 = Z0
plt.close(fig)
np.savez_compressed("geom2d.npz", S0=S0, S45=S45, mask=mask, extent=np.array([Y0, Y1, Z0, Z1]))
print("sections", len(S0), len(S45), "mask", mask.shape, mask.mean())
