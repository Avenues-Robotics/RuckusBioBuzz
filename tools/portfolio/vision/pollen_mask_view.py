"""Autonomous page, Blob Detection card: "camera" vs "what OpenCV sees" (tool).

Runs the SHIPPED PollenDetectionPipeline steps (pipelines/PollenDetectionPipeline.java, Config
defaults - PollenCamera only overrides rotateDeg) on one saved camera frame and renders two stacked
panels of the same zoomed region: the camera frame, and the cleaned yellow mask the code groups into
clumps. The robot does not detect edges: it is an HSV colour threshold -> open -> close -> connected
components -> area / aspect filter.

Source frame: tools/swervetune/runs/auto_blue9/frames/027.50.jpg (the frame already on the slide).
Saved frames are the pipeline's annotated output at 320 x 427, i.e. the 480 x 640 rotated capture at
2/3 scale, so every pixel-sized tunable is scaled by 2/3 (areas by 4/9). The zoom is the "~5 balls"
clump; its overlay is green, which is outside the yellow HSV ranges, so the mask is computed straight
from the saved pixels with no retouching. (The "~1 ball" clump's overlay is amber - inside the yellow
range - so it is left out of the zoom rather than edited out.)

Run: python3 tools/portfolio/vision/pollen_mask_view.py -> docs/portfolio/pollen_mask_view.png
"""
import os
import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, "..", "..", ".."))
SRC = os.path.join(ROOT, "tools", "swervetune", "runs", "auto_blue9", "frames", "027.50.jpg")
FONT = os.path.join(ROOT, "tools", "pedro-visualizer", "public", "fonts", "Poppins-SemiBold.ttf")
OUT = os.path.join(ROOT, "docs", "portfolio", "pollen_mask_view.png")

# PollenDetectionPipeline.Config defaults (640 x 480 capture).
HSV_A = ((15, 90, 70), (38, 255, 255))
HSV_B = ((12, 60, 45), (42, 255, 255))
OPEN_RADIUS, CLOSE_H_GAP, CLOSE_V_RADIUS = 3, 16, 12
MIN_AREA, MAX_AREA, MAX_ASPECT = 350.0, 100_000.0, 5.0
K = 2.0 / 3.0  # saved frame scale

# Layout, in points: the slide's image column is 94.8 pt wide.
W_PT, PANEL_H_PT, GAP_PT = 94.8, 65.5, 5.0
CUT_PT = 11.0            # diagonal corner cut, same motif as the slide's other photos
TAG_PT, TAG_PAD_PT, TAG_INSET_PT = 5.2, 2.2, 3.5
DPI = 600
TAG_BG, TAG_FG, CREAM = (62, 92, 18), (255, 255, 255), (251, 248, 230)

# Zoom region in saved-frame pixels: the "~5 balls" clump, right of the amber "~1 ball" box.
ZOOM_X0, ZOOM_X1, ZOOM_YC = 178, 320, 282


def pipeline_mask(rgb):
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, *HSV_A) | cv2.inRange(hsv, *HSV_B)
    r = max(0, round(OPEN_RADIUS * K))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1)))
    gh, gv = round(CLOSE_H_GAP * K), round(CLOSE_V_RADIUS * K)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                            cv2.getStructuringElement(cv2.MORPH_RECT, (2 * gh + 1, 2 * gv + 1)))
    n, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8, cv2.CV_32S)
    kept = np.zeros_like(mask)
    for i in range(1, n):
        x, y, w, h, area = stats[i]
        if not (MIN_AREA * K * K <= area <= MAX_AREA * K * K):
            continue
        if max(w, h) / max(1, min(w, h)) > MAX_ASPECT:
            continue
        kept[labels == i] = 255
    return kept


def px(pt):
    return int(round(pt * DPI / 72.0))


def panel(img_rgb, w, h, smooth_mask=False):
    """Resize the zoom crop to w x h px. A mask is resampled smoothly and re-thresholded."""
    if smooth_mask:
        big = cv2.resize(img_rgb, (w, h), interpolation=cv2.INTER_CUBIC)
        big = np.where(big > 127, 255, 0).astype(np.uint8)
        return Image.fromarray(cv2.cvtColor(big, cv2.COLOR_GRAY2RGB))
    return Image.fromarray(cv2.resize(img_rgb, (w, h), interpolation=cv2.INTER_CUBIC))


def tag(draw, font, text, x, y):
    pad, l, t, r, b = px(TAG_PAD_PT), *draw.textbbox((0, 0), text, font=font)
    draw.rectangle((x, y, x + (r - l) + 2 * pad, y + (b - t) + 2 * pad), fill=TAG_BG)
    draw.text((x + pad - l, y + pad - t), text, font=font, fill=TAG_FG)


def main():
    rgb = cv2.cvtColor(cv2.imread(SRC), cv2.COLOR_BGR2RGB)
    mask = pipeline_mask(rgb)

    W, PH, G = px(W_PT), px(PANEL_H_PT), px(GAP_PT)
    zw = ZOOM_X1 - ZOOM_X0
    zh = round(zw * PANEL_H_PT / W_PT)
    y0 = ZOOM_YC - zh // 2
    cam = panel(rgb[y0:y0 + zh, ZOOM_X0:ZOOM_X1], W, PH)
    seen = panel(mask[y0:y0 + zh, ZOOM_X0:ZOOM_X1], W, PH, smooth_mask=True)

    out = Image.new("RGBA", (W, 2 * PH + G), CREAM + (0,))
    out.paste(cam, (0, 0))
    out.paste(seen, (0, PH + G))

    d = ImageDraw.Draw(out)
    c = px(CUT_PT)
    clear = CREAM + (0,)
    d.polygon([(W - c, 0), (W, 0), (W, c)], fill=clear)                       # top-right cut
    d.polygon([(0, 2 * PH + G - c), (0, 2 * PH + G), (c, 2 * PH + G)], fill=clear)  # bottom-left cut

    font = ImageFont.truetype(FONT, px(TAG_PT))
    ins = px(TAG_INSET_PT)
    tag(d, font, "CAMERA", ins, ins)
    tag(d, font, "WHAT OPENCV SEES", ins, PH + G + ins)

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    out.save(OUT, dpi=(DPI, DPI))
    print(OUT, out.size, "zoom", (ZOOM_X0, y0, ZOOM_X1, y0 + zh), "mask px in zoom",
          int((mask[y0:y0 + zh, ZOOM_X0:ZOOM_X1] > 0).sum()))


if __name__ == "__main__":
    main()
