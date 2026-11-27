"""Autonomous page, Blob Detection card: "camera" vs "what OpenCV sees" (tool).

Shows the robot's real camera frame next to the RAW output of the shipped PollenDetectionPipeline's
first step (pipelines/PollenDetectionPipeline.java, Config defaults - PollenCamera only overrides
rotateDeg): the two HSV yellow ranges OR-ed, before any cleanup. Ragged edges and stray specks are
left in on purpose - that is what the code really gets. The robot does not detect edges: it is an HSV
colour threshold -> open -> close -> connected components -> area / aspect filter.

Source frame: tools/swervetune/runs/auto_blue12/frames/025.29.jpg - a frame where the robot itself
labelled the clump "~3 balls" (the label is the robot's own drawing, unedited). Saved frames are the
pipeline's annotated output at 320 x 427 (the 480 x 640 rotated capture at 2/3 scale), so pixel
tunables are scaled by 2/3 (areas by 4/9).

The only retouch: pixels the robot DREW over the camera image inside the zoom (its green centre dot
and the white aim line) are not camera data and would punch false holes in the mask, so those
pixels alone take the value of the pipeline's own close step; every other mask pixel is the raw
threshold. The zoom stops left of the two amber "~1 ball" detections, whose amber labels fall inside
the yellow range and would show up as text in the mask.

Run: python3 tools/portfolio/vision/pollen_mask_view.py -> docs/portfolio/pollen_mask_view.png
"""
import os
import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, "..", "..", ".."))
SRC = os.path.join(ROOT, "tools", "swervetune", "runs", "auto_blue12", "frames", "025.29.jpg")
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

# Zoom region in saved-frame pixels: the "~3 balls" clump, right of the amber "~1 ball" boxes.
ZOOM_X0, ZOOM_X1, ZOOM_YC = 95, 255, 283


def raw_mask(rgb):
    """Pipeline step 1: the two HSV yellow ranges, OR-ed. No cleanup."""
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    return cv2.inRange(hsv, *HSV_A) | cv2.inRange(hsv, *HSV_B)


def close(mask):
    gh, gv = round(CLOSE_H_GAP * K), round(CLOSE_V_RADIUS * K)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                            cv2.getStructuringElement(cv2.MORPH_RECT, (2 * gh + 1, 2 * gv + 1)))


def drawn_overlay(rgb):
    """Pixels the robot drew INSIDE a clump: the filled green centre dot and the white aim line.
    The thin green box outline is left alone - it sits on the clump's bounding box, and filling it
    would square off the blob's real edges."""
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV).astype(int)
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    green = (h >= 55) & (h <= 85) & (s >= 150) & (v >= 150)
    white = (s < 40) & (v >= 235)
    cx = rgb.shape[1] // 2
    white[:, :cx - 2] = False
    white[:, cx + 3:] = False
    dot = cv2.morphologyEx(green.astype(np.uint8), cv2.MORPH_OPEN,
                           cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))) > 0   # lines vanish
    over = (dot | white).astype(np.uint8)
    return cv2.dilate(over, np.ones((3, 3), np.uint8)) > 0


def seen_mask(rgb):
    """Raw threshold, with only the robot-drawn pixels filled from the pipeline's close step."""
    raw = raw_mask(rgb)
    over = drawn_overlay(rgb)
    out = raw.copy()
    out[over] = close(raw)[over]
    return out


def pipeline_mask(rgb):
    """Whole pipeline mask (kept components). Not drawn; used to pick frames."""
    mask = raw_mask(rgb)
    r = max(0, round(OPEN_RADIUS * K))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1)))
    mask = close(mask)
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


def panel(img_rgb, w, h, mask=False):
    """Resize the zoom crop to w x h px. A mask is scaled nearest-neighbour: real pixels, no smoothing."""
    if mask:
        big = cv2.resize(img_rgb, (w, h), interpolation=cv2.INTER_NEAREST)
        return Image.fromarray(cv2.cvtColor(big, cv2.COLOR_GRAY2RGB))
    return Image.fromarray(cv2.resize(img_rgb, (w, h), interpolation=cv2.INTER_CUBIC))


def tag(draw, font, text, x, y):
    pad, l, t, r, b = px(TAG_PAD_PT), *draw.textbbox((0, 0), text, font=font)
    draw.rectangle((x, y, x + (r - l) + 2 * pad, y + (b - t) + 2 * pad), fill=TAG_BG)
    draw.text((x + pad - l, y + pad - t), text, font=font, fill=TAG_FG)


def main():
    rgb = cv2.cvtColor(cv2.imread(SRC), cv2.COLOR_BGR2RGB)
    mask = seen_mask(rgb)

    W, PH, G = px(W_PT), px(PANEL_H_PT), px(GAP_PT)
    zw = ZOOM_X1 - ZOOM_X0
    zh = round(zw * PANEL_H_PT / W_PT)
    y0 = ZOOM_YC - zh // 2
    cam = panel(rgb[y0:y0 + zh, ZOOM_X0:ZOOM_X1], W, PH)
    seen = panel(mask[y0:y0 + zh, ZOOM_X0:ZOOM_X1], W, PH, mask=True)

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
    print(OUT, out.size, "zoom", (ZOOM_X0, y0, ZOOM_X1, y0 + zh), "raw mask px in zoom",
          int((mask[y0:y0 + zh, ZOOM_X0:ZOOM_X1] > 0).sum()))


if __name__ == "__main__":
    main()
