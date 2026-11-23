package org.firstinspires.ftc.teamcode.pipelines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import android.graphics.Canvas;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/**
 * PollenDetectionPipeline - finds clumps of BIOBUZZ Pollen (yellow plastic balls, ~2.8 in) in a webcam frame.
 *
 * <p>Per frame: optional rotation (Config.rotateDeg, for a camera mounted on its side) -> HSV threshold, two
 * overlapping yellow ranges OR-ed -> morphological open (specks out) -> morphological close (gaps between touching
 * balls filled) -> connected components with stats -> area / aspect filter -> components whose boxes are within
 * clumpMergeGap merged into Clumps -> sorted by estimated ball count. Boxes are drawn onto the frame.
 *
 * <p>2026-11-23: steps 2-5 used to be a hand-rolled run-length (RLE) open / close / labelling. Its close merged
 * every run within closeVRadius (12) rows INTO the previous row's run, so a ball kept one run every 13 rows; the
 * labelling joins only runs on adjacent rows, so every ball fell apart into single-row strips of one ball-width of
 * area (60-170 px at 640 x 480), all under minArea 350, and nothing was ever detected (first camera test, balls in
 * plain view, 0 clumps). Replaced by OpenCV's native morphologyEx + connectedComponentsWithStats; the tunables
 * mean the same thing. Cost on the Control Hub is not measured.
 *
 * <p>Usage: {@code pipeline = new PollenDetectionPipeline(null, config)}, add it to a VisionPortal
 * (pipelines/PollenCamera), read {@link #getBestClump()} / {@link #getSteeringError()} from the OpMode loop.
 * Calibrate singleBallAreaPx: one ball at intake distance, read getBestClump().areaPx.
 */
public class PollenDetectionPipeline implements VisionProcessor {

    // ─────────────────────────────────────────────────────────────────────────
    // Tuning
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Every tunable, as one object. The defaults are the values this pipeline
     * has always shipped with, so {@code new PollenDetectionPipeline(telemetry)}
     * behaves exactly as it did before Config existed. Autos exported from the
     * Pedro visualizer build one from its Settings → Vision tab.
     */
    public static class Config {
        // HSV color space (OpenCV): H ∈ [0,179]  S ∈ [0,255]  V ∈ [0,255]
        // BIOBUZZ pollen is daffodil yellow. Two overlapping ranges handle the
        // shift in apparent hue between warm fluorescent (common in gymnasiums)
        // and cooler LED rigs used at championship venues.
        public double[] hsvLowA  = {15,  90,  70};
        public double[] hsvHighA = {38, 255, 255};
        public double[] hsvLowB  = {12,  60,  45}; // wider net for dim venues
        public double[] hsvHighB = {42, 255, 255};

        // Morphological open (noise removal), kernel radius in px: specks smaller than ~2r+1 vanish.
        // A single pollen ball at 3ft spans ~22px wide; noise is typically 1–4px.
        public int openRadius = 3;

        // Morphological close (gap filling), kernel half-width in px: fills horizontal gaps between touching balls.
        // Two adjacent balls at 3ft may have a 5–15px dark gap between them.
        public int closeHGap = 16;

        // Close kernel half-height in px.
        public int closeVRadius = 12;

        // Component area filter — pixels²
        public double minArea = 350.0;
        public double maxArea = 100_000.0;

        // Bounding-box aspect ratio filter (width / height).
        // One ball ≈ 1:1; three in a row ≈ 3:1. Cap at 5 to reject thin streaks.
        public double maxAspect = 5.0;

        // Spatial distance within which two blobs are merged into one Clump.
        public int clumpMergeGap = 28;

        // Estimated pixel area of one pollen ball at typical intake distance.
        // ⚠ CALIBRATE THIS: place one ball at your intake, call getBestClump()
        //   .areaPx, then paste that number here.
        public double singleBallAreaPx = 1_600.0;

        // Camera mounted rotated: the frame is turned this many degrees CLOCKWISE (0, 90, 180, 270) before
        // anything else, so detection, steering error and the annotated feed are all upright.
        public int rotateDeg = 0;
    }

    private final Scalar HSV_LO_A;
    private final Scalar HSV_HI_A;
    private final Scalar HSV_LO_B;
    private final Scalar HSV_HI_B;
    private final int    OPEN_RADIUS;
    private final int    CLOSE_H_GAP;
    private final int    CLOSE_V_RADIUS;
    private final double MIN_AREA;
    private final double MAX_AREA;
    private final double MAX_ASPECT;
    private final int    CLUMP_MERGE_GAP;
    private final double SINGLE_BALL_AREA_PX;
    private final int    ROTATE_CODE;   // Core.ROTATE_*, or -1 for none

    // OpenCV Mats - allocated once, reused each frame
    private final Mat hsvMat    = new Mat();
    private final Mat maskA     = new Mat();
    private final Mat maskB     = new Mat();
    private final Mat mask      = new Mat();
    private final Mat rotated   = new Mat();
    private final Mat labels    = new Mat();
    private final Mat stats     = new Mat();
    private final Mat centroids = new Mat();
    private final Mat openKernel;
    private final Mat closeKernel;
    private int[]    statBuf = new int[0];
    private double[] centBuf = new double[0];
    private Mat display;

    // Shared state - written by vision thread, read by robot thread
    private final Object     lock     = new Object();
    private final List<Clump> results  = new ArrayList<>();
    private Clump   best          = null;
    private double  steeringError = 0.0;
    private int     fw = 640, fh = 480;  // frame dimensions after rotation

    private final Telemetry telemetry;

    // ─────────────────────────────────────────────────────────────────────────
    // Public result type
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * A detected pollen clump. All coordinates are in full-frame pixels.
     *
     * estimatedBallCount  — rough count derived from total blob area.
     *                       Calibrate SINGLE_BALL_AREA_PX for accuracy.
     *
     * steeringError       — normalized horizontal offset in [-1, +1].
     *                       Negative = clump is left of frame center.
     *                       Positive = right. Feed directly into a P-loop:
     *                         power = KP * pipeline.getSteeringError()
     */
    public static class Clump {
        public final double centerX, centerY;
        public final double areaPx;
        public final int    boundX, boundY, boundW, boundH;
        public final int    estimatedBallCount;
        public final double steeringError;

        Clump(double cx, double cy, double area,
              int bx, int by, int bw, int bh, int frameWidth, double singleBallAreaPx) {
            centerX            = cx;
            centerY            = cy;
            areaPx             = area;
            boundX = bx; boundY = by; boundW = bw; boundH = bh;
            estimatedBallCount = Math.max(1, (int) Math.round(area / singleBallAreaPx));
            steeringError      = (cx - frameWidth / 2.0) / (frameWidth / 2.0);
        }

        @Override
        public String toString() {
            return String.format("Clump{~%d balls  cx=%.0f  cy=%.0f  err=%.3f}",
                    estimatedBallCount, centerX, centerY, steeringError);
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Constructor
    // ─────────────────────────────────────────────────────────────────────────

    public PollenDetectionPipeline(Telemetry telemetry) {
        this(telemetry, new Config());
    }

    public PollenDetectionPipeline(Telemetry telemetry, Config config) {
        this.telemetry = telemetry;
        Config c = config != null ? config : new Config();
        HSV_LO_A            = new Scalar(c.hsvLowA);
        HSV_HI_A            = new Scalar(c.hsvHighA);
        HSV_LO_B            = new Scalar(c.hsvLowB);
        HSV_HI_B            = new Scalar(c.hsvHighB);
        OPEN_RADIUS         = c.openRadius;
        CLOSE_H_GAP         = c.closeHGap;
        CLOSE_V_RADIUS      = c.closeVRadius;
        MIN_AREA            = c.minArea;
        MAX_AREA            = c.maxArea;
        MAX_ASPECT          = c.maxAspect;
        CLUMP_MERGE_GAP     = c.clumpMergeGap;
        SINGLE_BALL_AREA_PX = c.singleBallAreaPx;
        int rot = ((c.rotateDeg % 360) + 360) % 360;
        ROTATE_CODE = rot == 90 ? Core.ROTATE_90_CLOCKWISE : rot == 180 ? Core.ROTATE_180
                : rot == 270 ? Core.ROTATE_90_COUNTERCLOCKWISE : -1;
        openKernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE,
                new Size(2 * Math.max(0, OPEN_RADIUS) + 1, 2 * Math.max(0, OPEN_RADIUS) + 1));
        closeKernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT,
                new Size(2 * Math.max(0, CLOSE_H_GAP) + 1, 2 * Math.max(0, CLOSE_V_RADIUS) + 1));
    }

    // ─────────────────────────────────────────────────────────────────────────
    // VisionProcessor
    // ─────────────────────────────────────────────────────────────────────────

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        boolean quarter = ROTATE_CODE == Core.ROTATE_90_CLOCKWISE || ROTATE_CODE == Core.ROTATE_90_COUNTERCLOCKWISE;
        synchronized (lock) { fw = quarter ? height : width; fh = quarter ? width : height; }
    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        Mat frame = input;
        if (ROTATE_CODE >= 0) {
            Core.rotate(input, rotated, ROTATE_CODE);
            frame = rotated;
        }
        int W = frame.cols(), H = frame.rows();

        // 1. HSV threshold: two merged yellow ranges (warm and cool venue lighting).
        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsvMat, HSV_LO_A, HSV_HI_A, maskA);
        Core.inRange(hsvMat, HSV_LO_B, HSV_HI_B, maskB);
        Core.bitwise_or(maskA, maskB, mask);

        // 2. Open (specks out), close (gaps between touching balls filled).
        if (OPEN_RADIUS > 0) Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, openKernel);
        if (CLOSE_H_GAP > 0 || CLOSE_V_RADIUS > 0) Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, closeKernel);

        // 3. Connected components with stats (label 0 is the background).
        int n = Imgproc.connectedComponentsWithStats(mask, labels, stats, centroids, 8, CvType.CV_32S);

        // 4. Filter, merge into Clumps, sort.
        List<Clump> clumps = buildClumps(n, W);
        Collections.sort(clumps, CLUMP_ORDER);

        // 5. Annotate + publish.
        drawAnnotations(frame, clumps, W, H);
        display = frame;
        synchronized (lock) {
            fw = W;
            fh = H;
            results.clear();
            results.addAll(clumps);
            best          = clumps.isEmpty() ? null : clumps.get(0);
            steeringError = best != null ? best.steeringError : 0.0;
        }
        if (telemetry != null) {
            telemetry.addData("[Pollen] clumps", clumps.size());
            telemetry.addData("[Pollen] best",   best != null ? best : "none");
        }
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int ow, int oh,
                            float bmpScale, float canvasScale, Object ctx) { }

    // ─────────────────────────────────────────────────────────────────────────
    // Filter components and merge spatially close ones into Clumps
    // ─────────────────────────────────────────────────────────────────────────

    private List<Clump> buildClumps(int numLabels, int W) {
        List<double[]> comps = new ArrayList<>();
        if (numLabels > 1) {
            if (statBuf.length < numLabels * 5) statBuf = new int[numLabels * 5];
            if (centBuf.length < numLabels * 2) centBuf = new double[numLabels * 2];
            stats.get(0, 0, statBuf);        // rows of LEFT, TOP, WIDTH, HEIGHT, AREA
            centroids.get(0, 0, centBuf);    // rows of cx, cy
            for (int i = 1; i < numLabels; i++) {
                int x = statBuf[i * 5 + Imgproc.CC_STAT_LEFT];
                int y = statBuf[i * 5 + Imgproc.CC_STAT_TOP];
                int bw = statBuf[i * 5 + Imgproc.CC_STAT_WIDTH];
                int bh = statBuf[i * 5 + Imgproc.CC_STAT_HEIGHT];
                double area = statBuf[i * 5 + Imgproc.CC_STAT_AREA];
                if (area < MIN_AREA || area > MAX_AREA || bh == 0) continue;
                double asp = (double) bw / bh;
                if (asp > MAX_ASPECT || asp < 1.0 / MAX_ASPECT) continue;
                // [cx, cy, area, x1, y1, x2, y2]
                comps.add(new double[]{centBuf[i * 2], centBuf[i * 2 + 1], area, x, y, x + bw - 1, y + bh - 1});
            }
        }

        // Spatial merge: union components whose bounding boxes are within CLUMP_MERGE_GAP
        int n = comps.size();
        int[] p = new int[n];
        for (int i = 0; i < n; i++) p[i] = i;
        for (int i = 0; i < n; i++)
            for (int j = i + 1; j < n; j++)
                if (boxesClose(comps.get(i), comps.get(j))) smallUnion(p, i, j);

        // Accumulate per-clump stats
        double[] ax = new double[n], ay = new double[n], ar = new double[n];
        double[] x1 = new double[n], y1 = new double[n];
        double[] x2 = new double[n], y2 = new double[n];
        for (int i = 0; i < n; i++) { x1[i] = y1[i] = Double.MAX_VALUE; x2[i] = y2[i] = -Double.MAX_VALUE; }
        for (int i = 0; i < n; i++) {
            int r = smallFind(p, i);
            double[] c = comps.get(i);
            ar[r] += c[2];
            ax[r] += c[0] * c[2];
            ay[r] += c[1] * c[2];
            if (c[3] < x1[r]) x1[r] = c[3];
            if (c[4] < y1[r]) y1[r] = c[4];
            if (c[5] > x2[r]) x2[r] = c[5];
            if (c[6] > y2[r]) y2[r] = c[6];
        }

        List<Clump> out = new ArrayList<>();
        for (int i = 0; i < n; i++) {
            if (smallFind(p, i) != i || ar[i] < MIN_AREA) continue;
            double cx = ax[i] / ar[i];
            double cy = ay[i] / ar[i];
            int bx = (int) x1[i], by = (int) y1[i];
            int bw = (int)(x2[i] - x1[i]) + 1;
            int bh = (int)(y2[i] - y1[i]) + 1;
            out.add(new Clump(cx, cy, ar[i], bx, by, bw, bh, W, SINGLE_BALL_AREA_PX));
        }
        return out;
    }

    private boolean boxesClose(double[] a, double[] b) {
        int g = CLUMP_MERGE_GAP;
        return a[3]-g < b[5] && a[5]+g > b[3] && a[4]-g < b[6] && a[6]+g > b[4];
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Annotation
    // ─────────────────────────────────────────────────────────────────────────

    private static final Scalar GREEN  = new Scalar(  0, 255,  80);
    private static final Scalar AMBER  = new Scalar(255, 190,   0);
    private static final Scalar WHITE  = new Scalar(255, 255, 255);
    private static final Scalar BLUE   = new Scalar( 80, 160, 255);
    private static final Scalar DARK   = new Scalar( 25,  25,  25);

    private void drawAnnotations(Mat frame, List<Clump> clumps, int W, int H) {
        // Vertical center reference
        Imgproc.line(frame, new Point(W/2, 0), new Point(W/2, H), WHITE, 1, Imgproc.LINE_AA, 0);

        for (int i = 0; i < clumps.size(); i++) {
            Clump  c   = clumps.get(i);
            Scalar col = (i == 0) ? GREEN : AMBER;
            int    th  = (i == 0) ? 3 : 1;

            // Bounding box
            Imgproc.rectangle(frame,
                    new Point(c.boundX, c.boundY),
                    new Point(c.boundX + c.boundW, c.boundY + c.boundH),
                    col, th, Imgproc.LINE_AA, 0);

            // Centroid dot
            Imgproc.circle(frame, new Point(c.centerX, c.centerY),
                    6, col, -1, Imgproc.LINE_AA, 0);

            // Label (with dark background for readability on any field colour)
            String text = "~" + c.estimatedBallCount
                    + (c.estimatedBallCount == 1 ? " ball" : " balls");
            int[] bl = {0};
            Size  ts = Imgproc.getTextSize(text, Imgproc.FONT_HERSHEY_SIMPLEX, 0.52, 2, bl);
            Point tl = new Point(c.boundX + 4, c.boundY - 7);
            Imgproc.rectangle(frame,
                    new Point(tl.x - 2, tl.y - ts.height - 2),
                    new Point(tl.x + ts.width + 2, tl.y + 2),
                    DARK, -1);
            Imgproc.putText(frame, text, tl,
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.52, col, 2, Imgproc.LINE_AA, false);

            // Steering arrow for the primary target only
            if (i == 0) {
                int ay = H - 20;
                Imgproc.arrowedLine(frame,
                        new Point(W/2, ay), new Point(c.centerX, ay),
                        BLUE, 2, Imgproc.LINE_AA, 0, 0.15);
                Imgproc.putText(frame,
                        String.format("err=%.3f", c.steeringError),
                        new Point(8, H - 6),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.4, BLUE, 1, Imgproc.LINE_AA, false);
            }
        }
    }

    // Simple union-find for the small clump-merge step (no rank needed)
    private int smallFind(int[] p, int i) {
        while (p[i] != i) { p[i] = p[p[i]]; i = p[i]; } return i;
    }
    private void smallUnion(int[] p, int a, int b) {
        a = smallFind(p, a); b = smallFind(p, b); if (a != b) p[b] = a;
    }


    private static final Comparator<Clump> CLUMP_ORDER =
            new Comparator<Clump>() {
                @Override public int compare(Clump a, Clump b) {
                    return Integer.compare(b.estimatedBallCount, a.estimatedBallCount);
                }
            };

    // ─────────────────────────────────────────────────────────────────────────
    // Public accessors — thread-safe, callable from the robot loop
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * The highest-priority (most balls) clump detected this frame.
     * Returns null when no pollen is visible.
     */
    public Clump getBestClump() {
        synchronized (lock) { return best; }
    }

    /**
     * Signed horizontal offset toward the best clump, normalized to [-1, +1].
     * Negative = clump is left of frame center; positive = right.
     * Returns 0.0 when no clump is visible.
     *
     * Ready to feed into a P-controller:
     *   double power = KP * pipeline.getSteeringError();
     */
    public double getSteeringError() {
        synchronized (lock) { return steeringError; }
    }

    /** Frame width the pipeline was initialised with, pixels. */
    public int getFrameWidth() {
        synchronized (lock) { return fw; }
    }

    /** Frame height the pipeline was initialised with, pixels. */
    public int getFrameHeight() {
        synchronized (lock) { return fh; }
    }

    /** True if at least one pollen clump is visible this frame. */
    public boolean isPollenVisible() {
        synchronized (lock) { return best != null; }
    }

    /** Snapshot of all clumps this frame, sorted by estimated ball count. */
    public List<Clump> getAllClumps() {
        synchronized (lock) { return new ArrayList<>(results); }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Lifecycle
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Free OpenCV Mats. Call in your OpMode's stop() or a try-finally block.
     *
     *   try { ... } finally { pipeline.release(); visionPortal.close(); }
     */
    public void release() {
        hsvMat.release();
        maskA.release();
        maskB.release();
        mask.release();
        rotated.release();
        labels.release();
        stats.release();
        centroids.release();
        openKernel.release();
        closeKernel.release();
    }

    /**
     * The frame this pipeline last processed, upright (rotateDeg applied) and annotated. Vision thread only:
     * for a processor added after this one in the same VisionPortal (fieldview.CameraFeed). Null before the first frame.
     */
    public Mat displayFrame() {
        return display;
    }
}
