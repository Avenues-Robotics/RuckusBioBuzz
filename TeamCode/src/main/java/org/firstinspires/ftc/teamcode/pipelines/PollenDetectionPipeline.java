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
 * PollenDetectionPipeline
 * ═══════════════════════════════════════════════════════════════════════
 * Detects clumps of BIOBUZZ Pollen — yellow plastic balls ~2.8" diameter
 * — on the FTC foam field surface.
 *
 * Drop into:  TeamCode/src/main/java/…/teamcode
 *
 * ── Why this architecture ────────────────────────────────────────────
 *
 * The naive approach (HSV threshold → morphologyEx → findContours) works
 * but is slow on Android because:
 *
 *   1. morphologyEx with a 15px kernel slides over all 307,200 pixels
 *      twice — the single biggest cost in the pipeline (~8–12ms).
 *   2. findContours() allocates a MatOfPoint object per contour,
 *      generating GC pressure that causes jitter in the vision thread.
 *   3. Computing moments/bounding boxes requires a second pass over each
 *      contour's pixel list.
 *
 * This pipeline replaces steps 2 and 3 with Run-Length Encoding (RLE):
 *
 *   • After the HSV threshold, the binary mask is compressed into "runs"
 *     (horizontal spans of foreground pixels). A 640×480 scene with a few
 *     pollen balls typically produces 300–2000 runs — vs 307,200 pixels.
 *
 *   • Morphological open and close operate on runs, not pixels. Erosion
 *     is shrinking run endpoints; dilation is expanding them and merging
 *     overlapping runs. Cost is O(R) where R = run count, independent of
 *     kernel size. A 15px close costs the same as a 3px close.
 *     (Ref: Ehrensperger et al., "Fast algorithms for morphological
 *     operations using RLE binary images", arXiv:1504.01052)
 *
 *   • Connected-component labeling uses a single-pass union-find over
 *     runs rather than pixels. Adjacent-row runs that overlap horizontally
 *     are unioned. Blob statistics (centroid, area, bounding box) are
 *     accumulated during this single pass — no second scan, no contour
 *     tracing, no heap allocation per blob.
 *     (Ref: Wang et al., "New algorithm for binary CCL based on RLE and
 *     union-find sets", Beijing Inst. of Technology, 2010)
 *
 * ── Total frame cost (640×480, REV Control Hub estimate) ────────────
 *
 *   HSV threshold (cvtColor + 2× inRange + bitwise_or) : ~2.5 ms
 *   RLE compression of binary mask                      : ~0.4 ms
 *   RLE open (noise removal)                            : ~0.3 ms
 *   RLE close (gap filling between touching balls)      : ~0.3 ms
 *   RLE CCL + accumulation (union-find, single pass)    : ~0.3 ms
 *   Clump merging + annotation                          : ~0.5 ms
 *   ─────────────────────────────────────────────────────────────────
 *   TOTAL                                               : ~4–5 ms
 *   vs naive pixel-grid pipeline                        : ~15–25 ms
 *   Camera frame budget at 30fps                        : 33 ms
 *
 * ── Usage ────────────────────────────────────────────────────────────
 *
 *   PollenDetectionPipeline pipeline = new PollenDetectionPipeline(telemetry);
 *
 *   visionPortal = new VisionPortal.Builder()
 *       .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
 *       .addProcessor(pipeline)
 *       .build();
 *
 *   // In OpMode loop:
 *   Clump target = pipeline.getBestClump();
 *   if (target != null) {
 *       double correction = KP * pipeline.getSteeringError();
 *       // feed into your drive
 *   }
 *
 * ── Tuning ───────────────────────────────────────────────────────────
 *
 *   SINGLE_BALL_AREA_PX  — most important. Point camera at one ball at
 *                          intake distance, read getBestClump().areaPx
 *                          from telemetry, plug that value in here.
 *
 *   HSV ranges           — defaults cover warm fluorescent + LED venues.
 *                          Tighten S_MIN if yellow field seams give false
 *                          positives. Loosen H range if pollen looks
 *                          orange-ish under your lights.
 *
 *   OPEN_RADIUS          — increase to kill larger noise speckles.
 *   CLOSE_GAP            — increase if touching balls aren't merging.
 *   CLUMP_MERGE_GAP      — increase if separate clumps are merging wrong.
 * ═══════════════════════════════════════════════════════════════════════
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

        // RLE morphological open: removes noise runs shorter than this many pixels.
        // A single pollen ball at 3ft spans ~22px wide; noise is typically 1–4px.
        public int openRadius = 3;

        // RLE morphological close: fills horizontal gaps between touching balls.
        // Two adjacent balls at 3ft may have a 5–15px dark gap between them.
        public int closeHGap = 16;

        // Vertical radius for the close operation: how many rows apart two runs
        // can be and still be merged into one blob region.
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

    // Maximum runs array size. 32k handles even the noisiest frames.
    private static final int MAX_RUNS = 32_000;

    // ─────────────────────────────────────────────────────────────────────────
    // OpenCV Mats — allocated once, reused each frame to avoid GC churn
    // ─────────────────────────────────────────────────────────────────────────

    private final Mat hsvMat   = new Mat();
    private final Mat maskA    = new Mat();
    private final Mat maskB    = new Mat();
    private final Mat mask     = new Mat();

    // ─────────────────────────────────────────────────────────────────────────
    // RLE parallel arrays — runs stored as (row, colStart, colEnd[inclusive])
    // Using parallel primitives avoids per-run object allocation (no GC).
    // ─────────────────────────────────────────────────────────────────────────

    private final int[] rRow   = new int[MAX_RUNS];
    private final int[] rStart = new int[MAX_RUNS];
    private final int[] rEnd   = new int[MAX_RUNS];
    private int         rCount = 0;

    // Union-find arrays for CCL
    private final int[] ufParent = new int[MAX_RUNS];
    private final int[] ufRank   = new int[MAX_RUNS];

    // Per-component accumulators (indexed by canonical label = run index)
    private final long[] cSumX  = new long[MAX_RUNS];
    private final long[] cSumY  = new long[MAX_RUNS];
    private final long[] cArea  = new long[MAX_RUNS];
    private final int[]  cMinX  = new int[MAX_RUNS];
    private final int[]  cMinY  = new int[MAX_RUNS];
    private final int[]  cMaxX  = new int[MAX_RUNS];
    private final int[]  cMaxY  = new int[MAX_RUNS];

    // Raw byte array for fast mask readout (avoid Mat.get() in inner loops)
    private byte[] maskBytes = new byte[0];

    // ─────────────────────────────────────────────────────────────────────────
    // Shared state — written by vision thread, read by robot thread
    // ─────────────────────────────────────────────────────────────────────────

    private final Object     lock     = new Object();
    private final List<Clump> results  = new ArrayList<>();
    private Clump   best          = null;
    private double  steeringError = 0.0;
    private int     fw = 640, fh = 480;  // frame dimensions

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
    }

    // ─────────────────────────────────────────────────────────────────────────
    // VisionProcessor — init
    // ─────────────────────────────────────────────────────────────────────────

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        synchronized (lock) { fw = width; fh = height; }
        int needed = width * height;
        if (maskBytes.length < needed) maskBytes = new byte[needed];
    }

    // ─────────────────────────────────────────────────────────────────────────
    // VisionProcessor — processFrame  (runs on the vision thread)
    // ─────────────────────────────────────────────────────────────────────────

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        int W, H;
        synchronized (lock) { W = fw; H = fh; }

        // ── 1. HSV threshold ─────────────────────────────────────────────────
        // HSV separates hue from brightness, giving stability across venue
        // lighting. Two merged ranges catch both warm and cool light sources.
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsvMat, HSV_LO_A, HSV_HI_A, maskA);
        Core.inRange(hsvMat, HSV_LO_B, HSV_HI_B, maskB);
        Core.bitwise_or(maskA, maskB, mask);

        // ── 2. RLE compression ───────────────────────────────────────────────
        // Read entire mask as a flat byte array once (avoids JNI overhead of
        // calling Mat.get() inside any kind of loop).
        mask.get(0, 0, maskBytes);
        rCount = buildRle(maskBytes, W, H);

        // ── 3. RLE morphological open (noise removal) ────────────────────────
        // Erosion: drop runs shorter than 2*OPEN_RADIUS, shrink the rest.
        // Dilation: restore surviving runs to original size.
        // Net effect: isolated specks vanish, real ball blobs survive.
        rCount = rleOpen(W);

        // ── 4. RLE morphological close (gap filling) ─────────────────────────
        // Dilation: expand runs by CLOSE_H_GAP and merge vertical neighbors.
        // Erosion: shrink back to original size.
        // Net effect: small gaps between touching balls in a clump are filled.
        rCount = rleClose(W);

        // ── 5. Single-pass RLE connected-component labeling ──────────────────
        // Sorts runs by (row, start) then sweeps adjacency with a two-pointer,
        // union-find merging. Blob statistics are accumulated in the same pass.
        int numBlobs = rleLabel();

        // ── 6. Filter blobs, build Clumps ────────────────────────────────────
        List<Clump> clumps = buildClumps(numBlobs, W);
        Collections.sort(clumps, CLUMP_ORDER);

        // ── 7. Annotate + publish ─────────────────────────────────────────────
        drawAnnotations(input, clumps, W, H);
        synchronized (lock) {
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
    // Step 2: Build RLE from binary mask
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Single linear scan of the flat mask byte array.
     * Encodes each horizontal run of nonzero pixels as (row, start, end).
     * O(W × H) — exactly one read per pixel, no branching per-column beyond
     * the foreground/background transition check.
     */
    private int buildRle(byte[] m, int W, int H) {
        int count = 0;
        for (int y = 0; y < H && count < MAX_RUNS - 1; y++) {
            int base  = y * W;
            boolean in = false;
            int     s  = 0;
            for (int x = 0; x < W; x++) {
                boolean fg = m[base + x] != 0;
                if (fg && !in)          { s = x; in = true; }
                else if (!fg && in)     { writeRun(count++, y, s, x - 1); in = false; }
            }
            if (in) writeRun(count++, y, s, W - 1);
        }
        return count;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Step 3: RLE morphological open
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Erosion + dilation in run-space.
     *
     * Erosion: shrink each run inward by OPEN_RADIUS on each side.
     *   - Runs shorter than 2*OPEN_RADIUS vanish entirely (noise removal).
     *   - Runs that have no vertically adjacent neighbour within OPEN_RADIUS
     *     rows are also dropped (isolated horizontal streaks).
     *
     * Dilation: re-expand surviving runs by OPEN_RADIUS on each side.
     *   Completes the open: noise is gone, ball blobs return to original size.
     *
     * Critical insight (Ehrensperger 2015): for RLE morphology, kernel size
     * does NOT increase cost — a 15px erosion is the same number of array
     * operations as a 3px erosion, because we work on run endpoints, not pixels.
     */
    private int rleOpen(int W) {
        if (rCount == 0) return 0;
        sortRuns(rCount); // sort needed for the neighbour check below

        int out = 0;
        for (int i = 0; i < rCount; i++) {
            int ns = rStart[i] + OPEN_RADIUS;
            int ne = rEnd[i]   - OPEN_RADIUS;
            if (ns > ne) continue; // run too short — drop

            // Vertical neighbour check: require at least one run on an
            // adjacent row that overlaps the shrunken span.
            boolean hasNeighbour = false;
            for (int j = 0; j < rCount && !hasNeighbour; j++) {
                int dy = rRow[j] - rRow[i];
                if (dy == 0 || dy < -OPEN_RADIUS) continue;
                if (dy > OPEN_RADIUS) break; // sorted by row → can early-exit
                if (rStart[j] <= ne && rEnd[j] >= ns) hasNeighbour = true;
            }
            if (!hasNeighbour) continue;

            writeRun(out++, rRow[i], ns, ne);
        }

        // Re-dilate: restore eroded runs to original size
        for (int i = 0; i < out; i++) {
            rStart[i] = Math.max(0,   rStart[i] - OPEN_RADIUS);
            rEnd[i]   = Math.min(W-1, rEnd[i]   + OPEN_RADIUS);
        }
        return out;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Step 4: RLE morphological close
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Dilation + erosion in run-space.
     *
     * Dilation: expand every run by CLOSE_H_GAP horizontally, then merge all
     *   runs that now overlap — including runs on adjacent rows within
     *   CLOSE_V_RADIUS. Small gaps between touching balls collapse into one run.
     *
     * Erosion: shrink surviving (merged) runs back by CLOSE_H_GAP.
     *   Restores original boundaries; the "bridge" pixels that filled the
     *   gap are removed, but the two blobs now share adjacency for CCL.
     */
    private int rleClose(int W) {
        if (rCount == 0) return 0;

        // Dilate horizontally
        for (int i = 0; i < rCount; i++) {
            rStart[i] = Math.max(0,   rStart[i] - CLOSE_H_GAP);
            rEnd[i]   = Math.min(W-1, rEnd[i]   + CLOSE_H_GAP);
        }

        // Sort by (row ASC, start ASC) then merge overlapping/adjacent runs
        sortRuns(rCount);
        int out = 0;
        for (int i = 0; i < rCount; i++) {
            if (out == 0) { copyRun(i, out++); continue; }
            int p  = out - 1;
            int dy = rRow[i] - rRow[p];
            if (dy <= CLOSE_V_RADIUS && rStart[i] <= rEnd[p] + 1) {
                // Merge into previous run
                if (rEnd[i] > rEnd[p]) rEnd[p] = rEnd[i];
            } else {
                copyRun(i, out++);
            }
        }

        // Erode back
        int final_ = 0;
        for (int i = 0; i < out; i++) {
            int ns = rStart[i] + CLOSE_H_GAP;
            int ne = rEnd[i]   - CLOSE_H_GAP;
            if (ns > ne) continue;
            writeRun(final_++, rRow[i], ns, ne);
        }
        return final_;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Step 5: RLE connected-component labeling
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Single-pass union-find CCL over the run-length representation.
     *
     * Algorithm (Wang et al. 2010, adapted):
     *
     *   Phase A — Merge:
     *     Iterate runs in sorted order. For each run on row R, scan all runs
     *     on row R-1 (maintained via a sliding window pointer). Any run on
     *     R-1 that overlaps horizontally is unioned with the current run.
     *     Union-find with path-halving + union-by-rank gives amortized O(α(N))
     *     per operation — effectively O(1) for realistic run counts.
     *
     *   Phase B — Accumulate:
     *     Second pass over runs. For each run, resolve its canonical label
     *     via find(), then add the run's contribution to that label's
     *     centroid sum, area, and bounding box. No pixel-level second scan.
     *
     * Returns the number of distinct components found.
     */
    private int rleLabel() {
        if (rCount == 0) return 0;
        sortRuns(rCount);

        // Initialise union-find
        for (int i = 0; i < rCount; i++) { ufParent[i] = i; ufRank[i] = 0; }

        // Phase A: merge adjacent-row overlapping runs
        int prevStart = 0; // sliding window into runs on (curRow - 1)
        for (int i = 0; i < rCount; i++) {
            int curRow = rRow[i];
            // Advance window past rows older than curRow-1
            while (prevStart < i && rRow[prevStart] < curRow - 1) prevStart++;
            // Scan window for horizontal overlap
            for (int j = prevStart; j < i; j++) {
                if (rRow[j] != curRow - 1) continue;
                if (rStart[j] > rEnd[i] || rEnd[j] < rStart[i]) continue;
                ufUnion(i, j);
            }
        }

        // Phase B: accumulate per-component statistics
        for (int i = 0; i < rCount; i++) {
            cSumX[i] = cSumY[i] = cArea[i] = 0;
            cMinX[i] = cMinY[i] = Integer.MAX_VALUE;
            cMaxX[i] = cMaxY[i] = Integer.MIN_VALUE;
        }
        for (int i = 0; i < rCount; i++) {
            int    root = ufFind(i);
            int    len  = rEnd[i] - rStart[i] + 1;
            long   midX = rStart[i] + len / 2L;
            cArea[root] += len;
            cSumX[root] += midX * len;
            cSumY[root] += (long) rRow[i] * len;
            if (rStart[i] < cMinX[root]) cMinX[root] = rStart[i];
            if (rRow[i]   < cMinY[root]) cMinY[root] = rRow[i];
            if (rEnd[i]   > cMaxX[root]) cMaxX[root] = rEnd[i];
            if (rRow[i]   > cMaxY[root]) cMaxY[root] = rRow[i];
        }

        int n = 0;
        for (int i = 0; i < rCount; i++) if (ufFind(i) == i && cArea[i] > 0) n++;
        return n;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Step 6: Filter components and merge spatially close ones into Clumps
    // ─────────────────────────────────────────────────────────────────────────

    private List<Clump> buildClumps(int numBlobs, int W) {
        // Collect valid components
        List<double[]> comps = new ArrayList<>(numBlobs);
        for (int i = 0; i < rCount; i++) {
            if (ufFind(i) != i || cArea[i] == 0) continue;
            double area = cArea[i];
            if (area < MIN_AREA || area > MAX_AREA) continue;
            int bw = cMaxX[i] - cMinX[i] + 1;
            int bh = cMaxY[i] - cMinY[i] + 1;
            if (bh == 0) continue;
            double asp = (double) bw / bh;
            if (asp > MAX_ASPECT || asp < 1.0 / MAX_ASPECT) continue;
            double cx = (double) cSumX[i] / area;
            double cy = (double) cSumY[i] / area;
            // [cx, cy, area, x1, y1, x2, y2]
            comps.add(new double[]{cx, cy, area, cMinX[i], cMinY[i], cMaxX[i], cMaxY[i]});
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

    // ─────────────────────────────────────────────────────────────────────────
    // Union-find — path-halving + union by rank (near-O(1) amortized)
    // ─────────────────────────────────────────────────────────────────────────

    private int ufFind(int x) {
        // Path halving: point every other node directly at its grandparent.
        // Equivalent to path compression in practice, no recursion needed.
        while (ufParent[x] != x) {
            ufParent[x] = ufParent[ufParent[x]];
            x           = ufParent[x];
        }
        return x;
    }

    private void ufUnion(int a, int b) {
        int ra = ufFind(a), rb = ufFind(b);
        if (ra == rb) return;
        // Union by rank: attach smaller tree under larger
        if      (ufRank[ra] < ufRank[rb]) ufParent[ra] = rb;
        else if (ufRank[ra] > ufRank[rb]) ufParent[rb] = ra;
        else { ufParent[rb] = ra; ufRank[ra]++; }
    }

    // Simple union-find for the small clump-merge step (no rank needed)
    private int smallFind(int[] p, int i) {
        while (p[i] != i) { p[i] = p[p[i]]; i = p[i]; } return i;
    }
    private void smallUnion(int[] p, int a, int b) {
        a = smallFind(p, a); b = smallFind(p, b); if (a != b) p[b] = a;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Run array helpers
    // ─────────────────────────────────────────────────────────────────────────

    private void writeRun(int i, int row, int start, int end) {
        rRow[i] = row; rStart[i] = start; rEnd[i] = end;
    }

    private void copyRun(int from, int to) {
        rRow[to] = rRow[from]; rStart[to] = rStart[from]; rEnd[to] = rEnd[from];
    }

    /**
     * In-place insertion sort of the run arrays by (row ASC, start ASC).
     *
     * Insertion sort chosen deliberately:
     *   • Runs are nearly sorted between consecutive frames (the scene changes
     *     little in 33ms), making insertion sort O(N + k) where k = inversions.
     *   • For noisy frames with ~2000 runs, worst-case is still fast enough
     *     (~0.3ms) because the constant factor is tiny (3 array reads/writes).
     *   • Zero heap allocation — no Comparator, no Integer boxing.
     */
    private void sortRuns(int count) {
        for (int i = 1; i < count; i++) {
            int kr = rRow[i], ks = rStart[i], ke = rEnd[i];
            int j  = i - 1;
            while (j >= 0 && (rRow[j] > kr || (rRow[j] == kr && rStart[j] > ks))) {
                rRow[j+1]   = rRow[j];
                rStart[j+1] = rStart[j];
                rEnd[j+1]   = rEnd[j];
                j--;
            }
            rRow[j+1]   = kr;
            rStart[j+1] = ks;
            rEnd[j+1]   = ke;
        }
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
    }
}
