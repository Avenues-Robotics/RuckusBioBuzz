package org.firstinspires.ftc.teamcode.fieldview;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.SystemConstants;
import org.firstinspires.ftc.teamcode.pipelines.PollenDetectionPipeline;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.io.ByteArrayOutputStream;
import java.util.List;
import java.util.Locale;

/**
 * The last camera frame as a JPEG for the field page's Camera tab (/field/camera.jpg, stats at /field/camera).
 * Added to the VisionPortal AFTER {@link PollenDetectionPipeline}, which draws its boxes onto the frame, so the
 * feed shows what the pipeline found. Runs on the vision thread, never on the OpMode loop: it encodes at most
 * SystemConstants.Camera.STREAM_FPS frames a second, downscaled to STREAM_WIDTH, and does nothing at all while
 * {@link #streaming} is false (comp branch 2, no telemetry).
 */
public final class CameraFeed implements VisionProcessor {
    private static volatile byte[] jpeg;
    private static volatile long jpegAtNs;
    private static volatile String stats = "{\"live\":false}";

    private final PollenDetectionPipeline pipeline;
    private final boolean streaming;
    private final Mat small = new Mat();
    private final ByteArrayOutputStream out = new ByteArrayOutputStream(64 * 1024);
    private Bitmap bitmap;
    private long lastEncodeNs;
    private long fpsWindowStartNs;
    private int fpsFrames;
    private double cameraFps;
    private int width, height;

    public CameraFeed(PollenDetectionPipeline pipeline, boolean streaming) {
        this.pipeline = pipeline;
        this.streaming = streaming;
    }

    /** Forget the last OpMode's frame, so the page never shows a stale picture as live. */
    public static void clear() {
        jpeg = null;
        jpegAtNs = 0;
        stats = "{\"live\":false}";
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        this.width = width;
        this.height = height;
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        long now = System.nanoTime();
        fpsFrames++;
        if (fpsWindowStartNs == 0) fpsWindowStartNs = now;
        if (now - fpsWindowStartNs >= 1_000_000_000L) {
            cameraFps = fpsFrames * 1e9 / (now - fpsWindowStartNs);
            fpsFrames = 0;
            fpsWindowStartNs = now;
        }
        if (!streaming) return null;
        double fps = Math.max(1, SystemConstants.Camera.STREAM_FPS);
        if (now - lastEncodeNs < 1e9 / fps) return null;
        lastEncodeNs = now;
        // The pipeline's own frame: rotated upright (Camera.ROTATE_DEG) with its boxes drawn on it.
        Mat shown = pipeline != null ? pipeline.displayFrame() : null;
        if (shown != null && !shown.empty()) frame = shown;
        width = frame.cols();
        height = frame.rows();

        int w = (int) Math.max(80, Math.min(frame.cols(), SystemConstants.Camera.STREAM_WIDTH));
        int h = Math.max(1, Math.round(w * (float) frame.rows() / frame.cols()));
        Imgproc.resize(frame, small, new Size(w, h), 0, 0, Imgproc.INTER_AREA);
        if (bitmap == null || bitmap.getWidth() != w || bitmap.getHeight() != h) {
            bitmap = Bitmap.createBitmap(w, h, Bitmap.Config.ARGB_8888);
        }
        Utils.matToBitmap(small, bitmap);
        out.reset();
        bitmap.compress(Bitmap.CompressFormat.JPEG,
                (int) Math.max(10, Math.min(95, SystemConstants.Camera.JPEG_QUALITY)), out);
        jpeg = out.toByteArray();
        jpegAtNs = now;
        stats = statsJson();
        return null;
    }

    private String statsJson() {
        StringBuilder sb = new StringBuilder(256);
        sb.append("{\"live\":true,\"width\":").append(width).append(",\"height\":").append(height)
                .append(",\"fps\":").append(String.format(Locale.US, "%.1f", cameraFps));
        List<PollenDetectionPipeline.Clump> clumps = pipeline != null ? pipeline.getAllClumps() : null;
        int balls = 0;
        if (clumps != null) {
            for (PollenDetectionPipeline.Clump c : clumps) balls += c.estimatedBallCount;
            sb.append(",\"clumps\":").append(clumps.size()).append(",\"balls\":").append(balls);
            if (!clumps.isEmpty()) {
                PollenDetectionPipeline.Clump b = clumps.get(0);
                sb.append(",\"best\":{\"balls\":").append(b.estimatedBallCount)
                        .append(",\"cx\":").append(Math.round(b.centerX))
                        .append(",\"cy\":").append(Math.round(b.centerY))
                        .append(",\"area\":").append(Math.round(b.areaPx))
                        .append(",\"steer\":").append(String.format(Locale.US, "%.3f", b.steeringError))
                        .append('}');
            }
        }
        return sb.append('}').toString();
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx,
                            float scaleCanvasDensity, Object userContext) { }

    static byte[] latestJpeg() {
        return jpeg;
    }

    /** Stats plus the frame's age; "live" goes false once no frame has come for a second. */
    static String statsState() {
        String s = stats;
        long at = jpegAtNs;
        if (at == 0) return s;
        long ageMs = (System.nanoTime() - at) / 1_000_000L;
        return s.substring(0, s.length() - 1) + ",\"ageMs\":" + ageMs + (ageMs > 1000 ? ",\"stale\":true" : "") + "}";
    }
}
