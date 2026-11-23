package org.firstinspires.ftc.teamcode.pipelines;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.SystemConstants;
import org.firstinspires.ftc.teamcode.fieldview.CameraFeed;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;

/**
 * The webcam (Logitech on the Control Hub's USB 3.0 port) running {@link PollenDetectionPipeline}, with
 * {@link CameraFeed} after it for the field page's Camera tab. Vision runs on its own thread; the OpMode loop only
 * pays for reading results. Never throws: with no camera, {@link #status()} says why and everything else is a no-op.
 *
 * <p>Camera lookup: SystemConstants.Camera.WEBCAM_NAME in the active robot configuration if it is there, else the
 * first webcam attached to the hub - the built-in ruckusbiobuzz.xml has no webcam entry (a webcam entry needs the
 * camera's serial number).
 */
public final class PollenCamera {
    private static final String TAG = "PollenCamera";

    private final PollenDetectionPipeline pipeline;
    private final VisionPortal portal;
    private final String status;

    private PollenCamera(PollenDetectionPipeline pipeline, VisionPortal portal, String status) {
        this.pipeline = pipeline;
        this.portal = portal;
        this.status = status;
    }

    /** @param streaming encode frames for /field/camera.jpg (false in comp branch 2). */
    public static PollenCamera open(HardwareMap hardwareMap, boolean streaming) {
        CameraFeed.clear();
        try {
            WebcamName cam = hardwareMap.tryGet(WebcamName.class, SystemConstants.Camera.WEBCAM_NAME);
            String how = "\"" + SystemConstants.Camera.WEBCAM_NAME + "\" from the config";
            if (cam == null) {
                List<WebcamName> all = ClassFactory.getInstance().getCameraManager().getAllWebcams();
                // The serial is what a <Webcam name=".." serialNumber=".."/> config entry needs: logged for that.
                for (WebcamName w : all) {
                    RobotLog.ii(TAG, "attached webcam serialNumber=\"" + w.getSerialNumber() + "\" " + w.getDeviceName());
                }
                if (all.isEmpty()) {
                    return new PollenCamera(null, null, "no webcam attached");
                }
                cam = all.get(0);
                how = "first attached webcam, NOT in the config (serial " + cam.getSerialNumber() + ")";
            }
            PollenDetectionPipeline pipeline = new PollenDetectionPipeline(null);
            VisionPortal portal = new VisionPortal.Builder()
                    .setCamera(cam)
                    .setCameraResolution(new android.util.Size(SystemConstants.Camera.WIDTH, SystemConstants.Camera.HEIGHT))
                    .addProcessors(pipeline, new CameraFeed(pipeline, streaming))
                    .enableLiveView(false)
                    .build();
            RobotLog.ii(TAG, "opened " + how);
            return new PollenCamera(pipeline, portal, "camera: " + how);
        } catch (RuntimeException e) {
            RobotLog.ee(TAG, e, "camera failed to open");
            return new PollenCamera(null, null, "camera failed: " + e.getMessage());
        }
    }

    /** Null when no camera opened. */
    public PollenDetectionPipeline pipeline() {
        return pipeline;
    }

    public String status() {
        return portal == null ? status : status + " · " + portal.getCameraState();
    }

    public void close() {
        if (portal != null) {
            try {
                portal.close();
            } catch (RuntimeException e) {
                RobotLog.ee(TAG, e, "close failed");
            }
        }
        if (pipeline != null) pipeline.release();
        CameraFeed.clear();
    }
}
