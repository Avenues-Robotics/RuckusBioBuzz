package org.firstinspires.ftc.teamcode.auto.modules;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.AutoConstants;
import org.firstinspires.ftc.teamcode.Config;
import org.firstinspires.ftc.teamcode.SystemConstants;
import org.firstinspires.ftc.teamcode.helpers.Alliance;
import org.firstinspires.ftc.teamcode.pipelines.PollenCamera;
import org.firstinspires.ftc.teamcode.pipelines.PollenDetectionPipeline;

/**
 * The autos' camera pick (operator 2026-11-23): partway down a path the robot reads ONE frame of the pollen
 * pipeline, picks the biggest clump against its alliance's end wall and is re-routed into it, ending pressed on the
 * wall. Red drives toward the audience wall (y = 0) facing 270; blue is the same play turned 180 deg about the field
 * centre (far wall, facing 90). Every AutoConstants.Autonomous value is in the RED frame and mirrored here.
 *
 * <p>The generated autos (tools/pedro-visualizer, a path's "Camera pick") only call {@link #pickY()},
 * {@link #decide(Pose)} and {@link #slamPath(Pose, Pose)}; the visualizer previews the same rule (pick x = the clump's
 * wall x, clamped to PICK_X_MIN..MAX; the same curve).
 */
public final class CameraPick {
    private static final String TAG = "CameraPick";

    private final Alliance alliance;
    private final PollenCamera camera;
    private String note = "not yet";

    public CameraPick(HardwareMap hardwareMap, Alliance alliance) {
        this.alliance = alliance;
        camera = AutoConstants.Autonomous.CAMERA_PICK ? PollenCamera.open(hardwareMap, Config.telemetryEnabled()) : null;
    }

    private static double field() {
        return SystemConstants.Zone.FIELD_SIZE;
    }

    private boolean blue() {
        return alliance == Alliance.BLUE;
    }

    /** Red-frame pose -> this alliance's field pose (blue: turned 180 deg about the field centre). */
    private Pose toField(double x, double y, double heading) {
        return blue() ? new Pose(field() - x, field() - y, heading + Math.PI) : new Pose(x, y, heading);
    }

    /** Field y on the path where the frame is read. */
    public double pickY() {
        return blue() ? field() - AutoConstants.Autonomous.PICK_Y : AutoConstants.Autonomous.PICK_Y;
    }

    /**
     * Reads the pipeline's latest frame now and returns where to end against the wall (field frame, heading facing
     * the wall), or null to carry on with the planned path (no camera, nothing seen in range).
     */
    public Pose decide(Pose robot) {
        PollenDetectionPipeline pipeline = camera != null ? camera.pipeline() : null;
        if (pipeline == null) {
            note = camera == null ? "off - planned path" : "no camera (" + camera.status() + ") - planned path";
            RobotLog.ii(TAG, note);
            return null;
        }
        // Work in the red frame: the robot pose turned with the field for blue, so one set of constants serves both.
        Pose red = blue() ? new Pose(field() - robot.x(), field() - robot.y(), robot.heading() + Math.PI) : robot;
        WallClumpPicker.Pick pick = WallClumpPicker.pick(red, pipeline.getAllClumps(),
                AutoConstants.Autonomous.BALL_LINE_Y, AutoConstants.Autonomous.PICK_X_MIN,
                AutoConstants.Autonomous.PICK_X_MAX);
        if (pick == null) {
            note = "no clump in range - planned path";
            RobotLog.ii(TAG, note);
            return null;
        }
        Pose end = toField(pick.x, AutoConstants.Autonomous.WALL_Y, Math.toRadians(270));
        note = String.format(java.util.Locale.US, "(%.1f, %.1f) %s", end.x(), end.y(), pick);
        RobotLog.ii(TAG, "pick at (" + Math.round(robot.x()) + ", " + Math.round(robot.y()) + "): " + note);
        return end;
    }

    /**
     * From {@code from} into {@code end}: straight on toward the wall first (clear of the HIVE rail beside the
     * lane), then across to the clump's x, holding the end heading the whole way.
     */
    public static Path slamPath(Pose from, Pose end) {
        double dy = end.y() - from.y();
        final double heading = end.heading();
        return Paths.curve(
                new Pose(from.x(), from.y()),
                new Pose(from.x(), from.y() + 0.45 * dy),
                new Pose(end.x(), end.y() - 0.35 * dy),
                new Pose(end.x(), end.y())
        ).heading((curve, t) -> heading);
    }

    public String note() {
        return note;
    }

    public String status() {
        return camera != null ? camera.status() : "off (AutoConstants.Autonomous.CAMERA_PICK)";
    }

    public void close() {
        if (camera != null) camera.close();
    }
}
