package org.firstinspires.ftc.teamcode.diagnostics.autosim;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.auto.BlueSoloAuto;
import org.firstinspires.ftc.teamcode.auto.RedSoloAuto;
import org.firstinspires.ftc.teamcode.auto.modules.AutoMechanisms;
import org.firstinspires.ftc.teamcode.fieldview.FieldView;
import org.firstinspires.ftc.teamcode.helpers.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.lang.reflect.Field;
import java.util.ArrayDeque;
import java.util.Locale;

/**
 * DIAGNOSTIC ONLY - never ships. Runs the REAL competition auto (RedSoloAuto / BlueSoloAuto: same paths,
 * events, shooter, intake, turret, gate, scoopula) with the robot ON BLOCKS and a simulated pose: a
 * {@link SimLocalizer} replaces the Pinpoint and integrates the follower's own commanded powers, so the pose
 * travels the field as the auto intends. Watch it at http://192.168.43.1:8080/field.
 *
 * <p>Everything except the pose is real hardware: with MOVE_PODS the pods steer and the wheels spin in the
 * air; the turret aims at the hive from the simulated pose; flywheel, gate and intake follow the auto's
 * events. LOAD NO PIECES - the turret points wherever the simulated robot sees the hive.
 *
 * <p>Bypasses Constants.requireForesightMeasured() for the simulated follower only: the hooks in Constants are
 * set just around the auto's init() and cleared in a finally. Every change of path step / event / shooter
 * state is logged (RobotLog tag AutoDryRun) and shown in telemetry (Driver Station + FTC Dashboard).
 */
@Config
@Configurable
public class AutoDryRun extends OpMode {
    /** RED or BLUE. */
    public static String AUTO = "RED";
    /** Drive the real pods on blocks (true) or keep the drivetrain still and only simulate (false). */
    public static boolean MOVE_PODS = true;
    /** First-order lag of the simulated chassis, s. */
    public static double SIM_TAU_S = 0.15;
    /** Pod distance from the robot centre, in: hypot(146.42, 154.24) mm (bring-up pod geometry). */
    public static double POD_RADIUS_IN = 8.37;

    private OpMode auto;
    private SimDrive drive;
    private SimLocalizer localizer;
    private AutoMechanisms mechanisms;
    private Field sequenceField;
    private String lastLine = "";
    private long startNs;
    private final ArrayDeque<String> events = new ArrayDeque<>();

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        auto = "BLUE".equalsIgnoreCase(AUTO) ? new BlueSoloAuto() : new RedSoloAuto();
        auto.hardwareMap = hardwareMap;
        auto.telemetry = telemetry;
        auto.gamepad1 = gamepad1;
        auto.gamepad2 = gamepad2;
        localizer = new SimLocalizer(() -> drive, Constants.maxAchievableForwardVelocity, POD_RADIUS_IN, SIM_TAU_S);
        Constants.dryRunLocalizer = localizer;
        Constants.dryRunDriveWrap = real -> drive = new SimDrive(real, MOVE_PODS);
        try {
            auto.init();
        } finally {
            Constants.dryRunLocalizer = null;
            Constants.dryRunDriveWrap = null;
        }
        mechanisms = (AutoMechanisms) read(auto, "mechanisms");
        sequenceField = field(auto, "sequenceIndex");
        log("init " + AUTO + " auto, simulated pose " + fmt(localizer.pose()) + ", MOVE_PODS " + MOVE_PODS);
    }

    @Override
    public void init_loop() {
        auto.init_loop();
        FieldView.publish(localizer.pose(), localizer.pose());
    }

    @Override
    public void start() {
        startNs = System.nanoTime();
        auto.start();
        log("start");
    }

    @Override
    public void loop() {
        // Lines added before the auto's own telemetry.update() inside its loop().
        telemetry.addData("DRY RUN", AUTO + " auto, robot on blocks, simulated pose");
        telemetry.addData("sim pose", fmt(localizer.pose()));
        for (String e : events) {
            telemetry.addLine(e);
        }
        auto.loop();
        FieldView.publish(localizer.pose(), localizer.pose());
        if (localizer.diverged()) {
            log("SIM DIVERGED - non-finite pose refused, stopping the dry run");
            requestOpModeStop();
            return;
        }
        String line = "seq=" + readInt(sequenceField) + " " + (mechanisms == null ? "" : mechanisms.stateLine());
        if (!line.equals(lastLine)) {
            lastLine = line;
            log(line + "  wheel " + (mechanisms == null ? "?" : Math.round(mechanisms.pollenVelocity()))
                    + "  at " + fmt(localizer.pose()));
        }
    }

    @Override
    public void stop() {
        if (auto != null) {
            auto.stop();
        }
        PoseStorage.clear();   // the auto saved the SIMULATED pose for TeleOp - never hand that on
        log("stop");
    }

    private void log(String what) {
        double t = startNs == 0 ? 0 : (System.nanoTime() - startNs) / 1e9;
        String e = String.format(Locale.US, "%6.2fs %s", t, what);
        RobotLog.ii("AutoDryRun", e);
        events.addLast(e);
        while (events.size() > 14) {
            events.removeFirst();
        }
    }

    private static String fmt(Pose p) {
        return String.format(Locale.US, "(%.1f, %.1f, %.0f deg)", p.x(), p.y(), Math.toDegrees(p.heading()));
    }

    private static Field field(Object o, String name) {
        try {
            Field f = o.getClass().getDeclaredField(name);
            f.setAccessible(true);
            return f;
        } catch (NoSuchFieldException e) {
            return null;
        }
    }

    private static Object read(Object o, String name) {
        try {
            Field f = field(o, name);
            return f == null ? null : f.get(o);
        } catch (IllegalAccessException e) {
            return null;
        }
    }

    private int readInt(Field f) {
        try {
            return f == null ? -1 : f.getInt(auto);
        } catch (IllegalAccessException e) {
            return -1;
        }
    }
}
