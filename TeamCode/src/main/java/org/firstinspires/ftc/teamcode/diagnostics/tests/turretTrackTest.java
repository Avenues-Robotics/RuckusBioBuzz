package org.firstinspires.ftc.teamcode.diagnostics.tests;

import org.firstinspires.ftc.teamcode.SystemConstants;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.helpers.Alliance;
import org.firstinspires.ftc.teamcode.helpers.HivePosition;
import org.firstinspires.ftc.teamcode.modules.PredictiveAiming;
import org.firstinspires.ftc.teamcode.systems.Turret;

/**
 * DIAGNOSTIC ONLY. Runs the real {@link Turret} against a SIMULATED robot pose, so turret
 * tracking (wrap, unwind, target switch at the field half) can be checked on blocks with nothing
 * driving. Only the two turret servos move.
 *
 * <p>SCENARIO 0 SPIN: robot parked at (SPIN_X, SPIN_Y), heading turning at SPIN_DEG_PER_S, so the
 * turret must counter-rotate through its travel limit and unwind. 1 LAP: robot drives a rectangle
 * (LAP_MIN..LAP_MAX in x and y) at LAP_IN_PER_S facing its direction of travel - crosses the y half
 * line (hive target flips) and turns a full 360 per lap. 2 MANUAL: MAN_X / MAN_Y / MAN_HEADING_DEG.
 * 3 IMU: robot parked at MAN_X / MAN_Y, heading = Control Hub IMU yaw (device "imu"), zeroed to
 * MAN_HEADING_DEG when the scenario starts - turn the robot by hand and the turret counter-rotates.
 * Diagnostic stand-in only; nothing shipped reads the Control Hub IMU (odometry is the Pinpoint).
 *
 * <p>Logcat tag turretTrackTest, every 100 ms: sim pose, the aim angle theta, commanded servo
 * position, the turret angle that position implies, and their difference (err, wrapped to +-180).
 * err is a SOFTWARE check of the commanded position; the servo's real angle is not sensed.
 */
@Config
@Configurable
public class turretTrackTest extends LinearOpMode {
    public static int SCENARIO = 0;
    public static boolean RED = true;

    public static double SPIN_X = 72, SPIN_Y = 40, SPIN_DEG_PER_S = 30;

    public static double LAP_MIN = 30, LAP_MAX = 114, LAP_IN_PER_S = 15;

    public static double MAN_X = 72, MAN_Y = 40, MAN_HEADING_DEG = 0;

    /** Flip if turning the robot left makes the IMU heading go down (hub mounted logo-down). */
    public static boolean IMU_REVERSED = false;

    private static IMU imu;

    /** The simulated pose that SimAiming hands to turret. */
    private static Pose sim = new Pose(72, 40, 0);

    /** predictiveAiming that ignores the follower and returns the simulated pose (no lead). */
    private static final class SimAiming extends PredictiveAiming {
        SimAiming() { super(null); }

        @Override public Pose leadPose() { return sim; }

        @Override public Pose leadPose(double[] offset) {
            double c = Math.cos(sim.heading()), s = Math.sin(sim.heading());
            return new Pose(sim.x() + offset[0] * c - offset[1] * s,
                    sim.y() + offset[0] * s + offset[1] * c, sim.heading());
        }
    }

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        boolean tRed = RED;
        Turret t = new Turret(hardwareMap, new SimAiming(), new HivePosition(), tRed ? Alliance.RED : Alliance.BLUE);
        Servo pollen = hardwareMap.get(Servo.class, "pollenTurretServo");
        imu = null;
        try {
            imu = hardwareMap.get(IMU.class, "imu");
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        } catch (IllegalArgumentException e) {
            imu = null; // no "imu" in the active config: SCENARIO 3 falls back to MANUAL
        }
        telemetry.addLine("Ready. SCENARIO 0 SPIN / 1 LAP / 2 MANUAL. Runs from INIT.");
        telemetry.update();

        long start = System.nanoTime();
        long lastLog = 0;
        int lastScenario = -1;
        while (!isStopRequested()) {
            if (SCENARIO != lastScenario) {
                start = System.nanoTime();
                lastScenario = SCENARIO;
                if (imu != null) imu.resetYaw();
            }
            double s = (System.nanoTime() - start) / 1e9;
            sim = simPose(SCENARIO, s);
            if (RED != tRed) { // alliance is a constructor arg now: rebuild on a live Dashboard edit
                tRed = RED;
                t = new Turret(hardwareMap, new SimAiming(), new HivePosition(), tRed ? Alliance.RED : Alliance.BLUE);
            }
            t.update();

            double thetaDeg = Math.toDegrees(t.aimPollen().theta);
            double pos = pollen.getPosition();
            double range = SystemConstants.Turret.SERVO_RANGE_DEG * SystemConstants.Turret.POLLEN_TURRET_GEAR_RATIO[1]
                    / SystemConstants.Turret.POLLEN_TURRET_GEAR_RATIO[0];
            double impliedDeg = (SystemConstants.Turret.POLLEN_TURRET_REVERSED ? -1 : 1)
                    * (pos - SystemConstants.Turret.POLLEN_TURRET_CENTER) * range;
            double err = AngleUnit.normalizeDegrees(impliedDeg - thetaDeg);

            telemetry.addData("scenario", SCENARIO);
            if (SCENARIO == 3 && imu == null) telemetry.addLine("NO \"imu\" IN ACTIVE CONFIG - using MANUAL heading");
            telemetry.addData("sim pose", "%.1f, %.1f, %.0f deg", sim.x(), sim.y(),
                    Math.toDegrees(sim.heading()));
            telemetry.addData("aim theta deg", thetaDeg);
            telemetry.addData("pollen servo pos", pos);
            telemetry.addData("implied turret deg", impliedDeg);
            telemetry.addData("err deg", err);
            telemetry.addData("aimed / unwinding", t.pollenAimed() + " / " + t.unwinding());
            telemetry.update();

            long now = System.currentTimeMillis();
            if (now - lastLog >= 100) {
                lastLog = now;
                Log.i("turretTrackTest", String.format(java.util.Locale.US,
                        "sc=%d t=%.2f x=%.1f y=%.1f h=%.1f theta=%.1f pos=%.4f implied=%.1f err=%.1f aimed=%b unwinding=%b",
                        SCENARIO, s, sim.x(), sim.y(), Math.toDegrees(sim.heading()), thetaDeg,
                        pos, impliedDeg, err, t.pollenAimed(), t.unwinding()));
            }
            sleep(20);
        }
    }

    private static Pose simPose(int scenario, double s) {
        switch (scenario) {
            case 1: {
                double side = LAP_MAX - LAP_MIN;
                double d = (s * LAP_IN_PER_S) % (4 * side);
                int leg = (int) (d / side);
                double u = d - leg * side;
                switch (leg) {
                    case 0:  return new Pose(LAP_MIN + u, LAP_MIN, 0);
                    case 1:  return new Pose(LAP_MAX, LAP_MIN + u, Math.PI / 2);
                    case 2:  return new Pose(LAP_MAX - u, LAP_MAX, Math.PI);
                    default: return new Pose(LAP_MIN, LAP_MAX - u, -Math.PI / 2);
                }
            }
            case 3:
                if (imu != null) {
                    YawPitchRollAngles a = imu.getRobotYawPitchRollAngles();
                    double yaw = (IMU_REVERSED ? -1 : 1) * a.getYaw(AngleUnit.DEGREES);
                    return new Pose(MAN_X, MAN_Y, Math.toRadians(MAN_HEADING_DEG + yaw));
                }
                return new Pose(MAN_X, MAN_Y, Math.toRadians(MAN_HEADING_DEG));
            case 2:
                return new Pose(MAN_X, MAN_Y, Math.toRadians(MAN_HEADING_DEG));
            default:
                return new Pose(SPIN_X, SPIN_Y, Math.toRadians(s * SPIN_DEG_PER_S));
        }
    }
}
