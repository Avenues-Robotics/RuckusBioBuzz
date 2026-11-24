package org.firstinspires.ftc.teamcode.auto.modules;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.AutoConstants;
import org.firstinspires.ftc.teamcode.Config;
import org.firstinspires.ftc.teamcode.diagnostics.swerve.TeleLoopProbe;
import org.firstinspires.ftc.teamcode.helpers.Alliance;
import org.firstinspires.ftc.teamcode.modules.PredictiveAiming;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Scoopula;
import org.firstinspires.ftc.teamcode.systems.Shooter;

import java.util.List;

public final class AutoMechanisms {
    private final List<LynxModule> hubs;
    private final Follower follower;
    /** Gate-open time accumulated during the current Shoot event (eventDone). */
    private double gateOpenMs;
    private long lastUpdateNs;
    private boolean preAiming;
    private final Shooter shooter;
    private final Intake intake;
    private final Scoopula scoopula;
    private boolean shooting;
    private boolean intaking;
    private boolean scoopDown;
    /**
     * Drive trace (DIAGNOSTIC, 2026-11-24): the drive probe's recorder for the whole auto - pose, the follower's
     * heading target, the drive command (forward / strafe / turn) and every pod's target, error, turn power and flip,
     * one row per loop. Pull it from /swerve/rec.csv after the run. Off in comp (Config.telemetryEnabled false).
     */
    private final TeleLoopProbe trace;
    private double loopHzEma;
    private boolean traceDisabled;

    public AutoMechanisms(HardwareMap hardwareMap, Follower follower, Alliance alliance) {
        this.follower = follower;
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        // Foresight's heading kP for the autos, set every init (the config is static and shared by every OpMode).
        double kp = Double.isNaN(AutoConstants.Autonomous.HEADING_KP)
                ? org.firstinspires.ftc.teamcode.pedroPathing.Constants.headingKP : AutoConstants.Autonomous.HEADING_KP;
        org.firstinspires.ftc.teamcode.pedroPathing.Constants.foresightConfig.headingFeedback.set(
                com.pedropathing.controllers.Controller.proportional(kp));
        double tkp = AutoConstants.Autonomous.TRANSLATIONAL_KP;
        double fP = Double.isNaN(tkp) ? org.firstinspires.ftc.teamcode.pedroPathing.Constants.forwardTranslationalPrimaryKP : tkp;
        double fS = Double.isNaN(tkp) ? org.firstinspires.ftc.teamcode.pedroPathing.Constants.forwardTranslationalSecondaryKP : tkp;
        double sP = Double.isNaN(tkp) ? org.firstinspires.ftc.teamcode.pedroPathing.Constants.strafeTranslationalPrimaryKP : tkp;
        double sS = Double.isNaN(tkp) ? org.firstinspires.ftc.teamcode.pedroPathing.Constants.strafeTranslationalSecondaryKP : tkp;
        org.firstinspires.ftc.teamcode.pedroPathing.Constants.foresightConfig.forwardTranslational.set(
                org.firstinspires.ftc.teamcode.pedroPathing.Constants.translationalController(fP, fS,
                        org.firstinspires.ftc.teamcode.pedroPathing.Constants.translationalKI,
                        org.firstinspires.ftc.teamcode.pedroPathing.Constants.translationalKD,
                        org.firstinspires.ftc.teamcode.pedroPathing.Constants.translationalSwitchIn));
        org.firstinspires.ftc.teamcode.pedroPathing.Constants.foresightConfig.strafeTranslational.set(
                org.firstinspires.ftc.teamcode.pedroPathing.Constants.translationalController(sP, sS,
                        org.firstinspires.ftc.teamcode.pedroPathing.Constants.translationalKI,
                        org.firstinspires.ftc.teamcode.pedroPathing.Constants.translationalKD,
                        org.firstinspires.ftc.teamcode.pedroPathing.Constants.translationalSwitchIn));
        // Pod steering high-speed floor for the autos (robot-wide static; createSwerve re-applies Constants for
        // every other OpMode, so TeleOp is unaffected).
        double floorHigh = Double.isNaN(AutoConstants.Autonomous.STEER_FLOOR_HIGH)
                ? org.firstinspires.ftc.teamcode.pedroPathing.Constants.turnScheduleFloorHigh
                : AutoConstants.Autonomous.STEER_FLOOR_HIGH;
        com.pedropathing.revhub.drivetrains.CoaxialPod.setScheduleHighSpeed(floorHigh,
                org.firstinspires.ftc.teamcode.pedroPathing.Constants.turnScheduleHighStart,
                org.firstinspires.ftc.teamcode.pedroPathing.Constants.turnScheduleHighFull);
        com.qualcomm.robotcore.util.RobotLog.ii("AutoMechanisms", "pod schedule high floor " + floorHigh);
        com.pedropathing.revhub.drivetrains.Swerve.setDemandSlewDegPerSec(
                Double.isNaN(AutoConstants.Autonomous.DEMAND_SLEW_DEG_S)
                        ? org.firstinspires.ftc.teamcode.pedroPathing.Constants.demandSlewDegPerSec
                        : AutoConstants.Autonomous.DEMAND_SLEW_DEG_S);
        com.qualcomm.robotcore.util.RobotLog.ii("AutoMechanisms", "demand slew "
                + com.pedropathing.revhub.drivetrains.Swerve.getDemandSlewDegPerSec() + " deg/s");
        {
            double fl = org.firstinspires.ftc.teamcode.pedroPathing.Constants.forwardBrakeLinear;
            double fq = org.firstinspires.ftc.teamcode.pedroPathing.Constants.forwardBrakeQuadratic;
            double sl = AutoConstants.Autonomous.ISOTROPIC_BRAKING ? fl : org.firstinspires.ftc.teamcode.pedroPathing.Constants.strafeBrakeLinear;
            double sq = AutoConstants.Autonomous.ISOTROPIC_BRAKING ? fq : org.firstinspires.ftc.teamcode.pedroPathing.Constants.strafeBrakeQuadratic;
            org.firstinspires.ftc.teamcode.pedroPathing.Constants.foresightConfig.linearBrakeCoefficients.set(
                    com.pedropathing.math.Matrix.diag(fl, sl));
            org.firstinspires.ftc.teamcode.pedroPathing.Constants.foresightConfig.quadraticBrakeCoefficients.set(
                    com.pedropathing.math.Matrix.diag(fq, sq));
            com.qualcomm.robotcore.util.RobotLog.ii("AutoMechanisms", "braking fwd " + fl + "/" + fq + " strafe " + sl + "/" + sq);
        }
        double hb = Double.isNaN(AutoConstants.Autonomous.HEADING_BRAKE_SCALE) ? 1 : AutoConstants.Autonomous.HEADING_BRAKE_SCALE;
        org.firstinspires.ftc.teamcode.pedroPathing.Constants.foresightConfig.headingBrakeCoefficients.set(
                com.pedropathing.math.Vector2D.cartesian(
                        org.firstinspires.ftc.teamcode.pedroPathing.Constants.headingBrakeLinear * hb,
                        org.firstinspires.ftc.teamcode.pedroPathing.Constants.headingBrakeQuadratic * hb));
        com.qualcomm.robotcore.util.RobotLog.ii("AutoMechanisms", "heading braking x" + hb);
        double tolDeg = Double.isNaN(AutoConstants.Autonomous.HEADING_TOLERANCE_DEG)
                ? 11.25 : AutoConstants.Autonomous.HEADING_TOLERANCE_DEG;
        org.firstinspires.ftc.teamcode.pedroPathing.Constants.foresightConfig.headingDeviationTolerance.set(
                Math.toRadians(tolDeg));
        com.qualcomm.robotcore.util.RobotLog.ii("AutoMechanisms", "Foresight heading tolerance " + tolDeg + " deg");
        com.qualcomm.robotcore.util.RobotLog.ii("AutoMechanisms", "Foresight heading kP " + kp
                + ", translational kP fwd " + fP + "/" + fS + " strafe " + sP + "/" + sS);
        shooter = new Shooter(hardwareMap, new PredictiveAiming(follower), alliance);
        intake = new Intake(hardwareMap, alliance);
        scoopula = new Scoopula(hardwareMap);
        if (Config.telemetryEnabled() && TeleLoopProbe.ENABLED) {
            trace = new TeleLoopProbe();
            trace.setSource("Auto " + alliance);
            trace.init(hardwareMap);
        } else {
            trace = null;
        }
    }

    public void clearCache() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
    }

    public void update() {
        long now = System.nanoTime();
        double dtMs = lastUpdateNs == 0 ? 0 : (now - lastUpdateNs) / 1e6;
        lastUpdateNs = now;
        // Pre-aim: while a path is followed, aim from its end pose (AutoConstants.Autonomous.PRE_AIM).
        preAiming = AutoConstants.Autonomous.PRE_AIM && follower.following() && follower.currentPath() != null;
        shooter.setAimPose(preAiming ? follower.currentPath().endPose() : null);
        long a = System.nanoTime();
        shooter.update();
        long b = System.nanoTime();
        if (shooting && shooter.shooting()) {
            gateOpenMs += dtMs;
        }
        if (shooting) {
            intake.update(false, shooter.shooting());
        } else {
            intake.update(false, intaking);
        }
        long c = System.nanoTime();
        scoopula.update(scoopDown);
        long d = System.nanoTime();
        if (trace != null && !traceDisabled) {
            // A diagnostic must never stop the auto: on any exception the trace turns itself off for this run.
            try {
                traceLoop(dtMs / 1000.0);
            } catch (RuntimeException ex) {
                com.qualcomm.robotcore.util.RobotLog.ee("AutoMechanisms", ex, "drive trace failed - trace off");
                traceDisabled = true;
            }
        }
        long e = System.nanoTime();
        lastNs[0] = b - a;
        lastNs[1] = c - b;
        lastNs[2] = d - c;
        lastNs[3] = e - d;
    }

    /** Loop-time breakdown (DIAGNOSTIC): ns of the last update() in shooter, intake, scoopula, drive trace. */
    public final long[] lastNs = new long[4];

    private void traceLoop(double dt) {
        if (dt <= 0) {
            trace.startRecording("auto");   // first loop = START
            return;
        }
        loopHzEma = loopHzEma == 0 ? 1 / dt : 0.9 * loopHzEma + 0.1 / dt;
        com.pedropathing.math.Pose pose = follower.pose();
        double target = Double.NaN;
        double[] follow = new double[org.firstinspires.ftc.teamcode.diagnostics.swerve.PodRecorder.FOLLOW_COLS];
        java.util.Arrays.fill(follow, Double.NaN);
        if (follower.following()) {
            com.pedropathing.math.Pose closest = follower.closestPose();
            if (closest != null) {
                target = Math.toDegrees(closest.heading());
                // fterr: signed cross-track error, in (+ = left of the path tangent); fherr: heading error, deg
                com.pedropathing.math.Vector2D n = follower.closestNormal();
                double ex = pose.x() - closest.x(), ey = pose.y() - closest.y();
                follow[0] = n != null ? ex * n.x() + ey * n.y() : Math.hypot(ex, ey);
                follow[1] = Math.toDegrees(com.pedropathing.utils.Angle.normalizeSigned(closest.heading() - pose.heading()));
                follow[2] = closest.x();
                follow[3] = closest.y();
                follow[4] = Math.toDegrees(closest.heading());
            }
            // Not follower.tangentialVelocity(): its tangent is null on a path's first loop (NPE killed blue run 8).
            com.pedropathing.math.Vector2D tan = follower.closestTangent();
            if (tan != null) follow[5] = follower.velocity().toVector2D().dot(tan);
        }
        follow[6] = follower.isBusy() ? 1 : 0;
        // No drivetrain.debug(): it reads every drive motor's power off the hub (4 Lynx reads a loop) and would slow
        // the loop being measured. The follower's command is recoverable offline (Foresight heading = kP x error).
        double[] drive = new double[org.firstinspires.ftc.teamcode.diagnostics.swerve.PodRecorder.DRIVE_COLS];
        for (int i = 0; i < 4; i++) {
            com.pedropathing.revhub.drivetrains.CoaxialPod pod = org.firstinspires.ftc.teamcode.pedroPathing.Constants.builtPods[i];
            drive[i] = pod == null ? Double.NaN : pod.getLastDrivePower();
        }
        double[] cmd = follower.drivetrain instanceof com.pedropathing.revhub.drivetrains.Swerve
                ? ((com.pedropathing.revhub.drivetrains.Swerve) follower.drivetrain).lastCommand()
                : new double[]{Double.NaN, Double.NaN, Double.NaN};
        drive[4] = cmd[0];
        drive[5] = cmd[1];
        drive[6] = cmd[2];
        com.pedropathing.math.Twist tw = follower.twist();
        drive[7] = tw.vx;      // body forward, in/s
        drive[8] = tw.vy;      // body left, in/s
        drive[9] = tw.omega;   // rad/s
        trace.update(dt, loopHzEma, Math.toDegrees(pose.heading()), target, pose.x(), pose.y(),
                cmd[0], cmd[1], cmd[2], follow, drive);
    }

    public void start(String event) {
        switch (event) {
            case "Shoot":
                shooting = true;
                gateOpenMs = 0;
                shooter.setForceZone(true);
                break;
            case "Intake":
            case "Intake On":
                intaking = true;
                break;
            case "Intake Off":
                intaking = false;
                break;
            case "Scoopula Down":
                scoopDown = true;
                break;
            case "Scoopula Up":
                scoopDown = false;
                break;
            default:
                break;
        }
    }

    public void finish(String event) {
        switch (event) {
            case "Shoot":
                shooting = false;
                shooter.setForceZone(false);
                // The balls collected since the last shot are gone: the intake stops until the next collect
                // ("Intake On" with each corner loop, Scoopula Down and the camera-pick path).
                intaking = false;
                break;
            case "Intake":
                intaking = false;
                break;
            default:
                break;
        }
    }

    /**
     * Whether a timed event step is over. A Shoot step ends once the gate has been open SHOOT_OPEN_TIME in total,
     * or at {@code durationMs} (SHOOT_TIME, the cap) - so a turret unwind at its start delays the shot instead
     * of eating it. Every other event: {@code durationMs}.
     */
    public boolean eventDone(String event, long elapsedMs, long durationMs) {
        if ("Shoot".equals(event)) {
            return gateOpenMs >= AutoConstants.Autonomous.SHOOT_OPEN_TIME || elapsedMs >= durationMs;
        }
        return elapsedMs >= durationMs;
    }

    public void stop() {
        shooter.setAimPose(null);
        shooting = false;
        intaking = false;
        shooter.setForceZone(false);
        intake.update(false, false);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Shooting", shooting + (shooting ? " (gate " + (shooter.shooting() ? "open" : "waiting") + ")" : ""));
        telemetry.addData("Intake", intaking);
        telemetry.addData("Scoopula", scoopDown ? "down" : "up");
        telemetry.addData("Shooter state", shooter.stateName() + (shooter.inZone() ? " in-zone" : "")
                + (shooter.pollenAimed() ? " pollen-aimed" : "") + (shooter.nectarAimed() ? " nectar-aimed" : ""));
        telemetry.addData("Pollen wheel (ticks/s)", Math.round(shooter.pollenVelocity()) + " / "
                + Math.round(shooter.pollenCommand()));
    }

    /** One line for logs: event flags plus the shooter's state (Auto Dry Run diffs this between loops). */
    public String stateLine() {
        return "shoot=" + (shooting ? 1 : 0) + " intake=" + (intaking ? 1 : 0) + " scoop=" + (scoopDown ? "down" : "up")
                + " shooter=" + shooter.stateName() + (shooter.inZone() ? "/zone" : "")
                + (shooter.pollenAimed() ? "/aimP" : "") + (shooter.nectarAimed() ? "/aimN" : "")
                + (preAiming ? " preaim" : "") + (shooting ? " open=" + Math.round(gateOpenMs / 100) * 100 : "");
    }

    public double pollenVelocity() {
        return shooter.pollenVelocity();
    }
}
