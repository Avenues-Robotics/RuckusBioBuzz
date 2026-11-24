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

    public AutoMechanisms(HardwareMap hardwareMap, Follower follower, Alliance alliance) {
        this.follower = follower;
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
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
        shooter.update();
        if (shooting && shooter.shooting()) {
            gateOpenMs += dtMs;
        }
        if (shooting) {
            intake.update(false, shooter.shooting());
        } else {
            intake.update(false, intaking);
        }
        scoopula.update(scoopDown);
        if (trace != null) {
            traceLoop(dtMs / 1000.0);
        }
    }

    private void traceLoop(double dt) {
        if (dt <= 0) {
            trace.startRecording("auto");   // first loop = START
            return;
        }
        loopHzEma = loopHzEma == 0 ? 1 / dt : 0.9 * loopHzEma + 0.1 / dt;
        com.pedropathing.math.Pose pose = follower.pose();
        double target = Double.NaN;
        if (follower.following()) {
            com.pedropathing.math.Pose closest = follower.closestPose();
            if (closest != null) target = Math.toDegrees(closest.heading());
        }
        // No drivetrain.debug(): it reads every drive motor's power off the hub (4 Lynx reads a loop) and would slow
        // the loop being measured. The follower's command is recoverable offline (Foresight heading = kP x error).
        double[] drive = new double[org.firstinspires.ftc.teamcode.diagnostics.swerve.PodRecorder.DRIVE_COLS];
        for (int i = 0; i < 4; i++) {
            com.pedropathing.revhub.drivetrains.CoaxialPod pod = org.firstinspires.ftc.teamcode.pedroPathing.Constants.builtPods[i];
            drive[i] = pod == null ? Double.NaN : pod.getLastDrivePower();
        }
        drive[4] = drive[5] = drive[6] = Double.NaN;
        com.pedropathing.math.Twist tw = follower.twist();
        drive[7] = tw.vx;      // body forward, in/s
        drive[8] = tw.vy;      // body left, in/s
        drive[9] = tw.omega;   // rad/s
        trace.update(dt, loopHzEma, Math.toDegrees(pose.heading()), target, pose.x(), pose.y(),
                Double.NaN, Double.NaN, Double.NaN, drive);
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
