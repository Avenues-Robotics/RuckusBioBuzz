package org.firstinspires.ftc.teamcode.tele;

import org.firstinspires.ftc.teamcode.Config;
import org.firstinspires.ftc.teamcode.SystemConstants;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.drivetrains.Swerve;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.systems.Intake;

import java.util.List;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.helpers.AimOffsets;

import org.firstinspires.ftc.teamcode.fieldview.FieldView;
import org.firstinspires.ftc.teamcode.helpers.Alliance;
import org.firstinspires.ftc.teamcode.helpers.PinpointGuard;
import org.firstinspires.ftc.teamcode.helpers.PoseStorage;
import org.firstinspires.ftc.teamcode.modules.Cache;
import org.firstinspires.ftc.teamcode.modules.PredictiveAiming;
import org.firstinspires.ftc.teamcode.systems.Scoopula;
import org.firstinspires.ftc.teamcode.systems.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Competition TeleOp. Controls (2026-11-15):
 * <pre>
 * DRIVER 1 (gamepad1)
 *   left stick          drive - robot-centric, or field-centric with SystemConstants.TeleOp.FIELD_CENTRIC
 *   OPTIONS / START     field-centric: re-zero "forward" to where the robot faces now (point it away from you)
 *   right stick x       turn - in SHOOTER LOCK the robot turns itself to the goal; the stick still overrides.
 *                       Centred while driving: the robot HOLDS its heading (SystemConstants.TeleOp.HEADING_HOLD)
 *   right bumper        intake (hold)            left bumper   reject / outtake (hold)
 *   right trigger       shoot now (hold)         in the zone the robot also shoots by itself (not in lock)
 *   left trigger        scoopula (hold)
 *   SHARE / BACK        press: turret parks (clear of the back); back into the alliance's corner facing up the field,
 *                       left side on the alliance wall, back on the end wall (red: audience end, blue: far end);
 *                       release after 0.5 s: pose reset (SystemConstants.TeleOp.RELOCALIZE_*); turret resumes 2 s later
 * DRIVER 2 (gamepad2) - aiming only
 *   d-pad left / right  aim point one step left / right, as the coach sees it from the last shot ("2 RIGHT")
 *   d-pad up / down     flywheel speed one step up / down (offset on the speed curve)
 *   OPTIONS / START     SHOOTER LOCK on / off: turret straight ahead, driver 1's robot heading aims, no zone
 *   SHARE / BACK        clear both offsets
 * INIT: driver 1 X = blue, B = red.
 * </pre>
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "TeleOp")
public class TeleOp extends OpMode {

    private Follower follower;
    private PredictiveAiming predictor;
    private Shooter shooterSystem;
    private List<LynxModule> hubs;
    private Alliance alliance = Alliance.RED;
    private Intake intake;
    private Scoopula scoopula;
    private final double[] dtMs = new double[256];
    private int dtCount;
    private long lastLoopNs;
    private final Cache telemetryCache = new Cache();
    private MultipleTelemetry out;
    private PinpointGuard pinpointGuard;
    private Pose autoEndPose;
    private boolean frozenWarned;
    private final AimOffsets offsets = new AimOffsets();
    private boolean lockForward;
    private double aimErrorRad = Double.NaN;
    /** Heading hold target (rad), NaN while not holding. See SystemConstants.TeleOp.HEADING_HOLD. */
    private double holdTargetRad = Double.NaN;
    private final ElapsedTime sinceTurnStick = new ElapsedTime();
    private final ElapsedTime relocalizeHeld = new ElapsedTime();
    private final ElapsedTime sinceRelocalizeRelease = new ElapsedTime();
    private boolean turretParked;
    private boolean relocalizeHeldPrev;
    private boolean relocalizeDone;
    private String relocalizeNote = "";
    private boolean prevLeft, prevRight, prevUp, prevDown, prevOptions, prevShare;
    /** Field-centric re-zero (driver 1 OPTIONS): added to the alliance's drivers' heading. Cleared by relocalize. */
    private double fieldCentricOffset;
    private boolean prevD1Options;
    /** The drive command handed to follower.manual this loop (after the hold / lock / field-centric stages). */
    private double lastForward, lastLeft, lastTurn;

    @Override
    public void init() {
        out = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        follower = Constants.createFollower(hardwareMap);
        autoEndPose = PoseStorage.take();
        if (autoEndPose != null) {
            follower.setPose(autoEndPose);
        }
        // Pinpoint heading has frozen after app restarts; reset position + IMU while the robot stands still at
        // init, then put the auto's pose back once the reset is done (helpers/PinpointGuard).
        pinpointGuard = new PinpointGuard(hardwareMap);
        pinpointGuard.begin();
        predictor = new PredictiveAiming(follower);
        scoopula = new Scoopula(hardwareMap);
        // Shooter and Intake are alliance-dependent: built in start(), after init_loop's X/B pick.
    }

    @Override
    public void init_loop() {
        if (gamepad1.x) alliance = Alliance.BLUE;
        if (gamepad1.b) alliance = Alliance.RED;
        if (pinpointGuard.update()) {
            restorePose();
        }
        out.addData("pinpoint", pinpointGuard.status());
        out.addData("alliance (X blue, B red)", alliance);
        out.addData("D1", "LT scoopula, RT shoot, hold SHARE = relocalize (corner, facing your wall), OPTIONS = "
                + "field-centric re-zero (" + (SystemConstants.TeleOp.FIELD_CENTRIC ? "field" : "robot") + "-centric now)");
        out.addData("D2", "d-pad aim L/R + speed U/D, OPTIONS shooter lock, SHARE clear");
        out.update();
    }

    @Override
    public void start() {
        if (pinpointGuard.finish()) {
            restorePose();
        }
        if (mechanismsEnabled()) {
            shooterSystem = new Shooter(hardwareMap, predictor, alliance);
            intake = new Intake(hardwareMap, alliance);
        }
    }

    @Override
    public void loop() {
        long now = System.nanoTime();
        if (lastLoopNs != 0) {
            dtMs[dtCount++ % dtMs.length] = (now - lastLoopNs) / 1e6;
        }
        lastLoopNs = now;
        Pose pose = follower.pose();
        if (Config.telemetryEnabled()) {
            FieldView.publish(pose, predictor.predictPose());
        }
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        if (mechanismsEnabled()) {
            driver2();
        }
        relocalize(pose);

        Swerve.setTranslationPriority(SystemConstants.TeleOp.TRANSLATION_PRIORITY);
        Gamepad sticks = driveSticks();
        double turn = -sticks.right_stick_x;
        double forward = -sticks.left_stick_y;
        double left = -sticks.left_stick_x;
        aimErrorRad = Double.NaN;
        if (lockForward && Math.abs(sticks.right_stick_x) < SystemConstants.TeleOp.AIM_STICK_OVERRIDE) {
            turn = aimTurn(pose);
            holdTargetRad = Double.NaN;
        } else {
            turn = headingHoldTurn(pose, forward, left, turn);
        }
        boolean d1Options = gamepad1.options || gamepad1.start;
        if (d1Options && !prevD1Options) {
            fieldCentricOffset = AngleUnit.normalizeRadians(pose.heading() - driversHeading(0));
            gamepad1.rumbleBlips(1);
        }
        prevD1Options = d1Options;
        if (SystemConstants.TeleOp.FIELD_CENTRIC) {
            // The stick is in the drivers' frame; turn it into the robot's: rotate by (drivers' heading - robot's).
            double a = driversHeading(fieldCentricOffset) - pose.heading();
            double c = Math.cos(a), s = Math.sin(a);
            double rf = forward * c - left * s;
            double rl = forward * s + left * c;
            forward = rf;
            left = rl;
        }
        lastForward = forward;
        lastLeft = left;
        lastTurn = turn;
        follower.manual(forward, left, turn);
        follower.update();
        // Heading not moving by a single bit while turning hard = frozen Pinpoint: rumble once (works with
        // telemetry off) and say so below. Recovery is a restart of TeleOp with the robot still.
        boolean frozen = pinpointGuard.frozenWhileTurning(pose.heading(), turn);
        if (frozen && !frozenWarned) {
            gamepad1.rumble(600);
        }
        frozenWarned = frozen;
        if (mechanismsEnabled()) {
            updateMechanisms();
        }

        if (!Config.telemetryEnabled() || !telemetryCache.due(SystemConstants.Cache.TELEMETRY_INTERVAL_MS)) {
            return;
        }
        addTelemetry(pose, frozen);
    }

    private void updateMechanisms() {
        shooterSystem.setLockForward(lockForward);
        shooterSystem.setZoneAutoShoot(!lockForward);
        shooterSystem.setAimOffsets(offsets.goalDx(), offsets.goalDy(), offsets.speedOffset());
        // Right trigger held = shoot now: gate open, flywheel at full target, intake feeding (below).
        shooterSystem.setManualShoot(gamepad1.right_trigger > 0.5);
        // Shooter first, so the intake follows this loop's SHOOT decision, not last loop's.
        shooterSystem.update();
        if(shooterSystem.shooting()){
            intake.update(false, true); //just sends intake command that's overridable
        }else {
            intake.update(gamepad1.left_bumper, gamepad1.right_bumper);
        }
        scoopula.update(gamepad1.left_trigger > 0.5);
    }

    private void addTelemetry(Pose pose, boolean frozen) {
        if (frozen) {
            out.addData("WARNING", "Pinpoint heading FROZEN - aiming and odometry are wrong");
        }
        out.addData("alliance", alliance);
        out.addData("drive", SystemConstants.TeleOp.FIELD_CENTRIC
                ? String.format(java.util.Locale.US, "FIELD-centric (re-zero %+.0f deg, D1 OPTIONS)",
                        Math.toDegrees(fieldCentricOffset))
                : "robot-centric");
        out.addData("mode", lockForward
                ? "SHOOTER LOCK - turret forward, heading aims, no zone (D2 OPTIONS)"
                : "turret tracking, auto-shoot in zone");
        out.addData("heading hold", !SystemConstants.TeleOp.HEADING_HOLD ? "OFF"
                : Double.isNaN(holdTargetRad) ? "idle" : String.format(java.util.Locale.US, "holding %.1f deg",
                        Math.toDegrees(holdTargetRad)));
        if (lockForward && !Double.isNaN(aimErrorRad)) {
            out.addData("aim error (deg)", Math.round(Math.toDegrees(aimErrorRad) * 10) / 10.0);
        }
        out.addData("aim offset", String.format(java.util.Locale.US, "%+d %s (%.1f, %.1f in)  speed %+d (%+.0f t/s)",
                Math.abs(offsets.lateralClicks()), offsets.lateralClicks() >= 0 ? "R" : "L",
                offsets.goalDx(), offsets.goalDy(), offsets.speedClicks(), offsets.speedOffset()));
        if (!relocalizeNote.isEmpty()) {
            out.addData("relocalize", relocalizeNote);
        }
        out.addData("x", pose.x());
        out.addData("y", pose.y());
        out.addData("heading (deg)", Math.toDegrees(pose.heading()));
        if (shooterSystem != null) {
            out.addData("in zone", shooterSystem.inZone());
            out.addData("ready", shooterSystem.ready());
            out.addData("pollen wheel (ticks/s)", Math.round(shooterSystem.pollenVelocity()) + " / " + Math.round(shooterSystem.pollenCommand()));
            out.addData("F volt scale", Math.round(shooterSystem.flywheelFScale() * 1000) / 1000.0);
        }
        addLoopStats();
        out.update();
    }

    /**
     * Where the drive sticks (left x/y, right x) are read from. Driver 1's gamepad here;
     * diagnostics/swerve/TeleDriveProbe feeds laptop-scripted sticks through the same drive code.
     */
    protected Gamepad driveSticks() {
        return gamepad1;
    }

    /** False skips the shooter, intake, scoopula and driver 2 (TeleDriveProbe: drivetrain only). */
    protected boolean mechanismsEnabled() {
        return true;
    }

    protected Follower drivetrainFollower() {
        return follower;
    }

    /** Heading-hold target (rad), NaN while not holding. */
    protected double headingHoldTargetRad() {
        return holdTargetRad;
    }

    /** {forward, left, turn} handed to follower.manual on the last loop. */
    protected double[] lastDriveCommand() {
        return new double[] {lastForward, lastLeft, lastTurn};
    }

    /** Driver 2: aim offsets on the d-pad, shooter lock on OPTIONS, clear on SHARE. Rising edges only. */
    private void driver2() {
        boolean left = gamepad2.dpad_left, right = gamepad2.dpad_right;
        boolean up = gamepad2.dpad_up, down = gamepad2.dpad_down;
        boolean options = gamepad2.options || gamepad2.start;
        boolean share = gamepad2.share || gamepad2.back;
        Pose now = follower.pose();
        if (left && !prevLeft) offsets.nudgeLateral(-1, shooterSystem.lastShotPose(), now, alliance);
        if (right && !prevRight) offsets.nudgeLateral(1, shooterSystem.lastShotPose(), now, alliance);
        if (up && !prevUp) offsets.nudgeSpeed(1);
        if (down && !prevDown) offsets.nudgeSpeed(-1);
        if (share && !prevShare) offsets.reset();
        if (options && !prevOptions) {
            lockForward = !lockForward;
            // Both drivers feel it: two blips = lock on, one = off.
            gamepad1.rumbleBlips(lockForward ? 2 : 1);
            gamepad2.rumbleBlips(lockForward ? 2 : 1);
        }
        prevLeft = left;
        prevRight = right;
        prevUp = up;
        prevDown = down;
        prevOptions = options;
        prevShare = share;
    }

    /**
     * Shooter lock: turn so the (straight-ahead) shooter faces the goal. The pollen turret sits on the robot's
     * centre line, so pointing the heading from the robot centre at the goal lines the shot up. Aimed from the
     * lead pose (PredictiveAiming), so it leads a moving robot like the turret does. Controller: Foresight's
     * heading shape - P on the heading projected ahead by the braking lookahead - so it slows before it arrives.
     */
    private double aimTurn(Pose pose) {
        Pose lead = predictor.leadPose();
        Pose goal = shooterSystem.goal(lead);
        double target = Math.atan2(goal.y() - lead.y(), goal.x() - lead.x());
        double w = follower.velocity().omega;
        double projected = pose.heading() + w * SystemConstants.TeleOp.AIM_BRAKE_LINEAR
                + w * Math.abs(w) * SystemConstants.TeleOp.AIM_BRAKE_QUADRATIC;
        aimErrorRad = AngleUnit.normalizeRadians(target - pose.heading());
        double err = AngleUnit.normalizeRadians(target - projected);
        double max = SystemConstants.TeleOp.AIM_MAX_TURN;
        return Math.max(-max, Math.min(max, SystemConstants.TeleOp.AIM_KP * err));
    }

    /**
     * Heading hold while driving (SystemConstants.TeleOp.HEADING_HOLD): returns the turn command - the stick's own
     * when the hold is not active. Robot-frame vs field-frame sticks do not matter: it only looks at magnitude.
     */
    private double headingHoldTurn(Pose pose, double forward, double left, double stickTurn) {
        if (Math.abs(stickTurn) >= SystemConstants.TeleOp.HOLD_STICK_DEADBAND) {
            sinceTurnStick.reset();
        }
        boolean translating = Math.hypot(forward, left) > SystemConstants.TeleOp.HOLD_DRIVE_DEADBAND;
        if (!SystemConstants.TeleOp.HEADING_HOLD || !translating
                || Math.abs(stickTurn) >= SystemConstants.TeleOp.HOLD_STICK_DEADBAND) {
            holdTargetRad = Double.NaN;
            return stickTurn;
        }
        double w = follower.velocity().omega;
        if (Double.isNaN(holdTargetRad)) {
            if (sinceTurnStick.seconds() < SystemConstants.TeleOp.HOLD_CAPTURE_S
                    && Math.abs(w) > Math.toRadians(SystemConstants.TeleOp.HOLD_CAPTURE_RATE_DEG_S)) {
                return stickTurn;   // still coasting out of a stick turn: let it finish, then hold where it ends
            }
            holdTargetRad = pose.heading();
        }
        double projected = pose.heading() + w * SystemConstants.TeleOp.AIM_BRAKE_LINEAR
                + w * Math.abs(w) * SystemConstants.TeleOp.AIM_BRAKE_QUADRATIC;
        double err = AngleUnit.normalizeRadians(holdTargetRad - projected);
        double max = SystemConstants.TeleOp.HOLD_MAX_TURN;
        return Math.max(-max, Math.min(max, SystemConstants.TeleOp.HOLD_KP * err));
    }

    /**
     * Driver 1 relocalize, on BACK (SHARE on a PS pad - the SDK reports both from one button):
     * press -> the turret parks at Turret.RESET_POSE so it does not stick out past the back; back into the corner;
     * release after at least RELOCALIZE_HOLD_S -> the pose is set to the spot; the turret resumes tracking
     * RESET_TIMER_SEC after the release (time to drive off the wall). A shorter press only parks and resumes.
     * Until 2026-11-23 the park / resume calls were swapped: the turret sat parked except for 2 s after a relocalize.
     */
    private void relocalize(Pose pose) {
        boolean held = gamepad1.back;
        boolean released = !held && relocalizeHeldPrev;
        if (held && !relocalizeHeldPrev) {
            relocalizeHeld.reset();
            relocalizeDone = false;
            turretParked = true;
            shooterSystem.setTurretParked(true);
        }
        if (released) {
            sinceRelocalizeRelease.reset();
        }

        if (released && !relocalizeDone && relocalizeHeld.seconds() >= SystemConstants.TeleOp.RELOCALIZE_HOLD_S) {
            Pose spot = relocalizePose();
            follower.setPose(spot);
            fieldCentricOffset = 0;   // the heading is exact again: the drivers' frame needs no re-zero
            holdTargetRad = Double.NaN;   // the heading jumped: never hold a target from the old frame
            relocalizeDone = true;
            gamepad1.rumble(400);
            relocalizeNote = String.format(java.util.Locale.US, "set to (%.1f, %.1f, %.0f deg), was (%.1f, %.1f, %.0f deg)",
                    spot.x(), spot.y(), Math.toDegrees(spot.heading()), pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        }
        if (turretParked && !held && sinceRelocalizeRelease.seconds() >= SystemConstants.TeleOp.RESET_TIMER_SEC) {
            turretParked = false;
            shooterSystem.setTurretParked(false);
        }
        relocalizeHeldPrev = held;
    }

    /**
     * The field heading that is "stick forward" for this alliance's drivers: away from their wall - red stands at
     * x = 0 looking toward +x (0 rad), blue at the far x wall looking toward -x (pi) - plus the re-zero offset.
     */
    private double driversHeading(double offset) {
        return (alliance == Alliance.BLUE ? Math.PI : 0) + offset;
    }

    /** The relocalize spot: SystemConstants.TeleOp.RELOCALIZE_* in the red frame, rotated 180 deg for blue. */
    private Pose relocalizePose() {
        double x = SystemConstants.TeleOp.RELOCALIZE_X;
        double y = SystemConstants.TeleOp.RELOCALIZE_Y;
        double h = Math.toRadians(SystemConstants.TeleOp.RELOCALIZE_HEADING_DEG);
        if (alliance == Alliance.BLUE) {
            double field = SystemConstants.Zone.FIELD_SIZE;
            return new Pose(field - x, field - y, AngleUnit.normalizeRadians(h + Math.PI));
        }
        return new Pose(x, y, AngleUnit.normalizeRadians(h));
    }

    /** After the init IMU reset: the auto's pose if there was one, else the reset's 0,0,0 stands. */
    private void restorePose() {
        if (autoEndPose != null) {
            follower.setPose(autoEndPose);
        }
    }

    private void addLoopStats() {
        int n = Math.min(dtCount, dtMs.length);
        if (n == 0) {
            return;
        }
        double[] sorted = java.util.Arrays.copyOf(dtMs, n);
        double sum = 0;
        for (double d : sorted) {
            sum += d;
        }
        java.util.Arrays.sort(sorted);
        double mean = sum / n;
        double p90 = sorted[Math.min(n - 1, (int) Math.ceil(0.9 * n) - 1)];
        out.addData("loop Hz (1/mean dt)", String.valueOf(Math.round(1000.0 / mean)));
        out.addData("loop dt mean / p90 ms", Math.round(mean * 10) / 10.0 + " / " + Math.round(p90 * 10) / 10.0);
        out.addData("loop samples", n);
    }

    @Override
    public void stop() {
        follower.stop();
        follower.update();
        Swerve.setTranslationPriority(0);
    }
}
