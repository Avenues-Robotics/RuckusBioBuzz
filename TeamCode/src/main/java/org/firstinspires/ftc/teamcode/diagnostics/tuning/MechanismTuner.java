package org.firstinspires.ftc.teamcode.diagnostics.tuning;

import org.firstinspires.ftc.teamcode.SystemConstants;

import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.helpers.Alliance;
import org.firstinspires.ftc.teamcode.helpers.HivePosition;
import org.firstinspires.ftc.teamcode.modules.ShootingRegression;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.systems.ColorCheck;
import org.firstinspires.ftc.teamcode.systems.Flywheel;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Shooter;
import org.firstinspires.ftc.teamcode.systems.Turret;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Mechanism tuning OpMode behind {@code http://192.168.43.1:8080/tune}. DIAGNOSTIC ONLY - never
 * ships.
 *
 * <p>Covers everything that is not the drivetrain: both flywheels (hub velocity PIDF, step
 * response, shot / synthetic-disturbance recovery, feed-forward sweep, idle fraction), the
 * shooting sequence ({@code shooter.java}'s gate logic: entry band, hold band, minimum open time,
 * fed by the shipped intake), both turret servos (center, direction, gear ratio, mount offset),
 * the gate and scoopula servo positions, and the intake (color thresholds with labelled samples,
 * reverse brake, per-motor direction, live reject check through the shipped {@link Intake} class). The drivetrain is
 * never touched: swerve tuning stays in Swerve Bring-Up.
 *
 * <p>Starting values are read from the shipped statics at init, and anything changed here is
 * exported as declarations for the shipped files; shipped statics the tool has to write so the
 * shipped intake logic sees them are restored in {@link #stop()}.
 *
 * <p>Safety: nothing moves before START, flywheel targets are capped at {@link #VEL_CAP_TPS}, and
 * every actuator stops if no web client has read state or sent a command for
 * {@link #CLIENT_TIMEOUT_MS}.
 */
public class MechanismTuner extends OpMode {

    // ------------------------------------------------------------------------- limits
    /** Bare 6000 rpm motor with a 28 cpr encoder tops out near 2800 ticks/s. */
    static final double VEL_CAP_TPS = 3000;
    static final long CLIENT_TIMEOUT_MS = 30_000;
    static final long PUBLISH_INTERVAL_NS = 50_000_000L;
    static final long SLOW_INTERVAL_NS = 200_000_000L;
    static final long COLOR_INTERVAL_NS = 50_000_000L;

    // Dip detector: a wheel must sit in its at-speed band this long before a drop counts as a shot.
    static final long ARM_NS = 300_000_000L;
    static final long DIP_TIMEOUT_NS = 3_000_000_000L;
    static final long POST_WINDOW_NS = 300_000_000L;
    static final int MAX_EVENTS = 60;

    // Shoot sequence: give up if the wheels never reach the entry band.
    static final long SHOOT_READY_TIMEOUT_NS = 8_000_000_000L;
    static final int MAX_SHOOT_WINDOWS = 30;

    static final int MODE_OFF = 0, MODE_VEL = 1, MODE_POWER = 2;
    static final String[] MODE_NAMES = {"OFF", "VEL", "POWER"};

    private List<LynxModule> hubs;
    private VoltageSensor voltageSensor;
    private double volts;

    private final Wheel[] wheels = new Wheel[2];
    private final TurretCal[] turrets = new TurretCal[2];
    private final ServoCal[] servos = new ServoCal[2];

    private Intake intakeSys;
    private String intakeError = "";
    private String intakeMode = "OFF";

    private final DcMotorEx[] intakeMotors = new DcMotorEx[2];
    private boolean breakIn, breakInResting;
    private double breakInPower = 0.6, breakInFlipS = 30, breakInRestS = 1, breakInTotalS = 600;
    private double breakInStallA = 9, breakInStallS = 1;
    private int breakInDir = 1, breakInFlips;
    private long breakInLastNs, breakInPhaseNs, breakInStallSinceNs;
    private double breakInRunS;
    private String breakInEnd = "";
    private final double[] intakeAmps = new double[2];
    private final double[] intakeMaxAmps = new double[2];
    // Intake direction tab: run LEFT, RIGHT or BOTH motors open loop (direction flags applied) to
    // see which roller pulls the wrong way. "" = off.
    private String intakeTest = "";
    private double intakeTestPower = 0.4;
    private boolean origIntakeRev, origIntakeLeftRev, origIntakeRightRev;

    private int[][] origMapPollen, origMapNectar;
    private int origXQuality, origYQuality, origOffPollen, origOffNectar;
    private Alliance alliance = Alliance.RED;

    // Shooting regression (Regression tab). The Pinpoint is read for the pose only - the drivetrain
    // is never built. Its pose is NOT reset here (Constants.pinpointConfig resetMode NONE), so it
    // carries whatever the last OpMode left, until the operator sets one.
    private PinpointLocalizer localizer;
    private String poseError = "";
    private Pose pose;
    private String hive = "AUTO";   // AUTO (shipped rule: by robot y) | UP | DOWN
    private boolean regAim;
    private final boolean[] aimReachable = {true, true};
    private final RegressionSession reg = new RegressionSession();

    private ColorSensor color;
    private DistanceSensor colorDistance;
    private boolean colorStream = true;
    private int cr, cg, cb, ca;
    private double cdist = Double.NaN;
    private String sampleLabel = "";
    private int sampleRemaining;
    private final List<String> sampleLabels = new ArrayList<>();
    private final List<int[]> samples = new ArrayList<>();
    private final List<Double> sampleDist = new ArrayList<>();
    private String colorSummary = "{}";

    private double atSpeedFrac;
    // Tool copies of the shooter statics; the tool runs its own copy of the SHOOT logic with them.
    private double holdFrac, minGateOpenS, idleFrac;
    private long sessionId;
    private final FlyRecorder recorder = new FlyRecorder();
    private boolean recCurrent;
    private final List<String> events = new ArrayList<>();
    private int eventSeq;

    private boolean started;
    private String message = "Init. Nothing moves until START.";
    private long cmdSeq;
    private boolean watchdogTripped;

    private long lastLoopNs, lastPublishNs, lastSlowNs, lastColorNs, lastTelemetryNs;
    private final double[] dtRing = new double[128];
    private int dtCount;

    // Shipped statics the tool writes so the shipped intake class runs with them; restored on stop.
    private double origRedOverBlue, origBlueOverRed, origMaxDist, origBrakeMs;

    /** Export values as they stood at init (the shipped statics), keyed "file#name". */
    private final Map<String, String> shippedValues = new LinkedHashMap<>();

    // Shoot sequence (mirrors shooter.java TRACK/SHOOT without aiming or zone): 0 off, 1 TRACK
    // (waiting for both wheels in the entry band), 2 SHOOT (gate open, intake feeding).
    private int shootState;
    private int shootLeft;
    private boolean shootFeed, shootOwnsRec, shootHeldByTime;
    private long shootPhaseNs, shootOpenNs;
    private double shootReadyMs, shootWorstP, shootWorstN;
    private String shootEnd = "";
    private int shootSeq;
    private final List<String> shootWindows = new ArrayList<>();

    // ------------------------------------------------------------------------- lifecycle

    @Override
    public void init() {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        voltageSensor = hardwareMap.voltageSensor.iterator().hasNext()
                ? hardwareMap.voltageSensor.iterator().next() : null;

        sessionId = System.currentTimeMillis();
        atSpeedFrac = SystemConstants.Flywheel.AT_SPEED_FRACTION;
        idleFrac = SystemConstants.Flywheel.IDLE_FRACTION;
        holdFrac = SystemConstants.Shooter.HOLD_SPEED_FRACTION;
        minGateOpenS = SystemConstants.Shooter.MIN_GATE_OPEN_S;
        wheels[0] = new Wheel("pollen", "pollenTurret", SystemConstants.Flywheel.POLLEN_FLYWHEEL_REVERSED,
                SystemConstants.Flywheel.POLLEN_PIDF);
        wheels[1] = new Wheel("nectar", "nectarTurret", SystemConstants.Flywheel.NECTAR_FLYWHEEL_REVERSED,
                SystemConstants.Flywheel.NECTAR_PIDF);

        turrets[0] = new TurretCal("pollen", "pollenTurretServo", SystemConstants.Turret.POLLEN_TURRET_CENTER,
                SystemConstants.Turret.POLLEN_TURRET_REVERSED, SystemConstants.Turret.POLLEN_TURRET_GEAR_RATIO,
                SystemConstants.Turret.pollenTurretOffset, SystemConstants.Turret.POLLEN_TURRET_MIN, SystemConstants.Turret.POLLEN_TURRET_MAX,
                SystemConstants.Turret.POLLEN_TURRET_START);
        turrets[1] = new TurretCal("nectar", "nectarTurretServo", SystemConstants.Turret.NECTAR_TURRET_CENTER,
                SystemConstants.Turret.NECTAR_TURRET_REVERSED, SystemConstants.Turret.NECTAR_TURRET_GEAR_RATIO,
                SystemConstants.Turret.nectarTurretOffset, SystemConstants.Turret.NECTAR_TURRET_MIN, SystemConstants.Turret.NECTAR_TURRET_MAX,
                SystemConstants.Turret.NECTAR_TURRET_START);

        servos[0] = new ServoCal("gate", "turretGate", "gate",
                new String[]{"open", "openPos", "close", "closePos"});
        servos[1] = new ServoCal("scoopula", "scoopula", "scoopula",
                new String[]{"scoop", "scoopPos", "unscoop", "unscoopPos"});

        origRedOverBlue = SystemConstants.ColorCheck.RED_OVER_BLUE;
        origBlueOverRed = SystemConstants.ColorCheck.BLUE_OVER_RED;
        origMaxDist = SystemConstants.ColorCheck.MAX_DIST_CM;
        origBrakeMs = SystemConstants.Intake.REVERSE_BRAKE_MS;
        origIntakeRev = SystemConstants.Intake.INTAKE_REVERSED;
        origIntakeLeftRev = SystemConstants.Intake.INTAKE_LEFT_REVERSED;
        origIntakeRightRev = SystemConstants.Intake.INTAKE_RIGHT_REVERSED;
        origMapPollen = copyMap(SystemConstants.Zone.mapPollen);
        origMapNectar = copyMap(SystemConstants.Zone.mapNectar);
        origXQuality = SystemConstants.Zone.xQuality;
        origYQuality = SystemConstants.Zone.yQuality;
        origOffPollen = SystemConstants.Zone.yOffsetPollen;
        origOffNectar = SystemConstants.Zone.yOffsetNectar;
        try {
            intakeSys = new Intake(hardwareMap, alliance);
            intakeMotors[0] = hardwareMap.tryGet(DcMotorEx.class, "intakeLeft");
            intakeMotors[1] = hardwareMap.tryGet(DcMotorEx.class, "intakeRight");
        } catch (RuntimeException e) {
            intakeSys = null;
            intakeError = shortError(e);
        }
        color = hardwareMap.tryGet(ColorSensor.class, "colorSensor");
        colorDistance = hardwareMap.tryGet(DistanceSensor.class, "colorSensor");
        try {
            localizer = Constants.createLocalizer(hardwareMap);
            readPose();
        } catch (RuntimeException e) {
            localizer = null;
            poseError = shortError(e);
        }
        reg.load();

        TuningHub.INSTANCE.clearCommands();
        TuningHub.INSTANCE.setRecorder(recorder);
        TuningHub.INSTANCE.setRegression(reg);
        lastLoopNs = System.nanoTime();
        rebuildColorSummary();
        for (String[] e : exportEntries()) {
            shippedValues.put(e[0] + "#" + e[1], e[2]);
        }
        // A fit saved in an earlier run reloads from the hub file; "in the code" is still the class's own value.
        shippedValues.put("regression#POLLEN_COEFFS", reg.shippedCoeffSource(0));
        shippedValues.put("regression#POLLEN_RANGE", reg.shippedRangeSource(0));
        shippedValues.put("regression#NECTAR_COEFFS", reg.shippedCoeffSource(1));
        shippedValues.put("regression#NECTAR_RANGE", reg.shippedRangeSource(1));
    }

    @Override
    public void init_loop() {
        service();
    }

    @Override
    public void start() {
        started = true;
        message = "Started. Mechanisms can move now.";
    }

    @Override
    public void loop() {
        service();
    }

    @Override
    public void stop() {
        stopAll();
        recorder.stop();
        SystemConstants.ColorCheck.RED_OVER_BLUE = origRedOverBlue;
        SystemConstants.ColorCheck.BLUE_OVER_RED = origBlueOverRed;
        SystemConstants.ColorCheck.MAX_DIST_CM = origMaxDist;
        SystemConstants.Intake.REVERSE_BRAKE_MS = origBrakeMs;
        SystemConstants.Intake.INTAKE_REVERSED = origIntakeRev;
        SystemConstants.Intake.INTAKE_LEFT_REVERSED = origIntakeLeftRev;
        SystemConstants.Intake.INTAKE_RIGHT_REVERSED = origIntakeRightRev;
        SystemConstants.Zone.mapPollen = origMapPollen;
        SystemConstants.Zone.mapNectar = origMapNectar;
        SystemConstants.Zone.xQuality = origXQuality;
        SystemConstants.Zone.yQuality = origYQuality;
        SystemConstants.Zone.yOffsetPollen = origOffPollen;
        SystemConstants.Zone.yOffsetNectar = origOffNectar;
        TuningHub.INSTANCE.markStopped();
    }

    // ------------------------------------------------------------------------- loop

    private void service() {
        long now = System.nanoTime();
        double dt = (now - lastLoopNs) / 1e9;
        lastLoopNs = now;
        dtRing[dtCount++ % dtRing.length] = dt;

        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        boolean slow = now - lastSlowNs >= SLOW_INTERVAL_NS;
        if (slow) {
            lastSlowNs = now;
            readVolts();
        }
        for (Wheel w : wheels) {
            w.read(slow || recCurrent);
        }

        if (regAim || slow) {
            readPose();
        }

        Map<String, String> cmd;
        while ((cmd = TuningHub.INSTANCE.poll()) != null) {
            handle(cmd);
        }
        if (regAim) {
            aimTurrets();
        }

        if (breakIn) {
            updateBreakIn(now, slow);
        } else if (slow) {
            readIntakeAmps();
        }

        boolean anythingMoving = breakIn || !intakeTest.isEmpty() || (intakeSys != null && !"OFF".equals(intakeMode));
        for (Wheel w : wheels) {
            w.update(now);
            anythingMoving |= w.mode != MODE_OFF;
        }
        updateShoot(now);
        if (anythingMoving && TuningHub.INSTANCE.msSinceClient() > CLIENT_TIMEOUT_MS) {
            stopAll();
            watchdogTripped = true;
            message = "WATCHDOG: no web client for " + (CLIENT_TIMEOUT_MS / 1000)
                    + " s - everything stopped.";
        }

        if (breakIn) {
            intakeMode = "OFF";
        } else if (!intakeTest.isEmpty()) {
            // Re-applied every loop so a direction tick-box takes effect while the motors run.
            for (int i = 0; i < 2; i++) {
                if (intakeMotors[i] == null) continue;
                boolean on = "BOTH".equals(intakeTest) || ("LEFT".equals(intakeTest) ? i == 0 : "RIGHT".equals(intakeTest) && i == 1);
                intakeMotors[i].setPower(on ? intakeTestPower * intakeSign(i) : 0);
            }
        } else if (intakeSys != null && shootState != 0 && shootFeed) {
            // As TeleOp: the intake feeds while SHOOT holds, and idles (reject logic still live) otherwise.
            intakeSys.update(false, shootState == 2);
        } else if (intakeSys != null && !"OFF".equals(intakeMode)) {
            intakeSys.update("OUT".equals(intakeMode), "IN".equals(intakeMode));
        }

        if (color != null && (colorStream || sampleRemaining > 0) && !recorder.isRecording()
                && now - lastColorNs >= COLOR_INTERVAL_NS) {
            lastColorNs = now;
            readColor();
        }

        recorder.add(dt, wheels[0].target, wheels[0].vel, wheels[1].target, wheels[1].vel,
                recCurrent ? wheels[0].cur : Double.NaN, recCurrent ? wheels[1].cur : Double.NaN, volts, wheels[0].mode, wheels[1].mode);

        if (now - lastPublishNs >= PUBLISH_INTERVAL_NS) {
            lastPublishNs = now;
            TuningHub.INSTANCE.publish(stateJson());
        }
        if (now - lastTelemetryNs >= SLOW_INTERVAL_NS) {
            lastTelemetryNs = now;
            telemetryUpdate();
        }
    }

    private void readVolts() {
        if (voltageSensor == null) {
            return;
        }
        try {
            volts = voltageSensor.getVoltage();
        } catch (RuntimeException e) {
            volts = 0;
        }
    }

    private void readColor() {
        try {
            cr = color.red();
            cg = color.green();
            cb = color.blue();
            ca = color.alpha();
            cdist = colorDistance == null ? Double.NaN : colorDistance.getDistance(DistanceUnit.CM);
        } catch (RuntimeException e) {
            return;
        }
        if (sampleRemaining > 0) {
            sampleLabels.add(sampleLabel);
            samples.add(new int[]{cr, cg, cb, ca});
            sampleDist.add(cdist);
            if (--sampleRemaining == 0) {
                rebuildColorSummary();
                message = "Color: captured samples for " + sampleLabel + ".";
            }
        }
    }

    private void telemetryUpdate() {
        String open = TuningHub.INSTANCE.openNoteText();
        if (!open.isEmpty()) {
            telemetry.addLine(">>> " + open);
        }
        telemetry.addLine("Dashboard: http://192.168.43.1:8080/tune");
        telemetry.addData("State", started ? "RUNNING" : "INIT (press START to move)");
        telemetry.addData("Battery", FlyRecorder.r(volts, 2) + " V");
        for (Wheel w : wheels) {
            telemetry.addData(w.name, MODE_NAMES[w.mode] + "  tgt " + FlyRecorder.r(w.target, 0)
                    + "  vel " + FlyRecorder.r(w.vel, 0));
        }
        if (breakIn) {
            telemetry.addData("Intake break-in", (breakInResting ? "REST" : breakInDir > 0 ? "IN" : "OUT")
                    + "  " + FlyRecorder.r(breakInRunS, 0) + " / " + FlyRecorder.r(breakInTotalS, 0) + " s"
                    + "  " + FlyRecorder.r(intakeAmps[0], 2) + " / " + FlyRecorder.r(intakeAmps[1], 2) + " A");
        }
        if (shootState != 0) {
            telemetry.addData("Shoot", (shootState == 2 ? "SHOOT (gate open)" : "TRACK (waiting for speed)")
                    + "  windows left " + shootLeft);
        }
        if (regAim && pose != null) {
            telemetry.addData("Regression aim", alliance + " " + hiveFor().name + " hive  pollen "
                    + FlyRecorder.r(aim(turrets[0])[0], 1) + " in  nectar " + FlyRecorder.r(aim(turrets[1])[0], 1) + " in");
        }
        telemetry.addData("Message", message);
        telemetry.update();
    }

    // ------------------------------------------------------------------------- commands

    private static boolean isMotion(String action) {
        switch (action) {
            case "fly": case "flyPower": case "step": case "kick": case "ffSweep":
            case "turretPos": case "turretDeg": case "servoPos": case "gatePulse": case "intake": case "intakeTest":
            case "shoot": case "breakInStart": case "regAim": case "regSpinFit":
                return true;
            default:
                return false;
        }
    }

    /** Commands that take over a wheel, the gate or the intake, so a running shoot sequence ends first. */
    private static boolean endsShoot(String action) {
        switch (action) {
            case "flyPower": case "flyOff": case "step": case "ffSweep": case "servoPos":
            case "gatePulse": case "intake": case "intakeTest": case "shoot":
                return true;
            default:
                return false;
        }
    }

    private void handle(Map<String, String> cmd) {
        String action = cmd.get("action") == null ? "" : cmd.get("action");
        cmdSeq = (long) num(cmd, "_seq", cmdSeq);
        if (isMotion(action) && !started) {
            message = "REFUSED " + action + ": press START on the Driver Station first.";
            return;
        }
        if (isMotion(action)) {
            watchdogTripped = false;
        }
        if (breakIn && ("intake".equals(action) || "intakeTest".equals(action) || "shoot".equals(action)
                || "breakInStart".equals(action))) {
            endBreakIn("interrupted by " + action);
        }
        if (!intakeTest.isEmpty() && ("intake".equals(action) || "shoot".equals(action) || "breakInStart".equals(action))) {
            endIntakeTest();
        }
        if ("breakInStart".equals(action) && shootState != 0) {
            message = "REFUSED breakInStart: stop the shoot sequence first.";
            return;
        }
        if (shootState != 0 && endsShoot(action)) {
            endShoot("interrupted by " + action);
        }
        if (regAim && ("turretPos".equals(action) || "turretDeg".equals(action))) {
            regAim = false;
        }
        try {
            dispatch(action, cmd);
        } catch (RuntimeException e) {
            message = "ERROR in " + action + ": " + shortError(e);
        }
    }

    private void dispatch(String action, Map<String, String> c) {
        switch (action) {
            case "stopAll":
                stopAll();
                message = "Everything stopped.";
                break;

            // ---- flywheels
            case "fly":
                for (Wheel w : wheelsFor(c)) {
                    w.cancelTests();
                    w.setVelocity(num(c, "vel", 0));
                }
                message = "Flywheel velocity " + FlyRecorder.r(num(c, "vel", 0), 0) + " ticks/s.";
                break;
            case "flyPower":
                for (Wheel w : wheelsFor(c)) {
                    w.cancelTests();
                    w.setPower(num(c, "p", 0));
                }
                message = "Flywheel open-loop power " + num(c, "p", 0) + ".";
                break;
            case "flyOff":
                for (Wheel w : wheelsFor(c)) {
                    w.cancelTests();
                    w.off();
                }
                message = "Flywheel off (coasting).";
                break;
            case "flyDir":
                for (Wheel w : wheelsFor(c)) {
                    w.reversed = num(c, "reversed", 0) != 0;
                    w.applyDirection();
                }
                message = "Flywheel direction updated.";
                break;
            case "pidf":
                for (Wheel w : wheelsFor(c)) {
                    w.kP = num(c, "p", w.kP);
                    w.kI = num(c, "i", w.kI);
                    w.kD = num(c, "d", w.kD);
                    w.kF = num(c, "f", w.kF);
                    w.applyPidf();
                }
                message = "Hub velocity PIDF written.";
                break;
            case "atSpeed":
                atSpeedFrac = clamp(num(c, "frac", atSpeedFrac), 0.005, 0.5);
                message = "At-speed band now " + FlyRecorder.r(atSpeedFrac * 100, 1) + "%.";
                break;
            case "shooterParams":
                atSpeedFrac = clamp(num(c, "entry", atSpeedFrac), 0.005, 0.5);
                holdFrac = clamp(num(c, "hold", holdFrac), 0.005, 0.9);
                minGateOpenS = clamp(num(c, "minOpen", minGateOpenS), 0, 5);
                idleFrac = clamp(num(c, "idle", idleFrac), 0, 1);
                message = "Shooter values: entry " + FlyRecorder.r(atSpeedFrac * 100, 1) + "%, hold "
                        + FlyRecorder.r(holdFrac * 100, 1) + "%, min open " + FlyRecorder.r(minGateOpenS, 3)
                        + " s, idle " + FlyRecorder.r(idleFrac * 100, 0) + "% (tool only until exported).";
                break;
            case "shoot":
                startShoot(num(c, "pollen", wheels[0].target), num(c, "nectar", wheels[1].target),
                        (int) clamp(num(c, "count", 1), 1, 20), num(c, "feed", 1) != 0);
                break;
            case "shootStop":
                if (shootState != 0) {
                    endShoot("stopped from the dashboard");
                }
                break;
            case "step": {
                Wheel w = oneWheel(c);
                w.startStep(num(c, "to", 0), c.containsKey("from") ? num(c, "from", 0) : Double.NaN,
                        num(c, "pre", 1.5), num(c, "sec", 3.0), c.get("label"));
                message = "Step started on " + w.name + ".";
                break;
            }
            case "kick": {
                for (Wheel w : wheelsFor(c)) {
                    w.kick(num(c, "ms", 120));
                }
                message = "Synthetic disturbance: motor output cut for " + num(c, "ms", 120) + " ms.";
                break;
            }
            case "ffSweep": {
                Wheel w = oneWheel(c);
                w.startFf(parseList(c.get("powers"), new double[]{0.3, 0.45, 0.6, 0.75, 0.9, 1.0}),
                        num(c, "hold", 2.0));
                message = "Feed-forward sweep started on " + w.name + ".";
                break;
            }
            case "clearEvents":
                events.clear();
                message = "Shot events cleared.";
                break;
            case "recStart":
                recCurrent = num(c, "cur", 0) != 0;
                recorder.start(c.get("label"));
                message = "Recording " + recorder.label() + (recCurrent ? " with current." : ".");
                break;
            case "recStop":
                recorder.stop();
                recCurrent = false;
                message = "Recording stopped, " + recorder.size() + " samples.";
                break;

            // ---- turrets
            case "turretPos":
                for (TurretCal t : turretsFor(c)) {
                    // free=1 (Turret limits tab) may go past the limits, to find them.
                    t.setPosition(num(c, "pos", t.pos), "1".equals(c.get("free")));
                }
                message = "Turret position set.";
                break;
            case "turretDeg":
                for (TurretCal t : turretsFor(c)) {
                    t.setPosition(t.positionFor(num(c, "deg", 0)), false);
                }
                message = "Turret pointed at " + num(c, "deg", 0) + " deg (shipped formula).";
                break;
            case "turretCal":
                for (TurretCal t : turretsFor(c)) {
                    if ("1".equals(c.get("here"))) {
                        t.center = t.pos;
                    }
                    t.center = clamp(num(c, "center", t.center), 0, 1);
                    t.reversed = num(c, "reversed", t.reversed ? 1 : 0) != 0;
                    t.gearA = num(c, "grA", t.gearA);
                    t.gearB = num(c, "grB", t.gearB);
                    t.servoRange = num(c, "servoRange", t.servoRange);
                    t.offset[0] = num(c, "offX", t.offset[0]);
                    t.offset[1] = num(c, "offY", t.offset[1]);
                    t.min = clamp(num(c, "min", t.min), 0, 1);
                    t.max = clamp(num(c, "max", t.max), 0, 1);
                    t.start = clamp(num(c, "start", t.start), 0, 1);
                }
                message = "Turret calibration updated (tool only until exported).";
                break;
            case "turretMark": {
                String mark = c.get("mark") == null ? "" : c.get("mark");
                StringBuilder m = new StringBuilder();
                for (TurretCal t : turretsFor(c)) {
                    if (Double.isNaN(t.pos)) {
                        m.append(t.name).append(": move it first. ");
                        continue;
                    }
                    switch (mark) {
                        case "min": t.min = t.pos; break;
                        case "max": t.max = t.pos; break;
                        case "start": t.start = t.pos; break;
                        case "center": t.center = t.pos; break;
                        default: m.append("unknown mark ").append(mark).append(". "); continue;
                    }
                    m.append(t.name).append(' ').append(mark).append(" = ")
                            .append(FlyRecorder.r(t.pos, 4)).append(". ");
                }
                message = m.append("(tool only until exported)").toString();
                break;
            }

            // ---- gate / scoopula
            case "servoPos": {
                ServoCal s = servoFor(c);
                s.setPosition(num(c, "pos", s.pos));
                message = s.name + " position " + FlyRecorder.r(s.pos, 3) + ".";
                break;
            }
            case "servoMark": {
                ServoCal s = servoFor(c);
                String as = c.get("as");
                if (!s.marks.containsKey(as)) {
                    message = "servoMark: as= must be one of " + s.marks.keySet();
                    break;
                }
                s.marks.put(as, Double.isNaN(s.pos) ? null : s.pos);
                message = s.name + " '" + as + "' marked at " + FlyRecorder.r(s.pos, 3) + ".";
                break;
            }
            case "gatePulse": {
                ServoCal s = servos[0];
                Double open = s.marks.get("open");
                Double close = s.marks.get("close");
                if (open == null || close == null) {
                    message = "gatePulse needs gate 'open' and 'close' marked first.";
                    break;
                }
                s.pulse(open, close, num(c, "ms", 250));
                message = "Gate pulsed open for " + num(c, "ms", 250) + " ms.";
                break;
            }

            // ---- intake / color
            case "intake": {
                String m = c.get("mode") == null ? "OFF" : c.get("mode").toUpperCase();
                if (intakeSys == null) {
                    message = "Intake unavailable: " + intakeError;
                    break;
                }
                if (!Arrays.asList("OFF", "IN", "OUT", "WATCH").contains(m)) {
                    message = "intake mode= must be OFF, IN, OUT or WATCH.";
                    break;
                }
                intakeMode = m;
                if ("OFF".equals(m)) {
                    stopIntake();
                }
                message = "Intake " + m + " (shipped intake logic, alliance " + alliance + ").";
                break;
            }
            case "alliance":
                alliance = "BLUE".equalsIgnoreCase(c.get("side")) ? Alliance.BLUE : Alliance.RED;
                if (intakeSys != null) {
                    intakeSys.setAlliance(alliance);
                }
                message = "Alliance " + alliance + ".";
                break;
            case "colorStream":
                colorStream = num(c, "on", 1) != 0;
                message = "Color streaming " + (colorStream ? "on." : "off.");
                break;
            case "colorSample": {
                String label = c.get("label") == null ? "" : c.get("label").trim().toUpperCase();
                if (label.isEmpty() || color == null) {
                    message = color == null ? "No color sensor named colorSensor."
                            : "colorSample needs label=";
                    break;
                }
                sampleLabel = label;
                sampleRemaining = (int) clamp(num(c, "n", 40), 1, 500);
                message = "Sampling " + sampleRemaining + " reads as " + label + "...";
                break;
            }
            case "colorClear": {
                String label = c.get("label");
                for (int i = samples.size() - 1; i >= 0; i--) {
                    if (label == null || label.equalsIgnoreCase(sampleLabels.get(i))) {
                        samples.remove(i);
                        sampleLabels.remove(i);
                        sampleDist.remove(i);
                    }
                }
                rebuildColorSummary();
                message = "Color samples cleared" + (label == null ? "." : " for " + label + ".");
                break;
            }
            case "intakeTest": {
                String m = c.get("motor") == null ? "OFF" : c.get("motor").toUpperCase(java.util.Locale.US);
                if (!Arrays.asList("OFF", "LEFT", "RIGHT", "BOTH").contains(m)) {
                    message = "intakeTest motor= must be OFF, LEFT, RIGHT or BOTH.";
                    break;
                }
                if ("OFF".equals(m)) {
                    endIntakeTest();
                    message = "Intake direction test stopped.";
                    break;
                }
                if (intakeMotors[0] == null && intakeMotors[1] == null) {
                    message = "Intake test unavailable: no intakeLeft / intakeRight motor.";
                    break;
                }
                if (intakeSys != null && !"OFF".equals(intakeMode)) {
                    stopIntake();
                }
                intakeMode = "OFF";
                intakeTestPower = clamp(num(c, "power", intakeTestPower), 0.05, 1);
                intakeTest = m;
                message = "Intake direction test: " + m + " at power " + FlyRecorder.r(intakeTestPower, 2)
                        + " (intake direction, flags applied).";
                break;
            }
            case "intakeDir":
                SystemConstants.Intake.INTAKE_LEFT_REVERSED = num(c, "left", SystemConstants.Intake.INTAKE_LEFT_REVERSED ? 1 : 0) != 0;
                SystemConstants.Intake.INTAKE_RIGHT_REVERSED = num(c, "right", SystemConstants.Intake.INTAKE_RIGHT_REVERSED ? 1 : 0) != 0;
                SystemConstants.Intake.INTAKE_REVERSED = num(c, "both", SystemConstants.Intake.INTAKE_REVERSED ? 1 : 0) != 0;
                message = "Intake direction: left " + (SystemConstants.Intake.INTAKE_LEFT_REVERSED ? "REVERSED" : "normal")
                        + ", right " + (SystemConstants.Intake.INTAKE_RIGHT_REVERSED ? "REVERSED" : "normal")
                        + ", overall " + (SystemConstants.Intake.INTAKE_REVERSED ? "REVERSED" : "normal")
                        + " - applied live (restored on STOP unless exported).";
                break;
            case "intakeParams":
                SystemConstants.Intake.REVERSE_BRAKE_MS = clamp(num(c, "brakeMs", SystemConstants.Intake.REVERSE_BRAKE_MS), 0, 1000);
                message = "Intake REVERSE_BRAKE_MS " + FlyRecorder.r(SystemConstants.Intake.REVERSE_BRAKE_MS, 0)
                        + " applied live (restored on STOP unless exported).";
                break;
            case "colorParams":
                SystemConstants.ColorCheck.RED_OVER_BLUE = num(c, "red", SystemConstants.ColorCheck.RED_OVER_BLUE);
                SystemConstants.ColorCheck.BLUE_OVER_RED = num(c, "blue", SystemConstants.ColorCheck.BLUE_OVER_RED);
                SystemConstants.ColorCheck.MAX_DIST_CM = num(c, "dist", SystemConstants.ColorCheck.MAX_DIST_CM);
                rebuildColorSummary();
                message = "Intake color thresholds applied live (restored on STOP unless exported).";
                break;
            case "colorSuggestApply":
                rebuildColorSummary();
                if (suggestion != null) {
                    SystemConstants.ColorCheck.RED_OVER_BLUE = suggestion[0];
                    SystemConstants.ColorCheck.BLUE_OVER_RED = suggestion[1];
                    SystemConstants.ColorCheck.MAX_DIST_CM = suggestion[2];
                    rebuildColorSummary();
                    message = "Suggested thresholds applied live.";
                } else {
                    message = "No suggestion yet: sample RED, BLUE and EMPTY.";
                }
                break;

            case "breakInStart": {
                if (intakeMotors[0] == null && intakeMotors[1] == null) {
                    message = "Break-in unavailable: no intakeLeft / intakeRight motor.";
                    break;
                }
                breakInPower = clamp(num(c, "power", breakInPower), 0.05, 1);
                breakInFlipS = clamp(num(c, "flipS", breakInFlipS), 0, 3600);
                breakInRestS = clamp(num(c, "restS", breakInRestS), 0, 60);
                breakInTotalS = clamp(num(c, "totalS", breakInTotalS), 1, 7200);
                breakInStallA = clamp(num(c, "stallA", breakInStallA), 0.5, 20);
                breakInStallS = clamp(num(c, "stallS", breakInStallS), 0.2, 10);
                if (intakeSys != null && !"OFF".equals(intakeMode)) {
                    stopIntake();
                }
                intakeMode = "OFF";
                long now = System.nanoTime();
                breakIn = true;
                breakInResting = false;
                breakInDir = 1;
                breakInFlips = 0;
                breakInRunS = 0;
                breakInLastNs = now;
                breakInPhaseNs = now;
                breakInStallSinceNs = 0;
                breakInEnd = "";
                intakeMaxAmps[0] = 0;
                intakeMaxAmps[1] = 0;
                setIntakePower(breakInPower);
                message = "Intake break-in started: power " + FlyRecorder.r(breakInPower, 2) + ", "
                        + FlyRecorder.r(breakInTotalS / 60, 1) + " min"
                        + (breakInFlipS > 0 ? ", reversing every " + FlyRecorder.r(breakInFlipS, 0) + " s." : ".");
                break;
            }
            case "breakInStop":
                if (breakIn) {
                    endBreakIn("stopped from the dashboard");
                }
                message = "Intake break-in stopped.";
                break;

            // ---- shooting regression
            case "regHive": {
                String h = c.get("hive") == null ? "" : c.get("hive").toUpperCase(java.util.Locale.US);
                if (!h.equals("AUTO") && !h.equals("UP") && !h.equals("DOWN")) {
                    throw new IllegalArgumentException("hive= must be AUTO, UP or DOWN");
                }
                hive = h;
                message = "Aiming at the " + hive + " hive, alliance " + alliance + ".";
                break;
            }
            case "setPose": {
                if (localizer == null) throw new IllegalStateException("no Pinpoint: " + poseError);
                Pose p = new Pose(num(c, "x", 0), num(c, "y", 0), Math.toRadians(num(c, "deg", 0)));
                localizer.setPose(p);
                pose = p;
                message = "Pose set to x " + FlyRecorder.r(p.x(), 2) + ", y " + FlyRecorder.r(p.y(), 2)
                        + ", heading " + FlyRecorder.r(num(c, "deg", 0), 1) + " deg.";
                break;
            }
            case "regAim":
                regAim = num(c, "on", 1) != 0;
                if (regAim && localizer == null) {
                    regAim = false;
                    throw new IllegalStateException("no Pinpoint: " + poseError);
                }
                message = regAim ? "Turrets tracking the " + hive + " hive (" + alliance + ")." : "Turret tracking off.";
                break;
            case "regRecord": {
                readPose();
                if (pose == null) throw new IllegalStateException("no pose: " + poseError);
                StringBuilder out = new StringBuilder();
                for (Wheel w : wheelsFor(c)) {
                    int i = w == wheels[0] ? 0 : 1;
                    if (w.mode != MODE_VEL || w.target <= 0) {
                        out.append(w.name).append(": not spinning in velocity mode, skipped. ");
                        continue;
                    }
                    double[] a = aim(turrets[i]);
                    RegressionSession.Sample s = reg.add(i, a[0], w.target, w.vel, pose.x(), pose.y(),
                            Math.toDegrees(pose.heading()), alliance.toString(), hiveFor().name, volts);
                    out.append(w.name).append(" #").append(s.id).append(": ")
                            .append(FlyRecorder.r(w.target, 0)).append(" ticks/s at ")
                            .append(FlyRecorder.r(a[0], 1)).append(" in. ");
                }
                message = out.length() == 0 ? "Nothing recorded." : out.toString().trim();
                break;
            }
            case "regDelete":
                message = reg.delete((int) num(c, "id", -1)) ? "Sample deleted." : "No such sample.";
                break;
            case "regClear":
                message = "Cleared " + reg.clear(RegressionSession.wheelIndex(c.get("wheel"))) + " "
                        + c.get("wheel") + " samples.";
                break;
            case "regDegree":
                reg.setDegree(RegressionSession.wheelIndex(c.get("wheel")), (int) num(c, "degree", 1));
                message = c.get("wheel") + " fit degree " + (int) num(c, "degree", 1) + ".";
                break;
            case "regSave": {
                String why = reg.save(RegressionSession.wheelIndex(c.get("wheel")));
                message = why == null
                        ? c.get("wheel") + " fit saved for export (ShootingRegression.java via mechtune.py export --only regression --write)."
                        : "REFUSED regSave: " + why;
                break;
            }
            case "regUnsave":
                reg.unsave(RegressionSession.wheelIndex(c.get("wheel")));
                message = c.get("wheel") + " export reverted to the shipped regression.";
                break;
            case "regSpinFit": {
                readPose();
                if (pose == null) throw new IllegalStateException("no pose: " + poseError);
                StringBuilder out = new StringBuilder();
                for (Wheel w : wheelsFor(c)) {
                    int i = w == wheels[0] ? 0 : 1;
                    double d = aim(turrets[i])[0];
                    double v = reg.fitVelocity(i, d);
                    if (Double.isNaN(v)) {
                        out.append(w.name).append(": no fit yet. ");
                        continue;
                    }
                    w.cancelTests();
                    w.setVelocity(v);
                    out.append(w.name).append(" ").append(FlyRecorder.r(w.target, 0)).append(" ticks/s (fit at ")
                            .append(FlyRecorder.r(d, 1)).append(" in). ");
                }
                message = out.toString().trim();
                break;
            }

            case "zoneCell": {
                int[][] map = zoneMap(c.get("map"));
                int x = (int) num(c, "x", -1), y = (int) num(c, "y", -1);
                if (map == null || y < 0 || y >= map.length || map[y] == null || x < 0 || x >= map[y].length) {
                    message = "zoneCell: map=pollen|nectar and x, y inside the grid.";
                    break;
                }
                map[y][x] = num(c, "v", 1) != 0 ? 1 : 0;
                message = "Zone " + c.get("map") + " cell (" + x + ", " + y + ") = " + map[y][x]
                        + " (live; restored on STOP unless exported).";
                break;
            }
            case "zoneFill": {
                int v = num(c, "v", 0) != 0 ? 1 : 0;
                String which = c.get("map") == null ? "both" : c.get("map");
                for (String m : new String[]{"pollen", "nectar"}) {
                    if (!"both".equals(which) && !m.equals(which)) continue;
                    int[][] map = zoneMap(m);
                    if (map == null) continue;
                    for (int[] row : map) {
                        if (row != null) Arrays.fill(row, v);
                    }
                }
                message = "Zone " + which + " filled with " + v + ".";
                break;
            }
            case "zoneCopy": {
                String from = c.get("from");
                int[][] src = zoneMap(from);
                if (src == null) {
                    message = "zoneCopy: from=pollen|nectar.";
                    break;
                }
                int[][] copy = copyMap(src);
                if ("pollen".equals(from)) SystemConstants.Zone.mapNectar = copy;
                else SystemConstants.Zone.mapPollen = copy;
                message = "Zone " + from + " copied to the other map.";
                break;
            }
            case "zoneSize": {
                int nx = (int) clamp(num(c, "x", SystemConstants.Zone.xQuality), 1, 48);
                int ny = (int) clamp(num(c, "y", SystemConstants.Zone.yQuality), 1, 48);
                SystemConstants.Zone.mapPollen = resizeMap(SystemConstants.Zone.mapPollen, nx, ny);
                SystemConstants.Zone.mapNectar = resizeMap(SystemConstants.Zone.mapNectar, nx, ny);
                SystemConstants.Zone.xQuality = nx;
                SystemConstants.Zone.yQuality = ny;
                message = "Zone grid resized to " + nx + " x " + ny + " cells.";
                break;
            }
            case "zoneOffsets":
                SystemConstants.Zone.yOffsetPollen = (int) Math.round(num(c, "pollen", SystemConstants.Zone.yOffsetPollen));
                SystemConstants.Zone.yOffsetNectar = (int) Math.round(num(c, "nectar", SystemConstants.Zone.yOffsetNectar));
                message = "Zone shooter offsets applied live.";
                break;

            default:
                message = "UNKNOWN COMMAND: \"" + action + "\" - nothing was done.";
                break;
        }
    }

    private void stopAll() {
        regAim = false;
        if (shootState != 0) {
            endShoot("everything stopped");
        }
        for (Wheel w : wheels) {
            w.cancelTests();
            w.off();
        }
        if (breakIn) {
            endBreakIn("everything stopped");
        }
        endIntakeTest();
        if (intakeSys != null) {
            stopIntake();
        }
        intakeMode = "OFF";
    }

    private void updateBreakIn(long now, boolean slow) {
        double dt = (now - breakInLastNs) / 1e9;
        breakInLastNs = now;
        double phaseS = (now - breakInPhaseNs) / 1e9;
        if (breakInResting) {
            if (phaseS >= breakInRestS) {
                breakInResting = false;
                breakInDir = -breakInDir;
                breakInFlips++;
                breakInPhaseNs = now;
                setIntakePower(breakInDir * breakInPower);
            }
        } else {
            breakInRunS += dt;
            if (breakInRunS >= breakInTotalS) {
                endBreakIn("finished");
                message = "Intake break-in finished: " + FlyRecorder.r(breakInRunS / 60, 1) + " min, "
                        + breakInFlips + " reversals, peak " + FlyRecorder.r(intakeMaxAmps[0], 2) + " / "
                        + FlyRecorder.r(intakeMaxAmps[1], 2) + " A.";
                return;
            }
            if (breakInFlipS > 0 && phaseS >= breakInFlipS) {
                breakInResting = breakInRestS > 0;
                breakInPhaseNs = now;
                if (breakInResting) {
                    setIntakePower(0);
                } else {
                    breakInDir = -breakInDir;
                    breakInFlips++;
                    setIntakePower(breakInDir * breakInPower);
                }
            }
        }
        if (slow) {
            readIntakeAmps();
            boolean over = !breakInResting && Math.max(intakeAmps[0], intakeAmps[1]) > breakInStallA;
            if (!over) {
                breakInStallSinceNs = 0;
            } else if (breakInStallSinceNs == 0) {
                breakInStallSinceNs = now;
            } else if ((now - breakInStallSinceNs) / 1e9 >= breakInStallS) {
                endBreakIn("STALL: over " + FlyRecorder.r(breakInStallA, 1) + " A for "
                        + FlyRecorder.r(breakInStallS, 1) + " s");
                message = "Intake break-in STOPPED - current over " + FlyRecorder.r(breakInStallA, 1)
                        + " A (" + FlyRecorder.r(intakeAmps[0], 2) + " / " + FlyRecorder.r(intakeAmps[1], 2)
                        + " A). Check for a jam before restarting.";
            }
        }
    }

    private void endBreakIn(String why) {
        breakIn = false;
        breakInResting = false;
        breakInEnd = why;
        setIntakePower(0);
    }

    /** Positive p = the intake direction, with the shipped direction flags applied per motor (as Intake.update does). */
    private void setIntakePower(double p) {
        for (int i = 0; i < 2; i++) {
            if (intakeMotors[i] != null) {
                intakeMotors[i].setPower(p * intakeSign(i));
            }
        }
    }

    /** -1 when motor i (0 left, 1 right) runs reversed under the live SystemConstants.Intake flags. */
    private static double intakeSign(int i) {
        boolean motorRev = i == 0 ? SystemConstants.Intake.INTAKE_LEFT_REVERSED : SystemConstants.Intake.INTAKE_RIGHT_REVERSED;
        return (motorRev ^ SystemConstants.Intake.INTAKE_REVERSED) ? -1 : 1;
    }

    private void endIntakeTest() {
        if (intakeTest.isEmpty()) return;
        intakeTest = "";
        for (DcMotorEx m : intakeMotors) {
            if (m != null) m.setPower(0);
        }
    }

    private void readIntakeAmps() {
        for (int i = 0; i < 2; i++) {
            if (intakeMotors[i] == null) continue;
            try {
                intakeAmps[i] = intakeMotors[i].getCurrent(CurrentUnit.AMPS);
                if (breakIn) intakeMaxAmps[i] = Math.max(intakeMaxAmps[i], intakeAmps[i]);
            } catch (RuntimeException e) {
                intakeAmps[i] = Double.NaN;
            }
        }
    }

    private static int[][] zoneMap(String name) {
        if ("pollen".equals(name)) return SystemConstants.Zone.mapPollen;
        if ("nectar".equals(name)) return SystemConstants.Zone.mapNectar;
        return null;
    }

    private static int[][] copyMap(int[][] map) {
        if (map == null) return null;
        int[][] out = new int[map.length][];
        for (int i = 0; i < map.length; i++) {
            out[i] = map[i] == null ? new int[0] : map[i].clone();
        }
        return out;
    }

    private static int[][] resizeMap(int[][] map, int nx, int ny) {
        int[][] out = new int[ny][nx];
        if (map == null) return out;
        for (int y = 0; y < Math.min(ny, map.length); y++) {
            if (map[y] == null) continue;
            System.arraycopy(map[y], 0, out[y], 0, Math.min(nx, map[y].length));
        }
        return out;
    }

    private static String mapJson(int[][] map) {
        StringBuilder sb = new StringBuilder("[");
        if (map != null) {
            for (int y = 0; y < map.length; y++) {
                if (y > 0) sb.append(',');
                sb.append('[');
                int[] row = map[y] == null ? new int[0] : map[y];
                for (int x = 0; x < row.length; x++) {
                    if (x > 0) sb.append(',');
                    sb.append(row[x]);
                }
                sb.append(']');
            }
        }
        return sb.append(']').toString();
    }

    private static String mapSource(int[][] map) {
        return mapJson(map).replace('[', '{').replace(']', '}').replace(",", ", ");
    }

    /** The shipped intake has no stop; with a null alliance its reject branch cannot fire. */
    private void stopIntake() {
        intakeSys.setAlliance(null);
        intakeSys.update(false, false);
        intakeSys.setAlliance(alliance);
    }

    // ------------------------------------------------------------------------- shooting regression

    private void readPose() {
        if (localizer == null) return;
        try {
            localizer.update();
            Pose p = localizer.state().pose();
            if (Double.isNaN(p.x()) || Double.isNaN(p.y()) || Double.isNaN(p.heading())
                    || Double.isInfinite(p.x()) || Double.isInfinite(p.y())) {
                // The Pinpoint reports NaN while it is not ready (IMU calibrating, device fault).
                pose = null;
                poseError = "Pinpoint returned NaN (not ready or faulted) - check its wiring / power, or set the pose";
            } else {
                pose = p;
                poseError = "";
            }
        } catch (RuntimeException e) {
            poseError = shortError(e);
        }
    }

    private static final class HiveTarget {
        final String name;
        final Pose pose;

        HiveTarget(String name, Pose pose) {
            this.name = name;
            this.pose = pose;
        }
    }

    /**
     * AUTO is the shipped rule exactly ({@link HivePosition#target} on the robot-center pose, as
     * Turret.aimFrom calls it); UP / DOWN force that hive at the alliance's x.
     */
    private HiveTarget hiveFor() {
        Pose auto = HivePosition.target(offsetPose(SystemConstants.PredictiveAiming.CENTER), alliance);
        if ("AUTO".equals(hive)) {
            return new HiveTarget(auto.y() == SystemConstants.HivePosition.UP_POS ? "UP" : "DOWN", auto);
        }
        return new HiveTarget(hive, new Pose(auto.x(), "UP".equals(hive)
                ? SystemConstants.HivePosition.UP_POS : SystemConstants.HivePosition.DOWN_POS));
    }

    /** The pose shifted by a robot-frame {forward, left} offset, as PredictiveAiming.leadPose at rest. */
    private Pose offsetPose(double[] offset) {
        double c = Math.cos(pose.heading()), s = Math.sin(pose.heading());
        return new Pose(pose.x() + offset[0] * c - offset[1] * s,
                pose.y() + offset[0] * s + offset[1] * c, pose.heading());
    }

    /** {distance in, bearing rad relative to the robot heading} from this turret's pivot to the hive. */
    private double[] aim(TurretCal t) {
        Pose from = offsetPose(t.offset);
        Pose target = hiveFor().pose;
        double dx = target.x() - from.x(), dy = target.y() - from.y();
        double theta = Math.atan2(dy, dx) - from.heading();
        theta = Math.atan2(Math.sin(theta), Math.cos(theta));
        return new double[]{Math.hypot(dx, dy), theta};
    }

    /**
     * Points both turrets at the hive with the tool's calibration: of the aim angle and its copies a
     * whole turn away, the reachable one nearest the current position (turret.java unwinds the same
     * way, minus the hysteresis). Unreachable: held at the nearer limit and flagged.
     */
    private void aimTurrets() {
        if (pose == null) return;
        for (int i = 0; i < turrets.length; i++) {
            TurretCal t = turrets[i];
            if (t.servo == null) continue;
            double deg = Math.toDegrees(aim(t)[1]);
            double lo = Math.min(t.min, t.max), hi = Math.max(t.min, t.max);
            double cur = Double.isNaN(t.pos) ? t.center : t.pos;
            double best = Double.NaN;
            for (int k = -1; k <= 1; k++) {
                double p = t.positionFor(deg + 360 * k);
                if (p >= lo && p <= hi && (Double.isNaN(best) || Math.abs(p - cur) < Math.abs(best - cur))) {
                    best = p;
                }
            }
            aimReachable[i] = !Double.isNaN(best);
            if (!aimReachable[i]) best = clamp(t.positionFor(deg), lo, hi);
            if (Double.isNaN(t.pos) || Math.abs(best - t.pos) > 1e-4) {
                t.setPosition(best, false);
            }
        }
    }

    private String regJson() {
        StringBuilder sb = new StringBuilder(512);
        sb.append("{\"poseOk\":").append(pose != null)
                .append(",\"poseError\":\"").append(TuningHub.esc(poseError)).append('"');
        if (pose != null) {
            HiveTarget h = hiveFor();
            sb.append(",\"x\":").append(nanToNull(pose.x(), 2))
                    .append(",\"y\":").append(nanToNull(pose.y(), 2))
                    .append(",\"h\":").append(nanToNull(Math.toDegrees(pose.heading()), 1))
                    .append(",\"hiveUsed\":\"").append(h.name)
                    .append("\",\"hiveX\":").append(nanToNull(h.pose.x(), 2))
                    .append(",\"hiveY\":").append(nanToNull(h.pose.y(), 2))
                    .append(",\"turrets\":[");
            for (int i = 0; i < turrets.length; i++) {
                double[] a = aim(turrets[i]);
                double fit = reg.fitVelocity(i, a[0]);
                double[][] sh = reg.shipped[i];
                if (i > 0) sb.append(',');
                sb.append("{\"dist\":").append(nanToNull(a[0], 2))
                        .append(",\"deg\":").append(nanToNull(Math.toDegrees(a[1]), 1))
                        .append(",\"reachable\":").append(aimReachable[i])
                        .append(",\"fitVel\":").append(nanToNull(fit, 0))
                        .append(",\"shippedVel\":").append(nanToNull(
                                ShootingRegression.evaluate(sh[0], sh[1], a[0]), 0)).append('}');
            }
            sb.append(']');
        }
        sb.append(",\"alliance\":\"").append(alliance)
                .append("\",\"hive\":\"").append(hive)
                .append("\",\"aim\":").append(regAim)
                .append(",\"data\":").append(reg.json());
        return sb.append('}').toString();
    }

    // ------------------------------------------------------------------------- shoot sequence

    /**
     * The gate half of {@code shooter.java} with the tool's values: TRACK until both wheels are
     * inside the entry band (AT_SPEED_FRACTION), then SHOOT - gate open and, as TeleOp does, the
     * intake fed - held while the gate has been open less than MIN_GATE_OPEN_S or both wheels are
     * inside HOLD_SPEED_FRACTION. Aiming and the zone are not modelled: the turrets do not move.
     */
    private void startShoot(double pollenTps, double nectarTps, int count, boolean feed) {
        ServoCal g = servos[0];
        if (wheels[0].motor == null || wheels[1].motor == null) {
            message = "REFUSED shoot: both flywheels are needed (shooter.java waits on both).";
            return;
        }
        if (g.servo == null || g.marks.get("open") == null || g.marks.get("close") == null) {
            message = "REFUSED shoot: mark the gate 'open' and 'close' first (Turrets & servos tab).";
            return;
        }
        if (pollenTps <= 0 || nectarTps <= 0) {
            message = "REFUSED shoot: both targets must be positive ticks/s (shooter.java needs target > 0).";
            return;
        }
        if (feed && intakeSys == null) {
            message = "REFUSED shoot with feed: intake unavailable (" + intakeError + ").";
            return;
        }
        for (Wheel w : wheels) {
            w.cancelTests();
        }
        wheels[0].setVelocity(pollenTps);
        wheels[1].setVelocity(nectarTps);
        g.setPosition(g.marks.get("close"));
        shootFeed = feed;
        if (feed) {
            intakeMode = "OFF";
        }
        shootLeft = count;
        shootEnd = "";
        shootOwnsRec = !recorder.isRecording();
        if (shootOwnsRec) {
            recorder.start("shoot-" + (shootSeq + 1));
        }
        shootState = 1;
        shootPhaseNs = System.nanoTime();
        message = "Shoot sequence: waiting for both wheels in the entry band.";
    }

    private void updateShoot(long now) {
        if (shootState == 0) return;
        Wheel p = wheels[0], n = wheels[1];
        if (p.mode != MODE_VEL || n.mode != MODE_VEL || p.target <= 0 || n.target <= 0) {
            endShoot("a flywheel left velocity mode");
            return;
        }
        ServoCal g = servos[0];
        if (shootState == 1) {
            if (inBand(p, atSpeedFrac) && inBand(n, atSpeedFrac)) {
                g.setPosition(g.marks.get("open"));
                shootState = 2;
                shootOpenNs = now;
                shootReadyMs = (now - shootPhaseNs) / 1e6;
                shootWorstP = 0;
                shootWorstN = 0;
                shootHeldByTime = true;
                recorder.mark(5);
            } else if (now - shootPhaseNs > SHOOT_READY_TIMEOUT_NS) {
                endShoot("wheels never reached the entry band in "
                        + SHOOT_READY_TIMEOUT_NS / 1_000_000_000L + " s");
            }
            return;
        }
        shootWorstP = Math.min(shootWorstP, 100 * (p.vel - p.target) / p.target);
        shootWorstN = Math.min(shootWorstN, 100 * (n.vel - n.target) / n.target);
        boolean byTime = (now - shootOpenNs) / 1e9 < minGateOpenS;
        boolean byBand = inBand(p, holdFrac) && inBand(n, holdFrac);
        if (byTime || byBand) {
            shootHeldByTime = byTime;
            return;
        }
        g.setPosition(g.marks.get("close"));
        recorder.mark(6);
        logShootWindow((now - shootOpenNs) / 1e6,
                shootHeldByTime ? "min open time ended" : "speed left the hold band");
        if (--shootLeft > 0) {
            shootState = 1;
            shootPhaseNs = now;
        } else {
            endShoot("done");
        }
    }

    private boolean inBand(Wheel w, double frac) {
        return w.target > 0 && Math.abs(w.vel - w.target) <= frac * w.target;
    }

    private void logShootWindow(double openMs, String closedBy) {
        StringBuilder sb = new StringBuilder(260);
        sb.append("{\"id\":").append(++shootSeq)
                .append(",\"readyMs\":").append(FlyRecorder.r(shootReadyMs, 0))
                .append(",\"openMs\":").append(FlyRecorder.r(openMs, 0))
                .append(",\"closedBy\":\"").append(closedBy)
                .append("\",\"pWorstPct\":").append(FlyRecorder.r(shootWorstP, 1))
                .append(",\"nWorstPct\":").append(FlyRecorder.r(shootWorstN, 1))
                .append(",\"pTarget\":").append(FlyRecorder.r(wheels[0].target, 0))
                .append(",\"nTarget\":").append(FlyRecorder.r(wheels[1].target, 0))
                .append(",\"entry\":").append(FlyRecorder.r(atSpeedFrac, 4))
                .append(",\"hold\":").append(FlyRecorder.r(holdFrac, 4))
                .append(",\"minOpenS\":").append(FlyRecorder.r(minGateOpenS, 3))
                .append(",\"feed\":").append(shootFeed)
                .append(",\"volts\":").append(FlyRecorder.r(volts, 2)).append('}');
        shootWindows.add(sb.toString());
        while (shootWindows.size() > MAX_SHOOT_WINDOWS) {
            shootWindows.remove(0);
        }
        message = "Gate window " + shootSeq + ": ready after " + (int) shootReadyMs + " ms, open "
                + (int) openMs + " ms, " + closedBy + ".";
    }

    /** Closes the gate if it is open and hands the intake back; the wheels keep spinning. */
    private void endShoot(String reason) {
        ServoCal g = servos[0];
        if (shootState == 2 && g.servo != null && g.marks.get("close") != null) {
            g.setPosition(g.marks.get("close"));
            recorder.mark(6);
        }
        shootState = 0;
        if (shootFeed && intakeSys != null) {
            stopIntake();
        }
        if (shootOwnsRec) {
            recorder.stop();
            shootOwnsRec = false;
        }
        shootEnd = reason;
        message = "Shoot sequence ended: " + reason + ".";
    }

    private Wheel[] wheelsFor(Map<String, String> c) {
        String w = c.get("wheel") == null ? "both" : c.get("wheel");
        if ("pollen".equals(w)) return new Wheel[]{wheels[0]};
        if ("nectar".equals(w)) return new Wheel[]{wheels[1]};
        if ("both".equals(w)) return wheels;
        throw new IllegalArgumentException("wheel= must be pollen, nectar or both");
    }

    private Wheel oneWheel(Map<String, String> c) {
        String w = c.get("wheel");
        if ("pollen".equals(w)) return wheels[0];
        if ("nectar".equals(w)) return wheels[1];
        throw new IllegalArgumentException("this test needs wheel=pollen or wheel=nectar");
    }

    private TurretCal[] turretsFor(Map<String, String> c) {
        String w = c.get("which") == null ? "both" : c.get("which");
        if ("pollen".equals(w)) return new TurretCal[]{turrets[0]};
        if ("nectar".equals(w)) return new TurretCal[]{turrets[1]};
        if ("both".equals(w)) return turrets;
        throw new IllegalArgumentException("which= must be pollen, nectar or both");
    }

    private ServoCal servoFor(Map<String, String> c) {
        String n = c.get("name");
        for (ServoCal s : servos) {
            if (s.name.equals(n)) return s;
        }
        throw new IllegalArgumentException("name= must be gate or scoopula");
    }

    // ------------------------------------------------------------------------- flywheel

    private final class Wheel {
        final String name;
        final String device;
        DcMotorEx motor;
        String error = "";
        boolean reversed;
        int mode = MODE_OFF;
        DcMotor.RunMode runMode;
        double target, power, vel;
        double cur = Double.NaN;
        double kP, kI, kD, kF;
        final double[] shippedPidf;
        String hubPidf = "";

        // dip detector: 0 arming, 1 armed, 2 in dip, 3 post-recovery window
        int dip;
        long inBandSinceNs, dipStartNs, dipMinNs, recoveredNs, kickEventUntilNs;
        double dipMin, overMax, dipTarget, dipVolts;
        String dipSource = "shot";

        long kickEndNs;
        boolean kicking;

        // step test: 0 none, 1 pre (holding from), 2 running
        int stepPhase;
        long phaseEndNs;
        double stepTo, stepSec;
        int stepStartIdx;
        int stepSeq;
        boolean stepOwnsRec;
        String stepLabel;
        String lastStep = "null";

        // feed-forward sweep
        double[] ffPowers;
        int ffIdx = -1;
        double ffHold, ffSum;
        int ffCount;
        long ffLevelStartNs;
        final List<double[]> ffLevels = new ArrayList<>();
        String ffResult = "null";

        Wheel(String name, String device, boolean reversed,
              com.pedropathing.control.PIDFCoefficients shipped) {
            this.name = name;
            this.device = device;
            this.reversed = reversed;
            kP = shipped.P;
            kI = shipped.I;
            kD = shipped.D;
            kF = shipped.F;
            shippedPidf = new double[]{kP, kI, kD, kF};
            motor = hardwareMap.tryGet(DcMotorEx.class, device);
            if (motor == null) {
                error = "no motor named " + device;
                return;
            }
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            applyDirection();
            setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(0);
            applyPidf();
        }

        void applyDirection() {
            if (motor != null) {
                motor.setDirection(reversed ? DcMotorSimple.Direction.REVERSE
                        : DcMotorSimple.Direction.FORWARD);
            }
        }

        void applyPidf() {
            if (motor == null) return;
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(kP, kI, kD, kF));
            PIDFCoefficients back = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            hubPidf = FlyRecorder.r(back.p, 4) + ", " + FlyRecorder.r(back.i, 4) + ", "
                    + FlyRecorder.r(back.d, 4) + ", " + FlyRecorder.r(back.f, 4);
        }

        void setRunMode(DcMotor.RunMode m) {
            if (motor != null && runMode != m) {
                motor.setMode(m);
                runMode = m;
            }
        }

        void read(boolean withCurrent) {
            if (motor == null) return;
            vel = motor.getVelocity();
            if (withCurrent) {
                cur = motor.getCurrent(CurrentUnit.AMPS);
            }
        }

        void setVelocity(double v) {
            if (motor == null) return;
            v = clamp(v, -VEL_CAP_TPS, VEL_CAP_TPS);
            setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (mode != MODE_VEL || v != target) {
                motor.setVelocity(v);
                dip = 0;
                inBandSinceNs = 0;
            }
            mode = MODE_VEL;
            target = v;
            power = Double.NaN;
        }

        void setPower(double p) {
            if (motor == null) return;
            p = clamp(p, -1, 1);
            setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(p);
            mode = MODE_POWER;
            power = p;
            target = 0;
            dip = 0;
        }

        void off() {
            if (motor == null) return;
            if (kicking) {
                motor.setMotorEnable();
                kicking = false;
            }
            setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(0);
            mode = MODE_OFF;
            target = 0;
            power = 0;
            dip = 0;
        }

        void cancelTests() {
            if (stepPhase != 0 && stepOwnsRec) {
                recorder.stop();
            }
            stepPhase = 0;
            ffIdx = -1;
        }

        /** Cuts the motor output so the wheel coasts, then hands it back to the hub's PIDF. */
        void kick(double ms) {
            if (motor == null || mode != MODE_VEL) {
                throw new IllegalStateException(name + " must be in VEL mode to kick");
            }
            motor.setMotorDisable();
            kicking = true;
            long now = System.nanoTime();
            kickEndNs = now + (long) (clamp(ms, 10, 1000) * 1e6);
            kickEventUntilNs = kickEndNs + 200_000_000L;
            recorder.mark(2);
        }

        void startStep(double to, double from, double pre, double sec, String label) {
            if (motor == null) throw new IllegalStateException(error);
            cancelTests();
            stepTo = clamp(to, -VEL_CAP_TPS, VEL_CAP_TPS);
            stepSec = clamp(sec, 0.5, 20);
            stepLabel = label == null || label.isEmpty()
                    ? "step-" + name + "-" + (int) stepTo : label;
            lastStep = "null";
            if (!Double.isNaN(from)) {
                setVelocity(from);
                stepPhase = 1;
                phaseEndNs = System.nanoTime() + (long) (clamp(pre, 0, 10) * 1e9);
            } else {
                beginStepRun();
            }
        }

        void beginStepRun() {
            stepOwnsRec = !recorder.isRecording();
            if (stepOwnsRec) {
                recorder.start(stepLabel);
            }
            stepStartIdx = recorder.size();
            recorder.mark(1);
            setVelocity(stepTo);
            stepPhase = 2;
            phaseEndNs = System.nanoTime() + (long) (stepSec * 1e9);
        }

        void startFf(double[] powers, double hold) {
            if (motor == null) throw new IllegalStateException(error);
            cancelTests();
            ffPowers = powers;
            ffHold = clamp(hold, 0.5, 10);
            ffLevels.clear();
            ffResult = "null";
            ffIdx = 0;
            beginFfLevel();
        }

        void beginFfLevel() {
            setPower(ffPowers[ffIdx]);
            ffLevelStartNs = System.nanoTime();
            ffSum = 0;
            ffCount = 0;
            recorder.mark(4);
        }

        void update(long now) {
            if (motor == null) return;

            if (kicking && now >= kickEndNs) {
                motor.setMotorEnable();
                // Re-issue the target: whether the hub keeps it across a disable is not documented.
                motor.setVelocity(target);
                kicking = false;
            }

            if (stepPhase == 1 && now >= phaseEndNs) {
                beginStepRun();
            } else if (stepPhase == 2 && (now >= phaseEndNs || recorder.isFull()
                    || !recorder.isRecording())) {
                int end = recorder.size();
                if (stepOwnsRec) {
                    recorder.stop();
                }
                stepPhase = 0;
                lastStep = analyseStep(this, stepStartIdx, end, stepTo);
            }

            if (ffIdx >= 0) {
                double t = (now - ffLevelStartNs) / 1e9;
                if (t >= ffHold - Math.min(0.5, ffHold / 2)) {
                    ffSum += vel;
                    ffCount++;
                }
                if (t >= ffHold) {
                    ffLevels.add(new double[]{ffPowers[ffIdx], ffCount == 0 ? 0 : ffSum / ffCount,
                            volts});
                    ffIdx++;
                    if (ffIdx >= ffPowers.length) {
                        ffIdx = -1;
                        off();
                        ffResult = ffJson();
                        message = "Feed-forward sweep finished on " + name + ".";
                    } else {
                        beginFfLevel();
                    }
                }
            }

            updateDip(now);
        }

        private void updateDip(long now) {
            if (mode != MODE_VEL || target == 0 || stepPhase != 0) {
                dip = 0;
                inBandSinceNs = 0;
                return;
            }
            double band = atSpeedFrac * Math.abs(target);
            double e = (vel - target) * Math.signum(target);  // negative = slower than target
            switch (dip) {
                case 0:
                    if (Math.abs(e) <= band) {
                        if (inBandSinceNs == 0) inBandSinceNs = now;
                        if (now - inBandSinceNs >= ARM_NS) dip = 1;
                    } else {
                        inBandSinceNs = 0;
                    }
                    break;
                case 1:
                    if (e < -band) {
                        dip = 2;
                        dipStartNs = now;
                        dipMin = e;
                        dipMinNs = now;
                        dipTarget = target;
                        dipVolts = volts;
                        dipSource = kicking || now < kickEventUntilNs ? "kick" : "shot";
                        recorder.mark(3);
                    }
                    break;
                case 2:
                    if (e < dipMin) {
                        dipMin = e;
                        dipMinNs = now;
                    }
                    if (Math.abs(e) <= band) {
                        dip = 3;
                        recoveredNs = now;
                        overMax = 0;
                    } else if (now - dipStartNs > DIP_TIMEOUT_NS) {
                        finishDip(false);
                    }
                    break;
                case 3:
                    overMax = Math.max(overMax, e);
                    if (now - recoveredNs >= POST_WINDOW_NS) {
                        finishDip(true);
                    }
                    break;
                default:
                    dip = 0;
            }
        }

        private void finishDip(boolean recovered) {
            dip = 0;
            inBandSinceNs = 0;
            double t = Math.abs(dipTarget);
            double recoverMs = recovered ? (recoveredNs - dipStartNs) / 1e6 : -1;
            double dipPct = 100 * -dipMin / t;
            String summary = name + " " + dipSource + ": dip " + FlyRecorder.r(dipPct, 1)
                    + "%, " + (recovered ? "back at speed in " + (int) recoverMs + " ms"
                    : "NOT back at speed within 3 s");
            StringBuilder sb = new StringBuilder(200);
            sb.append("{\"id\":").append(++eventSeq)
                    .append(",\"wheel\":\"").append(name)
                    .append("\",\"source\":\"").append(dipSource)
                    .append("\",\"target\":").append(FlyRecorder.r(dipTarget, 0))
                    .append(",\"dipTps\":").append(FlyRecorder.r(-dipMin, 1))
                    .append(",\"dipPct\":").append(FlyRecorder.r(dipPct, 2))
                    .append(",\"tMinMs\":").append(FlyRecorder.r((dipMinNs - dipStartNs) / 1e6, 1))
                    .append(",\"recoverMs\":").append(FlyRecorder.r(recoverMs, 1))
                    .append(",\"overPct\":").append(recovered
                            ? FlyRecorder.r(100 * overMax / t, 2) : "null")
                    .append(",\"band\":").append(FlyRecorder.r(atSpeedFrac, 4))
                    .append(",\"pidf\":[").append(kP).append(',').append(kI).append(',')
                    .append(kD).append(',').append(kF).append(']')
                    .append(",\"volts\":").append(FlyRecorder.r(dipVolts, 2))
                    .append('}');
            events.add(sb.toString());
            while (events.size() > MAX_EVENTS) {
                events.remove(0);
            }
            if ("shot".equals(dipSource)) {
                TuningHub.INSTANCE.resolveAwait("shot:" + name, summary);
            }
            message = summary + ".";
        }

        private String ffJson() {
            // Least-squares slope through the origin: velocity per unit power.
            double spv = 0, spp = 0;
            StringBuilder lv = new StringBuilder();
            for (double[] l : ffLevels) {
                spv += l[0] * l[1];
                spp += l[0] * l[0];
                if (lv.length() > 0) lv.append(',');
                lv.append('[').append(l[0]).append(',').append(FlyRecorder.r(l[1], 1)).append(',')
                        .append(FlyRecorder.r(l[2], 2)).append(']');
            }
            double maxFit = spp == 0 ? 0 : spv / spp;
            double atFull = Double.NaN;
            for (double[] l : ffLevels) {
                if (l[0] >= 0.999) atFull = l[1];
            }
            // FIRST's hub velocity PIDF guide: F = 32767 / max ticks per second.
            return "{\"levels\":[" + lv + "],\"maxVelFit\":" + FlyRecorder.r(maxFit, 1)
                    + ",\"fFromFit\":" + (maxFit > 0 ? FlyRecorder.r(32767 / maxFit, 3) : "null")
                    + ",\"maxVelAtFull\":" + (Double.isNaN(atFull) ? "null" : FlyRecorder.r(atFull, 1))
                    + ",\"fFromFull\":" + (Double.isNaN(atFull) || atFull <= 0 ? "null"
                    : FlyRecorder.r(32767 / atFull, 3)) + "}";
        }
    }

    /**
     * Step metrics straight from the recorder, so the dashboard, the host scorer and the CSV all
     * describe the same samples. Band is the shipped at-speed band (AT_SPEED_FRACTION x target):
     * that is what decides when the shooter opens the gate.
     */
    private String analyseStep(Wheel w, int from, int to, double tgt) {
        double[] vel = w == wheels[0] ? recorder.pVel : recorder.nVel;
        int n = to - from;
        int seq = ++w.stepSeq;
        if (n < 5) {
            return "{\"seq\":" + seq + ",\"error\":\"too few samples (" + n + ")\"}";
        }
        double t0 = recorder.t[from];
        double v0 = vel[from];
        double delta = tgt - v0;
        double band = tgt == 0 ? 20 : atSpeedFrac * Math.abs(tgt);
        double sgn = Math.signum(delta);
        double t10 = Double.NaN, t90 = Double.NaN, tBand = Double.NaN, peak = 0;
        int lastOut = -1;
        for (int i = from; i < to; i++) {
            double frac = delta == 0 ? 1 : (vel[i] - v0) / delta;
            double t = recorder.t[i] - t0;
            if (Double.isNaN(t10) && frac >= 0.1) t10 = t;
            if (Double.isNaN(t90) && frac >= 0.9) t90 = t;
            boolean in = Math.abs(vel[i] - tgt) <= band;
            if (in && Double.isNaN(tBand)) tBand = t;
            if (!in) lastOut = i;
            peak = Math.max(peak, (vel[i] - tgt) * sgn);
        }
        double settle = lastOut == to - 1 ? Double.NaN
                : lastOut < 0 ? 0 : recorder.t[lastOut + 1] - t0;
        double tEnd = recorder.t[to - 1];
        double ssStart = tEnd - Math.min(1.0, (tEnd - t0) * 0.3);
        double sum = 0, sq = 0, mn = Double.MAX_VALUE, mx = -Double.MAX_VALUE;
        int ns = 0;
        double dtSum = 0, vSum = 0;
        double[] dts = new double[n];
        for (int i = from; i < to; i++) {
            dts[i - from] = recorder.dt[i];
            dtSum += recorder.dt[i];
            vSum += recorder.volts[i];
            if (recorder.t[i] >= ssStart) {
                double e = vel[i] - tgt;
                sum += e;
                sq += e * e;
                mn = Math.min(mn, e);
                mx = Math.max(mx, e);
                ns++;
            }
        }
        Arrays.sort(dts);
        double ssMean = ns == 0 ? Double.NaN : sum / ns;
        double ssStd = ns < 2 ? Double.NaN : Math.sqrt(Math.max(0, sq / ns - ssMean * ssMean));
        double span = recorder.t[to - 1] - recorder.t[from];
        StringBuilder sb = new StringBuilder(400);
        sb.append("{\"seq\":").append(seq)
                .append(",\"startIdx\":").append(from)
                .append(",\"endIdx\":").append(to)
                .append(",\"label\":\"").append(TuningHub.esc(w.stepLabel))
                .append("\",\"wheel\":\"").append(w.name)
                .append("\",\"from\":").append(FlyRecorder.r(v0, 1))
                .append(",\"to\":").append(FlyRecorder.r(tgt, 1))
                .append(",\"band\":").append(FlyRecorder.r(band, 1))
                .append(",\"rise10_90S\":").append(nanToNull(t90 - t10, 3))
                .append(",\"firstInBandS\":").append(nanToNull(tBand, 3))
                .append(",\"settleS\":").append(nanToNull(settle, 3))
                .append(",\"overshootPct\":").append(delta == 0 ? "null"
                        : FlyRecorder.r(100 * peak / Math.abs(delta), 2))
                .append(",\"ssErrMean\":").append(nanToNull(ssMean, 1))
                .append(",\"ssErrStd\":").append(nanToNull(ssStd, 1))
                .append(",\"ssErrP2p\":").append(ns == 0 ? "null" : FlyRecorder.r(mx - mn, 1))
                .append(",\"n\":").append(n)
                .append(",\"loopHzTrue\":").append(nanToNull((n - 1) / span, 1))
                .append(",\"dtMeanMs\":").append(FlyRecorder.r(1000 * dtSum / n, 2))
                .append(",\"dtP90Ms\":").append(FlyRecorder.r(1000 * dts[(int) (0.9 * (n - 1))], 2))
                .append(",\"volts\":").append(FlyRecorder.r(vSum / n, 2))
                .append(",\"pidf\":[").append(w.kP).append(',').append(w.kI).append(',')
                .append(w.kD).append(',').append(w.kF).append("]}");
        return sb.toString();
    }

    // ------------------------------------------------------------------------- turret / servos

    private final class TurretCal {
        final String name;
        final String device;
        Servo servo;
        double pos = Double.NaN;
        double center;
        boolean reversed;
        double gearA, gearB;
        double servoRange = SystemConstants.Turret.SERVO_RANGE_DEG;
        /** Turret pivot from the robot's pose center, inches, robot frame {forward, left}. */
        final double[] offset;
        /** Wiring limits and INIT position, servo units, as turret.*_TURRET_MIN / MAX / START. */
        double min, max, start;

        TurretCal(String name, String device, double center, boolean reversed, double[] gear,
                  double[] offset, double min, double max, double start) {
            this.name = name;
            this.min = min;
            this.max = max;
            this.start = start;
            this.offset = offset.clone();
            this.device = device;
            this.center = center;
            this.reversed = reversed;
            this.gearA = gear[0];
            this.gearB = gear[1];
            servo = hardwareMap.tryGet(Servo.class, device);
            Turret.configurePwm(servo); // same pulse range as the competition turret
        }

        /** Turret degrees across the full 0-1 servo travel, exactly as turret.TurretServo.track. */
        double range() {
            return servoRange * gearB / gearA;
        }

        double positionFor(double deg) {
            return center + (reversed ? -1 : 1) * deg / range();
        }

        double degFor(double p) {
            return (p - center) * range() * (reversed ? -1 : 1);
        }

        /** Commands are held inside the marked limits unless free (only the limits tab sends free). */
        void setPosition(double p, boolean free) {
            if (servo == null) throw new IllegalStateException("no servo named " + device);
            pos = free ? clamp(p, 0, 1) : clamp(p, Math.min(min, max), Math.max(min, max));
            servo.setPosition(pos);
        }
    }

    private final class ServoCal {
        final String name;
        final String device;
        final String file;
        final Map<String, Double> marks = new LinkedHashMap<>();
        final Map<String, String> fields = new LinkedHashMap<>();
        Servo servo;
        double pos = Double.NaN;

        ServoCal(String name, String device, String file, String[] markToField) {
            this.name = name;
            this.device = device;
            this.file = file;
            for (int i = 0; i < markToField.length; i += 2) {
                marks.put(markToField[i], null);
                fields.put(markToField[i], markToField[i + 1]);
            }
            servo = hardwareMap.tryGet(Servo.class, device);
        }

        void setPosition(double p) {
            if (servo == null) throw new IllegalStateException("no servo named " + device);
            pos = clamp(p, 0, 1);
            servo.setPosition(pos);
        }

        /** Opens, then closes on a background thread so the loop never sleeps. */
        void pulse(final double open, final double close, final double ms) {
            setPosition(open);
            final Servo s = servo;
            Thread t = new Thread(new Runnable() {
                @Override
                public void run() {
                    try {
                        Thread.sleep((long) clamp(ms, 20, 3000));
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    }
                    s.setPosition(close);
                }
            }, "MechanismTuner-gatePulse");
            t.setDaemon(true);
            t.start();
            pos = close;
        }
    }

    // ------------------------------------------------------------------------- color

    private double[] suggestion;

    private void rebuildColorSummary() {
        Map<String, List<int[]>> byLabel = new LinkedHashMap<>();
        StringBuilder csv = new StringBuilder("label,r,g,b,a,dist\n");
        for (int i = 0; i < samples.size(); i++) {
            String l = sampleLabels.get(i);
            List<int[]> list = byLabel.get(l);
            if (list == null) {
                list = new ArrayList<>();
                byLabel.put(l, list);
            }
            int[] s = samples.get(i);
            list.add(new int[]{s[0], s[1], s[2], s[3], i});
            csv.append(l).append(',').append(s[0]).append(',').append(s[1]).append(',')
                    .append(s[2]).append(',').append(s[3]).append(',')
                    .append(FlyRecorder.r(sampleDist.get(i), 2)).append('\n');
        }
        TuningHub.INSTANCE.setColorCsv(csv.toString());

        StringBuilder sb = new StringBuilder(800);
        sb.append("{\"labels\":[");
        boolean first = true;
        for (Map.Entry<String, List<int[]>> e : byLabel.entrySet()) {
            if (!first) sb.append(',');
            first = false;
            int[] mn = {Integer.MAX_VALUE, Integer.MAX_VALUE, Integer.MAX_VALUE, Integer.MAX_VALUE};
            int[] mx = {Integer.MIN_VALUE, Integer.MIN_VALUE, Integer.MIN_VALUE, Integer.MIN_VALUE};
            double[] mean = new double[4];
            int redHits = 0, blueHits = 0;
            for (int[] s : e.getValue()) {
                for (int k = 0; k < 4; k++) {
                    mn[k] = Math.min(mn[k], s[k]);
                    mx[k] = Math.max(mx[k], s[k]);
                    mean[k] += s[k] / (double) e.getValue().size();
                }
                double d = sampleDist.get(s[4]);
                if (ColorCheck.calledRed(s[0], s[2], d)) redHits++;
                if (ColorCheck.calledBlue(s[0], s[2], d)) blueHits++;
            }
            sb.append("{\"label\":\"").append(TuningHub.esc(e.getKey()))
                    .append("\",\"n\":").append(e.getValue().size())
                    .append(",\"mean\":[").append(FlyRecorder.r(mean[0], 1)).append(',')
                    .append(FlyRecorder.r(mean[1], 1)).append(',').append(FlyRecorder.r(mean[2], 1))
                    .append(',').append(FlyRecorder.r(mean[3], 1)).append(']')
                    .append(",\"min\":[").append(mn[0]).append(',').append(mn[1]).append(',')
                    .append(mn[2]).append(',').append(mn[3]).append(']')
                    .append(",\"max\":[").append(mx[0]).append(',').append(mx[1]).append(',')
                    .append(mx[2]).append(',').append(mx[3]).append(']')
                    .append(",\"redHits\":").append(redHits)
                    .append(",\"blueHits\":").append(blueHits).append('}');
        }
        sb.append(']');

        suggestion = null;
        if (byLabel.containsKey("RED") && byLabel.containsKey("BLUE") && byLabel.containsKey("EMPTY")) {
            double dist = fitDist();
            double[] sug = {fitRatio(true, dist), fitRatio(false, dist), dist};
            suggestion = sug;
            int[] cur = confusion(SystemConstants.ColorCheck.RED_OVER_BLUE, SystemConstants.ColorCheck.BLUE_OVER_RED, SystemConstants.ColorCheck.MAX_DIST_CM);
            int[] fit = confusion(sug[0], sug[1], sug[2]);
            sb.append(",\"suggest\":{\"redOverBlue\":").append(FlyRecorder.r(sug[0], 3))
                    .append(",\"blueOverRed\":").append(FlyRecorder.r(sug[1], 3))
                    .append(",\"maxDistCm\":").append(FlyRecorder.r(sug[2], 2))
                    .append(",\"errorsSuggested\":").append(fit[0])
                    .append(",\"errorsCurrent\":").append(cur[0])
                    .append(",\"total\":").append(fit[1]).append('}');
        } else {
            sb.append(",\"suggest\":null");
        }
        sb.append('}');
        colorSummary = sb.toString();
    }

    // Fits for the shipped ColorCheck rule (present && ratio). Each is a 1-D grid search that
    // maximises correct calls and takes the middle of the tied-best run, so the boundary sits as far
    // from both clusters as the data allows. Ratios use a log grid (middle = geometric midpoint).

    /** Piece-present gate: RED and BLUE samples must be under it, EMPTY over it. Other labels ignored. */
    private double fitDist() {
        List<Double> grid = new ArrayList<>();
        for (int k = 5; k <= 300; k++) grid.add(k * 0.05);
        int best = -1;
        List<Double> ties = new ArrayList<>();
        for (double g : grid) {
            int ok = 0;
            for (int i = 0; i < samples.size(); i++) {
                String l = sampleLabels.get(i);
                boolean piece = "RED".equals(l) || "BLUE".equals(l);
                if (!piece && !"EMPTY".equals(l)) continue;
                double d = sampleDist.get(i);
                if (Double.isNaN(d)) continue;
                if ((d < g) == piece) ok++;
            }
            if (ok > best) { best = ok; ties.clear(); }
            if (ok == best) ties.add(g);
        }
        return ties.get(ties.size() / 2);
    }

    /** RED_OVER_BLUE (red=true) or BLUE_OVER_RED, over every sample, with the distance gate applied. */
    private double fitRatio(boolean red, double maxDist) {
        String pos = red ? "RED" : "BLUE";
        int best = -1;
        List<Double> ties = new ArrayList<>();
        for (int k = 0; k <= 400; k++) {
            double g = 0.25 * Math.pow(10, k / 200.0); // 0.25 .. 25
            int ok = 0;
            for (int i = 0; i < samples.size(); i++) {
                int[] s = samples.get(i);
                boolean present = !(sampleDist.get(i) >= maxDist);
                boolean got = present && (red ? s[0] > g * s[2] : s[2] > g * s[0]);
                if (got == pos.equals(sampleLabels.get(i))) ok++;
            }
            if (ok > best) { best = ok; ties.clear(); }
            if (ok == best) ties.add(g);
        }
        return ties.get(ties.size() / 2);
    }

    /** {misclassified, total} over both calls for a parameter set. */
    private int[] confusion(double redOverBlue, double blueOverRed, double maxDist) {
        double r0 = SystemConstants.ColorCheck.RED_OVER_BLUE, b0 = SystemConstants.ColorCheck.BLUE_OVER_RED, d0 = SystemConstants.ColorCheck.MAX_DIST_CM;
        SystemConstants.ColorCheck.RED_OVER_BLUE = redOverBlue;
        SystemConstants.ColorCheck.BLUE_OVER_RED = blueOverRed;
        SystemConstants.ColorCheck.MAX_DIST_CM = maxDist;
        int wrong = 0;
        for (int i = 0; i < samples.size(); i++) {
            int[] s = samples.get(i);
            String l = sampleLabels.get(i);
            double d = sampleDist.get(i);
            if (ColorCheck.calledRed(s[0], s[2], d) != "RED".equals(l)) wrong++;
            if (ColorCheck.calledBlue(s[0], s[2], d) != "BLUE".equals(l)) wrong++;
        }
        SystemConstants.ColorCheck.RED_OVER_BLUE = r0;
        SystemConstants.ColorCheck.BLUE_OVER_RED = b0;
        SystemConstants.ColorCheck.MAX_DIST_CM = d0;
        return new int[]{wrong, samples.size() * 2};
    }

    // ------------------------------------------------------------------------- publish

    private String stateJson() {
        StringBuilder sb = new StringBuilder(4096);
        int n = Math.min(dtCount, dtRing.length);
        double[] dts = Arrays.copyOf(dtRing, n);
        Arrays.sort(dts);
        double dtSum = 0;
        for (double d : dts) dtSum += d;
        sb.append("{\"live\":true,\"sessionId\":").append(sessionId)
                .append(",\"started\":").append(started)
                .append(",\"message\":\"").append(TuningHub.esc(message))
                .append("\",\"cmdSeq\":").append(cmdSeq)
                .append(",\"volts\":").append(FlyRecorder.r(volts, 2))
                .append(",\"loopHzTrue\":").append(n == 0 || dtSum == 0 ? "null" : FlyRecorder.r(n / dtSum, 1))
                .append(",\"dtMeanMs\":").append(n == 0 ? "null" : FlyRecorder.r(1000 * dtSum / n, 2))
                .append(",\"dtP90Ms\":").append(n == 0 ? "null" : FlyRecorder.r(1000 * dts[(int) (0.9 * (n - 1))], 2))
                .append(",\"watchdog\":").append(watchdogTripped)
                .append(",\"clientAgeMs\":").append(TuningHub.INSTANCE.msSinceClient())
                .append(",\"clientTimeoutMs\":").append(CLIENT_TIMEOUT_MS)
                .append(",\"velCap\":").append(VEL_CAP_TPS)
                .append(",\"atSpeedFrac\":").append(FlyRecorder.r(atSpeedFrac, 4));

        sb.append(",\"wheels\":[");
        for (int i = 0; i < wheels.length; i++) {
            Wheel w = wheels[i];
            if (i > 0) sb.append(',');
            boolean atSpeed = w.mode == MODE_VEL && w.target != 0
                    && Math.abs(w.vel - w.target) <= atSpeedFrac * Math.abs(w.target);
            sb.append("{\"name\":\"").append(w.name)
                    .append("\",\"device\":\"").append(w.device)
                    .append("\",\"ok\":").append(w.motor != null)
                    .append(",\"error\":\"").append(TuningHub.esc(w.error))
                    .append("\",\"mode\":\"").append(MODE_NAMES[w.mode])
                    .append("\",\"target\":").append(FlyRecorder.r(w.target, 1))
                    .append(",\"power\":").append(nanToNull(w.power, 3))
                    .append(",\"vel\":").append(FlyRecorder.r(w.vel, 1))
                    .append(",\"cur\":").append(nanToNull(w.cur, 2))
                    .append(",\"atSpeed\":").append(atSpeed)
                    .append(",\"reversed\":").append(w.reversed)
                    .append(",\"kicking\":").append(w.kicking)
                    .append(",\"pidf\":[").append(w.kP).append(',').append(w.kI).append(',')
                    .append(w.kD).append(',').append(w.kF).append(']')
                    .append(",\"shippedPidf\":[").append(w.shippedPidf[0]).append(',')
                    .append(w.shippedPidf[1]).append(',').append(w.shippedPidf[2]).append(',')
                    .append(w.shippedPidf[3]).append(']')
                    .append(",\"hubPidf\":\"").append(w.hubPidf)
                    .append("\",\"dipState\":").append(w.dip)
                    .append(",\"stepPhase\":").append(w.stepPhase)
                    .append(",\"lastStep\":").append(w.lastStep)
                    .append(",\"ffRunning\":").append(w.ffIdx >= 0)
                    .append(",\"ffLevel\":").append(w.ffIdx)
                    .append(",\"ff\":").append(w.ffResult)
                    .append('}');
        }
        sb.append(']');

        sb.append(",\"events\":[");
        for (int i = Math.max(0, events.size() - 25); i < events.size(); i++) {
            if (i > Math.max(0, events.size() - 25)) sb.append(',');
            sb.append(events.get(i));
        }
        sb.append("],\"eventCount\":").append(eventSeq);

        sb.append(",\"rec\":{\"recording\":").append(recorder.isRecording())
                .append(",\"label\":\"").append(TuningHub.esc(recorder.label()))
                .append("\",\"size\":").append(recorder.size())
                .append(",\"capacity\":").append(FlyRecorder.CAPACITY)
                .append(",\"current\":").append(recCurrent).append('}');

        ServoCal g = servos[0];
        boolean gateOpen = shootState == 2;
        sb.append(",\"shoot\":{\"state\":\"").append(shootState == 0 ? "OFF" : shootState == 1 ? "TRACK" : "SHOOT")
                .append("\",\"gateOpen\":").append(gateOpen)
                .append(",\"gateReady\":").append(g.servo != null && g.marks.get("open") != null
                        && g.marks.get("close") != null)
                .append(",\"left\":").append(shootState == 0 ? 0 : shootLeft)
                .append(",\"feed\":").append(shootFeed)
                .append(",\"phaseMs\":").append(shootState == 0 ? "null"
                        : FlyRecorder.r((System.nanoTime() - (gateOpen ? shootOpenNs : shootPhaseNs)) / 1e6, 0))
                .append(",\"end\":\"").append(TuningHub.esc(shootEnd))
                .append("\",\"entry\":").append(FlyRecorder.r(atSpeedFrac, 4))
                .append(",\"hold\":").append(FlyRecorder.r(holdFrac, 4))
                .append(",\"minOpenS\":").append(FlyRecorder.r(minGateOpenS, 3))
                .append(",\"idle\":").append(FlyRecorder.r(idleFrac, 4))
                .append(",\"windows\":[");
        for (int i = 0; i < shootWindows.size(); i++) {
            if (i > 0) sb.append(',');
            sb.append(shootWindows.get(i));
        }
        sb.append("]}");

        sb.append(",\"turrets\":[");
        for (int i = 0; i < turrets.length; i++) {
            TurretCal t = turrets[i];
            if (i > 0) sb.append(',');
            sb.append("{\"name\":\"").append(t.name)
                    .append("\",\"device\":\"").append(t.device)
                    .append("\",\"ok\":").append(t.servo != null)
                    .append(",\"pos\":").append(nanToNull(t.pos, 4))
                    .append(",\"deg\":").append(nanToNull(t.degFor(t.pos), 1))
                    .append(",\"center\":").append(FlyRecorder.r(t.center, 4))
                    .append(",\"reversed\":").append(t.reversed)
                    .append(",\"grA\":").append(t.gearA)
                    .append(",\"grB\":").append(t.gearB)
                    .append(",\"servoRange\":").append(t.servoRange)
                    .append(",\"rangeDeg\":").append(FlyRecorder.r(t.range(), 2))
                    .append(",\"offX\":").append(t.offset[0])
                    .append(",\"offY\":").append(t.offset[1])
                    .append(",\"min\":").append(FlyRecorder.r(t.min, 4))
                    .append(",\"max\":").append(FlyRecorder.r(t.max, 4))
                    .append(",\"start\":").append(FlyRecorder.r(t.start, 4))
                    .append(",\"minDeg\":").append(FlyRecorder.r(t.degFor(t.min), 1))
                    .append(",\"maxDeg\":").append(FlyRecorder.r(t.degFor(t.max), 1))
                    .append(",\"startDeg\":").append(FlyRecorder.r(t.degFor(t.start), 1))
                    .append(",\"turnPos\":").append(FlyRecorder.r(360.0 / t.range(), 4)).append('}');
        }
        sb.append(']');

        sb.append(",\"servos\":[");
        for (int i = 0; i < servos.length; i++) {
            ServoCal s = servos[i];
            if (i > 0) sb.append(',');
            sb.append("{\"name\":\"").append(s.name)
                    .append("\",\"device\":\"").append(s.device)
                    .append("\",\"ok\":").append(s.servo != null)
                    .append(",\"pos\":").append(nanToNull(s.pos, 4))
                    .append(",\"marks\":{");
            boolean first = true;
            for (Map.Entry<String, Double> m : s.marks.entrySet()) {
                if (!first) sb.append(',');
                first = false;
                sb.append('"').append(m.getKey()).append("\":")
                        .append(m.getValue() == null ? "null" : FlyRecorder.r(m.getValue(), 4));
            }
            sb.append("}}");
        }
        sb.append(']');

        sb.append(",\"intake\":{\"ok\":").append(intakeSys != null)
                .append(",\"error\":\"").append(TuningHub.esc(intakeError))
                .append("\",\"mode\":\"").append(intakeMode)
                .append("\",\"alliance\":\"").append(alliance)
                .append("\",\"brakeMs\":").append(FlyRecorder.r(SystemConstants.Intake.REVERSE_BRAKE_MS, 1))
                .append(",\"motorsOk\":[").append(intakeMotors[0] != null).append(',').append(intakeMotors[1] != null)
                .append("],\"leftRev\":").append(SystemConstants.Intake.INTAKE_LEFT_REVERSED)
                .append(",\"rightRev\":").append(SystemConstants.Intake.INTAKE_RIGHT_REVERSED)
                .append(",\"rev\":").append(SystemConstants.Intake.INTAKE_REVERSED)
                .append(",\"test\":\"").append(intakeTest.isEmpty() ? "OFF" : intakeTest)
                .append("\",\"testPower\":").append(FlyRecorder.r(intakeTestPower, 2)).append('}');

        sb.append(",\"breakIn\":{\"ok\":").append(intakeMotors[0] != null || intakeMotors[1] != null)
                .append(",\"running\":").append(breakIn)
                .append(",\"resting\":").append(breakInResting)
                .append(",\"dir\":").append(breakInDir)
                .append(",\"runS\":").append(FlyRecorder.r(breakInRunS, 1))
                .append(",\"totalS\":").append(FlyRecorder.r(breakInTotalS, 1))
                .append(",\"power\":").append(FlyRecorder.r(breakInPower, 3))
                .append(",\"flipS\":").append(FlyRecorder.r(breakInFlipS, 1))
                .append(",\"restS\":").append(FlyRecorder.r(breakInRestS, 2))
                .append(",\"stallA\":").append(FlyRecorder.r(breakInStallA, 2))
                .append(",\"stallS\":").append(FlyRecorder.r(breakInStallS, 2))
                .append(",\"flips\":").append(breakInFlips)
                .append(",\"amps\":[").append(nanToNull(intakeAmps[0], 2)).append(',').append(nanToNull(intakeAmps[1], 2))
                .append("],\"maxAmps\":[").append(FlyRecorder.r(intakeMaxAmps[0], 2)).append(',').append(FlyRecorder.r(intakeMaxAmps[1], 2))
                .append("],\"end\":\"").append(TuningHub.esc(breakInEnd)).append("\"}");

        sb.append(",\"zone\":{\"x\":").append(SystemConstants.Zone.xQuality)
                .append(",\"y\":").append(SystemConstants.Zone.yQuality)
                .append(",\"field\":").append(SystemConstants.Zone.FIELD_SIZE)
                .append(",\"offPollen\":").append(SystemConstants.Zone.yOffsetPollen)
                .append(",\"offNectar\":").append(SystemConstants.Zone.yOffsetNectar)
                .append(",\"pollen\":").append(mapJson(SystemConstants.Zone.mapPollen))
                .append(",\"nectar\":").append(mapJson(SystemConstants.Zone.mapNectar)).append('}');

        sb.append(",\"color\":{\"ok\":").append(color != null)
                .append(",\"hasDistance\":").append(colorDistance != null)
                .append(",\"stream\":").append(colorStream)
                .append(",\"pausedForRec\":").append(recorder.isRecording())
                .append(",\"r\":").append(cr).append(",\"g\":").append(cg)
                .append(",\"b\":").append(cb).append(",\"a\":").append(ca)
                .append(",\"dist\":").append(nanToNull(cdist, 2))
                .append(",\"redDominant\":").append(ColorCheck.calledRed(cr, cb, cdist))
                .append(",\"blueDominant\":").append(ColorCheck.calledBlue(cr, cb, cdist))
                .append(",\"redOverBlue\":").append(SystemConstants.ColorCheck.RED_OVER_BLUE)
                .append(",\"blueOverRed\":").append(SystemConstants.ColorCheck.BLUE_OVER_RED)
                .append(",\"maxDistCm\":").append(SystemConstants.ColorCheck.MAX_DIST_CM)
                .append(",\"sampling\":").append(sampleRemaining)
                .append(",\"samplingLabel\":\"").append(TuningHub.esc(sampleLabel))
                .append("\",\"samples\":").append(samples.size())
                .append(",\"summary\":").append(colorSummary).append('}');

        sb.append(",\"reg\":").append(regJson());
        sb.append(",\"export\":").append(exportJson());
        return sb.append('}').toString();
    }

    /**
     * Every value this tool owns, as {file, name, value, shipped} with values written as Java
     * source; shipped is the value at init, or null when the tool cannot read it (the gate and
     * scoopula positions are instance fields). The host splicer (tools/mechtune/mechtune.py export
     * --write) replaces the right-hand side of that declaration by name. Unmarked servo positions
     * are left out rather than exported as 0.
     */
    private String exportJson() {
        List<String[]> out = exportEntries();
        StringBuilder sb = new StringBuilder(2048).append('[');
        for (int i = 0; i < out.size(); i++) {
            String[] e = out.get(i);
            String shipped = shippedValues.get(e[0] + "#" + e[1]);
            if (i > 0) sb.append(',');
            sb.append("{\"file\":\"").append(e.length > 3 ? e[3] : EXPORT_FILE)
                    .append("\",\"group\":\"").append(e[0])
                    .append("\",\"name\":\"").append(e[1])
                    .append("\",\"value\":\"").append(TuningHub.esc(e[2]))
                    .append("\",\"shipped\":").append(shipped == null ? "null"
                            : "\"" + TuningHub.esc(shipped) + "\"").append('}');
        }
        return sb.append(']').toString();
    }

    /**
     * Exported values live in SystemConstants.java unless the entry names its file in e[3]; e[0]
     * of an entry is its --only group.
     */
    static final String EXPORT_FILE = "SystemConstants.java";
    /** The regression fit is the one export that lands outside SystemConstants (e[3] of its entries). */
    static final String REGRESSION_FILE = "modules/ShootingRegression.java";

    private List<String[]> exportEntries() {
        List<String[]> out = new ArrayList<>();
        String fw = "flywheel";
        out.add(new String[]{fw, "POLLEN_FLYWHEEL_REVERSED", Boolean.toString(wheels[0].reversed)});
        out.add(new String[]{fw, "NECTAR_FLYWHEEL_REVERSED", Boolean.toString(wheels[1].reversed)});
        out.add(new String[]{fw, "AT_SPEED_FRACTION", Double.toString(atSpeedFrac)});
        out.add(new String[]{fw, "IDLE_FRACTION", Double.toString(idleFrac)});
        out.add(new String[]{fw, "POLLEN_PIDF", pidfSource(wheels[0])});
        out.add(new String[]{fw, "NECTAR_PIDF", pidfSource(wheels[1])});
        String tu = "turret";
        out.add(new String[]{tu, "POLLEN_TURRET_CENTER", FlyRecorder.r(turrets[0].center, 4)});
        out.add(new String[]{tu, "NECTAR_TURRET_CENTER", FlyRecorder.r(turrets[1].center, 4)});
        out.add(new String[]{tu, "POLLEN_TURRET_REVERSED", Boolean.toString(turrets[0].reversed)});
        out.add(new String[]{tu, "NECTAR_TURRET_REVERSED", Boolean.toString(turrets[1].reversed)});
        out.add(new String[]{tu, "POLLEN_TURRET_GEAR_RATIO", gearSource(turrets[0])});
        out.add(new String[]{tu, "NECTAR_TURRET_GEAR_RATIO", gearSource(turrets[1])});
        out.add(new String[]{tu, "SERVO_RANGE_DEG", trim(turrets[0].servoRange)});
        out.add(new String[]{tu, "POLLEN_TURRET_MIN", FlyRecorder.r(turrets[0].min, 4)});
        out.add(new String[]{tu, "POLLEN_TURRET_MAX", FlyRecorder.r(turrets[0].max, 4)});
        out.add(new String[]{tu, "NECTAR_TURRET_MIN", FlyRecorder.r(turrets[1].min, 4)});
        out.add(new String[]{tu, "NECTAR_TURRET_MAX", FlyRecorder.r(turrets[1].max, 4)});
        out.add(new String[]{tu, "POLLEN_TURRET_START", FlyRecorder.r(turrets[0].start, 4)});
        out.add(new String[]{tu, "NECTAR_TURRET_START", FlyRecorder.r(turrets[1].start, 4)});
        out.add(new String[]{tu, "pollenTurretOffset", arraySource(turrets[0].offset)});
        out.add(new String[]{tu, "nectarTurretOffset", arraySource(turrets[1].offset)});
        String sh = "shooter";
        out.add(new String[]{sh, "MIN_GATE_OPEN_S", Double.toString(minGateOpenS)});
        out.add(new String[]{sh, "HOLD_SPEED_FRACTION", Double.toString(holdFrac)});
        String in = "intake";
        String cc = "ColorCheck";
        out.add(new String[]{cc, "RED_OVER_BLUE", trim(SystemConstants.ColorCheck.RED_OVER_BLUE)});
        out.add(new String[]{cc, "BLUE_OVER_RED", trim(SystemConstants.ColorCheck.BLUE_OVER_RED)});
        out.add(new String[]{cc, "MAX_DIST_CM", trim(SystemConstants.ColorCheck.MAX_DIST_CM)});
        out.add(new String[]{in, "REVERSE_BRAKE_MS", trim(SystemConstants.Intake.REVERSE_BRAKE_MS)});
        out.add(new String[]{in, "INTAKE_REVERSED", Boolean.toString(SystemConstants.Intake.INTAKE_REVERSED)});
        out.add(new String[]{in, "INTAKE_LEFT_REVERSED", Boolean.toString(SystemConstants.Intake.INTAKE_LEFT_REVERSED)});
        out.add(new String[]{in, "INTAKE_RIGHT_REVERSED", Boolean.toString(SystemConstants.Intake.INTAKE_RIGHT_REVERSED)});
        String zn = "zone";
        out.add(new String[]{zn, "xQuality", Integer.toString(SystemConstants.Zone.xQuality)});
        out.add(new String[]{zn, "yQuality", Integer.toString(SystemConstants.Zone.yQuality)});
        out.add(new String[]{zn, "yOffsetPollen", Integer.toString(SystemConstants.Zone.yOffsetPollen)});
        out.add(new String[]{zn, "yOffsetNectar", Integer.toString(SystemConstants.Zone.yOffsetNectar)});
        out.add(new String[]{zn, "mapPollen", mapSource(SystemConstants.Zone.mapPollen)});
        out.add(new String[]{zn, "mapNectar", mapSource(SystemConstants.Zone.mapNectar)});
        // The fit saved on the Regression tab, or ShootingRegression's own value when none was saved.
        String rg = "regression";
        out.add(new String[]{rg, "POLLEN_COEFFS", reg.coeffSource(0), REGRESSION_FILE});
        out.add(new String[]{rg, "POLLEN_RANGE", reg.rangeSource(0), REGRESSION_FILE});
        out.add(new String[]{rg, "NECTAR_COEFFS", reg.coeffSource(1), REGRESSION_FILE});
        out.add(new String[]{rg, "NECTAR_RANGE", reg.rangeSource(1), REGRESSION_FILE});
        for (ServoCal s : servos) {
            for (Map.Entry<String, Double> m : s.marks.entrySet()) {
                if (m.getValue() != null) {
                    out.add(new String[]{s.file, s.fields.get(m.getKey()),
                            FlyRecorder.r(m.getValue(), 4)});
                }
            }
        }
        return out;
    }

    private static String pidfSource(Wheel w) {
        return "new PIDFCoefficients(" + trim(w.kP) + ", " + trim(w.kI) + ", " + trim(w.kD)
                + ", " + trim(w.kF) + ")";
    }

    private static String gearSource(TurretCal t) {
        return "{" + trim(t.gearA) + ", " + trim(t.gearB) + "}";
    }

    private static String arraySource(double[] v) {
        return "{" + trim(v[0]) + ", " + trim(v[1]) + "}";
    }

    private static String trim(double v) {
        return v == Math.rint(v) && Math.abs(v) < 1e9 ? Long.toString((long) v) : Double.toString(v);
    }

    // ------------------------------------------------------------------------- helpers

    private static double num(Map<String, String> c, String key, double fallback) {
        String v = c.get(key);
        if (v == null || v.trim().isEmpty()) return fallback;
        double d;
        try {
            d = Double.parseDouble(v.trim());
        } catch (NumberFormatException e) {
            throw new IllegalArgumentException(key + "=" + v + " is not a number");
        }
        // parseDouble accepts "NaN" (what an empty number box sends): it would reach the hub as a
        // gain and break the state JSON, so it is refused like any other non-number.
        if (Double.isNaN(d) || Double.isInfinite(d)) {
            throw new IllegalArgumentException(key + "=" + v + " is not a number");
        }
        return d;
    }

    private static double[] parseList(String s, double[] fallback) {
        if (s == null || s.trim().isEmpty()) return fallback;
        String[] parts = s.split(",");
        double[] out = new double[parts.length];
        for (int i = 0; i < parts.length; i++) {
            out[i] = clamp(Double.parseDouble(parts[i].trim()), -1, 1);
        }
        return out;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static String nanToNull(double v, int digits) {
        return Double.isNaN(v) || Double.isInfinite(v) ? "null" : FlyRecorder.r(v, digits);
    }

    private static String shortError(Throwable e) {
        return e.getClass().getSimpleName() + (e.getMessage() == null ? "" : ": " + e.getMessage());
    }
}
