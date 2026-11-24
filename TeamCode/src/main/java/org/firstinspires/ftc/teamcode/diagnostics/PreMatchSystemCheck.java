package org.firstinspires.ftc.teamcode.diagnostics;

import org.firstinspires.ftc.teamcode.SystemConstants;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.systems.Flywheel;
import org.firstinspires.ftc.teamcode.systems.Gate;
import org.firstinspires.ftc.teamcode.systems.Scoopula;
import org.firstinspires.ftc.teamcode.systems.Turret;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class PreMatchSystemCheck extends OpMode {
    private static final Pose START_POSE = new Pose(0.0, 0.0, 0.0);
    private static final double CHECK_POWER = 0.30;
    private static final double DRIVE_SECONDS = 0.85;
    private static final double STOP_SECONDS = 0.25;

    private static final double LOW_BATTERY_WARNING_VOLTS = 12.0;
    private static final double MIN_TRANSLATION_INCHES = 1.25;
    private static final double MIN_TURN_RADIANS = Math.toRadians(8.0);
    private static final double MAX_TRANSLATION_DRIFT_INCHES = 2.5;
    private static final double MAX_TURN_DRIFT_INCHES = 3.0;
    private static final double MAX_HEADING_DRIFT_RADIANS = Math.toRadians(18.0);
    private static final double MIN_COMMANDED_DRIVE_POWER = 0.04;
    private static final double MIN_OBSERVED_VELOCITY = 0.75;

    /** Pods Swerve.debug() must report, one nested map each. */
    private static final int EXPECTED_PODS = 4;

    // Mechanism thresholds - placeholders until the mechanisms exist. They separate "connected and
    // turning the configured way" from "unplugged / encoder unplugged / reversed", nothing finer.
    private static final double FLYWHEEL_CHECK_POWER = 0.40;
    private static final double MIN_FLYWHEEL_TPS = 100.0;
    private static final double INTAKE_CHECK_POWER = 0.50;
    private static final double MIN_MOTOR_AMPS = 0.05;

    private static final CheckStep[] STEPS = new CheckStep[] {
            new CheckStep("Settle before checks", CheckType.SETTLE, 0.0, 0.0, 0.0, 0.40),
            new CheckStep("Forward drive response", CheckType.FORWARD_POSITIVE,
                    CHECK_POWER, 0.0, 0.0, DRIVE_SECONDS),
            new CheckStep("Stop after forward", CheckType.SETTLE, 0.0, 0.0, 0.0, STOP_SECONDS),
            new CheckStep("Backward drive response", CheckType.FORWARD_NEGATIVE,
                    -CHECK_POWER, 0.0, 0.0, DRIVE_SECONDS),
            new CheckStep("Stop after backward", CheckType.SETTLE, 0.0, 0.0, 0.0, STOP_SECONDS),
            new CheckStep("Left strafe response", CheckType.STRAFE_POSITIVE,
                    0.0, CHECK_POWER, 0.0, DRIVE_SECONDS),
            new CheckStep("Stop after left strafe", CheckType.SETTLE, 0.0, 0.0, 0.0, STOP_SECONDS),
            new CheckStep("Right strafe response", CheckType.STRAFE_NEGATIVE,
                    0.0, -CHECK_POWER, 0.0, DRIVE_SECONDS),
            new CheckStep("Stop after right strafe", CheckType.SETTLE, 0.0, 0.0, 0.0, STOP_SECONDS),
            new CheckStep("Counterclockwise turn response", CheckType.TURN_POSITIVE,
                    0.0, 0.0, CHECK_POWER, DRIVE_SECONDS),
            new CheckStep("Stop after counterclockwise turn", CheckType.SETTLE,
                    0.0, 0.0, 0.0, STOP_SECONDS),
            new CheckStep("Clockwise turn response", CheckType.TURN_NEGATIVE,
                    0.0, 0.0, -CHECK_POWER, DRIVE_SECONDS)
    };

    private static final MechStep[] MECH_STEPS = MechStep.values();

    private final ElapsedTime stepTimer = new ElapsedTime();
    private final List<String> failures = new ArrayList<>();
    private final List<String> warnings = new ArrayList<>();
    /** One line per mechanism, shown under "Mechanisms" - PASS / FAIL / LOOK / SKIPPED. */
    private final List<String> mechanismResults = new ArrayList<>();

    private Follower follower;
    /** The sensor Pedro 2's Swerve.getVoltage() read; Pedro 3's Drivetrain has no accessor. */
    private VoltageSensor voltageSensor;
    private Pose stepStartPose = START_POSE;
    private int stepIndex;
    private Phase phase = Phase.WAITING;
    private boolean batteryChecked;
    private String fatalInitError;
    private String lastStepResult = "Waiting for start";
    private String lastDebugString = "";
    private double maxVelocityThisStep;
    private double maxDrivePowerThisStep;
    private double maxServoPowerThisStep;

    // Mechanisms - null when missing from the active config (already recorded as a failure).
    private DcMotorEx pollenFlywheel;
    private DcMotorEx nectarFlywheel;
    private DcMotorEx intakeLeft;
    private DcMotorEx intakeRight;
    private Servo pollenTurretServo;
    private Servo nectarTurretServo;
    private ColorSensor colorSensor;
    private Gate shooterGate;
    private Scoopula scoopula;

    private int mechIndex;
    /** Signed velocity of largest magnitude seen this step, ticks/s. */
    private double peakVelocityThisStep;
    private final double[] maxAmpsThisStep = new double[2];
    private boolean servoReturned;

    @Override
    public void init() {
        try {
            follower = Constants.createFollower(hardwareMap);
            voltageSensor = hardwareMap.voltageSensor.iterator().next();
            follower.setPose(START_POSE);
            // Pedro 2's update() with no path only refreshed the pose; Pedro 3's IDLE update()
            // writes every servo. Keep init actuator-silent.
            follower.localizer.update();
            validateDebug(follower.drivetrain.debug());
        } catch (RuntimeException e) {
            fatalInitError = e.getClass().getSimpleName() + ": " + e.getMessage();
            addFailure("Follower failed to initialize. Check hardware names and Pedro constants.");
        }

        initMechanisms();
        updateTelemetry();
    }

    @Override
    public void init_loop() {
        if (follower != null) {
            follower.localizer.update();
            validateDebug(follower.drivetrain.debug());
        }

        updateTelemetry();
    }

    @Override
    public void start() {
        checkBatteryVoltage();

        if (follower == null) {
            // No drivetrain to test; the mechanisms still get checked.
            beginMechanismPhase();
            return;
        }

        follower.setPose(START_POSE);
        follower.manual(DrivePowers.zero());
        follower.update();

        stepIndex = 0;
        phase = Phase.DRIVE;
        beginStep();
    }

    @Override
    public void loop() {
        switch (phase) {
            case DRIVE:
                runDriveStep();
                break;
            case MECHANISMS:
                runMechanismStep();
                break;
            default:
                stopFollower();
                break;
        }

        updateTelemetry();
    }

    @Override
    public void stop() {
        stopFollower();
        stopMechanisms();
    }

    private void runDriveStep() {
        CheckStep step = STEPS[stepIndex];
        follower.manual(new DrivePowers(step.forward, step.strafe, step.turn));
        follower.update();

        sampleStepHealth();

        if (stepTimer.seconds() >= step.durationSeconds) {
            finishStep(step);
            stepIndex++;

            if (stepIndex >= STEPS.length) {
                stopFollower();
                beginMechanismPhase();
            } else {
                beginStep();
            }
        }
    }

    private void beginStep() {
        stepStartPose = follower.pose();
        maxVelocityThisStep = 0.0;
        maxDrivePowerThisStep = 0.0;
        maxServoPowerThisStep = 0.0;
        lastStepResult = "Running: " + STEPS[stepIndex].name;
        stepTimer.reset();
    }

    private void finishStep(CheckStep step) {
        int failureCountBeforeStep = failures.size();
        Pose endPose = follower.pose();
        double dx = endPose.x() - stepStartPose.x();
        double dy = endPose.y() - stepStartPose.y();
        double headingDelta = angleDelta(endPose.heading(), stepStartPose.heading());

        validatePose(endPose);

        switch (step.type) {
            case FORWARD_POSITIVE:
                evaluateTranslation(step.name, dx, dy, headingDelta, 1, "X");
                break;
            case FORWARD_NEGATIVE:
                evaluateTranslation(step.name, dx, dy, headingDelta, -1, "X");
                break;
            case STRAFE_POSITIVE:
                evaluateTranslation(step.name, dy, dx, headingDelta, 1, "Y");
                break;
            case STRAFE_NEGATIVE:
                evaluateTranslation(step.name, dy, dx, headingDelta, -1, "Y");
                break;
            case TURN_POSITIVE:
                evaluateTurn(step.name, dx, dy, headingDelta, 1);
                break;
            case TURN_NEGATIVE:
                evaluateTurn(step.name, dx, dy, headingDelta, -1);
                break;
            case SETTLE:
            default:
                lastStepResult = step.name + " complete";
                return;
        }

        if (step.type != CheckType.SETTLE
                && maxDrivePowerThisStep < MIN_COMMANDED_DRIVE_POWER
                && maxVelocityThisStep < MIN_OBSERVED_VELOCITY) {
            addFailure(step.name + " saw almost no drivetrain output. Check module servo angles, "
                    + "drive motor wiring, and Pedro swerve constants.");
        }

        lastStepResult = failures.size() == failureCountBeforeStep
                ? "PASS: " + step.name
                : "FAIL: " + step.name;
    }

    private void evaluateTranslation(
            String stepName,
            double axisDelta,
            double crossAxisDelta,
            double headingDelta,
            int expectedSign,
            String axisName) {
        double signedDelta = axisDelta * expectedSign;
        double allowedCrossDrift = Math.max(MAX_TRANSLATION_DRIFT_INCHES,
                Math.abs(axisDelta) * 0.8);

        if (signedDelta < MIN_TRANSLATION_INCHES) {
            addFailure(stepName + " did not move in expected " + axisName + " direction. "
                    + "Delta=" + format(axisDelta) + " in.");
        }

        if (Math.abs(crossAxisDelta) > allowedCrossDrift) {
            addFailure(stepName + " drifted sideways too much. Cross delta="
                    + format(crossAxisDelta) + " in.");
        }

        if (Math.abs(headingDelta) > MAX_HEADING_DRIFT_RADIANS) {
            addFailure(stepName + " rotated while translating. Heading drift="
                    + format(Math.toDegrees(headingDelta)) + " deg.");
        }
    }

    private void evaluateTurn(String stepName, double dx, double dy, double headingDelta,
            int expectedSign) {
        double signedHeadingDelta = headingDelta * expectedSign;
        double translationDrift = Math.hypot(dx, dy);

        if (signedHeadingDelta < MIN_TURN_RADIANS) {
            addFailure(stepName + " did not turn in expected direction. Heading delta="
                    + format(Math.toDegrees(headingDelta)) + " deg.");
        }

        if (translationDrift > MAX_TURN_DRIFT_INCHES) {
            addFailure(stepName + " translated too much while turning. Drift="
                    + format(translationDrift) + " in.");
        }
    }

    private void sampleStepHealth() {
        Pose pose = follower.pose();
        validatePose(pose);

        double speed = follower.velocity().toVector2D().magnitude();
        if (isFinite(speed)) {
            maxVelocityThisStep = Math.max(maxVelocityThisStep, speed);
        }

        Map<String, Object> debug = follower.drivetrain.debug();
        validateDebug(debug);

        maxDrivePowerThisStep = Math.max(maxDrivePowerThisStep, maxAbsPodValue(debug, "drivePower"));
        maxServoPowerThisStep = Math.max(maxServoPowerThisStep, maxAbsPodValue(debug, "servoPower"));
    }

    // ------------------------------------------------------------------------- mechanisms

    /** Looks every mechanism device up by the names systems/ uses. Writes nothing. */
    private void initMechanisms() {
        pollenFlywheel = device(DcMotorEx.class, "pollenTurret", "pollen flywheel");
        nectarFlywheel = device(DcMotorEx.class, "nectarTurret", "nectar flywheel");
        intakeLeft = device(DcMotorEx.class, "intakeLeft", "intake");
        intakeRight = device(DcMotorEx.class, "intakeRight", "intake");
        pollenTurretServo = device(Servo.class, "pollenTurretServo", "pollen turret");
        nectarTurretServo = device(Servo.class, "nectarTurretServo", "nectar turret");
        Turret.configurePwm(pollenTurretServo); // same pulse range as the competition turret
        Turret.configurePwm(nectarTurretServo);
        colorSensor = hardwareMap.tryGet(ColorSensor.class, "colorSensor");   // not on the robot since 2026-11-24: optional
        if (device(Servo.class, "turretGate", "shooter gate") != null) {
            shooterGate = new Gate(hardwareMap);
        }
        if (device(Servo.class, "scoopula", "scoopula") != null) {
            scoopula = new Scoopula(hardwareMap);
        }

        checkTurretTravel("Pollen", SystemConstants.Turret.POLLEN_TURRET_GEAR_RATIO);
        checkTurretTravel("Nectar", SystemConstants.Turret.NECTAR_TURRET_GEAR_RATIO);
    }

    private <T> T device(Class<T> type, String name, String what) {
        T d = hardwareMap.tryGet(type, name);
        if (d == null) {
            addFailure("Missing " + what + " \"" + name + "\" (" + type.getSimpleName()
                    + ") in the active config.");
        }
        return d;
    }

    /** turret.track can only reach every heading if the turret covers a full turn. */
    private void checkTurretTravel(String name, double[] gearRatio) {
        double range = SystemConstants.Turret.SERVO_RANGE_DEG * gearRatio[1] / gearRatio[0];
        if (!isFinite(range) || range < 360.0) {
            addWarning(name + " turret covers " + format(range) + " deg over the servo's travel; "
                    + "under 360 it cannot aim at every heading.");
        }
    }

    private void beginMechanismPhase() {
        phase = Phase.MECHANISMS;
        checkColorSensor();
        mechIndex = 0;
        beginMechanismStep();
    }

    private void checkColorSensor() {
        if (colorSensor == null) {
            mechanismResults.add("Color sensor: SKIPPED (missing)");
            return;
        }
        try {
            int r = colorSensor.red();
            int g = colorSensor.green();
            int b = colorSensor.blue();
            int a = colorSensor.alpha();
            String reading = "r " + r + " g " + g + " b " + b + " a " + a;
            if (r == 0 && g == 0 && b == 0 && a == 0) {
                addFailure("Color sensor reads all zeros. Check its cable, and that the config's "
                        + "device type matches the fitted sensor.");
                mechanismResults.add("Color sensor: FAIL (" + reading + ")");
            } else {
                mechanismResults.add("Color sensor: PASS (" + reading + ")");
            }
        } catch (RuntimeException e) {
            addFailure("Color sensor read threw " + e.getClass().getSimpleName() + ".");
            mechanismResults.add("Color sensor: FAIL (read threw)");
        }
    }

    private void beginMechanismStep() {
        MechStep step = MECH_STEPS[mechIndex];
        peakVelocityThisStep = 0.0;
        maxAmpsThisStep[0] = 0.0;
        maxAmpsThisStep[1] = 0.0;
        servoReturned = false;
        lastStepResult = "Running: " + step.label;
        stepTimer.reset();

        switch (step) {
            case POLLEN_FLYWHEEL:
                startFlywheel(pollenFlywheel, SystemConstants.Flywheel.POLLEN_FLYWHEEL_REVERSED);
                break;
            case NECTAR_FLYWHEEL:
                startFlywheel(nectarFlywheel, SystemConstants.Flywheel.NECTAR_FLYWHEEL_REVERSED);
                break;
            case INTAKE:
                // Same sign systems/intake uses for INTAKE.
                setPower(intakeLeft, INTAKE_CHECK_POWER);
                setPower(intakeRight, INTAKE_CHECK_POWER);
                break;
            case TURRETS:
                if (pollenTurretServo != null) {
                    pollenTurretServo.setPosition(SystemConstants.Turret.POLLEN_TURRET_CENTER);
                }
                if (nectarTurretServo != null) {
                    nectarTurretServo.setPosition(SystemConstants.Turret.NECTAR_TURRET_CENTER);
                }
                break;
            case GATE:
                if (shooterGate != null && shooterGate.positionsSet()) {
                    shooterGate.openGate();
                }
                break;
            case SCOOPULA:
                if (scoopula != null && scoopula.positionsSet()) {
                    scoopula.update(true);
                }
                break;
        }
    }

    private void runMechanismStep() {
        MechStep step = MECH_STEPS[mechIndex];

        switch (step) {
            case POLLEN_FLYWHEEL:
                sampleMotor(pollenFlywheel, 0, true);
                break;
            case NECTAR_FLYWHEEL:
                sampleMotor(nectarFlywheel, 0, true);
                break;
            case INTAKE:
                sampleMotor(intakeLeft, 0, false);
                sampleMotor(intakeRight, 1, false);
                break;
            case GATE:
                // Open for the first half, closed for the second, so it ends where TeleOp starts.
                if (!servoReturned && stepTimer.seconds() >= step.seconds / 2
                        && shooterGate != null && shooterGate.positionsSet()) {
                    shooterGate.closeGate();
                    servoReturned = true;
                }
                break;
            case SCOOPULA:
                if (!servoReturned && stepTimer.seconds() >= step.seconds / 2
                        && scoopula != null && scoopula.positionsSet()) {
                    scoopula.update(false);
                    servoReturned = true;
                }
                break;
            default:
                break;
        }

        if (stepTimer.seconds() >= step.seconds) {
            finishMechanismStep(step);
            mechIndex++;
            if (mechIndex >= MECH_STEPS.length) {
                stopMechanisms();
                phase = Phase.DONE;
                lastStepResult = failures.isEmpty()
                        ? "All checks completed"
                        : "Checks completed with issues";
            } else {
                beginMechanismStep();
            }
        }
    }

    private void finishMechanismStep(MechStep step) {
        switch (step) {
            case POLLEN_FLYWHEEL:
                setPower(pollenFlywheel, 0.0);
                judgeFlywheel("Pollen flywheel", pollenFlywheel, "POLLEN_FLYWHEEL_REVERSED");
                break;
            case NECTAR_FLYWHEEL:
                setPower(nectarFlywheel, 0.0);
                judgeFlywheel("Nectar flywheel", nectarFlywheel, "NECTAR_FLYWHEEL_REVERSED");
                break;
            case INTAKE:
                setPower(intakeLeft, 0.0);
                setPower(intakeRight, 0.0);
                judgeCurrent("Intake left", intakeLeft, maxAmpsThisStep[0]);
                judgeCurrent("Intake right", intakeRight, maxAmpsThisStep[1]);
                break;
            case TURRETS:
                mechanismResults.add(pollenTurretServo == null && nectarTurretServo == null
                        ? "Turrets: SKIPPED (missing)"
                        : "Turrets: LOOK - both should have moved to their centre position");
                break;
            case GATE:
                judgeServo("Gate", shooterGate != null, shooterGate != null && shooterGate.positionsSet(),
                        "should have opened, then closed");
                break;
            case SCOOPULA:
                judgeServo("Scoopula", scoopula != null, scoopula != null && scoopula.positionsSet(),
                        "should have scooped, then stowed");
                break;
        }
        lastStepResult = step.label + " done";
    }

    private void startFlywheel(DcMotorEx motor, boolean reversed) {
        if (motor == null) {
            return;
        }
        // The direction systems/flywheel applies, open loop: this checks wiring, encoder and
        // direction without depending on the (untuned) velocity PIDF.
        motor.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(FLYWHEEL_CHECK_POWER);
    }

    private void sampleMotor(DcMotorEx motor, int slot, boolean trackVelocity) {
        if (motor == null) {
            return;
        }
        double amps = motor.getCurrent(CurrentUnit.AMPS);
        if (isFinite(amps)) {
            maxAmpsThisStep[slot] = Math.max(maxAmpsThisStep[slot], amps);
        }
        if (trackVelocity) {
            double v = motor.getVelocity();
            if (isFinite(v) && Math.abs(v) > Math.abs(peakVelocityThisStep)) {
                peakVelocityThisStep = v;
            }
        }
    }

    private void judgeFlywheel(String name, DcMotorEx motor, String reversedFlag) {
        if (motor == null) {
            mechanismResults.add(name + ": SKIPPED (missing)");
            return;
        }
        double v = peakVelocityThisStep;
        double amps = maxAmpsThisStep[0];
        String reading = format(v) + " ticks/s, " + format(amps) + " A";
        if (v <= -MIN_FLYWHEEL_TPS) {
            // shooter targets are positive and atSpeed compares against them, so this never shoots.
            addFailure(name + " turns backwards for positive power (" + reading + "). Flip "
                    + "flywheel." + reversedFlag + ", or the encoder is reversed.");
            mechanismResults.add(name + ": FAIL reversed (" + reading + ")");
        } else if (v < MIN_FLYWHEEL_TPS) {
            if (amps < MIN_MOTOR_AMPS) {
                addFailure(name + " drew no current at power " + format(FLYWHEEL_CHECK_POWER)
                        + ". Motor unplugged or on the wrong port?");
            } else {
                addFailure(name + " draws " + format(amps) + " A but its encoder reads "
                        + format(v) + " ticks/s. Encoder cable unplugged, or the wheel is jammed?");
            }
            mechanismResults.add(name + ": FAIL (" + reading + ")");
        } else {
            mechanismResults.add(name + ": PASS (" + reading + ")");
        }
    }

    private void judgeCurrent(String name, DcMotorEx motor, double amps) {
        if (motor == null) {
            mechanismResults.add(name + ": SKIPPED (missing)");
        } else if (amps < MIN_MOTOR_AMPS) {
            addFailure(name + " drew no current at power " + format(INTAKE_CHECK_POWER)
                    + ". Motor unplugged or on the wrong port?");
            mechanismResults.add(name + ": FAIL (" + format(amps) + " A)");
        } else {
            mechanismResults.add(name + ": PASS (" + format(amps) + " A) - LOOK: did it intake?");
        }
    }

    private void judgeServo(String name, boolean present, boolean positionsSet, String expected) {
        if (!present) {
            mechanismResults.add(name + ": SKIPPED (missing)");
        } else if (!positionsSet) {
            addWarning(name + " positions are still placeholders (both ends equal); not moved. "
                    + "Set them with the Mechanism Tuner.");
            mechanismResults.add(name + ": SKIPPED (positions not set)");
        } else {
            mechanismResults.add(name + ": LOOK - " + expected);
        }
    }

    private void setPower(DcMotorEx motor, double power) {
        if (motor != null) {
            motor.setPower(power);
        }
    }

    private void stopMechanisms() {
        setPower(pollenFlywheel, 0.0);
        setPower(nectarFlywheel, 0.0);
        setPower(intakeLeft, 0.0);
        setPower(intakeRight, 0.0);
    }

    // ------------------------------------------------------------------------- shared

    private void checkBatteryVoltage() {
        if (batteryChecked) {
            return;
        }

        batteryChecked = true;
        if (voltageSensor == null) {
            voltageSensor = hardwareMap.voltageSensor.iterator().hasNext()
                    ? hardwareMap.voltageSensor.iterator().next()
                    : null;
        }
        double voltage = readVoltage();
        if (!isFinite(voltage)) {
            addFailure("Battery voltage reading is invalid.");
        } else if (voltage < LOW_BATTERY_WARNING_VOLTS) {
            addWarning("Battery is low before match: " + format(voltage) + " V.");
        }
    }

    private double readVoltage() {
        return voltageSensor == null ? Double.NaN : voltageSensor.getVoltage();
    }

    private void validatePose(Pose pose) {
        if (pose == null
                || !isFinite(pose.x())
                || !isFinite(pose.y())
                || !isFinite(pose.heading())) {
            addFailure("Follower/localizer returned an invalid pose.");
        }
    }

    /**
     * Pedro 2 exposed a debug STRING and this check parsed it ("pod0".."pod3", "drive Power = ").
     * Pedro 3's Drivetrain.debug() is a map - scalar mixer fields plus one nested map per pod,
     * keyed by the pod's configured name - so the same checks run on the map directly.
     */
    private void validateDebug(Map<String, Object> debug) {
        lastDebugString = debug == null ? "" : debug.toString();

        if (debug == null || debug.isEmpty()) {
            addFailure("Pedro drivetrain debug output is empty.");
            return;
        }

        if (!allFinite(debug)) {
            addFailure("Pedro drivetrain debug output contains invalid numeric values.");
        }

        int pods = 0;
        for (Object value : debug.values()) {
            if (value instanceof Map) {
                pods++;
            }
        }
        if (pods != EXPECTED_PODS) {
            addFailure("Pedro drivetrain debug reports " + pods + " pods, expected "
                    + EXPECTED_PODS + ".");
        }
    }

    private boolean allFinite(Map<?, ?> map) {
        for (Object value : map.values()) {
            if (value instanceof Map) {
                if (!allFinite((Map<?, ?>) value)) {
                    return false;
                }
            } else if (value instanceof Number && !isFinite(((Number) value).doubleValue())) {
                return false;
            }
        }
        return true;
    }

    private double maxAbsPodValue(Map<String, Object> debug, String key) {
        double max = 0.0;
        for (Object value : debug.values()) {
            if (!(value instanceof Map)) {
                continue;
            }
            Object field = ((Map<?, ?>) value).get(key);
            if (field instanceof Number) {
                double v = ((Number) field).doubleValue();
                if (isFinite(v)) {
                    max = Math.max(max, Math.abs(v));
                }
            }
        }
        return max;
    }

    private void updateTelemetry() {
        if (fatalInitError != null) {
            telemetry.addData("Fatal Init Error", fatalInitError);
        }

        telemetry.addData("Overall", overallStatus());
        telemetry.addData("Last Step", lastStepResult);

        if (phase == Phase.DRIVE) {
            CheckStep step = STEPS[stepIndex];
            telemetry.addData("Step", "drive %d/%d %s", stepIndex + 1, STEPS.length, step.name);
            telemetry.addData("Step Time", "%.2f / %.2f s", stepTimer.seconds(),
                    step.durationSeconds);
            telemetry.addData("Command", "f %.2f | s %.2f | t %.2f",
                    step.forward, step.strafe, step.turn);
        } else if (phase == Phase.MECHANISMS) {
            MechStep step = MECH_STEPS[mechIndex];
            telemetry.addData("Step", "mechanism %d/%d %s", mechIndex + 1, MECH_STEPS.length,
                    step.label);
            telemetry.addData("Step Time", "%.2f / %.2f s", stepTimer.seconds(), step.seconds);
        }

        if (follower != null) {
            Pose pose = follower.pose();
            telemetry.addData("X Position (in)", "%.2f", pose.x());
            telemetry.addData("Y Position (in)", "%.2f", pose.y());
            telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(pose.heading()));
            telemetry.addData("Max Velocity This Step", "%.2f", maxVelocityThisStep);
            telemetry.addData("Max Drive Power This Step", "%.2f", maxDrivePowerThisStep);
            telemetry.addData("Max Servo Power This Step", "%.2f", maxServoPowerThisStep);
        }
        telemetry.addData("Battery (V)", "%.2f", readVoltage());

        for (int i = 0; i < mechanismResults.size(); i++) {
            telemetry.addData("Mechanism " + (i + 1), mechanismResults.get(i));
        }
        addIssueTelemetry("Failures", failures);
        addIssueTelemetry("Warnings", warnings);
        telemetry.addData("Swerve Debug", lastDebugString);
        telemetry.update();
    }

    private void addIssueTelemetry(String caption, List<String> issues) {
        if (issues.isEmpty()) {
            telemetry.addData(caption, "none");
            return;
        }

        int limit = Math.min(issues.size(), 6);
        for (int i = 0; i < limit; i++) {
            telemetry.addData(caption + " " + (i + 1), issues.get(i));
        }

        if (issues.size() > limit) {
            telemetry.addData(caption + " More", issues.size() - limit);
        }
    }

    private String overallStatus() {
        if (fatalInitError != null || !failures.isEmpty()) {
            return "FAIL";
        }

        if (phase == Phase.WAITING) {
            return "READY";
        }

        if (phase != Phase.DONE) {
            return "RUNNING";
        }

        return warnings.isEmpty() ? "PASS" : "PASS WITH WARNINGS";
    }

    private void stopFollower() {
        if (follower == null) {
            return;
        }

        follower.manual(DrivePowers.zero());
        follower.update();
    }

    private void addFailure(String issue) {
        if (!failures.contains(issue)) {
            failures.add(issue);
        }
    }

    private void addWarning(String issue) {
        if (!warnings.contains(issue)) {
            warnings.add(issue);
        }
    }

    private double angleDelta(double current, double previous) {
        double delta = current - previous;
        while (delta > Math.PI) {
            delta -= 2.0 * Math.PI;
        }
        while (delta < -Math.PI) {
            delta += 2.0 * Math.PI;
        }
        return delta;
    }

    private boolean isFinite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }

    private String format(double value) {
        return String.format("%.2f", value);
    }

    private enum Phase {
        WAITING,
        DRIVE,
        MECHANISMS,
        DONE
    }

    private enum CheckType {
        SETTLE,
        FORWARD_POSITIVE,
        FORWARD_NEGATIVE,
        STRAFE_POSITIVE,
        STRAFE_NEGATIVE,
        TURN_POSITIVE,
        TURN_NEGATIVE
    }

    private enum MechStep {
        POLLEN_FLYWHEEL("Pollen flywheel spin", 1.0),
        NECTAR_FLYWHEEL("Nectar flywheel spin", 1.0),
        INTAKE("Intake run", 0.6),
        TURRETS("Turrets to centre", 0.8),
        GATE("Gate open / close", 0.8),
        SCOOPULA("Scoopula scoop / stow", 0.8);

        final String label;
        final double seconds;

        MechStep(String label, double seconds) {
            this.label = label;
            this.seconds = seconds;
        }
    }

    private static class CheckStep {
        final String name;
        final CheckType type;
        final double forward;
        final double strafe;
        final double turn;
        final double durationSeconds;

        CheckStep(String name, CheckType type, double forward, double strafe, double turn,
                double durationSeconds) {
            this.name = name;
            this.type = type;
            this.forward = forward;
            this.strafe = strafe;
            this.turn = turn;
            this.durationSeconds = durationSeconds;
        }
    }
}
