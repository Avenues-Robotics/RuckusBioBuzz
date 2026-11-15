package com.pedropathing.revhub.drivetrains;

import com.pedropathing.control.PIDFController;
import com.pedropathing.controllers.Controller;
import com.pedropathing.math.Vector2D;
import com.pedropathing.utils.Angle;
import com.pedropathing.utils.Utils;
import com.qualcomm.robotcore.hardware.*;

import java.util.HashMap;
import java.util.Map;

/**
 * CoaxialPod is a hardware-backed implementation of the core `SwervePod` interface. It owns the
 * drive motor, continuous rotation servo (turn), analog encoder and the PIDF controller used to
 * control pod rotation.
 *
 * <p>RUCKUS PATCH: every patch the 2.1.2 fork carried on this class is re-applied here on top of
 * the v3 structure - see RUCKUS_PATCHES.md. Search for {@code RUCKUS PATCH}.
 *
 * @author Kabir Goyal
 * @author Baron Henderson
 */
public class CoaxialPod implements SwervePod {
    private final String name;
    private final AnalogInput turnEncoder; // for rotation of servo
    private final CRServo turnServo;
    private final DcMotorEx driveMotor;

    private final CoaxialPodConfig config;

    private double lastDrivePower = 0;
    private double lastTurnPower = 0;

    /** RUCKUS PATCH: instrumentation only. See {@link #getLastErrorRad}. */
    private double lastErrorRad = 0;
    private boolean lastMoveFlipped = false;

    /**
     * RUCKUS PATCH: instrumentation only - the wheel-space theta this pod was last asked for,
     * before the shortest-path flip and before any encoder-frame conversion.
     *
     * <p>The recorder's {@code tgt} column used to come from SwerveBringUp.computeTargets, a
     * host-side MIRROR of the mixer. Mirrors drift: on 2026-08-15 the mirror still used the old
     * 0.05 rotation epsilon while the mixer had moved to 0.015, so 4.0% of the samples in
     * mydrive-001 recorded a demand the pods were never given (worst disagreement 15.0 deg).
     * This is the demand itself, taken from inside the pod, and it cannot disagree with what
     * the pod acted on.
     */
    private double lastTargetWheelRad = Double.NaN;

    /** RUCKUS PATCH: flip-decision hysteresis half-band. See the flip block in {@link #move}. */
    private static final double FLIP_HYSTERESIS_RAD = Math.toRadians(10);

    /** RUCKUS PATCH: continuous static-friction term. See {@link #setStaticFriction}. */
    private double staticFrictionPower = 0.0;
    private double staticFrictionBandRad = Math.toRadians(2.0);

    /** RUCKUS PATCH: see {@link #setPulsedApproach}. */
    private boolean pulsedApproach = false;
    private double pulseBandRad = Math.toRadians(3.0);
    private double pulseToleranceRad = Math.toRadians(0.5);
    private double pulsePower = 0.055;
    private double pulseSeconds = 0.015;
    private double pulseStationaryRadPerSec = Math.toRadians(20);
    private double pulseCoastSeconds = 0.10;
    private long pulseEndNano = 0;
    private double pulseSign = 0;
    private boolean lastMovePulsed = false;
    // Stall escalation: one pulse size cannot straddle this plant. Measured travel per pulse
    // spans 0.05-0.5 deg across pods at one power (stiction spread), and the friction itself
    // drifts as pods warm. So the base pulse is sized for the loosest case, and any pulse that
    // fails to shrink the error grows the next same-direction pulse instead - the pod finds the
    // size the plant needs right now. Overshoot (sign flip) or convergence resets to base, so
    // escalation can never feed a cycle: every pulse is still bounded and coast-separated.
    private double pulseFirePower = 0;
    private double pulseLastAbsErr = Double.NaN;
    // Below this commanded drive power the pod counts as approaching rest and pulses engage;
    // above it the wheels are rolling and the scheduled PD tracks the (possibly rotating)
    // demand continuously. Sits under the human driver's band (0.1-0.2).
    private static final double PULSE_DRIVE_GATE = 0.08;
    private static final double PULSE_ESCALATE = 1.5;
    // Must clear the stiffest pod's breakaway or a stalled ladder rides the cap forever; a 0.09
    // cap did exactly that on pod 3 (2026-08-15).
    // RUCKUS NOTE (2026-10-10): this used to cite "podMinV max 0.106 measured" as the breakaway.
    // podMinV is the ANALOG ENCODER's minimum voltage (Constants.podMinV -> analogMinVoltage;
    // 0.106 V is pod 3's), not a servo power. Measured open-loop breakaway is 0.050 on tiles and
    // 0.030 off the ground, uniform across pods after the lubrication pass (0.05-0.07 before it)
    // - see the turn-gain notes in Constants. So 0.13 is ~2.6x the tiles breakaway, and that a
    // 0.09 cap (1.8x) still stalled suggests a 20 ms pulse needs more than the steady-state
    // figure; why is unmeasured. Value unchanged.
    private static final double PULSE_MAX_POWER = 0.13;
    private static final double PULSE_PROGRESS_FRACTION = 0.75;
    private static final double PULSE_PROGRESS_MIN_RAD = Math.toRadians(0.35);

    /**
     * RUCKUS PATCH: largest change in turn-servo power per move() call. 0 disables. See the
     * comment at the slew clamp in {@link #move} for why this exists.
     */
    private double turnSlewPerUpdate = 0.0;

    /** RUCKUS PATCH: see {@link #turnSlewPerUpdate}. */
    public void setTurnSlewPerUpdate(double limit) {
        this.turnSlewPerUpdate = Math.abs(limit);
    }

    /**
     * RUCKUS PATCH: drive-power gain scheduling for the turn loop.
     *
     * <p>Measured 2026-08-14: a ROLLING wheel steers with a fraction of the friction of a
     * parked one - with the servos frozen, free-castering wheels held a 0.1-0.4 degree band
     * while driving, i.e. the rolling plant is nearly frictionless. Gains sized to break
     * parked stiction (kP 0.38-0.44 plus kS) are therefore several times over-gained the
     * moment the robot moves, and the loop limit-cycled at 3-4 Hz, 25 degrees, on every pod,
     * at every gain tried - kP 0.14 while rolling collapsed the wheel oscillation from 32-49
     * to 7-9 degrees. So the turn output scales with commanded drive power: full authority
     * parked (stiction is real there), ramping down to {@code SCHED_FLOOR} of full by
     * {@code SCHED_RAMP_DRIVE} of drive power.
     *
     * <p>Re-tuned 2026-08-15 against a RECORDED HUMAN SESSION: the driver lives at 0.05-0.20
     * drive power, a regime every scripted test overshot. Floor 0.24 reached by 0.10 drive,
     * velocity leg starting at 22 deg/s (slow 1-degree dither peaks near 30 deg/s and flew
     * under the old 40): wobble/chatter episodes in the driver-mimic pattern fell by 60-90%.
     * The error gate keeps transitions at full authority, which is what makes a floor this
     * low safe.
     */
    private static final boolean TURN_GAIN_SCHEDULING = true;
    private static double SCHED_FLOOR = 0.24;
    private static double SCHED_RAMP_DRIVE = 0.10;

    /** RUCKUS PATCH: runtime tuning of the schedule, robot-wide. NaN leaves a value alone. */
    public static void setScheduleTuning(double floor, double velStartRadS, double errGateRad) {
        if (!Double.isNaN(floor)) SCHED_FLOOR = Utils.clamp(floor, 0.05, 1.0);
        if (!Double.isNaN(velStartRadS)) SCHED_VEL_START_RAD_S = Math.abs(velStartRadS);
        if (!Double.isNaN(errGateRad)) SCHED_ERR_GATE_RAD = Math.abs(errGateRad);
    }

    /** RUCKUS PATCH: drive power at which the schedule reaches the floor. Runtime-tunable. */
    public static void setScheduleRamp(double rampDrive) {
        if (!Double.isNaN(rampDrive)) SCHED_RAMP_DRIVE = Math.max(0.05, Math.abs(rampDrive));
    }

    public static double[] getScheduleTuning() {
        return new double[] {SCHED_FLOOR, SCHED_VEL_START_RAD_S, SCHED_ERR_GATE_RAD};
    }

    /**
     * RUCKUS PATCH: the velocity leg of the schedule. A pod swinging fast has a moving contact
     * patch - partially rolling, low friction - even with zero drive power, which is how the
     * at-rest hunt sustained itself at full static gains: the oscillation kept the plant in
     * the regime that made full gains unstable. Scaling authority down with the pod's own
     * measured steering speed starves the cycle (swing fast -> gains drop -> energy drops ->
     * slows -> stiction catches it) while a slow, settling pod keeps full stiction-beating
     * authority. Ramp starts at {@code SCHED_VEL_START} (above noise and normal creep) and
     * reaches the floor by {@code SCHED_VEL_FULL}.
     */
    private static double SCHED_VEL_START_RAD_S = Math.toRadians(22);
    private static final double SCHED_VEL_FULL_RAD_S = Math.toRadians(260);

    /**
     * RUCKUS PATCH: the schedule only applies near the target. The limit cycles it suppresses
     * all live within ~15-18 degrees of the setpoint; a pod slewing a large transition needs
     * every bit of authority, and scheduling it down made X-to-forward convergence so slow the
     * robot crabbed 50 degrees off its commanded direction until the pods caught up. Above
     * this error the gains are simply the gains.
     */
    private static double SCHED_ERR_GATE_RAD = Math.toRadians(20);

    /**
     * RUCKUS PATCH (2026-11-15): demand-rate feed-forward, robot-wide, default OFF (kV 0 = upstream behaviour).
     *
     * <p>The turn loop only produces output from error, so a pod tracking a TURNING demand (a stick sweep, an
     * arc, every slew-limited direction change) has to trail it by whatever error makes the push the sweep
     * needs: measured on the 2x2 stick circle (circle.py, 110 deg/s, 0.25 drive, heading hold off) as 10-15
     * deg of mean error, 46-127 ms of lag, and a 3 Hz staircase/limit cycle as the schedule and the deadband
     * fight over it. This adds kV x (rate of the demand) - the push the sweep needs, up front - so feedback
     * only corrects what is left. The rate is taken from the UNFLIPPED target (a flip turns the target 180 deg
     * in one call but both frames rotate together), low-passed with time constant tau, and a single-call jump
     * faster than DEMAND_RATE_STEP_RAD_S counts as a step (X-lock entry, a flip-skipped slew), not a sweep,
     * and contributes nothing. Added after the gain schedule: a rolling pod still needs the full sweep push.
     */
    private static double demandRateKv = 0.0;
    private static double demandRateTauS = 0.05;
    private static final double DEMAND_RATE_STEP_RAD_S = Math.toRadians(900);
    private double prevDesiredRad = Double.NaN;
    private long prevDesiredNano = 0;
    private double demandRateRadS = 0;

    /** RUCKUS PATCH: runtime tuning of the demand-rate feed-forward, robot-wide. NaN leaves a value alone. */
    public static void setDemandRateFeedforward(double kvPowerPerRadS, double tauS) {
        if (!Double.isNaN(kvPowerPerRadS)) demandRateKv = Math.max(0.0, kvPowerPerRadS);
        if (!Double.isNaN(tauS)) demandRateTauS = Math.max(0.0, tauS);
    }

    /** {kV (power per rad/s), tau (s)}. */
    public static double[] getDemandRateFeedforward() {
        return new double[] {demandRateKv, demandRateTauS};
    }

    /** RUCKUS PATCH: see {@link #setDerivativeOnMeasurement}. */
    private boolean derivativeOnMeasurement = false;
    private double previousActualRad = Double.NaN;
    private long previousMoveNano = 0;
    private double lastMeasuredVelocityRad = 0;

    /**
     * @param config pod configuration. RUCKUS NOTE: the turn controller is stateful (integral,
     *               previous error, timestamps), so every pod needs its OWN instance - never
     *               share one Controller between pods.
     */
    public CoaxialPod(HardwareMap hardwareMap, CoaxialPodConfig config) {
        this.config = config;

        this.name = config.name.get();

        this.driveMotor = hardwareMap.get(DcMotorEx.class, config.motorName.get());
        this.turnServo = hardwareMap.get(CRServo.class, config.servoName.get());
        this.turnEncoder = hardwareMap.get(AnalogInput.class, config.servoEncoderName.get());

        setMotorToFloat();

        driveMotor.setDirection(config.driveDirection.get());
        turnServo.setDirection(config.servoDirection.get());

        turnServo.setPower(0);
    }

    /**
     * Returns the pod's offset from robot center.
     *
     * @return offset as a Vector2D
     */
    @Override
    public Vector2D getOffset() {
        return config.podOffset.get();
    }

    /**
     * Returns the current pod heading after applying the configured offset, in radians.
     *
     * @return heading in radians
     */
    @Override
    public double getAngle() {
        return getAngleAfterOffsetRad();
    }

    /**
     * Sets turn servo power in [-1, 1].
     *
     * @param power turn servo power
     */
    public void setServoPower(double power) {
        lastTurnPower = power;
        turnServo.setPower(power);
    }

    /**
     * Sets drive motor power in [-1, 1].
     *
     * @param power drive motor power
     */
    public void setMotorPower(double power) {
        lastDrivePower = power;
        driveMotor.setPower(power);
    }

    /**
     * Sets drive motor zero power behavior to FLOAT.
     */
    @Override
    public void setToFloat() {
        setMotorToFloat();
    }

    /**
     * Sets drive motor zero power behavior to BRAKE.
     */
    @Override
    public void setToBreak() {
        setMotorToBreak();
    }

    /**
     * Sets drive motor zero power behavior to FLOAT.
     */
    public void setMotorToFloat() {
        driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /**
     * Sets drive motor zero power behavior to BRAKE.
     */
    public void setMotorToBreak() {
        driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * @return encoder reversed status
     */
    public boolean isEncoderReversed() {
        return config.encoderReversed.get();
    }

    /**
     * Converts wheel-space theta (radians) to encoder-space theta.
     *
     * @param wheelTheta wheel-space heading in radians
     * @return encoder-space heading in radians
     */
    @Override
    public double adjustThetaForEncoder(double wheelTheta) {
        // wheelTheta is in radians. If encoder is reversed, use wheelTheta directly; otherwise invert.
        //if encoder is reversed, ccw (top down) is positive, if unreversed than cw is positive
        double t = config.encoderReversed.get() ? wheelTheta : (2 * Math.PI - wheelTheta);
        // servo zero offset: +90 degrees -> +pi/2 radians
        t += Math.PI / 2.0;
        return Angle.normalize(t);
    }

    /**
     * Commands pod to a wheel heading (radians) with a drive power in [-1, 1].
     *
     * @param targetAngleRad desired wheel heading in radians
     * @param drivePower drive power in [0, 1]
     * @param ignoreAngleChanges if true, turn servo power is set to 0 regardless of target angle
     */
    @Override
    public void move(double targetAngleRad, double drivePower, boolean ignoreAngleChanges) {
        lastTargetWheelRad = targetAngleRad;
        boolean encoderReversed = config.encoderReversed.get();

        // Convert hardware angle to radians and normalize
        double actualRad = getAngleAfterOffsetRad();
        actualRad = Angle.normalize(actualRad);

        //if encoder is reversed, ccw (top down) is positive, if unreversed than cw is positive
        double desiredRad = encoderReversed ? targetAngleRad : (2 * Math.PI - targetAngleRad);
        desiredRad += Math.PI / 2.0;
        desiredRad = Angle.normalize(desiredRad);

        // Shortest-path error in radians (signed)
        double mag = Angle.smallestDifference(actualRad, desiredRad);
        double dir = Angle.turnDirection(actualRad, desiredRad);
        double signedRad = (mag == Math.PI) ? -Math.PI : mag * dir;

        // PID uses radians (tune PIDF for radian error)
        double errorRad = signedRad;

        // Minimize rotation: flip + invert drive if > 90 degrees - WITH HYSTERESIS.
        //
        // RUCKUS PATCH: the stock decision re-evaluates |error| > 90 every call, so a demand
        // sitting near the boundary (a diagonal drive from X-parked pods puts it almost
        // exactly there) flips back and forth every loop: the pod dithers between "go to
        // theta" and "go to theta+180 reversed", the wheel sweeps 60-110 degrees, and the
        // asymmetry of four pods doing this shows up as chassis yaw wobble. Measured on
        // full-speed diagonals 2026-08-14. Inside a +/-10 degree band around the boundary the
        // previous flip state is kept; the worst cost is accepting a 100 degree rotation
        // instead of an 80 degree one, once.
        double desiredUnflippedRad = desiredRad;   // RUCKUS PATCH: demand-rate feed-forward input
        boolean flip = decideFlip(Math.abs(errorRad));
        lastMoveFlipped = flip;
        if (flip) {
            // add 180 degrees (pi radians)
            desiredRad = Angle.normalize(desiredRad + Math.PI);
            drivePower = -drivePower;

            // recompute signed error
            mag = Angle.smallestDifference(actualRad, desiredRad);
            dir = Angle.turnDirection(actualRad, desiredRad);
            signedRad = (mag == Math.PI) ? -Math.PI : mag * dir;
            errorRad = signedRad;
        }

        // Sign-only feed-forward input, zeroed inside 2 degrees, exactly as upstream.
        double feedForwardInput = (Math.abs(errorRad) < (2.0 * Math.PI / 180.0))
                ? 0
                : Angle.turnDirection(actualRad, desiredRad);

        // RUCKUS PATCH: measured pod velocity, wrapped correctly across the encoder's 0/2pi seam.
        long nowNano = System.nanoTime();
        if (!Double.isNaN(previousActualRad) && previousMoveNano != 0) {
            double dt = (nowNano - previousMoveNano) / 1.0e9;
            if (dt > 1e-6) {
                lastMeasuredVelocityRad =
                        Angle.normalizeSigned(actualRad - previousActualRad) / dt;
            }
        }
        previousActualRad = actualRad;
        previousMoveNano = nowNano;

        // RUCKUS PATCH: demand rate for the feed-forward (see demandRateKv). Always tracked, so switching it
        // on mid-run starts from a settled estimate.
        if (!Double.isNaN(prevDesiredRad) && prevDesiredNano != 0) {
            double dtD = (nowNano - prevDesiredNano) / 1.0e9;
            if (dtD > 1e-6 && dtD < 0.1) {
                double r = Angle.normalizeSigned(desiredUnflippedRad - prevDesiredRad) / dtD;
                if (Math.abs(r) > DEMAND_RATE_STEP_RAD_S) {
                    r = 0;
                }
                double alpha = demandRateTauS > 0 ? dtD / (demandRateTauS + dtD) : 1.0;
                demandRateRadS += alpha * (r - demandRateRadS);
            } else {
                demandRateRadS = 0;
            }
        }
        prevDesiredRad = desiredUnflippedRad;
        prevDesiredNano = nowNano;

        Controller turnController = config.turnController.get();
        double raw;
        if (derivativeOnMeasurement) {
            // d(error)/dt is -d(measurement)/dt whenever the setpoint is holding still, and unlike
            // differencing the error it does not spike when the setpoint steps or the pod flips.
            // Controller's three-argument form takes the measured velocity, v3's convention.
            raw = turnController.calculate(feedForwardInput, errorRad, lastMeasuredVelocityRad);
        } else {
            raw = turnController.calculate(feedForwardInput, errorRad);
        }

        // RUCKUS PATCH: continuous static-friction feed-forward. A no-op while
        // staticFrictionPower is 0, which is every stock configuration.
        if (staticFrictionPower != 0.0 && staticFrictionBandRad > 0.0) {
            raw += staticFrictionPower * Math.tanh(errorRad / staticFrictionBandRad);
        }

        // RUCKUS PATCH: scale the whole turn effort by the schedule - see the field comments.
        // Two legs, whichever demands more reduction wins: commanded drive power (the robot is
        // translating, wheels rolling) and the pod's own measured steering speed (the pod is
        // swinging, contact patch moving). Applied after kS so the feed-forward scales too: a
        // rolling pod has no stiction to break.
        if (TURN_GAIN_SCHEDULING && Math.abs(errorRad) < SCHED_ERR_GATE_RAD) {
            double aDrive = Math.min(1.0, Math.abs(drivePower) / SCHED_RAMP_DRIVE);
            double aVel = Utils.clamp(
                    (Math.abs(lastMeasuredVelocityRad) - SCHED_VEL_START_RAD_S)
                            / (SCHED_VEL_FULL_RAD_S - SCHED_VEL_START_RAD_S), 0.0, 1.0);
            double a = Math.max(aDrive, aVel);
            raw *= 1.0 - a * (1.0 - SCHED_FLOOR);
        }

        // RUCKUS PATCH: demand-rate feed-forward, after the schedule (see demandRateKv). Off while released.
        if (demandRateKv != 0.0 && !ignoreAngleChanges) {
            raw += demandRateKv * demandRateRadS;
        }

        double turnPower = Utils.clamp(raw, -1.0, 1.0);

        // RUCKUS PATCH: discrete pulses for the final approach. See setPulsedApproach.
        // Gated on drive power: pulses own the approach-to-rest, the scheduled PD owns rolling.
        // A pulse-coast cycle corrects at most ~5-10 deg/s, slower than an arcing demand
        // rotates, so with pulses active at speed every pod lagged to the band edge and rode
        // it - median tracking error 8-14 deg through a recorded driving session's turns,
        // worst on the stiff pod. Straight lines and parking never showed it (their demand
        // holds still); arcs need the continuous loop.
        lastMovePulsed = false;
        if (ignoreAngleChanges) {
            // RUCKUS PATCH: a released pod is not being steered - the servo is forced to 0 below -
            // so the pulse state machine must not run. Left running, it "fired" pulses the servo
            // never received, the pod never moved, every one counted as stalled, and the ladder
            // climbed 0.035 -> 0.13 in ~0.5 s of release; the first REAL pulse afterwards fired
            // at the cap. End the episode instead: no pending pulse or coast, stall history
            // cleared, fire power back to base.
            pulseEndNano = 0;
            pulseLastAbsErr = Double.NaN;
            pulseFirePower = pulsePower;
        } else if (pulsedApproach && Math.abs(drivePower) < PULSE_DRIVE_GATE
                && Math.abs(errorRad) < pulseBandRad) {
            lastMovePulsed = true;
            if (nowNano < pulseEndNano) {
                turnPower = pulseSign * pulseFirePower;
            } else if (nowNano < pulseEndNano + (long) (pulseCoastSeconds * 1.0e9)) {
                // Mandatory coast. The velocity gate alone is not enough: the hub's analog
                // registers update more slowly than the control loop, so a stale reading reads as
                // zero velocity while the pod is still moving, and pulses stack into an overshoot.
                turnPower = 0;
            } else if (Math.abs(errorRad) > pulseToleranceRad
                    && Math.abs(lastMeasuredVelocityRad) < pulseStationaryRadPerSec) {
                double sign = errorRad >= 0 ? 1.0 : -1.0;
                // Progress must beat an absolute floor as well as a fraction: near the
                // tolerance, 25% of a small error is inside encoder noise, and a noise dip
                // must not reset a ladder that is still climbing toward breakaway.
                double progressNeeded = Math.max(
                        (1.0 - PULSE_PROGRESS_FRACTION) * pulseLastAbsErr,
                        PULSE_PROGRESS_MIN_RAD);
                boolean stalled = sign == pulseSign && !Double.isNaN(pulseLastAbsErr)
                        && Math.abs(errorRad) > pulseLastAbsErr - progressNeeded;
                pulseFirePower = stalled
                        ? Math.min(PULSE_MAX_POWER, pulseFirePower * PULSE_ESCALATE)
                        : pulsePower;
                pulseSign = sign;
                pulseLastAbsErr = Math.abs(errorRad);
                pulseEndNano = nowNano + (long) (pulseSeconds * 1.0e9);
                turnPower = pulseSign * pulseFirePower;
            } else {
                // Inside tolerance, or still coasting from the last pulse. Let it stop.
                turnPower = 0;
                if (Math.abs(errorRad) <= pulseToleranceRad) {
                    pulseLastAbsErr = Double.NaN; // converged: next episode starts at base power
                }
            }
        } else if (Math.abs(lastMeasuredVelocityRad) > pulseStationaryRadPerSec) {
            // Stall history goes stale only when the pod actually moves. Resetting on a mere
            // band exit let border chatter (a stalled pod sitting at the band edge, PD pushing
            // it fractionally in and out at loop rate) wipe the ladder every loop, so
            // escalation never reached breakaway - the exact stall it exists to break.
            pulseLastAbsErr = Double.NaN;
        }

        // RUCKUS PATCH: instrumentation only, no control effect.
        lastErrorRad = errorRad;

        // RUCKUS PATCH: slew-limit the turn output. The corruption loop documented on
        // getRawAngleRad is fed by the SIZE of each actuation step - a fake 25 degree error
        // used to slam the servo from -0.45 to +0.45 in one write, and that slam is what
        // disturbs the next reading. Ramping the output caps the disturbance and breaks the
        // loop's gain without costing steady-state authority. Released (ignoreAngleChanges)
        // writes stay instant - letting go must never be rate-limited.
        if (turnSlewPerUpdate > 0 && !ignoreAngleChanges) {
            turnPower = Utils.clamp(turnPower,
                    lastTurnPower - turnSlewPerUpdate, lastTurnPower + turnSlewPerUpdate);
        }

        double servoCachingThreshold = config.servoCachingThreshold.get();
        double motorCachingThreshold = config.motorCachingThreshold.get();

        // please don't change the next 5 lines took like 5 hours to figure ts out
        if (ignoreAngleChanges) {
            lastTurnPower = 0;
            turnServo.setPower(0);
        } else if (Math.abs(turnPower - lastTurnPower) > servoCachingThreshold || (turnPower == 0 && lastTurnPower != 0)) {
            lastTurnPower = turnPower;
            turnServo.setPower(turnPower);
        }

        if (Math.abs(drivePower - lastDrivePower) > motorCachingThreshold || (drivePower == 0 && lastDrivePower != 0)) {
            lastDrivePower = drivePower;
            driveMotor.setPower(drivePower);
        }
    }

    /**
     * RUCKUS PATCH: the flip decision with hysteresis, given the unflipped |error|. The one copy
     * of it: {@link #move} acts on it and {@link #wouldChangeFlip} reports it to the mixer.
     */
    private boolean decideFlip(double absErrRad) {
        if (absErrRad > Math.PI / 2.0 + FLIP_HYSTERESIS_RAD) {
            return true;
        } else if (absErrRad < Math.PI / 2.0 - FLIP_HYSTERESIS_RAD) {
            return false;
        }
        return lastMoveFlipped;
    }

    /**
     * RUCKUS PATCH: whether {@link #move} with this target, issued now, would change the flip
     * state - i.e. whether the pod will answer the demand by flipping and reversing the drive
     * rather than by rotating to it. Hysteresis and flip memory included, so the answer is
     * exactly the decision move() will make. Lets Swerve's demand slew limiter skip only the
     * jumps the pod really resolves by flipping. No side effects.
     *
     * @param targetWheelRad wheel-space target, as move() takes it
     * @param encoderRad     the pod's current encoder-frame angle, as {@link #getAngle} returns
     *                       it - passed in so the caller's reading is reused, not re-read
     */
    public boolean wouldChangeFlip(double targetWheelRad, double encoderRad) {
        double absErr = Angle.smallestDifference(Angle.normalize(encoderRad),
                adjustThetaForEncoder(targetWheelRad));
        return decideFlip(absErr) != lastMoveFlipped;
    }

    /**
     * Returns the current pod heading after applying the configured offset, in radians.
     *
     * @return heading in radians
     */
    public double getAngleAfterOffsetRad() {
        return getRawAngleRad() - config.angleOffsetRad.get();
    }

    /**
     * RUCKUS PATCH: median filter master switch. OFF - measured 2026-08-14, the one sample
     * of lag it adds makes the rolling-plant oscillation WORSE (wheel p95 span tripled), the
     * same way every lag-adding measure did. Kept for reference, not for use.
     */
    private static final boolean RAW_MEDIAN_FILTER = false;

    /** RUCKUS PATCH: median-of-3 state. See {@link #getRawAngleRad}. */
    private double medA = Double.NaN;
    private double medB = Double.NaN;
    private double medLastPushed = Double.NaN;

    /**
     * Returns the raw encoder angle in radians, in [0, 2pi], median-of-3 filtered.
     *
     * <p>RUCKUS PATCH: measured 2026-08-14, the Axon's analog output corrupts when its own
     * servo motor is drawing current - with the servos silenced the encoders read a 0.1-0.4
     * degree band while driving, with them active 20-33% of samples implied impossible pod
     * rates. Each fake error makes the loop act, the act corrupts the next reading, and the
     * result is a self-sustaining 3-4 Hz, 25-degree oscillation that no gain could touch. A
     * median of the last three distinct readings deletes isolated fakes outright and costs one
     * sample of lag on real motion. Wrap-aware: the median is taken after unwrapping the
     * candidates around the newest reading. Same-loop repeats (bulk-cached, identical) do not
     * advance the window.
     *
     * @return filtered raw encoder angle in radians
     */
    public double getRawAngleRad() {
        double v = turnEncoder.getVoltage();
        double analogMinVoltage = config.analogMinVoltage.get();
        double analogMaxVoltage = config.analogMaxVoltage.get();
        double range = analogMaxVoltage - analogMinVoltage;
        if (range == 0)
            return 0;
        double normalized = (v - analogMinVoltage) / range;
        normalized = Utils.clamp(normalized, 0, 1);
        double raw = normalized * (2.0 * Math.PI);

        if (!RAW_MEDIAN_FILTER) {
            return raw;
        }
        if (raw != medLastPushed) {
            medA = medB;
            medB = medLastPushed;
            medLastPushed = raw;
        }
        if (Double.isNaN(medA) || Double.isNaN(medB)) {
            return raw;
        }
        // Unwrap the two older samples around the newest, take the middle value.
        double b = raw + Angle.normalizeSigned(medB - raw);
        double a = raw + Angle.normalizeSigned(medA - raw);
        double lo = Math.min(raw, Math.min(a, b));
        double hi = Math.max(raw, Math.max(a, b));
        double mid = raw + a + b - lo - hi;
        return Angle.normalize(mid);
    }

    /**
     * Returns the normalized raw angle after offset, in radians.
     *
     * @return normalized angle in radians
     */
    public double getOffsetAngleRad() {
        double rad = getRawAngleRad() - config.angleOffsetRad.get();
        return Angle.normalize(rad);
    }

    /**
     * RUCKUS PATCH: a continuous replacement for the sign-only {@code kF} relay.
     *
     * <p>{@code kF} enters {@link PIDFController} as {@code F * feedForwardInput}, and this class
     * feeds it {@code turnDirection(...)}, which is exactly +1 or -1. That makes kF a relay, not
     * a feed-forward: constant magnitude, switching sign at the target. Measured on this
     * drivetrain, a kF large enough to break stiction (0.035, matching the measured breakaway of
     * 0.025-0.050) produced a mean of 15.8 error sign changes per step and a 42.5 degree
     * post-settle peak-to-peak, while a kF small enough to be stable (0.005) was 5-10x below
     * breakaway and left the pod parked 3-6 degrees short.
     *
     * <p>This term is {@code power * tanh(error / band)}: the same authority far from the target,
     * tapering smoothly through zero so there is no switching discontinuity to limit-cycle on.
     * Set {@code power} to the measured breakaway and {@code band} to roughly the settling
     * tolerance you want.
     *
     * <p>A no-op at {@code power = 0}, which is the default and every stock configuration.
     *
     * @param power peak static-friction output, in the same units as the PIDF output
     * @param bandRadians error at which the term reaches tanh(1) = 76% of {@code power}
     */
    public void setStaticFriction(double power, double bandRadians) {
        this.staticFrictionPower = power;
        this.staticFrictionBandRad = bandRadians;
    }

    /**
     * RUCKUS PATCH: close the last few degrees with discrete pulses instead of a continuous loop.
     *
     * <p>The pod has no creep regime. Static friction breaks to kinetic and it goes from stopped to
     * tens of degrees per second with nothing in between, so any continuous controller with enough
     * authority to correct a small error commits to several degrees of travel before feedback -
     * 39 ms of transport delay plus a 42 ms velocity time constant - can act on it. That is a limit
     * cycle by construction, and it is why raising static-friction authority always bought ringing.
     *
     * <p>A pulse sidesteps it by being open loop and bounded. One pulse of {@code power} for
     * {@code seconds} injects a known impulse - travel is roughly the terminal speed at that power
     * times the pulse length - then the pod coasts to a stop and is measured again. The lag never
     * enters the stability question because nothing is being stabilised; the loop is a sequence of
     * measured, bounded steps.
     *
     * <p>Outside {@code bandRadians} the normal PD loop runs untouched. Inside it, a pulse fires
     * only when the error exceeds {@code toleranceRadians} <em>and</em> the pod has come to rest,
     * so pulses never stack on top of one another.
     *
     * <p>Pulse length wants to be short, and the servo PWM frame quantises it - a pulse shorter
     * than one frame may not be delivered at all. At the SDK default 20 ms frame the shortest
     * usable pulse is 20 ms; halving the frame to 10 ms halves the quantum, which is the real
     * argument for changing it.
     *
     * <p>A no-op at {@code enabled = false}, which is the default.
     */
    public void setPulsedApproach(boolean enabled, double bandRadians, double toleranceRadians,
            double power, double seconds, double stationaryRadPerSec, double coastSeconds) {
        this.pulsedApproach = enabled;
        this.pulseBandRad = bandRadians;
        this.pulseToleranceRad = toleranceRadians;
        this.pulsePower = power;
        this.pulseSeconds = seconds;
        this.pulseStationaryRadPerSec = stationaryRadPerSec;
        this.pulseCoastSeconds = coastSeconds;
    }

    /** RUCKUS PATCH: true when the last {@link #move} call was in pulsed mode. Instrumentation. */
    public boolean wasLastMovePulsed() {
        return lastMovePulsed;
    }

    /**
     * RUCKUS PATCH: take the derivative from the measurement rather than from the error.
     *
     * <p>Measured on this drivetrain, the pod's actuation lag is 106 ms equivalent - 41 ms of
     * transport plus a 64 ms velocity time constant - against a loop period of 8 ms. Lead
     * compensation is the only thing that answers a plant lag that large, so the D term has to be
     * usable. It was not: differencing the error means a 90 degree setpoint step produces
     * {@code kD * 1.571 rad / 0.008 s}, which saturates the output clamp at kD as small as 0.010,
     * and the 180 degree flip does the same thing with the opposite sign.
     *
     * <p>Routed through {@link Controller#calculate(double, double, double)}, so it works with any
     * controller that implements the three-argument form (the retained {@link PIDFController}
     * does). A no-op at {@code false}, which is the default and every stock configuration.
     */
    public void setDerivativeOnMeasurement(boolean enabled) {
        this.derivativeOnMeasurement = enabled;
    }

    public boolean isDerivativeOnMeasurement() {
        return derivativeOnMeasurement;
    }

    /** RUCKUS PATCH: last measured pod angular velocity, rad/s. Instrumentation. */
    public double getMeasuredVelocityRad() {
        return lastMeasuredVelocityRad;
    }

    /**
     * RUCKUS PATCH: configures the turn PID's integral behaviour.
     *
     * <p>Only meaningful when the configured turn controller is the retained
     * {@link PIDFController}: see {@link PIDFController#setIntegralLimit},
     * {@link PIDFController#setIntegralBand} and
     * {@link PIDFController#setIntegralResetThreshold}. All three are no-ops while I is 0.
     *
     * @param limit largest absolute contribution the integral may make to the output
     * @param bandRadians only accumulate while |error| is inside this band
     * @param resetThresholdRadians error a sign change must exceed to clear the accumulator
     * @throws IllegalStateException if the turn controller is not a {@link PIDFController}
     */
    public void setTurnIntegralSettings(double limit, double bandRadians,
            double resetThresholdRadians) {
        PIDFController turnPID = getTurnPIDF();
        turnPID.setIntegralLimit(limit);
        turnPID.setIntegralBand(bandRadians);
        turnPID.setIntegralResetThreshold(resetThresholdRadians);
    }

    /**
     * RUCKUS PATCH: the configured turn controller, when it is the retained
     * {@link PIDFController}. Lets diagnostics retune gains live without rebuilding the pod.
     *
     * @throws IllegalStateException if the turn controller is some other {@link Controller}
     */
    public PIDFController getTurnPIDF() {
        Controller turnController = config.turnController.get();
        if (!(turnController instanceof PIDFController)) {
            throw new IllegalStateException("Pod " + name + " turn controller is "
                    + turnController.getClass().getName() + ", not PIDFController");
        }
        return (PIDFController) turnController;
    }

    public double getStaticFrictionPower() {
        return staticFrictionPower;
    }

    public double getStaticFrictionBandRad() {
        return staticFrictionBandRad;
    }

    /**
     * RUCKUS PATCH: the turn power most recently <em>written</em> to the servo.
     *
     * <p>Not the same as the controller's output: {@link #setServoCachingThreshold} suppresses
     * writes smaller than its threshold, so this is what the hardware is actually holding. Reading
     * it from the {@code CRServo} object instead would go through that object's own direction flag,
     * which a diagnostic tool may have set differently.
     *
     * @return last written turn servo power, in [-1, 1]
     */
    public double getLastTurnPower() {
        return lastTurnPower;
    }

    /**
     * RUCKUS PATCH: the drive power most recently written to the motor, same caching caveat as
     * {@link #getLastTurnPower}. Exists so diagnostics can display it without paying a live
     * {@code DcMotorEx.getPower()} Lynx transaction.
     *
     * @return last written drive motor power, in [-1, 1]
     */
    public double getLastDrivePower() {
        return lastDrivePower;
    }

    /**
     * RUCKUS PATCH: signed pod heading error from the last {@link #move} call, in radians.
     *
     * <p>Measured after the 180 degree flip is resolved, so it is the error the PID actually acted
     * on, already wrapped to [-pi/2, pi/2]. Scoring a step response against this avoids having to
     * reproduce the wrap and flip logic outside the pod.
     *
     * @return signed error in radians
     */
    public double getLastErrorRad() {
        return lastErrorRad;
    }

    /**
     * RUCKUS PATCH: whether the last {@link #move} call took the 180 degree flip.
     *
     * @return true if the target was inverted and drive power negated
     */
    public boolean wasLastMoveFlipped() {
        return lastMoveFlipped;
    }

    /**
     * RUCKUS PATCH: the wheel-space theta passed to the last {@link #move} call, in radians.
     *
     * <p>The demand as the mixer produced it: pre-flip, pre-encoder-frame, and taken from inside
     * the pod so no host-side reimplementation can drift away from it. NaN until the first move.
     *
     * @return last commanded wheel heading in radians, or NaN
     */
    public double getLastTargetWheelRad() {
        return lastTargetWheelRad;
    }

    /**
     * RUCKUS PATCH: runtime setter kept from the 2.1.2 API; writes through to the config.
     *
     * @param motorCachingThreshold minimum delta before applying power update
     */
    public void setMotorCachingThreshold(double motorCachingThreshold) {
        config.motorCachingThreshold.set(motorCachingThreshold);
    }

    /**
     * RUCKUS PATCH: runtime setter kept from the 2.1.2 API; writes through to the config.
     *
     * @param servoCachingThreshold minimum delta before applying power update
     */
    public void setServoCachingThreshold(double servoCachingThreshold) {
        config.servoCachingThreshold.set(servoCachingThreshold);
    }

    public String name() {
        return name;
    }

    /**
     * @return debug for pod state
     */
    @Override
    public Map<String, Object> debug() {
        double rawAngleRad = getRawAngleRad();
        double offsetAngleRad = getAngleAfterOffsetRad();

        Map<String, Object> map = new HashMap<>();

        map.put("servoName", config.servoName.get());
        map.put("rawAngleRad", rawAngleRad);
        map.put("rawAngleDeg", Math.toDegrees(rawAngleRad));
        map.put("angleAfterOffsetRad", offsetAngleRad);
        map.put("angleAfterOffsetDeg", Math.toDegrees(offsetAngleRad));
        map.put("servoPower", turnServo.getPower());
        map.put("drivePower", driveMotor.getPower());

        return map;
    }
}
