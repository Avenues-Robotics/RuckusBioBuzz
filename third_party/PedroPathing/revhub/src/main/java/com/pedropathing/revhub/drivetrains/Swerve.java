package com.pedropathing.revhub.drivetrains;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.math.Vector2D;
import com.pedropathing.utils.Angle;
import com.pedropathing.utils.Pair;
import com.pedropathing.utils.Utils;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Swerve Drivetrain implementation.
 * Angles are in radians and positive rotation is to the left (CCW, top-down).
 *
 * <p>RUCKUS PATCH: the mixer below is the 2.1.2 fork's, re-applied on the v3 structure - see
 * RUCKUS_PATCHES.md. Search for {@code RUCKUS PATCH}.
 *
 * @author Kabir Goyal
 * @author Baron Henderson
 */
public class Swerve implements Drivetrain {
    private final List<SwervePod> pods;
    private final SwerveConfig config;

    private double lastForward = 0;
    private double lastStrafe = 0;
    private double lastRotation = 0;
    private double lastAvgScaling = 0;

    private double powerScaling;

    private final VoltageSensor voltageSensor;

    /** RUCKUS PATCH: see the zeroRotation comment in applyDrive. */
    private static final double ROTATION_EPSILON = 0.015;

    /**
     * RUCKUS PATCH: fade the translation and rotation terms to zero across their epsilon bands
     * instead of switching them off at the wall.
     *
     * <p>Measured 2026-08-16 on mydrive-001: of 601 azimuth-setpoint jumps over 15 degrees while
     * driving, 42% were rotation-epsilon crossings and 3% were translation-epsilon crossings -
     * and the translation ones were the biggest in the log, mean 74-95 degrees and up to 179.
     * The cause is structural rather than a tuning error: the pod demand is
     * {@code atan2(translation + rotation)}, so deleting either term in one step rotates the
     * demand by however far apart the two directions were. On a nearly square chassis the
     * rotation-only directions are +-43.5 / +-136.5 degrees, which is why the symptom reads as
     * "snapping to 45 degrees".
     *
     * <p>A smoothstep taper makes the demand direction continuous through the wall: the term
     * shrinks to nothing as the input approaches zero, so the demand migrates toward the other
     * term's direction instead of jumping to it. Value and slope both match at the band edge.
     */
    private static volatile boolean epsilonTaper = true;

    /**
     * RUCKUS PATCH: largest change in a pod's commanded azimuth per call, degrees per second.
     * 0 disables.
     *
     * <p>The taper fixes the demand where the input crosses a wall slowly. It cannot help when
     * the input jumps - a stick released in one loop still swings the demand ~90 degrees. This
     * bounds the rate directly, and the bound is free: the measured pod slew is 214 deg/s
     * (184-259), so a demand moving faster than that is asking for travel the hardware cannot
     * deliver and only guarantees a saturated, lagging pod. A 90 degree change now takes 420 ms
     * of demand travel against the 421 ms of gross travel the pod measured for the same step.
     *
     * <p>Simulated over mydrive-001 (51.3 s of real driving, the recorded commands replayed
     * through both mixers): physical consecutive-loop demand change p90 19.8 -> 13.2 deg, jumps
     * over 15 deg 2.9/s -> 0.9/s, demand reversals 4.25 -> 3.47/s. 300 deg/s was tried first and
     * is worse at this loop rate - it spreads one big jump into several 19 deg steps and the
     * count of violations goes UP.
     *
     * <p>Note what the limiter cannot do: it is a RATE, so a slow loop still turns it into a big
     * step. 53% of the jumps that survive it in simulation happen on loops longer than 70 ms
     * (= 15 deg at 214 deg/s). Criterion 1 is therefore a loop-rate criterion as much as a mixer
     * one; at 50 Hz true with a 25 ms p99 the same limit permits 5.4 deg per loop.
     *
     * <p>Applied on the shortest-angle difference so it never sends a pod the long way round, and
     * skipped where the pod flips and reverses the drive rather than rotating - rate-limiting a
     * flip would force a real 180 degree sweep with the drive pointing the wrong way throughout.
     * "Where the pod flips" is the pod's own decision (see {@code resolvesByFlip}), not a bare
     * quarter-turn test: with the pod's +-10 degree flip hysteresis the two disagree for jumps
     * of 90-100 degrees.
     */
    private static volatile double demandSlewDegPerSec = 214.0;

    /**
     * RUCKUS PATCH: smallest fraction of the commanded translation a manual command keeps when
     * rotation saturates a pod. 0 disables it, which is upstream: every pod vector is divided by
     * the largest, so full stick plus full turn drops translation to 54% on this chassis. Above
     * 0, rotation is cut only as far as needed to hold translation at this fraction; commands
     * that already keep it are untouched. Only {@code drive(powers, true)} applies it; paths and
     * the tools' {@link #applyDrive(DrivePowers)} never do.
     */
    private static volatile double translationPriority = 0.0;

    /** Per-pod anchor for the demand slew limiter, radians; NaN until first commanded. */
    private final double[] lastCommandedTheta;
    private long lastArcadeNano = 0;

    /** RUCKUS PATCH: when the drive inputs last carried a real command. See applyDrive. */
    private long lastActiveInputNano = 0;

    /**
     * RUCKUS PATCH (2026-11-15): small-turn roll-out. For this long after the translation input was last at or
     * above epsilon, a turn input under epsilon (but over the much lower rotation epsilon) does not steer the
     * pods - the input is treated as zero, so the pods are released holding the travel direction, exactly as
     * the X_LOCK engage delay does for zero input. Without it, a stop with a small heading correction still
     * running sent every pod to its rotation tangent within one loop (all at 270 -> 20/70/110/160 deg; the
     * demand slew limiter skips jumps a pod resolves by flipping) while the chassis rolled at 21-23 in/s, which
     * scrubbed it to a dead stop - the operator's "jolt on stop", never seen in pure turns (the pods are already
     * on the tangents). After the window the small turn acts again, so heading hold at rest is unchanged; turns
     * at or above epsilon are never affected. 0 disables (upstream behaviour).
     *
     * <p>DEFAULT OFF (0): A/B 2026-11-15 on the Foresight BRAKE stage (rollout_ab.py) - it does what it says, pod
     * rotation in the 0.3 s after a stop 59 -> 11 deg mean (servo power 0.14 -> 0.03), but the stop is not
     * gentler: peak deceleration 183 vs 199 in/s^2, stopped within 0.3 s either way (n = 9 / 6 stops). The
     * abruptness is the rebuilt drivetrain's own coast deceleration, not the pod snap. Kept switchable
     * (setSmallTurnRolloutS, Bring-Up setMixer rollout=) rather than shipped on.
     */
    private static volatile double smallTurnRolloutS = 0.0;

    /** RUCKUS PATCH: when the translation input was last at or above epsilon (small-turn roll-out). */
    private long lastActiveTransNano = 0;

    /**
     * RUCKUS PATCH: how long zero input must persist before X_LOCK engages. Stick release at
     * speed used to snap all four pods sideways against a still-rolling chassis - ground
     * reaction torque then buffets the pod azimuths and the whole robot judders. Measured on
     * 2026-08-13 with a scripted forward/pause/backward cycle: wheels reversed direction 6-7
     * times per second against a commanded 0.8/s, invariant to every gain tried, and cutting the
     * instant snap halved it. Within this window zero input holds pod headings (servos released,
     * the IGNORE_ANGLE_CHANGES behaviour) so the robot rolls out straight; the X still engages
     * once the chassis has had time to stop.
     */
    private static final double X_LOCK_ENGAGE_DELAY_S = 0.35;

    /**
     * RUCKUS PATCH: last zero-power behaviour written to the pods; null forces the next write.
     * Upstream v3 re-applies it on every drive() call - once per pod per loop. The 2.1.2 fork set
     * it only on mode changes, and whether a repeated write costs a Lynx transaction was never
     * measured here, so this keeps the old write pattern rather than finding out on the robot.
     */
    private DcMotor.ZeroPowerBehavior appliedZeroPowerBehavior = null;

    /**
     * @param pods SwervePods, coaxial or differential
     */
    public Swerve(HardwareMap hardwareMap, SwerveConfig config, SwervePod... pods) {
        this.config = config;
        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();
        this.pods = Arrays.asList(pods);
        this.lastCommandedTheta = new double[pods.length];
        Arrays.fill(this.lastCommandedTheta, Double.NaN);

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /** Runtime switches, so each fix can be A/B'd inside one session. */
    public static void setEpsilonTaper(boolean on) {
        epsilonTaper = on;
    }

    public static void setDemandSlewDegPerSec(double degPerSec) {
        demandSlewDegPerSec = Math.max(0, degPerSec);
    }

    public static boolean getEpsilonTaper() {
        return epsilonTaper;
    }

    public static double getDemandSlewDegPerSec() {
        return demandSlewDegPerSec;
    }

    public static void setSmallTurnRolloutS(double seconds) {
        smallTurnRolloutS = Math.max(0, seconds);
    }

    public static double getSmallTurnRolloutS() {
        return smallTurnRolloutS;
    }

    public static void setTranslationPriority(double fraction) {
        translationPriority = Range.clip(fraction, 0, 1);
    }

    public static double getTranslationPriority() {
        return translationPriority;
    }

    /**
     * Largest s in [0, 1] with every |translation + s * rotation_i| <= 1 / translationPriority,
     * so normalising by the largest pod still leaves that fraction of the translation. One
     * quadratic per pod; translation is at most unit length, so each root is in [0, 1].
     */
    private double rotationKeep(Vector2D translation, double rotation) {
        double limit = 1.0 / translationPriority;
        double keep = 1.0;
        double tt = translation.magnitudeSquared();
        for (SwervePod pod : pods) {
            Vector2D r = Vector2D.polar(rotation, rotationBaseAngle(pod) + Math.PI / 2);
            double rr = r.magnitudeSquared();
            double tr = translation.dot(r);
            if (tt + 2 * tr + rr <= limit * limit) continue;
            keep = Math.min(keep, (-tr + Math.sqrt(tr * tr - rr * (tt - limit * limit))) / rr);
        }
        return keep;
    }

    /**
     * Smoothstep from 0 at zero input to 1 at the band edge. Slope is zero at both ends, so the
     * taper adds no discontinuity of its own where it meets the untapered region.
     */
    private static double taper(double magnitude, double band) {
        if (band <= 0 || magnitude >= band) {
            return 1.0;
        }
        double u = magnitude / band;
        return u * u * (3.0 - 2.0 * u);
    }

    /**
     * RUCKUS PATCH: the direction of a pod's rotation-only vector before the quarter-turn, i.e.
     * its X-lock direction. Mixer frame: x = robot right, y = robot forward; offsets are
     * x = forward, y = left.
     *
     * <p>Upstream v3 uses {@code atan2(-x, -y)} with the turn input negated. For x-forward/y-left
     * offsets that mirrors the rotation field about the robot's lateral axis - checked
     * numerically on this chassis, CCW turn at 0.5: v3 demands LF 133.5 / RF 226.5 / LB 46.5 /
     * RB 313.5 deg, the true CCW tangents (and the 2.1.2 mixer) are 226.5 / 133.5 / 313.5 / 46.5.
     * That is not a rigid rotation; the pods would fight. This keeps 2.1.2's geometry.
     */
    private static double rotationBaseAngle(SwervePod pod) {
        return Math.atan2(pod.getOffset().x(), -pod.getOffset().y());
    }

    /**
     * RUCKUS PATCH: whether {@code pod} will answer demand {@code theta} by changing its flip
     * state, as the demand slew limiter's skip condition. A {@link CoaxialPod} answers with its
     * own decision (hysteresis and flip memory included); any other pod keeps the stock
     * quarter-turn assumption.
     */
    private static boolean resolvesByFlip(SwervePod pod, double theta, double encoderRad) {
        if (pod instanceof CoaxialPod) {
            return ((CoaxialPod) pod).wouldChangeFlip(theta, encoderRad);
        }
        return true;
    }

    /**
     * RUCKUS PATCH: the pod's current heading in wheel space, the frame move() takes - the inverse
     * of {@link SwervePod#adjustThetaForEncoder}. That map is {@code c +- wheelTheta (mod 2pi)} for
     * any pod (an offset and an optional reversal), so probing it at 0 and a quarter turn recovers
     * c and the sign without reaching past the interface.
     */
    private static double wheelThetaOf(SwervePod pod) {
        double c = pod.adjustThetaForEncoder(0.0);
        double sign = Angle.normalizeSigned(pod.adjustThetaForEncoder(Math.PI / 2.0) - c) > 0
                ? 1.0 : -1.0;
        return Angle.normalize(sign * Angle.normalizeSigned(pod.getAngle() - c));
    }

    /** RUCKUS PATCH: 2.1.2's Vector kept theta in [0, 2pi); keep feeding the pods that range. */
    private static double thetaOf(Vector2D v) {
        return Angle.normalize(v.theta());
    }

    /**
     * Stops following and holds pod angles while floating drive motors.
     */
    @Override
    public void stop() {
        stop(config.manualBrakeMode.get());
    }

    public void stop(boolean brake) {
        if (brake) {
            setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
            setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        for (int i = 0; i < pods.size(); i++) {
            SwervePod pod = pods.get(i);
            // RUCKUS PATCH (2026-10-10): upstream passes pod.getAngle() - the ENCODER-frame angle -
            // where move() takes a WHEEL-space theta. Released, so the servo never acts on it, but
            // move() still records it as the demand (the recorder's tgt column), decides the flip
            // state against it (which then breaks the next 80-100 degree tie) and feeds it to the
            // turn PID's error history. Re-send the pod its own last demand instead, so stop()
            // leaves that state as the last real command left it; a pod never commanded yet gets
            // its current heading, converted to wheel space.
            double theta = Double.isNaN(lastCommandedTheta[i])
                    ? wheelThetaOf(pod) : lastCommandedTheta[i];
            pod.move(theta, 0, true);
            pod.setToFloat();
        }
        // RUCKUS PATCH: the pods were just floated behind the cache's back.
        appliedZeroPowerBehavior = null;
    }

    @Override
    public Map<String, Object> debug() {
        Map<String, Object> map = new HashMap<>();

        map.put("forward", lastForward);
        map.put("strafe", lastStrafe);
        map.put("rotation", lastRotation);
        map.put("powerScaling", powerScaling);
        map.put("averageAngleScaling", lastAvgScaling);

        for (SwervePod pod : pods) {
            map.put(pod.name(), pod.debug());
        }

        return map;
    }

    @Override
    public double interpolateVelocity(double xRadius, double yRadius, double theta) {
        if (Math.abs(xRadius - yRadius) < 0.001) return xRadius;

        //just using an ellipse here in case someone wanted to account for friction or something, but it should be a circle for most cases
        double cos = Math.cos(theta);
        double sin = Math.sin(theta);

        return 1.0 / Math.sqrt(
                (cos * cos) / (xRadius * xRadius) +
                        (sin * sin) / (yRadius * yRadius)
        );
    }

    @Override
    public void drive(DrivePowers powers, boolean manual) {
        if (manual && config.manualBrakeMode.get())
            setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        else
            setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        applyDrive(powers, manual);
    }

    public void applyDrive(DrivePowers powers) {
        applyDrive(powers, false);
    }

    private void applyDrive(DrivePowers powers, boolean manual) {
        double forward = powers.forward();
        double strafe = -powers.strafe();
        // RUCKUS PATCH: CCW-positive, as DrivePowers.turn is everywhere in core (Foresight,
        // ManualDrive). Upstream negates it here to pair with its mirrored rotation geometry -
        // see rotationBaseAngle.
        double rotation = powers.turn();

        lastForward = forward;
        lastStrafe = strafe;
        lastRotation = rotation;

        double epsilon = config.epsilon.get();

        // stores forward and strafe values as the translation vector with max magnitude of 1
        Vector2D rawTrans = Vector2D.polar(Range.clip(Math.hypot(strafe, forward), 0, 1), Math.atan2(forward, strafe));

        boolean zeroTrans = rawTrans.magnitude() < epsilon;
        // RUCKUS PATCH: rotation gets its own, much lower epsilon. The shared 0.05 was a wall
        // that swallowed every fine heading-hold correction, forcing minimum-magnitude
        // workarounds upstream whose smallest allowed kick still deflects pod demands by
        // atan(0.05/translation) - 17+ degrees at crawl speeds, the visible periodic pod
        // wobble the driver kept reporting. 0.015 lets a damped PID correct continuously with
        // deflections too small to see. Translation keeps the original epsilon: the zero-input
        // path (X-lock, release semantics) is tuned around it.
        double rotEpsilon = Math.min(epsilon, ROTATION_EPSILON);
        boolean zeroRotation = Math.abs(rotation) < rotEpsilon;

        // RUCKUS PATCH (2026-11-15): small-turn roll-out - see smallTurnRolloutS.
        long rolloutNow = System.nanoTime();
        if (!zeroTrans) {
            lastActiveTransNano = rolloutNow;
        } else if (!zeroRotation && Math.abs(rotation) < epsilon && smallTurnRolloutS > 0
                && lastActiveTransNano != 0 && (rolloutNow - lastActiveTransNano) / 1.0e9 < smallTurnRolloutS) {
            zeroRotation = true;
        }

        // RUCKUS PATCH: X_LOCK only after zero input has persisted, so a stick release at speed
        // does not snap the pods sideways under a rolling chassis. Until then zero input behaves
        // like IGNORE_ANGLE_CHANGES: headings held, servos quiet, robot rolls out straight.
        if (!(zeroTrans && zeroRotation)) {
            lastActiveInputNano = System.nanoTime();
        }
        boolean xLockRipe = zeroTrans && zeroRotation
                && (System.nanoTime() - lastActiveInputNano) / 1.0e9 >= X_LOCK_ENGAGE_DELAY_S;
        boolean xLock = config.zeroPowerBehavior.get() == SwerveConfig.ZeroPowerBehavior.X_LOCK;

        // RUCKUS PATCH: the epsilon walls decide RELEASE semantics (X-lock, servos quiet) as
        // booleans, exactly as before - but the vectors handed to the mixer are tapered rather
        // than deleted, so the demand direction is continuous through each wall. See epsilonTaper.
        double transScale = epsilonTaper ? taper(rawTrans.magnitude(), epsilon) : 0.0;
        double rotScale = epsilonTaper ? taper(Math.abs(rotation), rotEpsilon) : 0.0;

        // Untapered, these collapse to the original hard switch: scale 0 below the wall, 1 above.
        double rotationScalar = zeroRotation ? rotation * rotScale : rotation;
        Vector2D translationVector = zeroTrans ? rawTrans.times(transScale) : rawTrans;
        if (manual && translationPriority > 0 && rotationScalar != 0) {
            rotationScalar *= rotationKeep(translationVector, rotationScalar);
        }

        Vector2D[] podVectors = new Vector2D[pods.size()];
        // RUCKUS PATCH: pod directions are carried separately. Vector2D is cartesian, so a
        // zero-magnitude vector has no direction - which is exactly what the X-lock vector is.
        // (Upstream v3's X-lock hands every pod theta 0 for that reason.)
        double[] podThetas = new double[pods.size()];

        for (int i = 0; i < pods.size(); i++) {
            SwervePod pod = pods.get(i);
            double base = rotationBaseAngle(pod);

            // this gets the perpendicular vector for the wheel
            Vector2D rotationVector = Vector2D.polar(rotationScalar, base + Math.PI / 2);

            podVectors[i] = translationVector.plus(rotationVector);
            podThetas[i] = thetaOf(podVectors[i]);
            if (xLock && xLockRipe) {
                // Zero magnitude, pod's own radius for the angle: the X pattern, and no drive
                // power whatever the taper left in the vectors above.
                podVectors[i] = Vector2D.zero();
                podThetas[i] = Angle.normalize(base);
            }
        }

        // finding if any vector has magnitude > maxPowerScaling
        //
        // RUCKUS PATCH: read the voltage sensor once per call, not once per pod. (The 2.1.2
        // stock mixer also discarded the scaled vector, making compensation a silent no-op;
        // v3 fixed that part upstream.)
        double maxMagnitude = 1;
        if (config.voltageCompensation.get()) {
            double voltageNormalized = getVoltageNormalized();
            for (int i = 0; i < podVectors.length; i++) {
                podVectors[i] = podVectors[i].times(voltageNormalized);
            }
        }
        for (Vector2D podVector : podVectors) {
            maxMagnitude = Math.max(maxMagnitude, podVector.magnitude());
        }

        powerScaling = 1 / maxMagnitude;

        // Find the avg scaling constant (avg of cos(angle error))
        double avgScaling = 0;
        // RUCKUS PATCH: kept for the slew limiter's flip check, so it costs no extra read.
        double[] currentRads = new double[pods.size()];

        for (int i = 0; i < pods.size(); i++) {
            double currentRad = pods.get(i).getAngle();
            currentRads[i] = currentRad;

            // ask the pod to translate the wheel-space theta into the encoder frame
            double targetRad = pods.get(i).adjustThetaForEncoder(podThetas[i]);

            // compute shortest signed error in radians using MathFunctions
            double mag = Angle.smallestDifference(currentRad, targetRad);
            double dir = Angle.turnDirection(currentRad, targetRad);
            double errorRad = (mag == Math.PI) ? -Math.PI : mag * dir;

            avgScaling += Math.abs(Math.cos(errorRad));
        }

        avgScaling /= pods.size();
        lastAvgScaling = avgScaling;

        long nowNano = System.nanoTime();
        double slewDt = lastArcadeNano == 0 ? 0 : (nowNano - lastArcadeNano) / 1.0e9;
        lastArcadeNano = nowNano;
        // A stalled caller (mode switch, OpMode pause) must not bank up allowance.
        double maxStep = (demandSlewDegPerSec > 0 && slewDt > 0 && slewDt < 0.5)
                ? Math.toRadians(demandSlewDegPerSec) * slewDt
                : Double.POSITIVE_INFINITY;

        for (int podNum = 0; podNum < pods.size(); podNum++) {
            // Normalizing if necessary while preserving relative sizes
            Vector2D finalVector = podVectors[podNum].times(1 / maxMagnitude);

            // RUCKUS PATCH: inside the X_LOCK engage delay the pod vectors are still the
            // degenerate zero vectors, so the pods must be released rather than sent chasing
            // a meaningless theta - same treatment IGNORE_ANGLE_CHANGES always gets.
            boolean release = zeroTrans && zeroRotation
                    && (config.zeroPowerBehavior.get() == SwerveConfig.ZeroPowerBehavior.IGNORE_ANGLE_CHANGES
                            || !xLockRipe);

            double theta = podThetas[podNum];
            if (release) {
                // Servos are off and the pods hold where they are, so the anchor stays valid and
                // is KEPT - dropping it made the first command after every release unlimited,
                // which is exactly the snap-out-of-park the limiter exists to smooth. Simulated
                // over mydrive-001: keeping it cut jumps of 85 deg or more from 4-6 per pod to
                // 1-2.
                lastCommandedTheta[podNum] = Double.isNaN(lastCommandedTheta[podNum])
                        ? theta : lastCommandedTheta[podNum];
            } else if (!Double.isNaN(lastCommandedTheta[podNum])
                    && maxStep != Double.POSITIVE_INFINITY) {
                double delta = Angle.normalizeSigned(theta - lastCommandedTheta[podNum]);
                // Past a quarter turn the pod does not rotate at all - it flips and reverses the
                // drive, which costs no travel and is the behaviour the flip hysteresis was tuned
                // for. Rate-limiting there would force a real 180 degree sweep with the drive
                // pointing the wrong way for the whole of it. So limit ordinary rotations only,
                // and re-anchor on the ones the pod will resolve by flipping.
                //
                // RUCKUS PATCH (2026-10-10): "past a quarter turn" is not the pod's rule. The pod
                // flips only past 90 + 10 degrees and keeps its previous flip state inside the
                // +-10 degree hysteresis band, so a 90-100 degree jump on an unflipped pod was
                // neither limited here nor flipped there - a ~95 degree snap at full authority.
                // Ask the pod: skip only when it will actually change its flip state.
                if (Math.abs(delta) <= Math.PI / 2.0
                        || !resolvesByFlip(pods.get(podNum), theta, currentRads[podNum])) {
                    if (delta > maxStep) {
                        theta = Angle.normalize(lastCommandedTheta[podNum] + maxStep);
                    } else if (delta < -maxStep) {
                        theta = Angle.normalize(lastCommandedTheta[podNum] - maxStep);
                    }
                }
                lastCommandedTheta[podNum] = theta;
            } else {
                lastCommandedTheta[podNum] = theta;
            }

            // RUCKUS PATCH (2026-10-10): a released pod gets exactly zero drive. The taper exists
            // to keep the demand DIRECTION continuous; the walls still decide release as booleans,
            // and before the taper (and in stock v3) a released pod's vectors were deleted, so it
            // was handed 0. With the taper the leftover sub-epsilon magnitude reached the motor
            // instead - and since the drive cache forces a write only at exactly 0, up to 0.05 of
            // stale power could sit on an unsteered wheel until X-lock (or forever under
            // IGNORE_ANGLE_CHANGES). Exact 0 restores the old contract and forces that write.
            double drive = release ? 0.0 : finalVector.magnitude() * avgScaling;
            pods.get(podNum).move(theta, drive, release);
        }
    }

    /**
     * Pod vectors for {@code powers}, stateless - used by {@link #maxScaling}. RUCKUS PATCH: same
     * geometry, rotation epsilon and taper as {@link #applyDrive}, so the scaling Foresight plans
     * with matches what the pods are given. X-lock is omitted: it only ever produces zero
     * vectors, and this is only read for magnitudes.
     *
     * <p>Known limitation (as upstream, less severe): {@link #maxScaling} assumes these vectors are
     * linear in {@code powers}, and the epsilon taper is not, so a sub-epsilon delta is
     * understated and over-granted - by at most epsilon of pod magnitude. See RUCKUS_PATCHES.md.
     */
    public Vector2D[] computePodPowers(DrivePowers powers) {
        double forward = powers.forward();
        double strafe = -powers.strafe();
        double rotation = powers.turn();

        double epsilon = config.epsilon.get();
        double rotEpsilon = Math.min(epsilon, ROTATION_EPSILON);

        Vector2D[] podVectors = new Vector2D[pods.size()];
        Vector2D rawTrans = Vector2D.polar(Range.clip(Math.hypot(strafe, forward), 0, 1), Math.atan2(forward, strafe));

        boolean zeroTrans = rawTrans.magnitude() < epsilon;
        boolean zeroRotation = Math.abs(rotation) < rotEpsilon;

        double transScale = epsilonTaper ? taper(rawTrans.magnitude(), epsilon) : 0.0;
        double rotScale = epsilonTaper ? taper(Math.abs(rotation), rotEpsilon) : 0.0;
        double rotationScalar = zeroRotation ? rotation * rotScale : rotation;
        Vector2D translationVector = zeroTrans ? rawTrans.times(transScale) : rawTrans;

        for (int i = 0; i < pods.size(); i++) {
            Vector2D rotationVector =
                    Vector2D.polar(rotationScalar, rotationBaseAngle(pods.get(i)) + Math.PI / 2);
            podVectors[i] = translationVector.plus(rotationVector);
        }

        return podVectors;
    }

    @Override
    public double maxScaling(DrivePowers current, DrivePowers delta) {
        double lambda = 1.0;

        Vector2D[] currentPowers = computePodPowers(current);
        Vector2D[] deltaPowers = computePodPowers(delta);

        for (int i = 0; i < currentPowers.length; i++) {
            Vector2D a = currentPowers[i];
            Vector2D b = deltaPowers[i];

            double quadraticTerm = b.magnitudeSquared();
            double linearTerm = 2 * a.dot(b);
            double constantTerm = a.magnitudeSquared() - 1;

            Pair<Double, Double> wheelSolution = Utils.solveQuadratic(quadraticTerm, linearTerm, constantTerm);

            double t1 =  wheelSolution.first();
            double t2 = wheelSolution.second();

            if (t1 >= 0.0 && t1 < lambda) lambda = t1;
            if (t2 >= 0.0 && t2 < lambda) lambda = t2;
        }

        return Utils.clamp(lambda, 0.0, 1.0);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        // RUCKUS PATCH: write only on change - see appliedZeroPowerBehavior.
        if (behavior == appliedZeroPowerBehavior) {
            return;
        }
        appliedZeroPowerBehavior = behavior;
        for (SwervePod pod : pods) {
            if (behavior == DcMotor.ZeroPowerBehavior.BRAKE)
                pod.setToBreak();
            else
                pod.setToFloat();
        }
    }

    /**
     * @return normalized voltage for voltage compensation
     */
    private double getVoltageNormalized() {
        double voltage = voltageSensor.getVoltage();
        double nominalVoltage = config.nominalVoltage.get();
        double staticFrictionCoefficient = config.staticFrictionCoefficient.get();
        return (nominalVoltage - (nominalVoltage * staticFrictionCoefficient)) / (voltage
                - ((nominalVoltage * nominalVoltage / voltage) * staticFrictionCoefficient));
    }
}
