package org.firstinspires.ftc.teamcode.diagnostics.swerve;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.revhub.drivetrains.CoaxialPod;
import com.pedropathing.revhub.drivetrains.Swerve;
import com.pedropathing.revhub.drivetrains.SwervePod;

import org.firstinspires.ftc.teamcode.SystemConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.PositionalPod;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.revhub.drivetrains.SwerveConfig;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.utils.Angle;
import com.pedropathing.utils.Utils;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCCommand;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * Interactive swerve bring-up, wiring verification and calibration tool.
 *
 * <p>Run this OpMode, then open <b>http://192.168.43.1:8080/swerve</b> on a laptop joined to the
 * robot's WiFi. Every control is also reachable from gamepad 1 so the tool still works when the
 * network does not.
 *
 * <p>The tool deliberately talks to raw {@link DcMotorEx}/{@link CRServo}/{@link AnalogInput}
 * devices for wiring work, so it functions even when the constants in
 * {@code Constants} are wrong or unknown. Once a pod is calibrated it is driven
 * through a real {@link CoaxialPod}, so PID tuning and kinematics behave exactly as they will in
 * competition code.
 *
 * <h2>What the automatic scans determine</h2>
 * <ul>
 *   <li><b>Encoder pairing</b> - which analog channel belongs to which pod. This matters because
 *       the Axon feedback wires are spliced two-per-analog-port, so the mapping is easy to get
 *       wrong and invisible until the robot drives strangely.</li>
 *   <li><b>Servo direction</b> - {@link CoaxialPod}'s turn PID requires that positive servo power
 *       makes the encoder reading increase. That relationship is measured, not guessed.</li>
 *   <li><b>Analog range</b> - the true min/max voltage of each Axon absolute output.</li>
 * </ul>
 *
 * <p>{@code encoderReversed} is the one value that cannot be measured from inside the robot: it
 * says whether a rising encoder reading means counter-clockwise pod rotation. The dashboard asks
 * the operator to watch one pod turn and answer once.
 */
public class SwerveBringUp extends OpMode {

    // ---------------------------------------------------------------- tuning knobs

    private static final int POD_COUNT = 4;

    /** Servo power used for wiring/sweep routines. Low enough to be safe on blocks. */
    private static final double SCAN_SERVO_POWER = 0.45;
    private static final double SCAN_SETTLE_S = 0.25;
    private static final double SCAN_MOVE_S = 0.70;
    private static final double SWEEP_SECONDS = 4.0;

    /** Drive-motor identification pulse. */
    private static final double MOTOR_PULSE_POWER = 0.25;
    private static final double MOTOR_PULSE_S = 0.60;

    /** Manual jog power for nudging a pod into alignment. */
    private static final double NUDGE_POWER = 0.25;

    /** Jog durations: a short tap for alignment, a longer run for watching rotation direction. */
    private static final double NUDGE_SECONDS = 0.35;
    private static final double SPIN_SECONDS = 3.0;

    /** Drive test stops if no command arrives within this window. */
    private static final long DRIVE_WATCHDOG_MS = 400;

    /**
     * Must match the {@link SwerveConfig} used to build the drivetrain, because the visualizer
     * reproduces {@code Swerve.applyDrive}'s kinematics to show the commanded pod state.
     */
    private static final double SWERVE_EPSILON = 0.05;

    /**
     * The mixer's rotation deadband, mirroring {@code Swerve.ROTATION_EPSILON}. Translation keeps
     * SWERVE_EPSILON; rotation gets its own, much lower wall so fine heading corrections survive.
     */
    private static final double SWERVE_ROTATION_EPSILON = 0.015;

    /** Mirrors {@code Swerve.X_LOCK_ENGAGE_DELAY_S}. */
    private static final double X_LOCK_ENGAGE_DELAY_S = 0.35;

    /** Mirrors the mixer's {@code lastActiveInputNano}, for the X-lock engage delay. */
    private long lastActiveTargetInputNano;
    private static final double SWERVE_MAX_POWER = 1.0;

    /** Minimum total analog movement (volts) for a channel to count as "responded". */
    private static final double SCAN_MIN_RESPONSE_V = 0.15;

    /** A channel must move this many times more than the runner-up to be an unambiguous match. */
    private static final double SCAN_AMBIGUITY_RATIO = 2.0;

    /**
     * Below this the analog channel is carrying no signal at all. A powered Axon outputs somewhere
     * in 0-3.3 V depending on shaft position, so a flat reading this low means the encoder is
     * unpowered or its wire is not landing on the port - not that the pod failed to turn.
     */
    private static final double DEAD_CHANNEL_VOLTS = 0.05;

    /** Voltage jumps larger than this fraction of range are treated as encoder wrap, not motion. */
    private static final double WRAP_FRACTION = 0.5;

    private static final int TRACE_LEN = 260;

    private static final File CAL_FILE = new File(AppUtil.FIRST_FOLDER, "swerve_bringup_cal.txt");

    // ---------------------------------------------------------------- modes

    private enum Mode {
        IDLE,
        WIRE_SCAN,
        MOTOR_PULSE,
        /** Manual turn-servo jog. Drives only the servo, and always times out. */
        JOG,
        ENC_SWEEP,
        PID,
        /** Robot heading step response: displace open-loop, then close the loop and record. */
        HEADING,
        AUTOTUNE,
        /** Guarded walk to a servo position, for measuring a positional pod's endpoints. */
        CAL_POS,
        DRIVE,
        /** The real Pedro follower driving, for translational/drive/centripetal PIDF tuning. */
        FOLLOW,
        /** Foresight Tuner stages on the follower's drivetrain, inside the fence (ForesightBoxTuner). */
        FORESIGHT
    }

    /** Actions that can move the robot, and so require START to have been pressed. */
    private static boolean isMotionCommand(String action) {
        return "wireScan".equals(action)
                || "sweep".equals(action)
                || "pulseMotor".equals(action)
                || "spinServo".equals(action)
                || "nudge".equals(action)
                || "pidStep".equals(action)
                || "pidStepAll".equals(action)
                || "rawServo".equals(action)
                || "autoTune".equals(action)
                || "headingStep".equals(action)
                || "headingGoto".equals(action)
                || "drive".equals(action)
                || "calGoto".equals(action)
                || "calHome".equals(action)
                // Drives the servo to an endpoint - physical motion, same as calGoto.
                || "calPositional".equals(action)
                // The follower drives the robot the moment it starts (holdPoint servoes).
                || "pedroStart".equals(action)
                || "pedroLine".equals(action)
                || "pedroHold".equals(action)
                || "pedroCurve".equals(action)
                // Starts the follower on a whole path; was missing (robot.py's MOTION had it).
                || "pedroChain".equals(action)
                // Foresight Tuner stages spin and drive the robot open loop.
                || "foresight".equals(action);
    }

    // ---------------------------------------------------------------- state

    private final PodCal[] cals = new PodCal[POD_COUNT];
    private final DcMotorEx[] motors = new DcMotorEx[POD_COUNT];
    private final CRServo[] servos = new CRServo[POD_COUNT];

    /**
     * The same ports, when configured as positional servos instead.
     *
     * <p>The SDK builds a different device class per port type, so a port declared {@code Servo}
     * cannot be fetched as a {@code CRServo} and vice versa - the get simply throws. During the
     * positional A/B one port is Servo and three are CR, so both arrays are populated and every
     * site uses whichever is non-null. Losing this would cost PWM enable/disable on exactly the
     * pod under test, which is what the holding-current measurement toggles.
     */
    private final Servo[] posServos = new Servo[POD_COUNT];
    private final AnalogInput[] encoders = new AnalogInput[POD_COUNT];
    private final double[] volts = new double[POD_COUNT];

    /**
     * Turn-servo power actually commanded this loop, per pod.
     *
     * <p>Tracked here rather than read back from the {@code CRServo}, because during closed-loop
     * modes the pod writes through its own device object and {@code CoaxialPod}'s output caching
     * means the controller's output and the servo's held power are different numbers. The recorder
     * wants the held one.
     */
    private final double[] servoCmd = new double[POD_COUNT];

    /**
     * Drive-motor power actually commanded this loop, per pod, for the same reason as
     * {@link #servoCmd}: {@code DcMotorEx.getPower()} is a live Lynx transaction, not a cached
     * field, and the dashboard was paying eight of those per publish just to display two numbers.
     */
    private final double[] motorCmd = new double[POD_COUNT];

    /** Loop-rate recording of every pod, pulled as CSV from {@code /swerve/rec.csv}. */
    private final PodRecorder recorder = new PodRecorder();

    // Scratch rows reused each loop so recording allocates nothing in the control path.
    private final double[] recWheel = new double[POD_COUNT];
    private final double[] recTarget = new double[POD_COUNT];
    /** The demand read back out of each pod, degrees. See PodRecorder's ctgt column. */
    private final double[] recCmdTarget = new double[POD_COUNT];
    private final double[] recError = new double[POD_COUNT];
    private final boolean[] recFlipped = new boolean[POD_COUNT];

    /**
     * Which pods went through {@code CoaxialPod.move()} this loop.
     *
     * <p>Only those pods have a meaningful error and turn power to read back; for the rest the
     * pod's cached values are left over from whenever it was last driven, and recording them would
     * put stale numbers in the trace that look like real measurements.
     */
    private final boolean[] podMoved = new boolean[POD_COUNT];

    /**
     * The servo PWM configuration actually in force, read back from the hardware.
     *
     * <p>Nothing in this codebase ever called {@code setPwmRange}, so these are whatever the SDK
     * defaults to - and the frame period in particular is a hard bound on actuation bandwidth that
     * no amount of loop rate can improve. Reported rather than assumed.
     */
    private final double[] pwmLower = new double[POD_COUNT];
    private final double[] pwmUpper = new double[POD_COUNT];
    private final double[] pwmFrame = new double[POD_COUNT];

    /**
     * Servo rail current, milliamps, for the whole hub.
     *
     * <p>{@code LynxModule} exposes GPIO and I2C bus current but not the servo rail; the channel
     * exists in the Lynx protocol and is reached by sending the ADC command directly.
     *
     * <p>Aggregate across all four ports, which is enough: with one servo's PWM enabled and the
     * rest limp, the difference between enabled and disabled is that servo's holding current, and
     * everything else on the rail cancels. Read at the slow sensor cadence, not every loop - it is
     * a bus round trip and a holding current is DC.
     */
    private LynxModule servoRailModule;
    private double servoRailMa;

    /** Worst-case clamp margin from the last positional coverage proof, degrees. */
    private double positionalCoverageDeg = Double.NaN;

    // ---- guarded endpoint calibration ----
    /**
     * The endpoints have to be found by driving to them, which is the one time an
     * uncalibrated positional pod moves. The pod's mechanical range may be smaller than the
     * servo's programmed travel, so this walks there in small steps and stops the moment the
     * encoder stops following - the difference between measuring a limit and grinding into one.
     */
    private static final double CAL_STEP = 0.01;
    private static final double CAL_DWELL_S = 0.15;
    private static final double CAL_MIN_MOVE_DEG = 1.0;

    /**
     * Dwells at an unchanged command before calling it a mechanical limit.
     *
     * <p>calHome measured a genuine 0.38 s stiction stall mid-travel, so a couple of dwells is far
     * too eager - it would abort on ordinary friction and never reach an end. Twelve dwells is
     * 1.8 s, five times that stall, and still well inside the servo's first overload stage at
     * 5.1 s, so a real end stop is caught long before anything is stressed.
     */
    private static final int CAL_MAX_STALLS = 12;

    private double calPos;
    private double calTargetPos;
    private double calLastRaw;
    private int calStalls;
    private int calSteps;

    /**
     * Whether {@link #calPos} is known to match where the servo physically is.
     *
     * <p>{@code Servo.getPosition()} returns the controller's last <em>commanded</em> position, not
     * a measurement - after a restart it is a default that has nothing to do with the pod. Walking
     * outward from it would make the first 0.02 step an absolute command to somewhere arbitrary,
     * and the pod would cross up to half its travel in one go. The incremental walk protects every
     * step except the one that matters, so calGoto refuses until calHome has established a true
     * starting position.
     */
    private boolean calHomed;
    private double batteryMa;
    private double totalMa;

    /**
     * The four analog channels addressed by their fixed config names, independent of which pod each
     * one currently belongs to. The wiring scan must work in this space: once a scan remaps a pod's
     * encoder, {@link #encoders} no longer lines up with channel numbering, so scanning against it
     * would mislabel the result on any re-run.
     */
    private final AnalogInput[] channels = new AnalogInput[POD_COUNT];
    private final String[] channelNames = new String[POD_COUNT];
    private final double[] channelVolts = new double[POD_COUNT];

    private final List<String> hwErrors = new ArrayList<>();
    private final List<String> scanNotes = new ArrayList<>();

    private SwervePod[] pods;
    private Swerve swerve;
    private boolean podsDirty = true;
    private String podBuildError;

    private VoltageSensor voltageSensor;

    private Mode mode = Mode.IDLE;
    private int selected = 0;
    private String message = "Ready. Put the robot on blocks before running any scan.";

    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime phaseTimer = new ElapsedTime();
    private double loopHz;

    /** Smoothed loop period, seconds. {@link #loopHz} is derived as its inverse. */
    private double loopDtEma;

    /** Hubs with MANUAL bulk caching, cleared once per loop at the top of serviceLoop. */
    private final List<LynxModule> lynxModules = new ArrayList<>();

    /**
     * The rotation value actually handed to {@code arcadeDrive} this loop. With heading hold on
     * this is the heading PID's output, not the stick - and the visualizer/recorder mirror in
     * {@link #computeTargets} has to use the same number or the recorded targets describe a
     * command that was never sent.
     */
    private double appliedTurn;
    private double appliedForward;
    private double appliedStrafe;

    /**
     * Field-oriented drive. Per drive command, not a mode: only commands that arrive with
     * foc=1 (the dashboard gamepad path when its toggle is on) are treated as field-frame
     * and rotated by live heading each loop. Bench scripts never send the flag, so their
     * robot-frame commands can never be double-rotated. The reference is captured by the
     * focRef command - the dashboard sends it as the toggle turns on, so "stick up" means
     * "the way the robot faced when the driver enabled it".
     */
    private boolean driveFieldOriented;
    private double focRefRad;

    /** Whether the heading was usable last DRIVE loop, to catch the sensor coming back. */
    private boolean headingWasOkInDrive;

    /**
     * Smoothed per-stage loop cost in milliseconds.
     *
     * <p>The pod turn loop runs at the OpMode's loop rate, so the loop rate is a control parameter,
     * not a diagnostic curiosity: at 30 Hz the derivative is differencing over 33 ms and the servo
     * is being told something new less often than its own 20 ms PWM frame. Knowing which stage
     * costs what is the difference between fixing that and guessing at it.
     */
    private double msEncoders;
    private double msHeading;
    private double msMode;
    private double msPublish;

    /** Smoothed per-section publish cost, milliseconds. See the "pub" block in {@link #publish}. */
    private final double[] msPubSection = new double[6];
    private double msTelemetry;

    /** Last unrecognised command action, surfaced in errors[] so a typo cannot pass unnoticed. */
    private volatile String unknownCommand;

    /**
     * Turn gains this tool is holding that differ from the ones the robot actually ships with.
     *
     * <p>Divergence is legitimate - holding non-shipped gains is what a tuning tool is for - so
     * this reports rather than corrects. What is not legitimate is measuring at gains nobody
     * intended, which is exactly what happened when the calibration file sat at kD 0.010 while
     * Constants said 0.022: every number taken through the tool in that window was
     * at a configuration no one had chosen, and nothing said so.
     *
     * @return one description per differing pod and coefficient, empty when they agree
     */
    private List<String> gainDivergences() {
        List<String> out = new ArrayList<>();
        for (int i = 0; i < POD_COUNT; i++) {
            PodCal c = cals[i];
            // Shipped gains are per-pod arrays (ss-indexed, same as this tool's pod index).
            appendIfDifferent(out, i, "kP", c.kP, Constants.turnKPPerPod[i]);
            appendIfDifferent(out, i, "kD", c.kD, Constants.turnKDPerPod[i]);
            appendIfDifferent(out, i, "kS", c.kS, Constants.turnKSPerPod[i]);
            appendIfDifferent(out, i, "kS band", c.kSBandDeg,
                    Constants.turnKSBandDegPerPod[i]);
            appendIfDifferent(out, i, "cache", c.servoCaching,
                    Constants.turnServoCaching);
            appendIfDifferent(out, i, "kF", c.kF, 0.0);
            appendIfDifferent(out, i, "kI", c.kI, 0.0);

            // Calibration, not just gains. Wrong gains cost tuning time; a wrong zero points the
            // wheel somewhere else entirely, and until 2026-08-13 nothing compared these at all -
            // a repair and a re-zero left this file 30-175 degrees out with no warning anywhere.
            // Tolerance is deliberately loose: these are hand-captured and a tenth of a degree of
            // disagreement is not worth shouting about, but tens of degrees is.
            double zeroDeg = Math.toDegrees(c.angleOffsetRad);
            double shippedZero = Constants.podZeroDeg[i];
            // Wrap-aware: 359 against 1 is two degrees apart, not 358.
            double zeroGap = Math.abs(((zeroDeg - shippedZero) % 360 + 540) % 360 - 180);
            if (zeroGap > 0.5) {
                out.add(String.format(Locale.US,
                        "pod %d zero: tool %.1f deg, shipped %.1f deg (%.1f apart) - the robot is "
                                + "NOT calibrated the way competition code will drive it",
                        i, zeroDeg, shippedZero, zeroGap));
            }
            appendIfDifferentTol(out, i, "min V", c.analogMin,
                    Constants.podMinV[i], 0.01, "V");
            appendIfDifferentTol(out, i, "max V", c.analogMax,
                    Constants.podMaxV[i], 0.01, "V");

            // Directions too. A flipped drive direction and a wrong zero produce the same
            // haywire drive, and on 2026-08-13 the tool held two direction flips the shipped
            // file knew nothing about - nothing compared them.
            if (c.driveReversed() != Constants.podDriveReversed[i]) {
                out.add(String.format(Locale.US,
                        "pod %d drive direction: tool %s, shipped %s - this corner will push the "
                                + "wrong way under competition code",
                        i, c.driveReversed() ? "REVERSE" : "FORWARD",
                        Constants.podDriveReversed[i] ? "REVERSE" : "FORWARD"));
            }
        }
        // Odometry geometry: a live odoConfig that never reached Constants.java means the pose -
        // and so the fence and every follower run here - is not the one competition code sees.
        if (!odometryShipped()) {
            out.add(String.format(Locale.US, "odometry: tool x pod %.3f in %s / y pod %.3f in %s, "
                            + "Constants.java %.3f %s / %.3f %s - poses here are NOT the ones "
                            + "competition code will compute",
                    odoXPodOffset, odoXPodReversed ? "REV" : "FWD",
                    odoYPodOffset, odoYPodReversed ? "REV" : "FWD",
                    Constants.pinpointXPodOffset, Constants.pinpointXPodReversed ? "REV" : "FWD",
                    Constants.pinpointYPodOffset, Constants.pinpointYPodReversed ? "REV" : "FWD"));
        }
        return out;
    }

    /**
     * Reports a divergence only when it exceeds {@code tol}, so hand-captured values that agree to
     * within measurement noise do not fill the error panel and train people to ignore it.
     */
    private void appendIfDifferentTol(List<String> out, int pod, String name, double tool,
            double shipped, double tol, String unit) {
        if (Math.abs(tool - shipped) <= tol) {
            return;
        }
        out.add(String.format(Locale.US,
                "pod %d %s: tool %.3f %s, shipped %.3f %s - the robot is NOT calibrated the way "
                        + "competition code will drive it",
                pod, name, tool, unit, shipped, unit));
    }

    private static void appendIfDifferent(List<String> out, int pod, String name,
            double tool, double shipped) {
        if (Math.abs(tool - shipped) > 1e-9) {
            out.add(String.format(Locale.US,
                    "pod %d %s: tool %.4f, shipped %.4f - measurements here are NOT at the "
                            + "competition configuration", pod, name, tool, shipped));
        }
    }

    private static double smooth(double previous, long nanos) {
        return 0.9 * previous + 0.1 * (nanos / 1.0e6);
    }

    /**
     * How often state is serialised and telemetry pushed.
     *
     * <p>Both were running every loop and together cost 8-13 ms of a 20-27 ms loop, which is to say
     * the dashboard was consuming half the control bandwidth. 20 Hz is far faster than anyone reads
     * a web page, well inside {@code SwerveBench}'s 1500 ms liveness window, and irrelevant to the
     * recorder, which samples every loop regardless.
     *
     * <p>NOT a constant any more, and the 8-13 ms above no longer holds: measured on 2026-08-13
     * the two together cost 42.7 ms of a 53.6 ms loop, so the dashboard is now eating four fifths
     * of the control bandwidth rather than half. publish() has grown a lot of fields since that
     * note was written. {@code setPublishHz} makes the rate switchable at runtime so the effect of
     * loop rate on pod tracking can be A/B'd without reflashing, and so a driving session can buy
     * control bandwidth back by giving up dashboard refresh.
     */
    private static final double PUBLISH_INTERVAL_DEFAULT_S = 0.05;

    /** Live publish period, seconds. Changed by {@code setPublishHz}. */
    private volatile double publishIntervalS = PUBLISH_INTERVAL_DEFAULT_S;

    /**
     * How often the Pinpoint and the raw analog channels are read when nothing needs them.
     *
     * <p>The Pinpoint costs 5.5 ms of I2C per read and pod rotation does not use heading at all;
     * the four {@code channels[]} reads exist only for the wiring scan. Both still refresh slowly
     * so the dashboard shows live numbers instead of frozen ones.
     */
    private static final double IDLE_SENSOR_INTERVAL_S = 0.2;

    private final ElapsedTime publishTimer = new ElapsedTime();
    private final ElapsedTime idleSensorTimer = new ElapsedTime();

    // routine bookkeeping
    private int routinePod;
    private int routinePhase;
    private boolean routineActive;

    /** Per-(pod, channel) accumulated absolute and signed movement during a wiring scan. */
    private double[][] scanAbs;
    private double[][] scanSigned;
    private double[] lastVolts = new double[POD_COUNT];

    /** Highest voltage each channel reached during a scan, used to tell "dead" from "didn't move". */
    private final double[] scanChannelPeak = new double[POD_COUNT];

    /** Observed min/max during a sweep. */
    private final double[] sweepMin = new double[POD_COUNT];
    private final double[] sweepMax = new double[POD_COUNT];

    // PID step-response trace
    /**
     * Rolling record of where the selected pod was told to point versus where it actually is.
     * Written whenever a target exists, so it covers PID steps, drive tests and auto-tuning alike.
     */
    private final double[] traceT = new double[TRACE_LEN];
    private final double[] traceTarget = new double[TRACE_LEN];
    private final double[] traceActual = new double[TRACE_LEN];
    private int traceCount;
    private int traceHead;
    private int tracePod = -1;
    private double pidTargetRad;
    private boolean pidHolding;


    /**
     * Step every pod together rather than one at a time.
     *
     * <p>On the ground a single pod turning alone has to scrub its tire while the other three pin
     * the chassis - a far harsher and more variable load than real driving, where all four rotate
     * together. Stepping them together is the representative test.
     */
    private boolean pidAllPods;

    // drive test
    private double driveForward;
    private double driveStrafe;
    private double driveTurn;
    /** Watchdog: drive stops if the browser stops sending commands (closed tab, WiFi drop). */
    private long lastDriveCmdMs;

    /** True once START has been pressed. Nothing is allowed to move before then. */
    private boolean started;

    /** Zero-input behaviour of the bench drivetrain: X_LOCK when true, hold heading when false. */
    private boolean xLock = true;

    // ---- robot heading (Pinpoint IMU; works with no odometry pods attached) ----
    private GoBildaPinpointDriver pinpoint;
    private double podoLastHeading = Double.NaN;
    private int podoSameHeadingReads;
    private double headingRad;
    private boolean headingOk;

    // ---- field pose (Pinpoint + odometry pods, attached 2026-08-14) ----
    /** Field-frame pose and velocity, inches and in/s, in the Pinpoint's frame since last reset. */
    private double poseXIn;
    private double poseYIn;
    private double poseVxIn;
    private double poseVyIn;
    private boolean poseOk;
    private boolean boxNeedsFrameCheck;
    // Well above the drivetrain's true maximum (~45 in/s flat out): only handling reads faster.
    private static final double POSE_SANITY_IN_S = 80.0;

    /**
     * Operator-defined bounding box the robot may drive in, field frame, inches.
     *
     * <p>Captured as two corners (drive to a corner, mark it, drive to the opposite corner, mark
     * it), normalised to min/max, persisted on the hub. It exists for two customers: a human
     * tuning session that wants a visual fence, and the autonomous tuning driver, which treats it
     * as a hard limit - {@link #applyBoxLimit} clamps any outward field-velocity component when
     * the robot is within a braking margin of a wall, and everywhere outside the box only inward
     * commands pass. The box lives in the Pinpoint's frame, so resetting the Pinpoint pose
     * invalidates it; resetImu therefore clears it rather than letting a stale fence point at
     * the wrong patch of floor.
     */
    private double boxMinX, boxMinY, boxMaxX, boxMaxY;
    private boolean boxValid;
    private double boxCorner0X, boxCorner0Y;
    private boolean boxMarked0;
    private boolean boxClampedNow;

    /**
     * Field fence (2026-10-01). A box armed by {@code fieldFence} is the FIELD - its rect is the
     * physical perimeter, so the robot's footprint (heading-aware) has to stay inside it rather
     * than its centre - plus the BIOBUZZ keep-outs from {@code field/biobuzz_field.json}, the
     * same file the Pedro visualizer draws and checks paths against. A box marked by driving to
     * two corners keeps the old meaning: a rect the robot CENTRE may occupy, no keep-outs.
     */
    private boolean boxBoundary;
    /** "marked" (corners A/B), "field" (BIOBUZZ perimeter + keep-outs) or "border" (a W x H in
     *  perimeter the operator defines, no keep-outs). */
    private String boxKind = "marked";
    private final List<FenceGeometry.Polygon> keepOuts = new ArrayList<>();
    /** /state's keep-out list, built once when the fence is armed - not on every publish. */
    private String keepOutsJson = "[]";
    /** Robot footprint the field fence protects, inches. Defaults to the FTC 18 in maximum. */
    private double fenceRobotL = SystemConstants.Zone.ROBOT_LENGTH;
    private double fenceRobotW = SystemConstants.Zone.ROBOT_WIDTH;

    private static final String FIELD_RESOURCE =
            "/org/firstinspires/ftc/teamcode/field/biobuzz_field.json";

    /** Keep-outs are checked against a circle of the footprint's half-diagonal: any heading. */
    private double fenceRobotRadius() {
        return Math.hypot(fenceRobotL, fenceRobotW) / 2;
    }

    private static final File BOX_FILE = new File(AppUtil.FIRST_FOLDER, "swerve_field_box.txt");

    /** Wall margin: base plus a braking allowance per in/s of speed toward that wall. */
    private static final double BOX_MARGIN_BASE_IN = 4.0;
    private static final double BOX_MARGIN_LOOKAHEAD_S = 0.30;

    // ---- Pedro follower bench ----
    /**
     * A real competition {@link com.pedropathing.follower.Follower}, driven by bench commands,
     * so the translational, drive and centripetal PIDFs are tuned through the exact control
     * stack that competes - host-side control over WiFi would tune against 150 ms of link
     * latency instead of the plant. Paths are always RELATIVE to the follower's current pose,
     * so the follower's internal coordinate convention never has to agree with the Pinpoint
     * frame; the box guard runs on the raw Pinpoint pose regardless and breaks the follower the
     * moment it strays past the fence.
     *
     * <p>Gain changes go through pedroPidf, which mutates the shared foresightConfig and
     * throws the follower away - the next pedroStart rebuilds from clean state, so no stale
     * internal copy of a coefficient can survive into a measurement.
     */
    private com.pedropathing.follower.Follower pedro;
    private String pedroJob = "idle";

    /**
     * Margin every follower waypoint must keep from the box walls, inches - the overshoot
     * allowance. A Bezier never leaves the convex hull of its control points, so validating
     * endpoints and control points against this bound is sufficient for the whole path.
     */
    private static final double PEDRO_TARGET_MARGIN_IN = 6.0;

    private boolean pedroBreach;
    private final ElapsedTime pedroBreachTimer = new ElapsedTime();

    /**
     * The rule the 2026-08-14 escape wrote in stone: NO follower motion command is accepted
     * unless every point of the path fits inside the box with margin. The host once sent a
     * 12 inch line whose target was already past the wall; the reactive fence broke the
     * follower at the wall and the robot coasted 8 inches off the mat on FLOAT motors. Bounds
     * are checked BEFORE anything moves, robot-side, so no host mistake can repeat that.
     */
    private boolean pedroPointOk(double x, double y) {
        // A field fence's rect is the perimeter; a path may turn the robot, so its footprint is
        // taken at its worst heading (the half-diagonal) on top of the margin.
        double inset = PEDRO_TARGET_MARGIN_IN + (boxBoundary ? fenceRobotRadius() : 0);
        return boxValid
                && x >= boxMinX + inset
                && x <= boxMaxX - inset
                && y >= boxMinY + inset
                && y <= boxMaxY - inset
                && keepOutClearance(x, y) >= PEDRO_TARGET_MARGIN_IN;
    }

    /** Heading PIDF under test. Units match FollowerConstants.headingPIDFCoefficients (radians). */
    // Seeded from what actually ships, so a heading session starts where the robot is rather than
    // where it used to be. These had been left at 1.75/0.003, the pre-tuning values, while
    // Constants moved to 1.20/0.030 - so the tool opened on gains the robot had
    // not used since 2026-08-11, and anything measured from that start would have been compared
    // against the wrong baseline. Not persisted, deliberately: the source of truth is
    // FollowerConstants.headingPIDFCoefficients, and a copy that outlived a session would just be
    // a second place to disagree.
    // Retuned 2026-08-13 late with the per-pod turn gains and the hold/correct hysteresis in
    // place: kD 0.030 (the 08-11 value, tuned at roughly a third of today's loop rate against a
    // slower inner loop) overshot 11-15 deg on a 90 deg step; 0.080 settles the same step in
    // 0.56 s with 2.3 deg worst overshoot, zero overshoot at 20 deg, and holds heading to 2.3
    // deg worst-case while translating with direction flips. kP above 1.2 bought no settle -
    // the swing is inertia-limited - and only overshoot. Keep in sync with
    // FollowerConstants.headingPIDFCoefficients, same as ever.
    private double headingKp = Constants.headingKP;
    private double headingKd = 0.080;
    private double headingKf = 0.0;

    /**
     * Follower gains as this tool last put them on the shared Foresight config: Constants at
     * init, then pedroPidf. Foresight's controllers are opaque lambdas, so these are the only
     * readable record of what the follower bench is actually running - and they are what export
     * writes, never the tool's heading hold.
     */
    private double fHeadingKP = Constants.headingKP;
    private double fFwdPrimKP = Constants.forwardTranslationalPrimaryKP;
    private double fFwdSecKP = Constants.forwardTranslationalSecondaryKP;
    private double fStrPrimKP = Constants.strafeTranslationalPrimaryKP;
    private double fStrSecKP = Constants.strafeTranslationalSecondaryKP;
    private double fTransKI = Constants.translationalKI;
    private double fTransKD = Constants.translationalKD;
    private double fFwdDecel = Constants.naturalForwardDeceleration;
    private double fStrDecel = Constants.naturalStrafeDeceleration;
    // Heading braking (lookahead) and pinned quadratic brake coefficients - NaN = derived as Constants does.
    private double fHeadBrakeLin = Constants.headingBrakeLinear;
    private double fHeadBrakeQuad = Constants.headingBrakeQuadratic;
    private double fFwdBrakeQuad = Double.NaN;
    private double fStrBrakeQuad = Double.NaN;

    /**
     * Writes the tracked follower gains onto the shared config. The quadratic brake coefficient
     * follows a deceleration change while it is still the coast-down derivation 1/(2a) in
     * Constants - so the bench brakes on the same model the splice will ship - and is left alone
     * once a Foresight Tuner measurement has replaced it.
     */
    private void applyFollowerGains() {
        ForesightConfig fc = Constants.foresightConfig;
        fc.headingFeedback.set(com.pedropathing.controllers.Controller.proportional(fHeadingKP));
        fc.forwardTranslational.set(Constants.translationalController(fFwdPrimKP, fFwdSecKP,
                fTransKI, fTransKD, Constants.translationalSwitchIn));
        fc.strafeTranslational.set(Constants.translationalController(fStrPrimKP, fStrSecKP,
                fTransKI, fTransKD, Constants.translationalSwitchIn));
        fc.naturalForwardDeceleration.set(fFwdDecel);
        fc.naturalStrafeDeceleration.set(fStrDecel);
        boolean fwdDerived = Constants.forwardBrakeQuadratic
                == 1.0 / (2 * Constants.naturalForwardDeceleration);
        boolean strDerived = Constants.strafeBrakeQuadratic
                == 1.0 / (2 * Constants.naturalStrafeDeceleration);
        fc.quadraticBrakeCoefficients.set(com.pedropathing.math.Matrix.diag(
                !Double.isNaN(fFwdBrakeQuad) ? fFwdBrakeQuad
                        : fwdDerived ? 1.0 / (2 * fFwdDecel) : Constants.forwardBrakeQuadratic,
                !Double.isNaN(fStrBrakeQuad) ? fStrBrakeQuad
                        : strDerived ? 1.0 / (2 * fStrDecel) : Constants.strafeBrakeQuadratic));
        fc.headingBrakeCoefficients.set(com.pedropathing.math.Vector2D.cartesian(fHeadBrakeLin, fHeadBrakeQuad));
    }

    /** Pinpoint geometry as last written to the device - Constants at init, then odoConfig. */
    private double odoXPodOffset = Constants.pinpointXPodOffset;
    private double odoYPodOffset = Constants.pinpointYPodOffset;
    private boolean odoXPodReversed = Constants.pinpointXPodReversed;
    private boolean odoYPodReversed = Constants.pinpointYPodReversed;

    /** Writes the tracked odometry geometry to the Pinpoint (does not touch the pose). */
    private void applyOdometry() {
        pinpoint.setOffsets(odoXPodOffset, odoYPodOffset, DistanceUnit.INCH);
        pinpoint.setEncoderDirections(
                odoXPodReversed ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                        : GoBildaPinpointDriver.EncoderDirection.FORWARD,
                odoYPodReversed ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                        : GoBildaPinpointDriver.EncoderDirection.FORWARD);
    }

    private boolean odometryShipped() {
        return odoXPodOffset == Constants.pinpointXPodOffset
                && odoYPodOffset == Constants.pinpointYPodOffset
                && odoXPodReversed == Constants.pinpointXPodReversed
                && odoYPodReversed == Constants.pinpointYPodReversed;
    }

    private double headingTargetRad;
    private boolean headingClosedLoop;
    private double headingOpenLoopPower;

    /**
     * Heading controller, driven exactly as Pedro drives it.
     *
     * <p>{@code ErrorCalculator} builds a SIGNED error as
     * {@code getTurnDirection(current, target) * getSmallestAngleDifference(current, target)}, and
     * {@code VectorCalculator.getHeadingVector} feeds the turn direction in as the feed-forward
     * input before clamping the output to max power. Reproducing that exactly is what makes gains
     * tuned here transfer to {@code FollowerConstants.headingPIDFCoefficients}.
     */
    private final PIDFController headingPidf = new PIDFController(
            new com.pedropathing.control.PIDFCoefficientSupplier() {
                @Override
                public PIDFCoefficients get(double error) {
                    // Supplied per call, because PIDFController.run() re-reads the supplier and
                    // discards anything set with setCoefficients().
                    return new PIDFCoefficients(headingKp, 0, headingKd, headingKf);
                }
            });

    /**
     * Right stick sweeps a heading SETPOINT rather than commanding rotation power directly.
     *
     * <p>Back ON by default. It was disabled 2026-08-13 late because it shook the robot
     * violently; the shake turned out to be the epsilon release/re-engage relay in the
     * actuation path (see {@link #headingCorrecting}), not the gains - though the gains were
     * also stale (kD retuned 0.030 -> 0.080 for the faster loop and inner turn loop). With the
     * hysteresis and the retune it settles a 90 degree step in 0.56 s with 2.3 deg worst
     * overshoot and disengages cleanly. The dashboard drive panel has the toggle.
     */
    private boolean headingHold = true;
    private boolean headingStickActive;

    /**
     * Heading-hold lifecycle at zero input: coast to a stop, adopt reality, hand over to X-lock.
     *
     * <p>ACTIVE runs the PIDF - stick input, translation, or a pending headingGoto. On release
     * the robot is usually still rotating faster than the swept setpoint (how much faster
     * changes with the battery), so chasing a latched target rubber-bands it backwards.
     * STOPPING instead commands zero rotation and watches the IMU until the chassis has
     * actually stopped; only then is the setpoint snapped to the measured heading, and RESTING
     * begins. In RESTING the heading PIDF never produces output: the pods form the X (the
     * Swerve-level engage delay has been counting since the input went quiet) and the X owns
     * the hold - mechanically, not by servoing. Any new demand re-adopts the current heading
     * before acting, so nothing stale can snap the robot.
     */
    private enum HeadingHoldPhase { ACTIVE, STOPPING, RESTING }

    private HeadingHoldPhase headingPhase = HeadingHoldPhase.RESTING;

    /** A commanded rotate-to-heading (headingGoto) counts as demand until it settles. */
    private boolean headingGotoActive;

    private final ElapsedTime headingStopTimer = new ElapsedTime();

    /** Chassis rotation below this reads as "stopped" for the STOPPING -> RESTING snap. */
    private static final double HEADING_STOPPED_RATE_RAD_S = Math.toRadians(15);

    /** STOPPING never waits longer than this - IMU noise must not stall the snap forever. */
    private static final double HEADING_STOP_TIMEOUT_S = 1.0;

    private static final double HEADING_EXIT_RAD = Math.toRadians(1.0);
    private static final double HEADING_EXIT_RATE_RAD_S = Math.toRadians(25);
    private static final double HEADING_MIN_ENGAGED_TURN = 0.06;

    /**
     * Soft-lock strength while translating. The epsilon floor alone (0.06) holds heading to
     * about a degree at crawl speed but lets it wander 3-9 degrees at 0.55 power - yaw
     * disturbance grows with speed, so the correction floor does too, up to this cap. The
     * engage/release pair is hysteresis so the trim never dithers on its own deadband.
     */
    private static final double HEADING_TRIM_MAX = 0.20;
    private static final double HEADING_TRIM_ENGAGE_RAD = Math.toRadians(1.2);
    private static final double HEADING_TRIM_RELEASE_RAD = Math.toRadians(0.5);
    private static final double HEADING_TRIM_RATE_GATE_RAD_S = Math.toRadians(10);
    // Just past arcadeDrive's 0.05 rotation epsilon: the smallest turn that reaches the pods.
    private static final double HEADING_EPS_BYPASS = 0.055;
    // Below this translation magnitude the bypass stays out of the mix (see the block comment).
    private static final double HEADING_TRIM_MIN_TRANS = 0.25;

    private boolean headingTrimEngaged;

    /** Measured heading rate, rad/s, for the CORRECTING exit gate. */
    private double headingRateRadS;
    private double headingPrevForRate = Double.NaN;
    private final ElapsedTime headingRateTimer = new ElapsedTime();

    /** Frozen-heading watchdog: a dead sensor plus a heading controller means a full-power spin. */
    private double headingLastSeen;
    private double headingStuckSeconds;
    private static final double HEADING_STUCK_LIMIT_S = 1.2;
    /**
     * Setpoint sweep rate at full stick, matched to what the robot can actually do.
     *
     * <p>Measured on this drivetrain: 0.3 stick -> 72 deg/s, 0.6 -> 165, 1.0 -> 328. Sweeping the
     * setpoint slower than the robot can turn wastes the difference - the controller only ever
     * needs a fraction of full power to keep up, so a held stick feels weak. Pedro's own teleop
     * maps the stick straight to rotation power, so full stick must mean full authority here too.
     */
    // Raised 5.5 -> 7.0 rad/s (~400 deg/s) on 2026-08-13 late: the robot out-turned the swept
    // setpoint on a fresh pack (measured 328 deg/s at full power, and that ceiling moves with
    // battery), so the sweep was the limiter and the felt speed changed through a match. Now
    // the sweep always outruns the chassis, the lead cap saturates the controller, and full
    // stick means the robot's true maximum whatever the pack voltage. The release rubber-band
    // this used to cause is gone: the hold phase machine stops, waits for the IMU to read
    // stationary, and snaps the setpoint to reality instead of chasing the latched lead.
    private static final double HEADING_STICK_RATE = 7.0;   // rad/s, ~400 deg/s

    /**
     * How far the setpoint may lead the measured heading.
     *
     * <p>Without this the setpoint outruns the robot, the error grows past 180 degrees, and
     * {@code getTurnDirection} flips to the now-shorter way round - so the robot reverses, stalls,
     * and surges. Capping the lead keeps the error inside the controller's linear region and makes
     * a held stick produce smooth continuous rotation.
     */
    private static final double HEADING_MAX_LEAD = Math.toRadians(60);

    /** How long the current jog should run before stopping itself. */
    private double jogSeconds;

    private String exportText = "";

    private final PodAutoTuner tuner = new PodAutoTuner();

    /** Pedro 3's Foresight Tuner stages, ported to run inside the fence (FORESIGHT mode). */
    private final ForesightBoxTuner fTuner = new ForesightBoxTuner();
    private final ElapsedTime autoTuneTimer = new ElapsedTime();
    private final ElapsedTime headingStickTimer = new ElapsedTime();

    /** Commanded pod state for the visualizer. NaN theta means "nothing commanded right now". */
    private final double[] targetTheta = new double[POD_COUNT];
    private final double[] targetPower = new double[POD_COUNT];

    // gamepad edge detection
    private boolean prevA;
    private boolean prevB;
    private boolean prevX;
    private boolean prevY;
    private boolean prevDpadUp;
    private boolean prevDpadDown;
    private boolean prevStart;

    // ---------------------------------------------------------------- lifecycle

    @Override
    public void init() {
        restoreShippedStatics();
        // Defaults mirror Constants: index 2 = LF, 1 = RF, 3 = LB, 0 = RB.
        double dtLength = 146.420;
        double dtWidth = 154.240;
        cals[0] = new PodCal(0, "RB", -dtLength, -dtWidth);
        cals[1] = new PodCal(1, "RF", dtLength, -dtWidth);
        cals[2] = new PodCal(2, "LF", dtLength, dtWidth);
        cals[3] = new PodCal(3, "LB", -dtLength, dtWidth);

        loadCalibration();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            // MANUAL, cleared once at the top of every loop, not AUTO. Under AUTO a REPEATED read
            // of the same channel inside one loop forces a fresh bulk transaction, and DRIVE reads
            // every pod encoder three times per loop (readEncoders, Swerve's avgScaling, and
            // CoaxialPod.move) - roughly eight extra bus round-trips per loop that MANUAL serves
            // from the cache. Within-loop consistency is a feature here: the recorded volts[] and
            // the error the pod acted on come from the same snapshot.
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            lynxModules.add(module);
            // The servos are all on the Control Hub itself, so the parent module owns the rail.
            if (servoRailModule == null || module.isParent()) {
                servoRailModule = module;
            }
        }

        acquireHardware();

        // After acquireHardware, not before: its first act is hwErrors.clear(), so anything
        // reported earlier was being thrown away unread. Guard, not a review - PodCal's serialiser
        // has fallen behind its fields more than once, and nothing fails when a field is simply
        // never written.
        for (String gap : PodCal.roundTripGaps()) {
            hwErrors.add("PodCal does not persist: " + gap);
        }

        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,
                    Constants.pinpointConfig.name.get());
            // Same configuration the competition localizer applies, referenced from the same
            // constants object so the two can never quietly disagree. Configuring does NOT
            // reset the pose - continuity across OpMode restarts is what makes a saved box
            // survive a redeploy. resetImu is the deliberate way to re-origin.
            // Mirrors PinpointLocalizer's constructor, minus its reset().
            com.pedropathing.revhub.localizers.PinpointConfig pc =
                    Constants.pinpointConfig;
            pinpoint.setOffsets(pc.xPodOffset.get(), pc.yPodOffset.get(), pc.offsetUnits.get());
            if (pc.ticksPerUnit.get().isPresent()) {
                pinpoint.setEncoderResolution(pc.ticksPerUnit.get().getAsDouble(),
                        pc.encoderResolutionUnit.get());
            } else {
                pinpoint.setEncoderResolution(pc.podType.get());
            }
            pinpoint.setEncoderDirections(pc.xPodDirection.get(), pc.yPodDirection.get());
            pinpoint.update();
        } catch (RuntimeException e) {
            pinpoint = null;
            hwErrors.add("No \"pinpoint\" device; heading tuning unavailable.");
        }
        loadBox();

        try {
            voltageSensor = hardwareMap.voltageSensor.iterator().next();
        } catch (RuntimeException e) {
            hwErrors.add("No voltage sensor found.");
        }
        // Seed the cache: the idle-sensor timer does not fire for another IDLE_SENSOR_INTERVAL_S,
        // and until it does every reader - dashboard, telemetry, and any recording started
        // immediately - would see 0 V and have no way to tell that from a dead battery.
        refreshBatteryVolts();

        SwerveBench.INSTANCE.clearCommands();
        SwerveBench.INSTANCE.setRecorder(recorder);
        computeTargets();
        publish();
        pushTelemetry();
    }

    @Override
    public void init_loop() {
        serviceLoop();
    }

    @Override
    public void start() {
        started = true;
        message = "Started. Robot can move now.";
        phaseTimer.reset();
        loopTimer.reset();
    }

    @Override
    public void loop() {
        serviceLoop();
    }

    @Override
    public void stop() {
        allStop();
        // The next run's reload check needs to know where the robot was LEFT, not where it was
        // when the box was marked - the robot drives between the two.
        refreshBoxWitness();
        restorePedroSpeedCap();
        restoreShippedStatics();
        SwerveBench.INSTANCE.markStopped();
    }

    // Vendored defaults (third_party/.../Swerve.java and CoaxialPod.java). Keep in step with them.
    private static final boolean VENDORED_EPSILON_TAPER = true;
    private static final double VENDORED_DEMAND_SLEW_DEG_S = 214.0;
    private static final double VENDORED_SMALL_TURN_ROLLOUT_S = 0.0;
    private static final double VENDORED_SCHED_FLOOR = 0.24;
    private static final double VENDORED_SCHED_VEL_START_RAD_S = Math.toRadians(22);
    private static final double VENDORED_SCHED_ERR_GATE_RAD = Math.toRadians(20);
    private static final double VENDORED_SCHED_RAMP_DRIVE = 0.10;

    /**
     * Puts every process-wide setting this tool can change back to what the installed build
     * ships: the Foresight gains on Constants.foresightConfig (pedroPidf), the mixer (setMixer)
     * and the turn-gain schedule. They are statics, so without this they outlive the OpMode - an
     * auto started after a tuning session, with no redeploy in between, would run on whatever
     * the session left, and so would this tool's own pods=shipped acceptance run. Called at init
     * (a previous run may have died without stop) and at stop.
     */
    private void restoreShippedStatics() {
        fHeadingKP = Constants.headingKP;
        fFwdPrimKP = Constants.forwardTranslationalPrimaryKP;
        fFwdSecKP = Constants.forwardTranslationalSecondaryKP;
        fStrPrimKP = Constants.strafeTranslationalPrimaryKP;
        fStrSecKP = Constants.strafeTranslationalSecondaryKP;
        fTransKI = Constants.translationalKI;
        fTransKD = Constants.translationalKD;
        fFwdDecel = Constants.naturalForwardDeceleration;
        fStrDecel = Constants.naturalStrafeDeceleration;
        fHeadBrakeLin = Constants.headingBrakeLinear;
        fHeadBrakeQuad = Constants.headingBrakeQuadratic;
        fFwdBrakeQuad = Double.NaN;
        fStrBrakeQuad = Double.NaN;
        applyFollowerGains();
        Swerve.setEpsilonTaper(VENDORED_EPSILON_TAPER);
        Swerve.setDemandSlewDegPerSec(VENDORED_DEMAND_SLEW_DEG_S);
        Swerve.setSmallTurnRolloutS(VENDORED_SMALL_TURN_ROLLOUT_S);
        CoaxialPod.setScheduleTuning(VENDORED_SCHED_FLOOR, VENDORED_SCHED_VEL_START_RAD_S,
                VENDORED_SCHED_ERR_GATE_RAD);
        CoaxialPod.setScheduleRamp(VENDORED_SCHED_RAMP_DRIVE);
    }

    private boolean followerGainsShipped() {
        return fHeadingKP == Constants.headingKP
                && fFwdPrimKP == Constants.forwardTranslationalPrimaryKP
                && fFwdSecKP == Constants.forwardTranslationalSecondaryKP
                && fStrPrimKP == Constants.strafeTranslationalPrimaryKP
                && fStrSecKP == Constants.strafeTranslationalSecondaryKP
                && fTransKI == Constants.translationalKI && fTransKD == Constants.translationalKD
                && fFwdDecel == Constants.naturalForwardDeceleration
                && fStrDecel == Constants.naturalStrafeDeceleration
                && fHeadBrakeLin == Constants.headingBrakeLinear
                && fHeadBrakeQuad == Constants.headingBrakeQuadratic
                && Double.isNaN(fFwdBrakeQuad) && Double.isNaN(fStrBrakeQuad);
    }

    /** True while the mixer and schedule are exactly what ships. */
    private boolean staticsShipped() {
        double[] sch = CoaxialPod.getScheduleTuning();
        return Swerve.getEpsilonTaper() == VENDORED_EPSILON_TAPER
                && Swerve.getDemandSlewDegPerSec() == VENDORED_DEMAND_SLEW_DEG_S
                && Swerve.getSmallTurnRolloutS() == VENDORED_SMALL_TURN_ROLLOUT_S
                && sch[0] == VENDORED_SCHED_FLOOR && sch[1] == VENDORED_SCHED_VEL_START_RAD_S
                && sch[2] == VENDORED_SCHED_ERR_GATE_RAD;
    }

    /** Shared body so the dashboard is fully usable during init, before START is pressed. */
    private void serviceLoop() {
        // MANUAL bulk caching: one fresh snapshot per loop, repeats served from cache.
        for (int i = 0; i < lynxModules.size(); i++) {
            lynxModules.get(i).clearBulkCache();
        }

        double dt = loopTimer.seconds();
        loopTimer.reset();
        if (dt > 0) {
            // Smooth dt and invert, not smooth 1/dt. Averaging instantaneous rates overweights
            // the short loops - the exact Jensen error that had every rate in this project
            // reading ~1.8x optimistic until 2026-08-13.
            loopDtEma = loopDtEma == 0 ? dt : 0.9 * loopDtEma + 0.1 * dt;
            loopHz = 1.0 / loopDtEma;
        }

        drainCommands();

        boolean refreshIdleSensors = idleSensorTimer.seconds() >= IDLE_SENSOR_INTERVAL_S;
        if (refreshIdleSensors) {
            idleSensorTimer.reset();
        }

        long mark = System.nanoTime();
        readEncoders(refreshIdleSensors);
        msEncoders = smooth(msEncoders, System.nanoTime() - mark);

        mark = System.nanoTime();
        readHeading(refreshIdleSensors);
        if (refreshIdleSensors || fastCurrent) {
            readServoRailCurrent();
        }
        if (refreshIdleSensors) {
            refreshBatteryVolts();
        }
        msHeading = smooth(msHeading, System.nanoTime() - mark);

        handleGamepad();

        mark = System.nanoTime();
        runMode();
        msMode = smooth(msMode, System.nanoTime() - mark);
        // Before record() so the trace carries this loop's commanded angles, not the previous
        // loop's; publish() then reuses what was computed here.
        computeTargets();
        record(dt);

        if (publishTimer.seconds() >= publishIntervalS) {
            publishTimer.reset();
            // Timed separately: msPublish used to cover both, which meant a 42 ms reading could
            // not distinguish "the JSON got too big" from "the SDK's telemetry push is slow".
            mark = System.nanoTime();
            publish();
            msPublish = smooth(msPublish, System.nanoTime() - mark);

            mark = System.nanoTime();
            pushTelemetry();
            msTelemetry = smooth(msTelemetry, System.nanoTime() - mark);
        }
    }

    /**
     * Appends one loop's worth of every pod's state to {@link #recorder}.
     *
     * <p>Runs after {@link #runMode()} so it captures the outputs this loop actually produced.
     */
    private void record(double dt) {
        if (!recorder.recording()) {
            return;
        }

        // While the follower drives, the pods that matter are ITS pods, and the tool's own
        // mixer mirror (targetTheta) is stale - so tgt is blanked and err/pwr/flip/ctgt come
        // from the follower's pods. That is what lets a Pedro run be scored on pod tracking.
        // FORESIGHT drives the follower's pods too (ForesightBoxTuner), so it records them like FOLLOW.
        boolean following = (mode == Mode.FOLLOW || mode == Mode.FORESIGHT) && pedro != null && pedroPods != null;
        SwervePod[] src = following ? pedroPods : pods;
        for (int i = 0; i < POD_COUNT; i++) {
            PodCal c = cals[i];
            recWheel[i] = Double.isNaN(volts[i])
                    ? Double.NaN
                    : Math.toDegrees(c.wheelThetaFromEncoder(c.zeroedAngleRad(volts[i])));
            recTarget[i] = following || Double.isNaN(targetTheta[i])
                    ? Double.NaN
                    : Math.toDegrees(normalizeTwoPi(targetTheta[i]));

            recCmdTarget[i] = Double.NaN;
            boolean moved = following || podMoved[i];
            if (moved && src != null && src[i] instanceof CoaxialPod) {
                CoaxialPod cp = (CoaxialPod) src[i];
                recError[i] = Math.toDegrees(cp.getLastErrorRad());
                recFlipped[i] = cp.wasLastMoveFlipped();
                servoCmd[i] = cp.getLastTurnPower();
                // The demand from inside the pod, not the computeTargets mirror. See PodRecorder.
                double ct = cp.getLastTargetWheelRad();
                recCmdTarget[i] = Double.isNaN(ct) ? Double.NaN
                        : Math.toDegrees(normalizeTwoPi(ct));
            } else if (moved && src != null && src[i] instanceof PositionalPod) {
                // Same error convention so one scorer reads both. Turn power is NaN by
                // construction - a positional pod has none - and everything downstream that keys
                // on it is meaningless here, which is why criterion 8 moved to holding current.
                PositionalPod pp = (PositionalPod) src[i];
                recError[i] = Math.toDegrees(pp.getLastErrorRad());
                recFlipped[i] = pp.wasLastMoveFlipped();
                servoCmd[i] = pp.getLastTurnPower();
            } else {
                recError[i] = Double.NaN;
                recFlipped[i] = false;
            }
        }

        recorder.add(dt, batteryVolts(), loopHz, mode.ordinal(), servoRailMa, batteryMa,
                volts, recWheel, recTarget, recCmdTarget, recError, servoCmd, recFlipped,
                headingOk ? Math.toDegrees(headingRad) : Double.NaN,
                (headingHold && headingOk) ? Math.toDegrees(headingTargetRad) : Double.NaN,
                poseOk ? poseXIn : Double.NaN,
                poseOk ? poseYIn : Double.NaN,
                // Post-fence, post-field-rotation robot frame: what actually drove the pods.
                appliedForward, appliedStrafe, appliedTurn,
                mode == Mode.FOLLOW && pedro != null && pedroPods != null ? followerSample() : null,
                driveSample(following, src));
    }

    private final double[] driveSample = new double[PodRecorder.DRIVE_COLS];

    /**
     * {p0..p3 drive power, command forward / strafe / turn, body vx, vy in/s, yaw rate rad/s}. Drive power is
     * what each pod's motor was last set to (the follower's pods while following, else the tool's motorCmd);
     * the command is the follower's / stage's DrivePowers as the vendored Swerve recorded them (Swerve.debug),
     * else the tool's applied mixer input. Velocity is the bring-up's own pose, rotated into the robot frame.
     */
    private double[] driveSample(boolean following, SwervePod[] src) {
        java.util.Arrays.fill(driveSample, Double.NaN);
        for (int i = 0; i < POD_COUNT; i++) {
            if (following && src != null && src[i] instanceof CoaxialPod) {
                driveSample[i] = ((CoaxialPod) src[i]).getLastDrivePower();
            } else {
                driveSample[i] = motorCmd[i];
            }
        }
        if (following) {
            try {
                Map<String, Object> dbg = pedro.drivetrain.debug();
                driveSample[4] = asDouble(dbg.get("forward"));
                driveSample[5] = asDouble(dbg.get("strafe"));
                driveSample[6] = asDouble(dbg.get("rotation"));
            } catch (RuntimeException e) {
                // leave NaN
            }
        } else {
            driveSample[4] = appliedForward;
            driveSample[5] = appliedStrafe;
            driveSample[6] = appliedTurn;
        }
        if (poseOk && headingOk) {
            double c = Math.cos(headingRad), sn = Math.sin(headingRad);
            driveSample[7] = poseVxIn * c + poseVyIn * sn;
            driveSample[8] = -poseVxIn * sn + poseVyIn * c;
            driveSample[9] = headingRateRadS;
        }
        return driveSample;
    }

    private static double asDouble(Object o) {
        return o instanceof Number ? ((Number) o).doubleValue() : Double.NaN;
    }

    private final double[] followSample = new double[PodRecorder.FOLLOW_COLS];

    /**
     * {fterr in, fherr deg, closest-point x, y, path heading deg, target velocity in/s, busy}
     * from Foresight, the numbers a Pedro validation is scored on. NaN where unavailable.
     */
    private double[] followerSample() {
        java.util.Arrays.fill(followSample, Double.NaN);
        try {
            com.pedropathing.algorithm.Algorithm a = pedro.algorithm();
            if (a instanceof com.pedropathing.algorithm.Foresight) {
                com.pedropathing.algorithm.Foresight f = (com.pedropathing.algorithm.Foresight) a;
                followSample[0] = f.translationalError();
                followSample[1] = Math.toDegrees(f.headingError());
                followSample[5] = f.targetVelocity();
            }
            if (pedro.following()) {
                com.pedropathing.math.Pose cp = pedro.closestPose();
                followSample[2] = cp.x();
                followSample[3] = cp.y();
                followSample[4] = Math.toDegrees(cp.heading());
            }
            followSample[6] = pedro.isBusy() ? 1 : 0;
        } catch (RuntimeException e) {
            // A follower between jobs can have no closest point; leave the row NaN.
        }
        return followSample;
    }

    // ---------------------------------------------------------------- hardware

    private void acquireHardware() {
        hwErrors.clear();
        for (int i = 0; i < POD_COUNT; i++) {
            PodCal c = cals[i];
            try {
                motors[i] = hardwareMap.get(DcMotorEx.class, c.motorName);
                motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                motors[i].setDirection(c.driveDirection);
            } catch (RuntimeException e) {
                motors[i] = null;
                hwErrors.add("Missing drive motor \"" + c.motorName + "\" (pod " + i + ").");
            }
            // A port declared Servo cannot be fetched as a CRServo and vice versa - the SDK
            // builds a different device class per port type and the get simply throws. During the
            // positional A/B one port is Servo and three are CR, so try both and keep whichever
            // answers; every site downstream uses the one that is non-null.
            servos[i] = null;
            posServos[i] = null;
            String crFailure = null;
            try {
                servos[i] = hardwareMap.get(CRServo.class, c.servoName);
                servos[i].setDirection(c.servoDirection);
                setServo(i, 0);
            } catch (RuntimeException e) {
                crFailure = e.getMessage();
            }
            if (servos[i] == null) {
                try {
                    posServos[i] = hardwareMap.get(Servo.class, c.servoName);
                } catch (RuntimeException e) {
                    // Name both attempts. "Missing" was misleading when the device was present
                    // and simply of the other type, which cost a bench session to work out.
                    hwErrors.add("Turn servo \"" + c.servoName + "\" (pod " + i
                            + ") is neither a ContinuousRotationServo nor a Servo. As CRServo: "
                            + crFailure + ". As Servo: " + e.getMessage());
                }
            }
            if (cals[i].positional && posServos[i] == null) {
                hwErrors.add("Pod " + i + " is configured positional but \"" + c.servoName
                        + "\" did not resolve as a Servo. Its port must be declared <Servo> in "
                        + "the active hardware configuration, not <ContinuousRotationServo>.");
            }
            if (!cals[i].positional && servos[i] == null && posServos[i] != null) {
                hwErrors.add("Pod " + i + " is configured continuous-rotation but \"" + c.servoName
                        + "\" resolved as a positional Servo. Set it positional, or change the "
                        + "port back to <ContinuousRotationServo>.");
            }
            readPwmRange(i);
            try {
                encoders[i] = hardwareMap.get(AnalogInput.class, c.encoderName);
            } catch (RuntimeException e) {
                encoders[i] = null;
                hwErrors.add("Missing analog encoder \"" + c.encoderName + "\" (pod " + i + ").");
            }

            // Fixed channel names, never remapped, so repeated wiring scans stay meaningful.
            channelNames[i] = "se" + i;
            try {
                channels[i] = hardwareMap.get(AnalogInput.class, channelNames[i]);
            } catch (RuntimeException e) {
                channels[i] = null;
            }
        }
    }

    /** True while heading is actually part of the control law rather than just a readout. */
    private boolean headingInUse() {
        return mode == Mode.HEADING || (mode == Mode.DRIVE && headingHold);
    }

    /**
     * True while the robot can translate under the box/fence: applyBoxLimit (DRIVE) and the
     * FOLLOW breach backstop act on pose, velocity and heading. On the 5 Hz idle path those are
     * up to 200 ms old - about 9 in at 45 in/s - so these modes read the Pinpoint every loop
     * whether or not heading hold is on (2026-10-10; it used to be heading-hold DRIVE only).
     */
    private boolean poseInUse() {
        // FORESIGHT added 2026-11-15: its fence check and room planning ran on the 5 Hz pose (staircase speed
        // in the jolt recordings) - up to 200 ms, ~5 in at 25 in/s, of stale room.
        return mode == Mode.DRIVE || mode == Mode.FOLLOW || mode == Mode.FORESIGHT;
    }

    /**
     * When set, the servo rail current is read every loop instead of on the 5 Hz idle path.
     *
     * <p>Off by default and deliberately so: each read is three Lynx transactions, which is
     * exactly the cost that made {@code publish()} expensive. It exists because current is the
     * only torque proxy available here, and a 5 Hz sample cannot see the peak of a 0.37 s rise -
     * the "max" it reports is wherever the sampler happened to land. Turn it on for a current
     * measurement, off again afterwards, and do not read pod-tracking numbers off a trace
     * recorded with it on.
     */
    private volatile boolean fastCurrent;

    /** Reads the hub's servo rail current in milliamps, or leaves the last value on failure. */
    private void readServoRailCurrent() {
        if (servoRailModule == null) {
            return;
        }
        try {
            LynxGetADCCommand cmd = new LynxGetADCCommand(servoRailModule,
                    LynxGetADCCommand.Channel.SERVO_CURRENT,
                    LynxGetADCCommand.Mode.ENGINEERING);
            servoRailMa = cmd.sendReceive().getValue();
            batteryMa = new LynxGetADCCommand(servoRailModule,
                    LynxGetADCCommand.Channel.BATTERY_CURRENT,
                    LynxGetADCCommand.Mode.ENGINEERING).sendReceive().getValue();
            totalMa = servoRailModule.getCurrent(CurrentUnit.MILLIAMPS);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        } catch (RuntimeException | LynxNackException e) {
            // A dropped ADC read is not worth failing a routine over; the stale value is obvious
            // in the trace because it repeats.
        }
    }

    /**
     * Enables or disables a servo's PWM output.
     *
     * <p>Disabling is how a holding current gets measured: the rail reading with one servo enabled
     * and holding, minus the reading with it disabled, is that servo's contribution with every
     * other draw on the rail cancelling out.
     */
    private void applyPwmEnable(int i, boolean enabled) {
        Object dev = servos[i] != null ? servos[i] : posServos[i];
        if (!(dev instanceof PwmControl)) {
            message = cals[i].servoName + " does not support PWM control.";
            return;
        }
        if (enabled) {
            ((PwmControl) dev).setPwmEnable();
        } else {
            setServo(i, 0);
            ((PwmControl) dev).setPwmDisable();
        }
        message = cals[i].servoName + " PWM " + (enabled ? "enabled" : "disabled");
    }

    /** Reads back the PWM pulse limits and frame period the hardware is actually using. */
    private void readPwmRange(int i) {
        pwmLower[i] = 0;
        pwmUpper[i] = 0;
        pwmFrame[i] = 0;
        Object dev = servos[i] != null ? servos[i] : posServos[i];
        if (!(dev instanceof PwmControl)) {
            return;
        }
        try {
            PwmControl.PwmRange r = ((PwmControl) dev).getPwmRange();
            pwmLower[i] = r.usPulseLower;
            pwmUpper[i] = r.usPulseUpper;
            pwmFrame[i] = r.usFrame;
        } catch (RuntimeException e) {
            hwErrors.add("Could not read PWM range for " + cals[i].servoName + ": " + e.getMessage());
        }
    }

    /**
     * Widens (or restores) the servo PWM pulse range.
     *
     * <p>The range is a property of the controller port, not of the Java object, so this applies to
     * the {@link CoaxialPod} writing to the same port as well.
     *
     * <p>Changing it rescales every gain: the same commanded power becomes a different pulse width
     * and therefore a different speed. Re-tune after, not before.
     */
    private void applyPwmRange(double lower, double upper, double frame, boolean allPods) {
        int from = allPods ? 0 : selected;
        int to = allPods ? POD_COUNT : selected + 1;
        for (int i = from; i < to; i++) {
            Object dev = servos[i] != null ? servos[i] : posServos[i];
            if (!(dev instanceof PwmControl)) {
                message = cals[i].servoName + " does not support PWM control.";
                continue;
            }
            try {
                setServo(i, 0);
                ((PwmControl) dev).setPwmRange(
                        new PwmControl.PwmRange(lower, upper, frame));
                readPwmRange(i);
            } catch (RuntimeException e) {
                message = "setPwmRange failed on " + cals[i].servoName + ": " + e.getMessage();
                return;
            }
        }
        message = String.format(Locale.US, "PWM range %.0f-%.0f us, frame %.0f us.",
                lower, upper, frame);
    }

    /** Heading comes off the Pinpoint's IMU, so it is valid with no odometry pods attached. */
    private void readHeading(boolean refreshIdleSensors) {
        if (pinpoint == null) {
            headingOk = false;
            return;
        }
        if (!headingInUse() && !poseInUse() && !refreshIdleSensors) {
            return;
        }
        try {
            pinpoint.update();
            double h = pinpoint.getHeading(AngleUnit.RADIANS);
            // Freeze diagnosis (2026-11-15): how many reads in a row returned the bit-identical heading.
            podoSameHeadingReads = h == podoLastHeading ? podoSameHeadingReads + 1 : 0;
            podoLastHeading = h;
            headingOk = !Double.isNaN(h);
            if (headingOk) {
                headingRad = h;
            }
            // Full pose from the odometry pods. getPosition/getVel read the snapshot update()
            // already fetched, so this costs no extra bus traffic.
            org.firstinspires.ftc.robotcore.external.navigation.Pose2D p =
                    pinpoint.getPosition();
            double x = p.getX(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH);
            double y = p.getY(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH);
            poseOk = headingOk && !Double.isNaN(x) && !Double.isNaN(y);
            if (poseOk) {
                poseXIn = x;
                poseYIn = y;
                poseVxIn = pinpoint.getVelX(
                        org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH);
                poseVyIn = pinpoint.getVelY(
                        org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH);
                if (boxValid && Math.hypot(poseVxIn, poseVyIn) > POSE_SANITY_IN_S) {
                    // Odometry reporting speeds this drivetrain cannot produce means the pods
                    // are not rolling on the ground - the robot is being carried or dragged -
                    // and the integrated position is garbage from that moment on. A fence in a
                    // corrupted frame is not a fence: session data showed pose jumping 10-20
                    // inches per tenth of a second, wheels parked, while the robot was
                    // repositioned by hand right after the box was marked.
                    boxValid = false;
                    if (BOX_FILE.exists()) {
                        BOX_FILE.delete();
                    }
                    message = "Box discarded: odometry jumped faster than the robot can move "
                            + "(picked up or dragged?). Re-mark the box.";
                }
                if (boxNeedsFrameCheck) {
                    boxNeedsFrameCheck = false;
                    // A box reloaded from disk is only meaningful if the Pinpoint still holds
                    // the frame it was marked in. A stale box is not a fence, it is a lie aimed
                    // at whatever now occupies those coordinates.
                    //
                    // The signature of a reset frame is the pose sitting at the origin. The
                    // tolerance used to be 0.05 in / 0.1 deg, which only catches a frame reset
                    // MOMENTS ago - and on 2026-08-16 that was not enough. Running DriveTeleOp
                    // re-origins the Pinpoint (Pedro's PinpointLocalizer constructor calls
                    // odo.setPosition on the hardware), and by the time bring-up came back the
                    // heading had drifted to -0.63 deg. The box would have loaded as valid and
                    // fenced a patch of floor 20 inches away from the real one.
                    //
                    // "Pose at the origin" was the only reset signature until 2026-10-10, and it
                    // is not enough: autos and TeleOp setPose a start pose such as (72, 8, 90),
                    // which is nowhere near the origin, and a witness saved within 6 in of the
                    // origin switched the check off. The rule now is positive confirmation:
                    // the box survives only if the robot is still where the witness says it
                    // was left - written by saveBox and refreshed at stop() - within a tight
                    // position AND heading tolerance. Anything else (no witness, a moved
                    // frame, a robot carried or pushed while no OpMode ran) discards it. When
                    // in doubt the operator re-marks; a false discard costs a minute, a false
                    // keep fences the wrong patch of floor.
                    if (boxValid) {
                        String mismatch = boxWitnessMismatch();
                        if (mismatch != null) {
                            String was = String.format(Locale.US, "%s, x %.1f..%.1f, y %.1f..%.1f",
                                    boxKind, boxMinX, boxMaxX, boxMinY, boxMaxY);
                            boxValid = false;
                            setMarkedFence();
                            if (BOX_FILE.exists()) {
                                BOX_FILE.delete();
                            }
                            message = "Saved box discarded: " + mismatch + ". The odometry frame "
                                    + "may have moved while this OpMode was not running (a power "
                                    + "cycle, an auto or TeleOp setting its start pose, any OpMode "
                                    + "that builds a Pedro Follower) or the robot was moved. "
                                    + "Re-arm it (it was " + was + ").";
                        }
                    }
                }
            }
        } catch (RuntimeException e) {
            headingOk = false;
            poseOk = false;
        }
    }

    // ---------------------------------------------------------------- field box

    /**
     * How far the reloaded pose may be from the witness before the box is discarded. Tight on
     * purpose (was a 12 in warning): a parked robot's Pinpoint does not move, so a real
     * difference means a moved frame or a moved robot, and either way the fence is unverified.
     */
    private static final double BOX_WITNESS_TOLERANCE_IN = 1.0;
    private static final double BOX_WITNESS_TOLERANCE_RAD = Math.toRadians(2.0);

    /** Where the robot was when the box was last written. See the frame check in readHeading. */
    private double boxWitnessX, boxWitnessY, boxWitnessHeadingRad;
    private boolean boxWitnessValid;

    /** Null when the current pose matches the box's witness, else why it does not. */
    private String boxWitnessMismatch() {
        if (!boxWitnessValid) {
            return "it has no witness pose, so the frame it was marked in cannot be verified";
        }
        double dPos = Math.hypot(poseXIn - boxWitnessX, poseYIn - boxWitnessY);
        double dHead = Math.abs(Angle.normalizeSigned(headingRad - boxWitnessHeadingRad));
        if (dPos <= BOX_WITNESS_TOLERANCE_IN && dHead <= BOX_WITNESS_TOLERANCE_RAD) {
            return null;
        }
        return String.format(Locale.US,
                "the pose is (%.1f, %.1f, %.1f deg) but the box was saved at (%.1f, %.1f, %.1f "
                        + "deg) - %.1f in / %.1f deg apart, tolerance %.1f in / %.1f deg",
                poseXIn, poseYIn, Math.toDegrees(headingRad), boxWitnessX, boxWitnessY,
                Math.toDegrees(boxWitnessHeadingRad), dPos, Math.toDegrees(dHead),
                BOX_WITNESS_TOLERANCE_IN, Math.toDegrees(BOX_WITNESS_TOLERANCE_RAD));
    }

    /**
     * Rewrites the box file so its witness is where the robot is NOW, from a fresh Pinpoint
     * read. Only for a box this run has already verified (or armed) - refreshing the witness of
     * a box whose frame check is still pending would launder an unverified fence. If the pose
     * cannot be read, the file is written without a witness, so the next reload discards it.
     */
    private void refreshBoxWitness() {
        if (!boxValid || boxNeedsFrameCheck) {
            return;
        }
        readHeading(true);
        if (boxValid) {   // the read's own sanity check may just have discarded it
            saveBox();
        }
    }

    private void saveBox() {
        FileWriter w = null;
        try {
            w = new FileWriter(BOX_FILE, false);
            w.write("# Field bounding box, inches, Pinpoint frame. minX|minY|maxX|maxY\n");
            w.write(boxMinX + "|" + boxMinY + "|" + boxMaxX + "|" + boxMaxY + "\n");
            if (boxBoundary) {
                // Field fence: the rect is the perimeter, and the keep-outs that were armed with
                // it travel with it, so a reload enforces exactly what the operator verified.
                w.write("# field/border fence: rect is the perimeter. robot|lengthIn|widthIn\n");
                w.write("kind|" + boxKind + "\n");
                w.write("robot|" + fenceRobotL + "|" + fenceRobotW + "\n");
                for (FenceGeometry.Polygon k : keepOuts) {
                    StringBuilder pts = new StringBuilder();
                    for (int i = 0; i < k.xs.length; i++) {
                        pts.append(i == 0 ? "" : ";").append(k.xs[i]).append(',').append(k.ys[i]);
                    }
                    w.write("keepout|" + k.name.replace('|', '/') + "|" + pts + "\n");
                }
            }
            // Frame witness: where the robot stood when this box was written. A box is only a
            // fence if the frame it was marked in is still the frame the Pinpoint is reporting,
            // and nothing else on the hub records which frame that was.
            if (poseOk) {
                w.write("# witness: pose when written. x|y|headingRad\n");
                w.write("witness|" + poseXIn + "|" + poseYIn + "|" + headingRad + "\n");
            }
            boxWitnessValid = poseOk;
            boxWitnessX = poseXIn;
            boxWitnessY = poseYIn;
            boxWitnessHeadingRad = headingRad;
        } catch (IOException e) {
            message = "Could not save box: " + e.getMessage();
        } finally {
            closeQuietly(w);
        }
    }

    private void loadBox() {
        if (!BOX_FILE.exists()) {
            return;
        }
        BufferedReader r = null;
        try {
            r = new BufferedReader(new FileReader(BOX_FILE));
            String line;
            boxBoundary = false;
            boxKind = "marked";
            keepOuts.clear();
            while ((line = r.readLine()) != null) {
                if (line.startsWith("#") || line.trim().isEmpty()) {
                    continue;
                }
                String[] p = line.split("\\|");
                if (p.length >= 2 && "kind".equals(p[0])) {
                    boxKind = p[1].trim();
                } else if (p.length >= 3 && "robot".equals(p[0])) {
                    fenceRobotL = Double.parseDouble(p[1]);
                    fenceRobotW = Double.parseDouble(p[2]);
                    boxBoundary = true;
                    if ("marked".equals(boxKind)) {
                        boxKind = "field";   // a file written before the kind line existed
                    }
                } else if (p.length >= 3 && "keepout".equals(p[0])) {
                    String[] pts = p[2].split(";");
                    double[] xs = new double[pts.length];
                    double[] ys = new double[pts.length];
                    for (int i = 0; i < pts.length; i++) {
                        String[] xy = pts[i].split(",");
                        xs[i] = Double.parseDouble(xy[0]);
                        ys[i] = Double.parseDouble(xy[1]);
                    }
                    keepOuts.add(new FenceGeometry.Polygon(p[1], xs, ys));
                } else if (p.length >= 4 && "witness".equals(p[0])) {
                    boxWitnessX = Double.parseDouble(p[1]);
                    boxWitnessY = Double.parseDouble(p[2]);
                    boxWitnessHeadingRad = Double.parseDouble(p[3]);
                    boxWitnessValid = true;
                } else if (p.length >= 4) {
                    boxMinX = Double.parseDouble(p[0]);
                    boxMinY = Double.parseDouble(p[1]);
                    boxMaxX = Double.parseDouble(p[2]);
                    boxMaxY = Double.parseDouble(p[3]);
                    boxValid = true;
                    boxNeedsFrameCheck = true;
                }
            }
        } catch (IOException | RuntimeException e) {
            // A half-read fence is worse than none: a perimeter without its keep-outs would
            // happily steer into a HIVE rail.
            boxValid = false;
            boxBoundary = false;
            boxKind = "marked";
            keepOuts.clear();
        } finally {
            closeQuietly(r);
        }
        keepOutsJson = buildKeepOutsJson();
    }

    /** Back to a plain marked box: centre-in-rect, no keep-outs. */
    private void setMarkedFence() {
        boxBoundary = false;
        boxKind = "marked";
        keepOuts.clear();
        keepOutsJson = "[]";
    }

    private String buildKeepOutsJson() {
        StringBuilder sb = new StringBuilder("[");
        for (int k = 0; k < keepOuts.size(); k++) {
            FenceGeometry.Polygon poly = keepOuts.get(k);
            sb.append(k == 0 ? "" : ",").append("{\"name\":\"").append(esc(poly.name))
                    .append("\",\"pts\":[");
            for (int i = 0; i < poly.xs.length; i++) {
                sb.append(i == 0 ? "" : ",").append('[').append(fmt(poly.xs[i])).append(',')
                        .append(fmt(poly.ys[i])).append(']');
            }
            sb.append("]}");
        }
        return sb.append(']').toString();
    }

    /**
     * The BIOBUZZ field from {@code field/biobuzz_field.json}, bundled into the APK through
     * TeamCode's resources srcDir: {size, keep-outs}. The same file the visualizer imports, so
     * the fence and the path editor can never disagree about where a rail is.
     */
    private double loadFieldKeepOuts(List<FenceGeometry.Polygon> out) throws IOException {
        java.io.InputStream in = SwerveBringUp.class.getResourceAsStream(FIELD_RESOURCE);
        if (in == null) {
            throw new IOException(FIELD_RESOURCE + " is not on the classpath");
        }
        try {
            java.io.ByteArrayOutputStream buf = new java.io.ByteArrayOutputStream();
            byte[] chunk = new byte[4096];
            int n;
            while ((n = in.read(chunk)) != -1) {
                buf.write(chunk, 0, n);
            }
            org.json.JSONObject field = new org.json.JSONObject(buf.toString("UTF-8"));
            org.json.JSONArray obs = field.getJSONArray("obstacles");
            for (int i = 0; i < obs.length(); i++) {
                org.json.JSONObject o = obs.getJSONObject(i);
                org.json.JSONArray vs = o.getJSONArray("vertices");
                double[] xs = new double[vs.length()];
                double[] ys = new double[vs.length()];
                for (int j = 0; j < vs.length(); j++) {
                    xs[j] = vs.getJSONObject(j).getDouble("x");
                    ys[j] = vs.getJSONObject(j).getDouble("y");
                }
                out.add(new FenceGeometry.Polygon(o.optString("name", o.optString("id")), xs, ys));
            }
            return field.getDouble("size");
        } catch (org.json.JSONException e) {
            throw new IOException("bad " + FIELD_RESOURCE + ": " + e.getMessage(), e);
        } finally {
            in.close();
        }
    }

    /** Walls the robot CENTRE must stay inside: {minX, minY, maxX, maxY}. */
    private double[] centreBounds(double heading) {
        if (!boxBoundary) {
            return new double[] {boxMinX, boxMinY, boxMaxX, boxMaxY};
        }
        double rx = FenceGeometry.reachX(fenceRobotL, fenceRobotW, heading);
        double ry = FenceGeometry.reachY(fenceRobotL, fenceRobotW, heading);
        return new double[] {boxMinX + rx, boxMinY + ry, boxMaxX - rx, boxMaxY - ry};
    }

    /** Clearance from the robot circle to the nearest keep-out, inches (infinite with none). */
    private double keepOutClearance(double x, double y) {
        return keepOuts.isEmpty() ? Double.POSITIVE_INFINITY
                : FenceGeometry.clearance(keepOuts, x, y) - fenceRobotRadius();
    }

    /**
     * Keep-out check for a follower path, as Bezier control points (a line is two). Returns
     * null when clear, else a refusal naming the obstacle.
     */
    private String pedroPathRefusal(double[][] cps) {
        if (keepOuts.isEmpty()) {
            return null;
        }
        double clear = FenceGeometry.bezierClearance(keepOuts, cps, 96) - fenceRobotRadius();
        if (clear >= PEDRO_TARGET_MARGIN_IN) {
            return null;
        }
        double worstX = cps[0][0];
        double worstY = cps[0][1];
        double worst = Double.POSITIVE_INFINITY;
        for (int s = 0; s <= 96; s++) {
            double[] pt = FenceGeometry.bezier(cps, s / 96.0);
            double c = FenceGeometry.clearance(keepOuts, pt[0], pt[1]);
            if (c < worst) {
                worst = c;
                worstX = pt[0];
                worstY = pt[1];
            }
        }
        FenceGeometry.Polygon near = FenceGeometry.nearest(keepOuts, worstX, worstY);
        return String.format(Locale.US,
                "REFUSED: the path passes %.1f in from %s near (%.1f, %.1f) - the robot (%.1f in "
                        + "half-diagonal) needs %.0f in of margin. Nothing moved.",
                clear, near == null ? "a keep-out" : near.name, worstX, worstY,
                fenceRobotRadius(), PEDRO_TARGET_MARGIN_IN);
    }

    /**
     * Clamps a commanded robot-frame translation so the robot cannot leave the box.
     *
     * <p>The command is rotated into the field frame; any component pointing at a wall the robot
     * is within braking distance of is removed (the margin grows with measured speed toward that
     * wall), and outside the box only inward components survive. Rotation is untouched. With a
     * box armed and the pose unreadable, translation is refused outright - an autonomous driver
     * with no fence and no eyes is how robots leave tables.
     *
     * @return {forward, strafe} to actually command
     */
    /**
     * How far outside the hard margin the outward component starts being reduced, inches.
     * At or inside the margin the factor is 0 - the fence is exactly as hard as it was.
     */
    private static final double BOX_TAPER_IN = 6.0;

    /**
     * Fraction of an outward velocity component to keep, given how far the robot still is past
     * the hard margin. Smoothstep, so both the value and its slope are continuous at each end.
     */
    private static double outwardScale(double slackIn) {
        return FenceGeometry.taper(slackIn, BOX_TAPER_IN);
    }

    private double[] applyBoxLimit(double forward, double strafe) {
        boxClampedNow = false;
        // NaN slips through every comparison below unclamped. Inputs are validated upstream;
        // this keeps the fence from ever passing one regardless.
        if (Double.isNaN(forward) || Double.isNaN(strafe)
                || Double.isInfinite(forward) || Double.isInfinite(strafe)) {
            boxClampedNow = true;
            return new double[] {0, 0};
        }
        if (!boxValid) {
            return new double[] {forward, strafe};
        }
        if (!poseOk) {
            if (forward != 0 || strafe != 0) {
                boxClampedNow = true;
                message = "Box armed but pose unreadable - translation refused.";
            }
            return new double[] {0, 0};
        }
        double ch = Math.cos(headingRad);
        double sh = Math.sin(headingRad);
        double vx = forward * ch - strafe * sh;
        double vy = forward * sh + strafe * ch;

        double[] cb = centreBounds(headingRad);
        double cMinX = cb[0], cMinY = cb[1], cMaxX = cb[2], cMaxY = cb[3];
        double hiX = BOX_MARGIN_BASE_IN + Math.max(0, poseVxIn) * BOX_MARGIN_LOOKAHEAD_S;
        double loX = BOX_MARGIN_BASE_IN + Math.max(0, -poseVxIn) * BOX_MARGIN_LOOKAHEAD_S;
        double hiY = BOX_MARGIN_BASE_IN + Math.max(0, poseVyIn) * BOX_MARGIN_LOOKAHEAD_S;
        double loY = BOX_MARGIN_BASE_IN + Math.max(0, -poseVyIn) * BOX_MARGIN_LOOKAHEAD_S;

        // Taper the outward component across BOX_TAPER_IN rather than deleting it at the
        // threshold. Zeroing one field axis is the right SHAPE for an axis-aligned wall - it
        // slides along it - but as a step it rotates the commanded direction in one loop, and
        // since the margin itself shrinks as the robot slows it then chatters: clamp, stop
        // advancing, unclamp, clamp. The taper keeps the fence hard at the wall (the factor
        // reaches 0 at exactly the same place) and makes the approach continuous.
        boolean clamped = false;
        double fx = outwardScale(cMaxX - hiX - poseXIn);
        if (vx > 0 && fx < 1.0) {
            vx *= fx;
            clamped = true;
        }
        double fxLo = outwardScale(poseXIn - (cMinX + loX));
        if (vx < 0 && fxLo < 1.0) {
            vx *= fxLo;
            clamped = true;
        }
        double fy = outwardScale(cMaxY - hiY - poseYIn);
        if (vy > 0 && fy < 1.0) {
            vy *= fy;
            clamped = true;
        }
        double fyLo = outwardScale(poseYIn - (cMinY + loY));
        if (vy < 0 && fyLo < 1.0) {
            vy *= fyLo;
            clamped = true;
        }
        // Keep-outs: the same taper and margin, along each obstacle's own normal instead of a
        // field axis, against the robot's half-diagonal circle. Applied after the walls so a
        // rail near a wall cannot be escaped by the wall clamp re-adding what it removed - each
        // clamp only ever shrinks the component pointing at its own obstacle.
        for (FenceGeometry.Polygon k : keepOuts) {
            double[] kv = FenceGeometry.clampKeepOut(k, poseXIn, poseYIn, vx, vy,
                    poseVxIn, poseVyIn, fenceRobotRadius(), BOX_MARGIN_BASE_IN,
                    BOX_MARGIN_LOOKAHEAD_S, BOX_TAPER_IN);
            if (kv[2] != 0) {
                vx = kv[0];
                vy = kv[1];
                clamped = true;
            }
        }
        boxClampedNow = clamped;
        if (!clamped) {
            return new double[] {forward, strafe};
        }
        return new double[] {vx * ch + vy * sh, -vx * sh + vy * ch};
    }

    /**
     * Reads the four pod encoders every loop, and the four raw channels only when something wants
     * them.
     *
     * <p>{@code channels[]} duplicates {@code encoders[]} through fixed names so a wiring scan
     * stays meaningful after a remap. Outside a scan nothing reads it, and it was costing half of
     * this method's 4.6 ms every loop.
     */
    private void readEncoders(boolean refreshIdleSensors) {
        for (int i = 0; i < POD_COUNT; i++) {
            volts[i] = readVoltage(encoders[i]);
        }
        if (mode == Mode.WIRE_SCAN || refreshIdleSensors) {
            for (int i = 0; i < POD_COUNT; i++) {
                channelVolts[i] = readVoltage(channels[i]);
            }
        }
    }

    private static double readVoltage(AnalogInput input) {
        if (input == null) {
            return Double.NaN;
        }
        try {
            return input.getVoltage();
        } catch (RuntimeException e) {
            return Double.NaN;
        }
    }

    /** Rebuilds the real Pedro pods after any calibration change. */
    private void rebuildPods() {
        podBuildError = null;
        try {
            SwervePod[] built = new SwervePod[POD_COUNT];
            for (int i = 0; i < POD_COUNT; i++) {
                built[i] = cals[i].toSwervePod(hardwareMap);
            }
            for (int pi = 0; pi < built.length; pi++) {
                SwervePod pod = built[pi];
                if (pod instanceof PositionalPod) {
                    // Coverage is proved against the real calibration before anything is commanded.
                    // A band narrower than 180 degrees leaves headings unreachable, and the pod
                    // would stop tracking silently rather than report it.
                    double margin = ((PositionalPod) pod).verifyCoverage(0.25);
                    positionalCoverageDeg = margin;
                    if (margin < 0) {
                        throw new IllegalStateException("positional pod cannot reach every "
                                + "heading: clamped band is " + fmt(margin + 180) + " deg wide, "
                                + "needs 180. Re-check the endpoint calibration.");
                    }
                    // Before anything commands it: a position-mode servo drives to whatever it is
                    // told the instant it has power. A refusal here means the boot read could not
                    // be trusted, and the pod stays uncommanded rather than being sent somewhere
                    // on the strength of a bad number.
                    if (!((PositionalPod) pod).initFromEncoder()
                            && ((PositionalPod) pod).hasInitReadFault()) {
                        hwErrors.add("Pod " + pi + ": boot encoder reads disagreed by more than "
                                + "2 deg, so nothing was commanded. Check the pod is not being "
                                + "moved, and that the encoder wrap is outside the travel band.");
                    }
                }
            }
            pods = built;

            SwerveConfig sc = new SwerveConfig(c -> {
                    c.voltageCompensation.set(false);
                    // X_LOCK now coexists with heading hold. It used to be disabled under hold
                    // because the heading PID output dipping under epsilon toggled the X on and
                    // off - but the hold phase machine passes EXACTLY zero rotation once it
                    // decides to rest, so the X engages once (after the Swerve-level delay) and
                    // stays. In RESTING the heading PIDF is silent by design: the X owns the
                    // hold and is never overridden.
                    c.zeroPowerBehavior.set(xLock
                            ? SwerveConfig.ZeroPowerBehavior.X_LOCK
                            : SwerveConfig.ZeroPowerBehavior.IGNORE_ANGLE_CHANGES);
                    c.manualBrakeMode.set(false);
                    // Set explicitly so it cannot drift from the visualizer's copy of the math.
                    // SWERVE_MAX_POWER has no setting in v3: the mixer always normalises to 1.0,
                    // which is the value it holds.
                    c.epsilon.set(SWERVE_EPSILON);
            });
            // Pedro's builder passes pods as leftFront, rightFront, leftBack, rightBack; each pod
            // carries its own offset so ordering only affects debug output.
            swerve = new Swerve(hardwareMap, sc, pods[2], pods[1], pods[3], pods[0]);
            podsDirty = false;
        } catch (RuntimeException e) {
            pods = null;
            swerve = null;
            podBuildError = e.getClass().getSimpleName() + ": " + e.getMessage();
        }
    }

    private void ensurePods() {
        if (podsDirty || pods == null) {
            rebuildPods();
        }
    }

    /**
     * Applies turn-servo power and records what was applied.
     *
     * <p>Every open-loop servo write in this class goes through here, so {@link #servoCmd} is a
     * faithful log of what the hardware was told rather than a reconstruction.
     */
    /** Drives the bench drivetrain and notes that every pod went through {@code move()}. */
    private void arcade(double forward, double strafe, double rotation) {
        swerve.applyDrive(new DrivePowers(forward, strafe, rotation));
        for (int i = 0; i < POD_COUNT; i++) {
            podMoved[i] = true;
        }
    }

    private void setServo(int i, double power) {
        servoCmd[i] = power;
        if (servos[i] != null) {
            servos[i].setPower(power);
        }
    }

    /** Every open-loop drive-motor write goes through here so {@link #motorCmd} stays faithful. */
    private void setMotor(int i, double power) {
        motorCmd[i] = power;
        if (motors[i] != null) {
            motors[i].setPower(power);
        }
    }

    private void allStop() {
        for (int i = 0; i < POD_COUNT; i++) {
            if (servos[i] != null) {
                setServo(i, 0);
            }
            setMotor(i, 0);
        }
        driveForward = 0;
        driveStrafe = 0;
        driveTurn = 0;
        appliedForward = 0;
        appliedStrafe = 0;
        appliedTurn = 0;
    }

    // ---------------------------------------------------------------- mode runner

    private void runMode() {
        // Re-established each loop by whichever branch actually drives pods through move().
        for (int i = 0; i < POD_COUNT; i++) {
            podMoved[i] = false;
        }

        // Nothing moves until the driver station START button is pressed. The dashboard stays
        // fully live during INIT so wiring can be inspected safely.
        if (!started && mode != Mode.IDLE) {
            allStop();
            mode = Mode.IDLE;
        }

        switch (mode) {
            case WIRE_SCAN:
                runWireScan();
                break;
            case ENC_SWEEP:
                runEncoderSweep();
                break;
            case MOTOR_PULSE:
                runMotorPulse();
                break;
            case JOG:
                runJog();
                break;
            case PID:
                runPidMode();
                break;
            case AUTOTUNE:
                runAutoTune();
                break;
            case CAL_POS:
                runCalPos();
                break;
            case HEADING:
                runHeadingTune();
                break;
            case DRIVE:
                runDriveMode();
                break;
            case FOLLOW:
                runFollowMode();
                break;
            case FORESIGHT:
                runForesightMode();
                break;
            case IDLE:
            default:
                // Servos limp so pods can be turned by hand for zeroing.
                for (int i = 0; i < POD_COUNT; i++) {
                    if (servos[i] != null) {
                        setServo(i, 0);
                    }
                    setMotor(i, 0);
                }
                break;
        }
    }

    private void setMode(Mode next) {
        // Leaving AUTOTUNE any way other than the tuner finishing must tell the tuner: its
        // update() simply stops being called, so without this isRunning() stays true forever.
        if (mode == Mode.AUTOTUNE && next != Mode.AUTOTUNE && tuner.isRunning()) {
            tuner.abort("mode changed to " + next);
        }
        // Leaving FOLLOW must break the follower - its pods are separate objects the bench's
        // allStop cannot reach, and an abandoned holdPoint would keep servoing forever.
        if (mode == Mode.FOLLOW && next != Mode.FOLLOW && pedro != null) {
            pedroBreak();
        }
        // Leaving FORESIGHT: the stage drives the follower's own pods, which allStop cannot reach.
        if (mode == Mode.FORESIGHT && next != Mode.FORESIGHT) {
            fTuner.abort("mode changed to " + next);
            if (pedro != null) {
                pedro.drivetrain.stop();
            }
        }
        // A wiring scan forces every turn servo to Direction.FORWARD so "positive power" is
        // unambiguous. Only finishWireScan restored them, so aborting a scan (STOP, another
        // mode) left reversed pods with a flipped feedback sign - closed loop runs away.
        if (mode == Mode.WIRE_SCAN && next != Mode.WIRE_SCAN) {
            for (int i = 0; i < POD_COUNT; i++) {
                if (servos[i] != null) {
                    servos[i].setDirection(cals[i].servoDirection);
                }
            }
        }
        allStop();
        mode = next;
        routineActive = false;
        routinePod = 0;
        routinePhase = 0;
        phaseTimer.reset();
    }

    // ---------------------------------------------------------------- wiring scan

    /**
     * Pulses each turn servo in turn while watching all analog channels.
     *
     * <p>The channel that moves is that pod's encoder, which resolves the spliced-wire mapping.
     * The direction it moves sets {@code servoDirection}, because {@link CoaxialPod} requires
     * positive servo power to increase the encoder reading.
     */
    private void runWireScan() {
        if (!routineActive) {
            // Pre-flight: if every channel is flat at zero there is no signal to measure, so say
            // that plainly instead of spending four seconds to report four identical failures.
            if (allChannelsDead()) {
                scanNotes.clear();
                scanNotes.add("All four analog channels read 0.00 V, so no encoder signal is "
                        + "reaching the Control Hub. This is a power or wiring fault common to all "
                        + "four pods, not four separate faults.");
                scanNotes.add("Check first: the Axon encoders only output a position voltage while "
                        + "the servos are powered. Confirm servo power, then that each spliced "
                        + "signal wire lands on analog 0-3 and shares ground with the hub.");
                scanNotes.add("Quick test needing no servo: leave this in IDLE so the pods are "
                        + "limp, turn a pod by hand and watch its volts on the pod card. Still "
                        + "0.00 V means the signal wire, not the servo.");
                message = "Scan aborted: no encoder signal on any channel.";
                setMode(Mode.IDLE);
                return;
            }

            scanAbs = new double[POD_COUNT][POD_COUNT];
            scanSigned = new double[POD_COUNT][POD_COUNT];
            for (int i = 0; i < POD_COUNT; i++) {
                scanChannelPeak[i] = 0;
            }
            scanNotes.clear();
            routineActive = true;
            routinePod = 0;
            routinePhase = 0;
            phaseTimer.reset();
            // Force a known hardware direction so "positive power" is unambiguous.
            for (int i = 0; i < POD_COUNT; i++) {
                if (servos[i] != null) {
                    servos[i].setDirection(DcMotorSimple.Direction.FORWARD);
                    setServo(i, 0);
                }
            }
            System.arraycopy(channelVolts, 0, lastVolts, 0, POD_COUNT);
            message = "Wiring scan running - watch which pod turns.";
        }

        if (routinePod >= POD_COUNT) {
            finishWireScan();
            return;
        }

        if (routinePhase == 0) {
            // settle
            if (servos[routinePod] != null) {
                setServo(routinePod, 0);
            }
            if (phaseTimer.seconds() >= SCAN_SETTLE_S) {
                System.arraycopy(channelVolts, 0, lastVolts, 0, POD_COUNT);
                routinePhase = 1;
                phaseTimer.reset();
            }
            return;
        }

        // moving phase
        if (servos[routinePod] != null) {
            setServo(routinePod, SCAN_SERVO_POWER);
        }
        accumulateScanDeltas(routinePod);

        if (phaseTimer.seconds() >= SCAN_MOVE_S) {
            if (servos[routinePod] != null) {
                setServo(routinePod, 0);
            }
            routinePod++;
            routinePhase = 0;
            phaseTimer.reset();
        }
    }

    /** True when no analog channel is carrying a usable signal. */
    private boolean allChannelsDead() {
        for (int ch = 0; ch < POD_COUNT; ch++) {
            double v = channelVolts[ch];
            if (!Double.isNaN(v) && v >= DEAD_CHANNEL_VOLTS) {
                return false;
            }
        }
        return true;
    }

    private void accumulateScanDeltas(int drivenPod) {
        double range = 3.3;
        for (int ch = 0; ch < POD_COUNT; ch++) {
            if (Double.isNaN(channelVolts[ch]) || Double.isNaN(lastVolts[ch])) {
                continue;
            }
            scanChannelPeak[ch] = Math.max(scanChannelPeak[ch], channelVolts[ch]);
            double d = channelVolts[ch] - lastVolts[ch];
            // Ignore wrap-around jumps; they are not real movement magnitude. A wrap then
            // contributes nothing, rather than a large delta with a misleading sign.
            if (Math.abs(d) < range * WRAP_FRACTION) {
                scanAbs[drivenPod][ch] += Math.abs(d);
                scanSigned[drivenPod][ch] += d;
            }
            lastVolts[ch] = channelVolts[ch];
        }
    }

    private void finishWireScan() {
        int[] assigned = new int[POD_COUNT];
        for (int p = 0; p < POD_COUNT; p++) {
            int best = -1;
            double bestVal = 0;
            double runnerUp = 0;
            for (int ch = 0; ch < POD_COUNT; ch++) {
                double v = scanAbs[p][ch];
                if (v > bestVal) {
                    runnerUp = bestVal;
                    bestVal = v;
                    best = ch;
                } else if (v > runnerUp) {
                    runnerUp = v;
                }
            }

            if (best < 0 || bestVal < SCAN_MIN_RESPONSE_V) {
                assigned[p] = -1;
                // Distinguish "the channel is dead" from "the channel is alive but nothing moved" -
                // they point at completely different faults.
                boolean anyChannelAlive = false;
                for (int ch = 0; ch < POD_COUNT; ch++) {
                    if (scanChannelPeak[ch] >= DEAD_CHANNEL_VOLTS) {
                        anyChannelAlive = true;
                        break;
                    }
                }

                if (!anyChannelAlive) {
                    scanNotes.add("Pod " + p + " (" + cals[p].servoName
                            + "): every analog channel is flat at 0 V, so there is no signal to "
                            + "measure. Check encoder power and the spliced signal wires.");
                } else {
                    scanNotes.add("Pod " + p + " (" + cals[p].servoName
                            + "): channels are live but none moved while this servo was driven. "
                            + "Either the servo is not turning (power, or a dead servo port), or "
                            + "this pod's encoder is unplugged.");
                }
                continue;
            }
            if (runnerUp > 0 && bestVal < runnerUp * SCAN_AMBIGUITY_RATIO) {
                scanNotes.add("Pod " + p + ": encoder match is ambiguous (best "
                        + fmt(bestVal) + "V vs " + fmt(runnerUp)
                        + "V). Two pods may be moving together, or a splice is shorted.");
            }

            assigned[p] = best;
            cals[p].discoveredEncoderIndex = best;
            cals[p].encoderName = channelNames[best];

            // CoaxialPod's PID assumes positive servo power raises the encoder reading.
            boolean needsReverse = scanSigned[p][best] < 0;
            cals[p].setServoReversed(needsReverse);
        }

        for (int p = 0; p < POD_COUNT; p++) {
            for (int q = p + 1; q < POD_COUNT; q++) {
                if (assigned[p] >= 0 && assigned[p] == assigned[q]) {
                    scanNotes.add("Pods " + p + " and " + q + " both mapped to "
                            + channelNames[assigned[p]]
                            + ". Re-seat the splice for that analog port.");
                }
            }
        }

        acquireHardware();
        podsDirty = true;
        routineActive = false;
        message = scanNotes.isEmpty()
                ? "Wiring scan complete - encoder pairing and servo directions set."
                : "Wiring scan finished with " + scanNotes.size() + " issue(s).";
        setMode(Mode.IDLE);
    }

    // ---------------------------------------------------------------- encoder sweep

    /** Spins each pod continuously and records the true analog endpoints of the Axon output. */
    private void runEncoderSweep() {
        if (!routineActive) {
            routineActive = true;
            routinePod = 0;
            phaseTimer.reset();
            for (int i = 0; i < POD_COUNT; i++) {
                sweepMin[i] = Double.MAX_VALUE;
                sweepMax[i] = -Double.MAX_VALUE;
                if (servos[i] != null) {
                    servos[i].setDirection(cals[i].servoDirection);
                }
            }
            message = "Sweeping pods to measure encoder range.";
        }

        if (routinePod >= POD_COUNT) {
            for (int i = 0; i < POD_COUNT; i++) {
                if (sweepMax[i] > sweepMin[i]) {
                    cals[i].analogMin = sweepMin[i];
                    cals[i].analogMax = sweepMax[i];
                }
            }
            podsDirty = true;
            routineActive = false;
            message = "Encoder range captured for all pods.";
            setMode(Mode.IDLE);
            return;
        }

        int p = routinePod;
        if (servos[p] != null) {
            setServo(p, SCAN_SERVO_POWER);
        }
        if (!Double.isNaN(volts[p])) {
            sweepMin[p] = Math.min(sweepMin[p], volts[p]);
            sweepMax[p] = Math.max(sweepMax[p], volts[p]);
        }

        if (phaseTimer.seconds() >= SWEEP_SECONDS) {
            if (servos[p] != null) {
                setServo(p, 0);
            }
            routinePod++;
            phaseTimer.reset();
        }
    }

    // ---------------------------------------------------------------- motor pulse

    private void runMotorPulse() {
        if (!routineActive) {
            routineActive = true;
            routinePod = selected;
            phaseTimer.reset();
            message = "Pulsing " + cals[routinePod].motorName + " - watch which wheel spins.";
        }
        if (phaseTimer.seconds() < MOTOR_PULSE_S) {
            setMotor(routinePod, MOTOR_PULSE_POWER);
        } else {
            allStop();
            routineActive = false;
            setMode(Mode.IDLE);
        }
    }

    /**
     * Holds a manual turn-servo jog until it times out. The servo power was already applied by the
     * command handler; this only enforces the deadline so a pod can never be left spinning.
     */
    private void runJog() {
        for (int i = 0; i < POD_COUNT; i++) {
            setMotor(i, 0);
        }
        if (phaseTimer.seconds() >= jogSeconds) {
            allStop();
            routineActive = false;
            setMode(Mode.IDLE);
        }
    }

    /** Applies servo power for a manual jog and arms the timeout. */
    private void beginJog(double power, double seconds, String note) {
        if (servos[selected] == null) {
            message = "Pod " + selected + " has no turn servo.";
            return;
        }
        // Through setMode, not a bare assignment: setMode owns the transition cleanup (aborting
        // a live autotune, restoring scan-forced servo directions) and a jog can be commanded
        // from any mode.
        setMode(Mode.JOG);
        servos[selected].setDirection(cals[selected].servoDirection);
        setServo(selected, power);
        jogSeconds = seconds;
        routineActive = true;
        phaseTimer.reset();
        message = note;
    }

    // ---------------------------------------------------------------- PID tuning

    private void runPidMode() {
        ensurePods();
        if (pods == null) {
            message = "Cannot build pods: " + podBuildError;
            setMode(Mode.IDLE);
            return;
        }

        for (int i = 0; i < POD_COUNT; i++) {
            setMotor(i, 0);
            // Pods not under test stay limp, unless every pod is being stepped together.
            if (!pidAllPods && i != selected && servos[i] != null) {
                setServo(i, 0);
            }
        }

        if (!pidHolding) {
            return;
        }

        if (pidAllPods) {
            for (int i = 0; i < POD_COUNT; i++) {
                pods[i].move(pidTargetRad, 0.0, false);
                podMoved[i] = true;
            }
        } else {
            pods[selected].move(pidTargetRad, 0.0, false);
            podMoved[selected] = true;
        }

        recordTrace(selected, pidTargetRad);
    }

    /**
     * Records commanded versus measured wheel heading for the graph.
     *
     * <p>Both are converted to wheel space, the frame the operator actually thinks in, rather than
     * the encoder frame {@link CoaxialPod} works in internally.
     */
    private void recordTrace(int pod, double targetWheelRad) {
        if (Double.isNaN(volts[pod])) {
            return;
        }

        PodCal c = cals[pod];
        double actualDeg = Math.toDegrees(c.wheelThetaFromEncoder(c.zeroedAngleRad(volts[pod])));
        double targetDeg = Math.toDegrees(normalizeTwoPi(targetWheelRad));

        // Keep the target on the same side of the wrap as the measurement, so a pod sitting at 359
        // chasing 1 draws as a small step rather than a full-scale cliff.
        double delta = targetDeg - actualDeg;
        if (delta > 180) {
            targetDeg -= 360;
        } else if (delta < -180) {
            targetDeg += 360;
        }

        // A pod treats target+180 as the same demand - move() flips it and drives in reverse when
        // that is the shorter rotation. Near a flip boundary the raw target snaps back and forth by
        // 180 degrees on input noise alone, while the pod sensibly does nothing. Plotting the
        // nearer equivalent shows what is actually being asked of the pod instead of that artefact.
        delta = targetDeg - actualDeg;
        if (delta > 90) {
            targetDeg -= 180;
        } else if (delta < -90) {
            targetDeg += 180;
        }

        if (pod != tracePod) {
            tracePod = pod;
            traceCount = 0;
            traceHead = 0;
        }

        traceT[traceHead] = phaseTimer.seconds();
        traceTarget[traceHead] = targetDeg;
        traceActual[traceHead] = actualDeg;
        traceHead = (traceHead + 1) % TRACE_LEN;
        if (traceCount < TRACE_LEN) {
            traceCount++;
        }
    }

    private void clearTrace(int pod) {
        tracePod = pod;
        traceCount = 0;
        traceHead = 0;
    }

    // ---------------------------------------------------------------- auto tuning

    private void runAutoTune() {
        ensurePods();
        if (pods == null) {
            message = "Cannot build pods: " + podBuildError;
            setMode(Mode.IDLE);
            return;
        }

        int pod = tuner.podIndex();
        if (Double.isNaN(volts[pod])) {
            message = "Pod " + pod + " encoder is unreadable; auto-tune stopped.";
            setMode(Mode.IDLE);
            return;
        }

        // Everything except the pod under test stays still.
        for (int i = 0; i < POD_COUNT; i++) {
            if (i != pod) {
                if (servos[i] != null) {
                    setServo(i, 0);
                }
            }
            setMotor(i, 0);
        }

        PodCal c = cals[pod];
        double wheelDeg = Math.toDegrees(c.wheelThetaFromEncoder(c.zeroedAngleRad(volts[pod])));
        double dt = Math.max(1e-4, autoTuneTimer.seconds());
        autoTuneTimer.reset();

        PodAutoTuner.Action action = tuner.update(dt, wheelDeg, c);

        if (tuner.rebuildRequested) {
            // Coefficients live in the pod's PIDF, which is set at construction, so the pod has to
            // be rebuilt for a new gain to take effect.
            rebuildPods();
            if (pods == null) {
                message = "Cannot rebuild pods: " + podBuildError;
                setMode(Mode.IDLE);
                return;
            }
        }

        switch (action) {
            case RAW_SERVO:
                if (servos[pod] != null) {
                    servos[pod].setDirection(c.servoDirection);
                    setServo(pod, tuner.rawServoPower);
                }
                break;
            case PID_HOLD:
                pods[pod].move(tuner.targetWheelRad, 0.0, false);
                podMoved[pod] = true;
                recordTrace(pod, tuner.targetWheelRad);
                break;
            case FINISHED:
            default:
                allStop();
                saveCalibration();
                message = "Auto-tune complete. " + tuner.status();
                setMode(Mode.IDLE);
                return;
        }

        message = "Auto-tuning pod " + pod + ": " + tuner.status();
    }

    /** One guarded step per dwell toward {@link #calTargetPos}, aborting on a stall. */
    private void runCalPos() {
        int i = selected;
        // Deliberately the raw Servo, not the pod object. Calibration is what produces the
        // endpoints a PositionalPod needs in order to exist, so requiring one here deadlocks:
        // the pod will not build without a valid band, and the band cannot be measured without
        // moving the servo.
        if (posServos[i] == null) {
            message = "Pod " + i + " is not on a Servo-configured port.";
            setMode(Mode.IDLE);
            return;
        }
        if (phaseTimer.seconds() < CAL_DWELL_S) {
            return;
        }
        phaseTimer.reset();

        double raw = Math.toDegrees(cals[i].rawAngleRad(volts[i]));
        boolean moved = calSteps == 0
                || Math.abs(((raw - calLastRaw) + 540.0) % 360.0 - 180.0) >= CAL_MIN_MOVE_DEG;

        // Hold the command when the pod is not following, rather than advancing anyway. Advancing
        // through a stall winds the servo's internal loop up against stiction, and calHome showed
        // what that produces: 25 degrees of accumulated error released at 390 deg/s. Waiting
        // instead keeps the command at most one step ahead of the pod, so a break is one step.
        if (!moved) {
            calStalls++;
            if (calStalls >= CAL_MAX_STALLS) {
                message = String.format(Locale.US,
                        "Calibration STOPPED at position %.3f, encoder %.2f deg: no movement for "
                                + "%.1f s at an unchanged command. This is a mechanical limit, not "
                                + "stiction. Mark it with calMark.",
                        calPos, raw, CAL_MAX_STALLS * CAL_DWELL_S);
                setMode(Mode.IDLE);
            }
            return;
        }

        calStalls = 0;
        calLastRaw = raw;

        if (Math.abs(calTargetPos - calPos) < 1e-6) {
            message = String.format(Locale.US,
                    "Reached commanded position %.3f, encoder %.2f deg. Mark it with calMark.",
                    calPos, raw);
            setMode(Mode.IDLE);
            return;
        }

        calPos += Utils.clamp(calTargetPos - calPos, -CAL_STEP, CAL_STEP);
        posServos[i].setPosition(Utils.clamp(calPos, 0.0, 1.0));
        calSteps++;
    }

    private void startAutoTune() {
        ensurePods();
        if (pods == null) {
            message = "Cannot build pods: " + podBuildError;
            return;
        }
        if (Double.isNaN(volts[selected])) {
            message = "Pod " + selected + " encoder is unreadable; cannot auto-tune.";
            return;
        }

        PodCal c = cals[selected];
        double wheelDeg = Math.toDegrees(c.wheelThetaFromEncoder(c.zeroedAngleRad(volts[selected])));

        // Through setMode, not a bare assignment, and before anything is armed: setMode owns the
        // transition cleanup (restoring scan-forced servo directions - a closed-loop step on a
        // FORWARD-forced reversed pod has its feedback sign flipped and spins forever - and
        // breaking the follower when leaving FOLLOW), and it resets routine state.
        setMode(Mode.AUTOTUNE);
        clearTrace(selected);
        tuner.start(selected, c, wheelDeg);
        autoTuneTimer.reset();
        phaseTimer.reset();
        routineActive = true;
        message = "Auto-tuning pod " + selected + ".";
    }


    /**
     * Rotation power to drive the robot's heading to {@link #headingTargetRad}, computed the way
     * {@code VectorCalculator.getHeadingVector} computes it.
     */
    private double headingCorrection() {
        double direction = Angle.turnDirection(headingRad, headingTargetRad);
        double error = direction
                * Angle.smallestDifference(headingRad, headingTargetRad);

        headingPidf.updateFeedForwardInput(direction);
        headingPidf.updateError(error);
        return Utils.clamp(headingPidf.run(), -SWERVE_MAX_POWER, SWERVE_MAX_POWER);
    }

    // ---------------------------------------------------------------- heading tuning

    /** Open-loop displacement time before the loop is closed. */
    private static final double HEADING_OPEN_LOOP_S = 0.55;

    /**
     * Robot heading step response.
     *
     * <p>The robot is first rotated open-loop away from the captured target, with the controller
     * off, then the loop is closed and the recovery recorded. Displacing under control would only
     * ever show the controller chasing itself; letting it start from a genuine offset is what
     * exposes the gain.
     */
    private void runHeadingTune() {
        ensurePods();
        if (swerve == null || !headingOk) {
            message = swerve == null
                    ? "Cannot build drivetrain: " + podBuildError
                    : "No heading from the Pinpoint; cannot tune heading.";
            setMode(Mode.IDLE);
            return;
        }

        double error = Angle.normalizeSigned(headingTargetRad - headingRad);

        if (!headingClosedLoop) {
            // Open loop: spin away from the target so the closed-loop phase starts displaced.
            if (phaseTimer.seconds() < HEADING_OPEN_LOOP_S) {
                arcade(0, 0, headingOpenLoopPower);
                message = String.format(Locale.US, "Displacing open-loop (%.0f deg so far)",
                        Math.toDegrees(Math.abs(error)));
                return;
            }
            // Close the loop and start the recording from here.
            headingClosedLoop = true;
            headingPidf.reset();
            clearTrace(selected);
            phaseTimer.reset();
        }

        arcade(0, 0, headingCorrection());

        recordHeadingTrace();

        if (phaseTimer.seconds() > 3.0) {
            arcade(0, 0, 0);
            message = String.format(Locale.US,
                    "Heading step done. Final error %.1f deg.", Math.toDegrees(error));
            setMode(Mode.IDLE);
        }
    }

    /** Reuses the pod trace buffer to graph robot heading against its target. */
    private void recordHeadingTrace() {
        double actualDeg = Math.toDegrees(headingRad);
        double targetDeg = Math.toDegrees(headingTargetRad);
        double delta = targetDeg - actualDeg;
        if (delta > 180) {
            targetDeg -= 360;
        } else if (delta < -180) {
            targetDeg += 360;
        }

        traceT[traceHead] = phaseTimer.seconds();
        traceTarget[traceHead] = targetDeg;
        traceActual[traceHead] = actualDeg;
        traceHead = (traceHead + 1) % TRACE_LEN;
        if (traceCount < TRACE_LEN) {
            traceCount++;
        }
    }

    private void startHeadingStep(double displaceDeg) {
        ensurePods();
        if (swerve == null) {
            message = "Cannot build drivetrain: " + podBuildError;
            return;
        }
        if (!headingOk) {
            message = "No heading from the Pinpoint; cannot tune heading.";
            return;
        }

        // setMode first, for the same transition cleanup as startAutoTune.
        setMode(Mode.HEADING);
        headingTargetRad = headingRad;
        headingOpenLoopPower = displaceDeg >= 0 ? 0.35 : -0.35;
        headingClosedLoop = false;
        clearTrace(selected);
        tracePod = -2;   // marks the trace as heading rather than a pod
        phaseTimer.reset();
        autoTuneTimer.reset();
        routineActive = true;
        message = "Heading step: displacing open-loop, then closing the loop.";
    }

    private void startPidStep(double targetDeg, boolean allPods) {
        pidAllPods = allPods;
        startPidStep(targetDeg);
    }

    private void startPidStep(double targetDeg) {
        ensurePods();
        if (pods == null) {
            message = "Cannot build pods: " + podBuildError;
            return;
        }
        // setMode first, for the same transition cleanup as startAutoTune.
        setMode(Mode.PID);
        pidTargetRad = Math.toRadians(targetDeg);
        clearTrace(selected);
        pidHolding = true;
        phaseTimer.reset();
        message = String.format(Locale.US, "Pod %d stepping to %.0f deg.", selected, targetDeg);
    }

    // ---------------------------------------------------------------- drive test

    private void runDriveMode() {
        ensurePods();
        if (swerve == null) {
            message = "Cannot build drivetrain: " + podBuildError;
            setMode(Mode.IDLE);
            return;
        }

        // If the browser stops sending commands - tab closed, laptop asleep, WiFi drop - the last
        // command would otherwise latch and the robot would keep driving.
        boolean commanded = driveForward != 0 || driveStrafe != 0 || driveTurn != 0;
        if (commanded && System.currentTimeMillis() - lastDriveCmdMs > DRIVE_WATCHDOG_MS) {
            driveForward = 0;
            driveStrafe = 0;
            driveTurn = 0;
            // Zeroing the sticks is not enough under heading hold: the setpoint may still be up
            // to the lead cap ahead of the robot, and the heading PID would finish that rotation
            // with nobody at the controls. A watchdog trip means stop, so the setpoint comes back
            // to wherever the robot is.
            if (headingHold && headingOk) {
                headingTargetRad = headingRad;
                headingPidf.reset();
            }
            message = "Drive watchdog tripped - no command from the dashboard.";
        }

        // Heading source recovering mid-drive: the target still says whatever it said when the
        // sensor died, anywhere up to 180 degrees away, and resuming the PID against it would
        // spin the robot at full power with no stick input. Adopt the current heading instead.
        if (headingHold && headingOk && !headingWasOkInDrive) {
            headingTargetRad = headingRad;
            headingPidf.reset();
        }
        headingWasOkInDrive = headingOk;

        double turn = driveTurn;

        if (headingHold && headingOk) {
            // The stick sets a heading RATE; the controller holds whatever setpoint it leaves
            // behind. Releasing it therefore locks the current heading instead of coasting, and a
            // shove off-heading is corrected rather than accepted.
            double dt = Math.min(0.25, Math.max(1e-3, headingStickTimer.seconds()));
            headingStickTimer.reset();

            // Threshold matches the browser gamepad deadband (0.06): the robot must never read
            // "stick active" from an axis value the dashboard would have zeroed - a slightly
            // leaky pad would otherwise sweep the setpoint with nobody touching the stick.
            boolean stickActive = Math.abs(driveTurn) > 0.055;
            headingStickActive = stickActive;

            // Wrap-aware chassis rotation rate, for the stopped/settled gates.
            double rateDt = Math.min(0.25, Math.max(1e-3, headingRateTimer.seconds()));
            headingRateTimer.reset();
            if (!Double.isNaN(headingPrevForRate)) {
                headingRateRadS = Angle.normalizeSigned(
                        headingRad - headingPrevForRate) / rateDt;
            }
            headingPrevForRate = headingRad;

            boolean translating = Math.abs(driveForward) >= SWERVE_EPSILON
                    || Math.abs(driveStrafe) >= SWERVE_EPSILON;
            double headingAbsErr =
                    Angle.smallestDifference(headingRad, headingTargetRad);

            if (stickActive) {
                // Driver input overrides any pending rotate-to-target.
                headingGotoActive = false;
            }
            boolean demand = stickActive || translating || headingGotoActive;

            if (demand) {
                if (headingPhase != HeadingHoldPhase.ACTIVE) {
                    headingPhase = HeadingHoldPhase.ACTIVE;
                    if (!headingGotoActive) {
                        // Re-adopt reality on entry. The resting target may be stale (the robot
                        // can be shoved while X-locked and nothing corrects it, by design), and
                        // acting on a stale setpoint snaps the robot somewhere nobody asked.
                        headingTargetRad = headingRad;
                        headingPidf.reset();
                    }
                }

                if (stickActive) {
                    headingTargetRad = Angle.normalize(
                            headingTargetRad + driveTurn * HEADING_STICK_RATE * dt);

                    // Never let the setpoint lead further than the controller can chase. Past
                    // 180 the error wraps and getTurnDirection flips, which reverses the robot
                    // mid-turn. The lead cap is also what makes the sweep rate battery-proof:
                    // the setpoint runs ahead until the cap, the output saturates, and the
                    // robot turns at whatever its true maximum is today.
                    double lead = Angle.turnDirection(headingRad, headingTargetRad)
                            * Angle.smallestDifference(headingRad, headingTargetRad);
                    if (lead > HEADING_MAX_LEAD) {
                        headingTargetRad = Angle.normalize(headingRad + HEADING_MAX_LEAD);
                    } else if (lead < -HEADING_MAX_LEAD) {
                        headingTargetRad = Angle.normalize(headingRad - HEADING_MAX_LEAD);
                    }
                }

                turn = headingCorrection();
                if (!stickActive) {
                    // The soft heading lock. Sub-epsilon corrections die inside arcadeDrive
                    // (|rotation| < 0.05 is treated as zero), so the hold has no small-signal
                    // authority at all - and every minimum-magnitude floor tried against that
                    // deadzone turned bang-bang: a constant translation-scaled floor relayed at
                    // 3.5 Hz / ±0.135 turn (10 deg of fishtail at 0.3 power), and rate-gating
                    // it only slowed the relay - each kick is a fixed yaw impulse far above
                    // what a degree of error needs, so overshoot re-arms it every swing. The
                    // deadzone wants a BYPASS, not a floor: shift the PID output past epsilon
                    // so authority is continuous from zero and the kD damping survives to the
                    // output. Engage past 1.2 deg with release below 0.5 (hysteresis), command
                    // genuinely zero rotation inside the deadband, and skip the epsilon shift
                    // while the chassis is already rotating fast - a moving chassis is past
                    // stiction and the raw PID owns the approach.
                    double dir = Angle.turnDirection(headingRad, headingTargetRad);
                    if (headingAbsErr > HEADING_TRIM_ENGAGE_RAD) {
                        headingTrimEngaged = true;
                    } else if (headingAbsErr < HEADING_TRIM_RELEASE_RAD) {
                        headingTrimEngaged = false;
                    }
                    if (headingTrimEngaged) {
                        // The bypass is for SPEED. Turn deflects pod demands by roughly
                        // atan(turn/translation), so at crawl the same 0.06-0.13 assisted
                        // correction that is invisible at 0.6 power swings every pod 30-40 deg
                        // - measured at 0.16 forward: quiet drift to the engage threshold,
                        // then a visible all-pod wobble burst every few seconds while the
                        // correction relayed itself back inside the deadband. Below the gate
                        // the raw damped PID runs instead: arcadeDrive's own 0.05 epsilon
                        // means corrections fire only past ~2.4 deg and at the gentlest
                        // magnitude the mixer can deliver.
                        double transMag = Math.hypot(driveForward, driveStrafe);
                        if (transMag >= HEADING_TRIM_MIN_TRANS
                                && Math.abs(headingRateRadS) < HEADING_TRIM_RATE_GATE_RAD_S) {
                            double assisted = Math.abs(turn) + HEADING_EPS_BYPASS;
                            if (assisted > HEADING_TRIM_MAX) {
                                assisted = Math.max(HEADING_TRIM_MAX, Math.abs(turn));
                            }
                            turn = assisted * (turn != 0 ? Math.signum(turn) : dir);
                        }
                    } else if (translating) {
                        // At speed the deadband stays: the bypass engages at 1.2 deg and the
                        // proven behavior there should not change. At crawl the raw damped PID
                        // runs CONTINUOUSLY instead - the mixer's lowered rotation epsilon
                        // (0.015) means its fine outputs actually reach the pods now, with
                        // deflections of a few degrees at most, so there is no deadband for
                        // the heading to wander in and nothing visible when it corrects.
                        if (Math.hypot(driveForward, driveStrafe) >= HEADING_TRIM_MIN_TRANS) {
                            turn = 0;
                        }
                    } else if (Math.abs(turn) < HEADING_MIN_ENGAGED_TURN) {
                        // Stationary goto tail keeps its original epsilon floor.
                        turn = HEADING_MIN_ENGAGED_TURN * (turn != 0 ? Math.signum(turn) : dir);
                    }
                }
                if (headingGotoActive && headingAbsErr < HEADING_EXIT_RAD
                        && Math.abs(headingRateRadS) < HEADING_EXIT_RATE_RAD_S) {
                    headingGotoActive = false;
                }
            } else {
                // Zero input. The release sequence the driver actually wants: stop correcting
                // immediately (chasing a latched target while the chassis still carries
                // momentum rubber-bands it), wait for the IMU to say rotation has genuinely
                // stopped, snap the setpoint to wherever physics parked us, and let the X-lock
                // - whose engage delay has been counting since input went quiet - own the hold.
                // In RESTING the heading PIDF produces nothing, ever: the X is not overridden.
                if (headingPhase == HeadingHoldPhase.ACTIVE) {
                    headingPhase = HeadingHoldPhase.STOPPING;
                    headingStopTimer.reset();
                }
                if (headingPhase == HeadingHoldPhase.STOPPING
                        && (Math.abs(headingRateRadS) < HEADING_STOPPED_RATE_RAD_S
                                || headingStopTimer.seconds() > HEADING_STOP_TIMEOUT_S)) {
                    headingTargetRad = headingRad;
                    headingPidf.reset();
                    headingPhase = HeadingHoldPhase.RESTING;
                }
                turn = 0;
            }

            // If we are commanding real rotation but the measured heading has not moved at all,
            // the sensor is dead and this loop would happily spin the robot at full power.
            if (Math.abs(turn) > 0.15) {
                if (Math.abs(headingRad - headingLastSeen) < 1e-6) {
                    headingStuckSeconds += dt;
                } else {
                    headingStuckSeconds = 0;
                }
                if (headingStuckSeconds > HEADING_STUCK_LIMIT_S) {
                    headingHold = false;
                    podsDirty = true;
                    driveForward = 0;
                    driveStrafe = 0;
                    driveTurn = 0;
                    headingStuckSeconds = 0;
                    message = "Heading has not changed while turning - sensor looks dead. "
                            + "Heading hold disabled; recalibrate the Pinpoint.";
                    appliedTurn = 0;
                    arcade(0, 0, 0);
                    return;
                }
            } else {
                headingStuckSeconds = 0;
            }
            headingLastSeen = headingRad;
        }

        // Field-oriented translation, rotated by LIVE heading here rather than at the 10-17 Hz
        // command rate: between stick updates a spinning robot rotates several degrees, and a
        // stale transform would drag the translation direction around with it.
        double effForward = driveForward;
        double effStrafe = driveStrafe;
        if (driveFieldOriented) {
            if (headingOk) {
                double rel = headingRad - focRefRad;
                double cos = Math.cos(rel);
                double sin = Math.sin(rel);
                effForward = driveForward * cos + driveStrafe * sin;
                effStrafe = -driveForward * sin + driveStrafe * cos;
            } else {
                // Guessing a frame with a dead heading sensor moves the robot somewhere nobody
                // asked. Stop translating; the driver can toggle back to robot frame.
                effForward = 0;
                effStrafe = 0;
                message = "Field-oriented drive lost heading - translation stopped.";
            }
        }

        // Hard limit: the saved box is enforced on every translation command, whoever sent it.
        double[] fenced = applyBoxLimit(effForward, effStrafe);
        appliedTurn = turn;
        appliedForward = fenced[0];
        appliedStrafe = fenced[1];
        arcade(fenced[0], fenced[1], turn);

        if (headingHold && headingOk) {
            // Heading is the interesting signal while holding, and it is what the circle test and
            // the rotation tests are judged on.
            recordHeadingTrace();
        } else if (!Double.isNaN(targetTheta[selected])) {
            // computeTargets() runs at publish time, so the value here is from the previous loop -
            // close enough for a graph, and it keeps arcadeDrive's kinematics as the single source.
            recordTrace(selected, targetTheta[selected]);
        }
    }

    // ---------------------------------------------------------------- pedro follower bench

    /**
     * Which pods the follower bench drives.
     *
     * <p>{@code shipped}: {@link Constants#createFollower} as it stands in the installed build -
     * the acceptance test, because it is byte-for-byte what an auto runs. {@code live}: the same
     * Follower, Foresight config and shipped SwerveConfig, but the four pods built from this
     * tool's live calibration ({@link PodCal#toSwervePod}) - so pod gains, zeros and ranges
     * tuned here are exercised by the REAL path follower before anyone exports or deploys them.
     * The tool's own drive and step tests run a different control law above the pods (its own
     * mixer config, its own heading hold); only the follower validates what a match does.
     */
    private boolean pedroLivePods;

    /** The follower's pods, by servo number, so the recorder can log them while following. */
    private SwervePod[] pedroPods;

    private void buildPedro(boolean livePods) {
        if (pedro != null) {
            pedroBreak();
            pedro = null;
        }
        pedroPods = new SwervePod[POD_COUNT];
        if (livePods) {
            for (int i = 0; i < POD_COUNT; i++) {
                pedroPods[i] = cals[i].toSwervePod(hardwareMap);
            }
            // Same pod order as Constants.createSwerve: leftFront ss2, rightFront ss1,
            // leftBack ss3, rightBack ss0.
            pedro = new com.pedropathing.follower.Follower(Constants.createLocalizer(hardwareMap),
                    new Swerve(hardwareMap, Constants.swerveConfig, pedroPods[2], pedroPods[1],
                            pedroPods[3], pedroPods[0]),
                    new com.pedropathing.algorithm.Foresight(Constants.foresightConfig));
        } else {
            pedro = Constants.createFollower(hardwareMap);
            System.arraycopy(Constants.builtPods, 0, pedroPods, 0, POD_COUNT);
        }
        pedroLivePods = livePods;
        pedro.setPose(new com.pedropathing.math.Pose(poseXIn, poseYIn, headingRad));
        pedro.update();
    }

    /**
     * Everything that rewrites the Pinpoint frame (setPose, resetImu, recalibrateImu, odoConfig)
     * goes through here first. allStop() alone left FOLLOW running: runFollowMode kept calling
     * pedro.update() toward a target in the old frame, with the box already cleared. IDLE through
     * setMode breaks the follower, and the follower is dropped because it carries the old frame -
     * the next pedroStart builds a fresh one, the same as fieldFence does.
     */
    private void stopForFrameChange() {
        pidHolding = false;
        setMode(Mode.IDLE);
        if (pedro != null) {
            pedroBreak();
            pedro = null;
        }
    }

    /**
     * 2.1.2's breakFollowing(). v3's Follower.stop() only changes mode - the drivetrain hears
     * about it on the next update(), which runFollowMode stops calling the moment it leaves
     * FOLLOW - so the drivetrain is stopped here directly to release the pods now.
     */
    private void pedroBreak() {
        pedro.stop();
        pedro.drivetrain.stop();
        restorePedroSpeedCap();
    }

    /**
     * The bench's "power" argument. v3 has no follower max power; the closest control is
     * Foresight's maxPathSpeed, a cap on target path SPEED as a fraction of the max achievable
     * velocity. It lives on the shared static config, so it is put back whenever the follower is
     * broken off or the OpMode stops - otherwise a bench cap would carry into the next auto.
     */
    private double pedroSavedMaxPathSpeed = Double.NaN;

    private void setPedroSpeedCap(double fraction) {
        ForesightConfig fc = Constants.foresightConfig;
        if (Double.isNaN(pedroSavedMaxPathSpeed)) {
            pedroSavedMaxPathSpeed = fc.maxPathSpeed.get();
        }
        fc.maxPathSpeed.set(fraction);
    }

    private void restorePedroSpeedCap() {
        if (!Double.isNaN(pedroSavedMaxPathSpeed)) {
            Constants.foresightConfig.maxPathSpeed.set(pedroSavedMaxPathSpeed);
            pedroSavedMaxPathSpeed = Double.NaN;
        }
    }

    /** Distance to the closest point on the path, when the algorithm is Foresight. */
    private double pedroTranslationalError() {
        com.pedropathing.algorithm.Algorithm a = pedro.algorithm();
        return a instanceof com.pedropathing.algorithm.Foresight
                ? ((com.pedropathing.algorithm.Foresight) a).translationalError()
                : Double.NaN;
    }

    private void runFollowMode() {
        if (pedro == null) {
            setMode(Mode.IDLE);
            return;
        }
        // The fence, enforced on the follower too: the follower drives its own pods and never
        // passes through applyBoxLimit. Command-time validation (pedroPointOk) is the primary
        // defence; this is the backstop, and it BRAKES rather than releasing - breakFollowing
        // alone left FLOAT motors coasting several inches past the wall on 2026-08-14. On a
        // breach the follower is redirected to hold a point pulled back inside the box, which
        // actively arrests the momentum, and only once that has had a second to work does the
        // mode drop to IDLE.
        if (!poseOk && boxValid) {
            pedroBreak();
            allStop();
            setMode(Mode.IDLE);
            message = "Pose unreadable while following - broken off and stopped.";
            return;
        }
        double[] cb = boxValid ? centreBounds(headingRad) : null;
        if (boxValid && (poseXIn < cb[0] || poseXIn > cb[2]
                || poseYIn < cb[1] || poseYIn > cb[3]
                || keepOutClearance(poseXIn, poseYIn) < 0)) {
            if (!pedroBreach) {
                pedroBreach = true;
                pedroBreachTimer.reset();
                double hx = Math.max(cb[0] + PEDRO_TARGET_MARGIN_IN,
                        Math.min(cb[2] - PEDRO_TARGET_MARGIN_IN, poseXIn));
                double hy = Math.max(cb[1] + PEDRO_TARGET_MARGIN_IN,
                        Math.min(cb[3] - PEDRO_TARGET_MARGIN_IN, poseYIn));
                if (!keepOuts.isEmpty()) {
                    double[] out = FenceGeometry.pushOut(keepOuts, hx, hy,
                            fenceRobotRadius() + PEDRO_TARGET_MARGIN_IN);
                    hx = out[0];
                    hy = out[1];
                }
                pedro.hold(new com.pedropathing.math.Pose(hx, hy, headingRad));
                pedroJob = "BREACH-recovery";
                message = "Follower hit the fence - braking back inside.";
            } else if (pedroBreachTimer.seconds() > 1.5) {
                pedroBreak();
                allStop();
                setMode(Mode.IDLE);
                message = "Fence breach handled; follower stopped.";
                return;
            }
        } else if (pedroBreach && pedroBreachTimer.seconds() > 1.0) {
            // Back inside and settled: stop cleanly rather than resuming the old job.
            pedroBreach = false;
            pedroBreak();
            setMode(Mode.IDLE);
            message = "Recovered inside the box; follower stopped.";
            return;
        }
        pedro.update();
    }

    // ---------------------------------------------------------------- foresight tuner

    /**
     * One FORESIGHT tick. The fence check is the bring-up's own pose against centreBounds, every loop, before
     * the stage acts: outside it, the stage is aborted and the follower's drivetrain stopped (BRAKE motors).
     * The stage itself plans its stops short of the fence - a coasting robot cannot be clamped.
     */
    private void runForesightMode() {
        if (pedro == null) {
            setMode(Mode.IDLE);
            return;
        }
        pedro.localizer.update();
        if (!poseOk || !boxValid) {
            fTuner.abort(!poseOk ? "pose unreadable" : "box lost");
            setMode(Mode.IDLE);
            message = fTuner.message();
            return;
        }
        double[] cb = centreBounds(headingRad);
        if (poseXIn < cb[0] || poseXIn > cb[2] || poseYIn < cb[1] || poseYIn > cb[3]
                || keepOutClearance(poseXIn, poseYIn) < 0) {
            fTuner.abort("left the fence at (" + fmt(poseXIn) + ", " + fmt(poseYIn) + ")");
            setMode(Mode.IDLE);
            message = fTuner.message();
            return;
        }
        fTuner.update(foresightIo);
        message = fTuner.message();
        if (!fTuner.running()) {
            setMode(Mode.IDLE);
            message = fTuner.message();
        }
    }

    /** Inches the robot centre can move from the current pose along (fx, fy) before the fence. */
    private double fenceRoom(double fx, double fy) {
        double[] cb = centreBounds(headingRad);
        double room = Double.POSITIVE_INFINITY;
        if (fx > 1e-9) room = Math.min(room, (cb[2] - poseXIn) / fx);
        if (fx < -1e-9) room = Math.min(room, (cb[0] - poseXIn) / fx);
        if (fy > 1e-9) room = Math.min(room, (cb[3] - poseYIn) / fy);
        if (fy < -1e-9) room = Math.min(room, (cb[1] - poseYIn) / fy);
        room = Math.max(0, room);
        if (!keepOuts.isEmpty()) {
            for (double d = 0; d < room; d += 0.5) {
                if (keepOutClearance(poseXIn + fx * d, poseYIn + fy * d) < 0) {
                    return d;
                }
            }
        }
        return room;
    }

    private final ForesightBoxTuner.Io foresightIo = new ForesightBoxTuner.Io() {
        @Override
        public void drive(com.pedropathing.drivetrain.DrivePowers powers) {
            pedro.drivetrain.drive(powers, false);
        }

        @Override
        public void coast() {
            pedro.drivetrain.stop(false);
        }

        @Override
        public com.pedropathing.math.Pose pose() {
            return pedro.localizer.pose();
        }

        @Override
        public double forwardVel() {
            return pedro.localizer.twist().vx;
        }

        @Override
        public double strafeVel() {
            return pedro.localizer.twist().vy;
        }

        @Override
        public double omega() {
            return pedro.localizer.twist().omega;
        }

        @Override
        public double room(double fx, double fy) {
            return fenceRoom(fx, fy);
        }

        @Override
        public double headingKpDefault() {
            return Constants.headingKP;
        }

        @Override
        public double headingBrakeLinearDefault() {
            return Constants.headingBrakeLinear;
        }

        @Override
        public double headingBrakeQuadraticDefault() {
            return Constants.headingBrakeQuadratic;
        }
    };

    // ---------------------------------------------------------------- zeroing

    /** Captures the current raw angle of every pod as its forward reference. */
    private void captureZeros(boolean allPods) {
        int from = allPods ? 0 : selected;
        int to = allPods ? POD_COUNT : selected + 1;
        List<String> failed = new ArrayList<>();
        for (int i = from; i < to; i++) {
            if (Double.isNaN(volts[i])) {
                failed.add(String.valueOf(i));
                continue;
            }
            cals[i].angleOffsetRad = cals[i].rawAngleRad(volts[i]);
        }
        podsDirty = true;
        saveCalibration();
        // The success line must not paper over a failure: it used to overwrite the "encoder not
        // readable" message, so zeroAll reported success while a dead pod silently kept its old
        // zero - the kind of thing that points a wheel the wrong way with no warning anywhere.
        if (!failed.isEmpty()) {
            message = "Zero NOT captured for pod(s) " + String.join(", ", failed)
                    + " - encoder unreadable. Their old zeros are still in effect.";
        } else {
            message = allPods
                    ? "Captured forward zero for all pods."
                    : "Captured forward zero for pod " + selected + ".";
        }
    }

    // ---------------------------------------------------------------- commands

    private void drainCommands() {
        Map<String, String> cmd;
        while ((cmd = SwerveBench.INSTANCE.poll()) != null) {
            try {
                handleCommand(cmd);
            } catch (RuntimeException e) {
                // One malformed parameter (pedroChain pts "1,2;3", a head spec "param:10", an
                // export after a NaN gain) used to throw out of loop() and kill the OpMode - and
                // the HTTP server's last snapshot with it. A command that throws has done an
                // unknown fraction of its work, so everything stops before it is reported.
                try {
                    pidHolding = false;
                    setMode(Mode.IDLE);
                } catch (RuntimeException stopFailure) {
                    allStop();
                    mode = Mode.IDLE;
                    pedro = null;
                }
                message = "ERROR in " + cmd.get("action") + ": " + e
                        + " - stopped (IDLE); fix the command and resend.";
            }
        }
    }

    private void handleCommand(Map<String, String> cmd) {
        String action = cmd.get("action");
        if (action == null) {
            return;
        }
        requireFiniteArgs(cmd);
        int pod = intArg(cmd, "pod", selected);
        if (pod >= 0 && pod < POD_COUNT) {
            selected = pod;
        }

        if (!started && isMotionCommand(action)) {
            message = "Press START on the driver station before running \"" + action + "\".";
            return;
        }

        switch (action) {
            case "select":
                // The selection was already applied from the "pod" parameter above; this exists so
                // the dashboard can change pods without triggering anything else.
                break;
            case "stop":
                allStop();
                pidHolding = false;
                setMode(Mode.IDLE);
                message = "Stopped.";
                break;
            case "wireScan":
                setMode(Mode.WIRE_SCAN);
                break;
            case "sweep":
                setMode(Mode.ENC_SWEEP);
                break;
            case "pulseMotor":
                setMode(Mode.MOTOR_PULSE);
                break;
            case "spinServo":
                beginJog(SCAN_SERVO_POWER, SPIN_SECONDS,
                        "Spinning " + cals[selected].servoName + " - watch it from above.");
                break;
            case "nudge": {
                double dir = doubleArg(cmd, "dir", 1);
                double sign = dir < 0 ? -1 : 1;
                beginJog(NUDGE_POWER * sign, NUDGE_SECONDS, "Nudging pod " + selected + ".");
                break;
            }
            case "zeroPod":
                captureZeros(false);
                break;
            case "zeroAll":
                captureZeros(true);
                break;
            case "setEncoderReversed": {
                // No "value" means toggle. The dashboard builds its controls once and reuses them,
                // so a handler cannot bake in the opposite of a state that keeps changing.
                boolean value = cmd.get("value") == null
                        ? !cals[selected].encoderReversed
                        : Boolean.parseBoolean(cmd.get("value"));
                if ("all".equals(cmd.get("scope"))) {
                    for (PodCal c : cals) {
                        c.encoderReversed = value;
                    }
                    message = "encoderReversed set to " + value + " on all pods.";
                } else {
                    cals[selected].encoderReversed = value;
                    message = "Pod " + selected + " encoderReversed = " + value;
                }
                podsDirty = true;
                saveCalibration();
                break;
            }
            case "setDriveReversed":
                cals[selected].setDriveReversed(cmd.get("value") == null
                        ? !cals[selected].driveReversed()
                        : Boolean.parseBoolean(cmd.get("value")));
                if (motors[selected] != null) {
                    motors[selected].setDirection(cals[selected].driveDirection);
                }
                podsDirty = true;
                saveCalibration();
                message = "Pod " + selected + " drive direction "
                        + cals[selected].driveDirection.name();
                break;
            case "setServoReversed":
                cals[selected].setServoReversed(cmd.get("value") == null
                        ? !cals[selected].servoReversed()
                        : Boolean.parseBoolean(cmd.get("value")));
                if (servos[selected] != null) {
                    servos[selected].setDirection(cals[selected].servoDirection);
                }
                podsDirty = true;
                saveCalibration();
                break;
            case "setLabel": {
                String corner = cmd.get("value");
                if (corner == null) {
                    break;
                }

                // Swap with whichever pod already holds this corner, so the four pods always
                // occupy four distinct corners. Two pods sharing one would silently break the
                // rotation kinematics.
                PodCal previousOwner = findByLabel(corner);
                String vacated = cals[selected].label;

                if (!cals[selected].setCorner(corner)) {
                    message = "Unknown corner \"" + corner + "\". Use LF, RF, LB or RB.";
                    break;
                }

                if (previousOwner != null && previousOwner != cals[selected]) {
                    previousOwner.setCorner(vacated);
                    message = "Pod " + selected + " is now " + corner
                            + "; pod " + previousOwner.index + " took " + vacated + ".";
                } else {
                    message = "Pod " + selected + " is now " + corner + ".";
                }

                podsDirty = true;
                saveCalibration();
                break;
            }
            case "setFastCurrent": {
                fastCurrent = boolArg(cmd, "value", !fastCurrent);
                message = fastCurrent
                        ? "Servo rail current now read EVERY LOOP. This costs loop rate - it is a "
                                + "measurement mode, not a monitoring mode. Turn it off after."
                        : "Servo rail current back on the idle-sensor path.";
                break;
            }

            case "setPublishHz": {
                // Clamped, not validated-and-rejected: 0 would stall the dashboard into looking
                // like a dead robot, and anything above the loop rate just publishes every loop.
                double hz = doubleArg(cmd, "value", 1 / PUBLISH_INTERVAL_DEFAULT_S);
                hz = Math.max(1.0, Math.min(200.0, hz));
                publishIntervalS = 1.0 / hz;
                message = String.format(Locale.US,
                        "Publish rate %.1f Hz. Lower frees control bandwidth; the recorder is "
                                + "unaffected because it samples every loop.", hz);
                break;
            }

            case "setPose": {
                // Writes a known pose into the Pinpoint. The recovery path for a frame that was
                // re-origined by something else: if the robot has not physically moved since,
                // restoring the pose it held restores the frame exactly, and a box marked in
                // that frame becomes valid again without re-marking corners (boxSet with the
                // coordinates the reload's discard message printed).
                //
                // It does NOT re-validate the box on its own - the operator has to confirm the
                // fence lines up with the mat, because "the robot has not moved" is a claim only
                // a human can make.
                if (pinpoint == null) {
                    message = "No pinpoint device.";
                    break;
                }
                double px = doubleArg(cmd, "x", Double.NaN);
                double py = doubleArg(cmd, "y", Double.NaN);
                double ph = doubleArg(cmd, "headingDeg", Double.NaN);
                if (Double.isNaN(px) || Double.isNaN(py) || Double.isNaN(ph)) {
                    message = "setPose needs x, y and headingDeg.";
                    break;
                }
                stopForFrameChange();
                pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, px, py,
                        AngleUnit.DEGREES, ph));
                pinpoint.update();
                // The pose just moved on purpose: an armed box keeps its meaning (the operator
                // vouches for it), so its witness moves with the pose rather than tripping the
                // next reload's frame check.
                refreshBoxWitness();
                message = String.format(Locale.US,
                        "Pose set to (%.4f, %.4f) at %.4f deg. Confirm the box lines up with the "
                                + "mat before driving.", px, py, ph);
                break;
            }

            case "setMixer": {
                // The Task 2 continuity fixes, switchable at runtime so each can be A/B'd
                // interleaved inside one session instead of across a redeploy. Absent
                // parameters are left alone.
                if (cmd.get("taper") != null) {
                    com.pedropathing.revhub.drivetrains.Swerve.setEpsilonTaper(
                            Boolean.parseBoolean(cmd.get("taper")));
                }
                if (cmd.get("slew") != null) {
                    com.pedropathing.revhub.drivetrains.Swerve.setDemandSlewDegPerSec(
                            doubleArg(cmd, "slew", 300));
                }
                if (cmd.get("rollout") != null) {
                    com.pedropathing.revhub.drivetrains.Swerve.setSmallTurnRolloutS(
                            doubleArg(cmd, "rollout", 0.3));
                }
                message = String.format(Locale.US,
                        "Mixer: epsilon taper %s, demand slew %.0f deg/s, small-turn roll-out %.2f s.",
                        com.pedropathing.revhub.drivetrains.Swerve.getEpsilonTaper() ? "on" : "off",
                        com.pedropathing.revhub.drivetrains.Swerve.getDemandSlewDegPerSec(),
                        com.pedropathing.revhub.drivetrains.Swerve.getSmallTurnRolloutS());
                break;
            }

            case "pedroChain": {
                // A whole PathChain from the host: "x,y;x,y;x,y;x,y" per cubic segment, segments
                // separated by "|", one heading spec per segment. Designed and validated
                // host-side (pathdesign.py), bounds-checked here - a Bezier never leaves the
                // convex hull of its control points, so checking every control point bounds the
                // whole curve.
                if (mode != Mode.FOLLOW || pedro == null) {
                    message = "pedroStart first.";
                    break;
                }
                String pts = cmd.get("pts");
                if (pts == null || pts.isEmpty()) {
                    message = "pedroChain needs pts.";
                    break;
                }
                String[] segTexts = pts.split("\\|");
                String[] headTexts = (cmd.get("head") == null ? "" : cmd.get("head")).split("\\|");
                List<com.pedropathing.math.Pose[]> segs = new ArrayList<>();
                boolean bad = false;
                for (String segText : segTexts) {
                    String[] ptTexts = segText.split(";");
                    // 2 points is a line; more is a Bezier of that degree - the visualizer's
                    // paths are usually degree 3-4, and pathdesign.py's are cubic.
                    if (ptTexts.length < 2 || ptTexts.length > 12) {
                        message = "pedroChain: each segment needs 2-12 control points, got "
                                + ptTexts.length;
                        bad = true;
                        break;
                    }
                    com.pedropathing.math.Pose[] cps =
                            new com.pedropathing.math.Pose[ptTexts.length];
                    for (int i = 0; i < ptTexts.length; i++) {
                        String[] xy = ptTexts[i].split(",");
                        double px = finiteDouble(xy[0]);
                        double py = finiteDouble(xy[1]);
                        if (!pedroPointOk(px, py)) {
                            message = String.format(Locale.US,
                                    "REFUSED: control point (%.1f, %.1f) leaves the box minus "
                                            + "%.0f in margin. Nothing moved.",
                                    px, py, PEDRO_TARGET_MARGIN_IN);
                            bad = true;
                            break;
                        }
                        cps[i] = new com.pedropathing.math.Pose(px, py);
                    }
                    if (bad) {
                        break;
                    }
                    double[][] segPts = new double[cps.length][];
                    for (int i = 0; i < cps.length; i++) {
                        segPts[i] = new double[] {cps[i].x(), cps[i].y()};
                    }
                    String segRefusal = pedroPathRefusal(segPts);
                    if (segRefusal != null) {
                        message = "Segment " + (segs.size() + 1) + ": " + segRefusal;
                        bad = true;
                        break;
                    }
                    segs.add(cps);
                }
                if (bad) {
                    break;
                }
                com.pedropathing.paths.Path[] chain =
                        new com.pedropathing.paths.Path[segs.size()];
                for (int i = 0; i < segs.size(); i++) {
                    com.pedropathing.math.Pose[] c = segs.get(i);
                    com.pedropathing.paths.Path seg = c.length == 2
                            ? com.pedropathing.api.Paths.line(c[0], c[1])
                            : com.pedropathing.api.Paths.curve(c);
                    String head = i < headTexts.length ? headTexts[i] : "tangent";
                    if (head.startsWith("param")) {
                        // param:startDeg:endDeg:curve - the visualizer's own interpolation, on
                        // the curve parameter t shaped by t^curve, exactly as an exported auto's
                        // headingInterpolator does. Pedro's linear() interpolates on distance
                        // instead, so a visualizer path run with linear is a different path.
                        String[] parts = head.split(":");
                        final double h0 = Math.toRadians(finiteDouble(parts[1]));
                        double h1 = Math.toRadians(finiteDouble(parts[2]));
                        final double shape = Math.max(0.25, Math.min(4.0,
                                parts.length > 3 ? finiteDouble(parts[3]) : 1.0));
                        final double dH = Angle.normalizeSigned(h1 - h0);
                        seg = seg.heading((curve, t) -> Angle.normalizeSigned(
                                h0 + dH * Math.pow(Math.max(0, Math.min(1, t)), shape)));
                    } else if (head.startsWith("reverseTangent")) {
                        seg = seg.heading(com.pedropathing.paths.interpolator.Interpolator.tangent.reverse());
                    } else if (head.startsWith("constant")) {
                        double deg = head.contains(":")
                                ? finiteDouble(head.substring(head.indexOf(':') + 1))
                                : Math.toDegrees(headingRad);
                        seg = seg.constant(Math.toRadians(deg));
                    } else if (head.startsWith("linear")) {
                        String[] parts = head.split(":");
                        seg = seg.linear(
                                Math.toRadians(finiteDouble(parts[1])),
                                Math.toRadians(finiteDouble(parts[2])));
                    } else {
                        seg = seg.tangent();
                    }
                    chain[i] = seg;
                }
                double chainPower = doubleArg(cmd, "power", 0.5);
                setPedroSpeedCap(Math.max(0.05, Math.min(1.0, chainPower)));
                pedro.follow(com.pedropathing.api.Paths.path(chain));
                pedroJob = "chain";
                message = String.format(Locale.US, "Following a %d-segment chain at %.2f.",
                        segs.size(), chainPower);
                break;
            }

            case "setFastFmt": {
                // Switchable at runtime so the two number formatters can be interleaved inside
                // one session - a redeploy between arms would confound the comparison with a
                // fresh JIT, a different battery state and a different trace length.
                fastFmt = cmd.get("value") == null
                        ? !fastFmt
                        : Boolean.parseBoolean(cmd.get("value"));
                message = "Publish formatter: " + (fastFmt ? "hand-rolled" : "String.format");
                break;
            }

            case "setPidf": {
                // Robot-wide schedule tuning rides along: floor (fraction), velstart (deg/s),
                // gate (deg). NaN (absent) leaves a value untouched.
                com.pedropathing.revhub.drivetrains.CoaxialPod.setScheduleTuning(
                        doubleArg(cmd, "floor", Double.NaN),
                        Math.toRadians(doubleArg(cmd, "velstart", Double.NaN)),
                        Math.toRadians(doubleArg(cmd, "gate", Double.NaN)));
                com.pedropathing.revhub.drivetrains.CoaxialPod.setScheduleRamp(
                        doubleArg(cmd, "ramp", Double.NaN));
                PodCal c = cals[selected];
                if ("all".equals(cmd.get("scope"))) {
                    for (PodCal each : cals) {
                        each.kP = doubleArg(cmd, "kp", each.kP);
                        each.kI = doubleArg(cmd, "ki", each.kI);
                        each.kD = doubleArg(cmd, "kd", each.kD);
                        each.kF = doubleArg(cmd, "kf", each.kF);
                        // Propagated with the rest: the output caching threshold is a control
                        // parameter, not a display preference, and leaving it out of scope=all
                        // meant a swept value silently applied to one pod out of four.
                        each.servoCaching = doubleArg(cmd, "cache", each.servoCaching);
                        each.servoSlewPerUpdate = doubleArg(cmd, "slew", each.servoSlewPerUpdate);
                        each.kS = doubleArg(cmd, "ks", each.kS);
                        each.kSBandDeg = doubleArg(cmd, "ksband", each.kSBandDeg);
                        each.kILimit = doubleArg(cmd, "kilimit", each.kILimit);
                        each.kIBandDeg = doubleArg(cmd, "kiband", each.kIBandDeg);
                        each.kIResetDeg = doubleArg(cmd, "kireset", each.kIResetDeg);
                        each.derivativeOnMeasurement = boolArg(cmd, "dom", each.derivativeOnMeasurement);
                        each.pulsed = boolArg(cmd, "pulsed", each.pulsed);
                        each.pulseBandDeg = doubleArg(cmd, "pband", each.pulseBandDeg);
                        each.pulseTolDeg = doubleArg(cmd, "ptol", each.pulseTolDeg);
                        each.pulsePower = doubleArg(cmd, "ppow", each.pulsePower);
                        each.pulseMs = doubleArg(cmd, "pms", each.pulseMs);
                        each.pulseCoastMs = doubleArg(cmd, "pcoast", each.pulseCoastMs);
                    }
                } else {
                    c.kP = doubleArg(cmd, "kp", c.kP);
                    c.kI = doubleArg(cmd, "ki", c.kI);
                    c.kD = doubleArg(cmd, "kd", c.kD);
                    c.kF = doubleArg(cmd, "kf", c.kF);
                    c.servoCaching = doubleArg(cmd, "cache", c.servoCaching);
                    c.servoSlewPerUpdate = doubleArg(cmd, "slew", c.servoSlewPerUpdate);
                    c.kS = doubleArg(cmd, "ks", c.kS);
                    c.kSBandDeg = doubleArg(cmd, "ksband", c.kSBandDeg);
                    c.kILimit = doubleArg(cmd, "kilimit", c.kILimit);
                    c.kIBandDeg = doubleArg(cmd, "kiband", c.kIBandDeg);
                    c.kIResetDeg = doubleArg(cmd, "kireset", c.kIResetDeg);
                    c.derivativeOnMeasurement = boolArg(cmd, "dom", c.derivativeOnMeasurement);
                    c.pulsed = boolArg(cmd, "pulsed", c.pulsed);
                    c.pulseBandDeg = doubleArg(cmd, "pband", c.pulseBandDeg);
                    c.pulseTolDeg = doubleArg(cmd, "ptol", c.pulseTolDeg);
                    c.pulsePower = doubleArg(cmd, "ppow", c.pulsePower);
                    c.pulseMs = doubleArg(cmd, "pms", c.pulseMs);
                    c.pulseCoastMs = doubleArg(cmd, "pcoast", c.pulseCoastMs);
                }
                podsDirty = true;
                saveCalibration();
                message = "PIDF updated.";
                break;
            }
            case "setRange":
                cals[selected].analogMin = doubleArg(cmd, "min", cals[selected].analogMin);
                cals[selected].analogMax = doubleArg(cmd, "max", cals[selected].analogMax);
                podsDirty = true;
                saveCalibration();
                break;
            case "setPositional": {
                if (!PodCal.POSITIONAL_ENABLED && boolArg(cmd, "value", true)) {
                    message = "Positional mode is shelved - see "
                            + "tools/swervetune/POSITIONAL_SHELVED.md. The drivetrain is "
                            + "continuous-rotation only; enabling this needs a source change to "
                            + "PodCal.POSITIONAL_ENABLED, not a command.";
                    break;
                }
                cals[selected].positional = boolArg(cmd, "value", !cals[selected].positional);
                cals[selected].rawDegAtPos0 = doubleArg(cmd, "raw0", cals[selected].rawDegAtPos0);
                cals[selected].rawDegAtPos1 = doubleArg(cmd, "raw1", cals[selected].rawDegAtPos1);
                cals[selected].posCalibrated = boolArg(cmd, "cal", cals[selected].posCalibrated);
                if (boolArg(cmd, "clearmarks", false)) {
                    cals[selected].posMarked0 = false;
                    cals[selected].posMarked1 = false;
                    cals[selected].posCalibrated = false;
                    // Drop the endpoints too. Leaving one real and one stale is what let a
                    // half-finished calibration present a 112 degree band as if it meant something.
                    cals[selected].rawDegAtPos0 = 0.0;
                    cals[selected].rawDegAtPos1 = 270.0;
                }
                calHomed = false;
                podsDirty = true;
                saveCalibration();
                message = "Pod " + selected + (cals[selected].positional
                        ? " is positional. Its port must be configured as Servo, not "
                          + "ContinuousRotationServo."
                        : " is back to continuous rotation.");
                break;
            }
            case "calPositional": {
                // Drives the servo to each end of its travel and records what the encoder reads
                // there. Two points is the whole calibration: shaft and pod are 1:1 and the
                // encoder is on that shaft, so between them the relationship is a straight line.
                if (posServos[selected] == null) {
                    message = "Pod " + selected + " is not on a Servo-configured port.";
                    break;
                }
                setMode(Mode.IDLE);
                posServos[selected].setPosition(doubleArg(cmd, "pos", 0.0));
                message = String.format(Locale.US,
                        "Pod %d driven to position %.3f. Wait for it to stop, read rawDeg, then "
                                + "send setPositional with raw0/raw1.",
                        selected, doubleArg(cmd, "pos", 0.0));
                break;
            }
            case "calHome": {
                // The one unavoidable uncontrolled move. Nothing can know where a position-mode
                // servo physically is until it has been commanded somewhere, so this commands
                // mid-travel - at most half the travel away from anywhere in the band, and Soft
                // Start bounds how fast. Everything after it walks from a known position.
                if (posServos[selected] == null) {
                    message = "Pod " + selected + " is not on a Servo-configured port.";
                    break;
                }
                allStop();
                double before = Math.toDegrees(cals[selected].rawAngleRad(volts[selected]));
                calPos = Utils.clamp(doubleArg(cmd, "pos", 0.5), 0.0, 1.0);
                posServos[selected].setPosition(calPos);
                calHomed = true;
                calLastRaw = before;
                calSteps = 0;
                calStalls = 0;
                phaseTimer.reset();
                message = String.format(Locale.US,
                        "Pod %d commanded to %.3f from encoder %.1f deg. FIRST COMMANDED MOVE - "
                                + "expect up to half the travel, Soft Start limited. Let it stop, "
                                + "check the encoder settled, then calGoto.", selected, calPos, before);
                break;
            }
            case "calGoto": {
                if (posServos[selected] == null) {
                    message = "Pod " + selected + " is not on a Servo-configured port.";
                    break;
                }
                if (!calHomed) {
                    message = "Run calHome first: without it the starting position is the "
                            + "controller's cached default, not where the pod is, and the first "
                            + "step would be an absolute jump.";
                    break;
                }
                // setMode, not a bare assignment: it owns the transition cleanup (scan-forced
                // servo directions, breaking the follower, aborting a live autotune).
                setMode(Mode.CAL_POS);
                // calPos is carried forward from calHome and each completed walk, so it tracks
                // where the servo actually is rather than what the controller last cached.
                // No ensurePods() here: see runCalPos.
                calTargetPos = Utils.clamp(doubleArg(cmd, "pos", 0.5), 0.0, 1.0);
                calStalls = 0;
                calSteps = 0;
                calLastRaw = Math.toDegrees(cals[selected].rawAngleRad(volts[selected]));
                phaseTimer.reset();
                routineActive = true;
                message = String.format(Locale.US,
                        "Walking pod %d from position %.3f to %.3f in %.3f steps, stopping on a "
                                + "stall.", selected, calPos, calTargetPos, CAL_STEP);
                break;
            }
            case "calMark": {
                if (posServos[selected] == null) {
                    message = "Pod " + selected + " is not on a Servo port.";
                    break;
                }
                double raw = Math.toDegrees(cals[selected].rawAngleRad(volts[selected]));
                PodCal mc = cals[selected];
                if (intArg(cmd, "which", 0) == 0) {
                    mc.rawDegAtPos0 = raw;
                    mc.posMarked0 = true;
                } else {
                    mc.rawDegAtPos1 = raw;
                    mc.posMarked1 = true;
                }
                // Both endpoints actually measured - not a span wide enough to look plausible.
                // One real endpoint against the other's stale default can span over 100 degrees
                // and would otherwise have declared the pod calibrated on a fiction.
                double span = Math.abs(mc.rawDegAtPos1 - mc.rawDegAtPos0);
                mc.posCalibrated = mc.posMarked0 && mc.posMarked1 && span > 100.0;
                podsDirty = true;
                saveCalibration();
                message = String.format(Locale.US,
                        "Marked endpoint %d at %.2f deg. Marked: pos0=%s pos1=%s. Span %.1f deg. "
                                + "calibrated=%s.",
                        intArg(cmd, "which", 0), raw, mc.posMarked0, mc.posMarked1, span,
                        mc.posCalibrated);
                break;
            }
            case "probeClamp": {
                // Deliberately asks for a position outside the clamp, in both directions, and
                // reports what was actually written. A clamp that has never been exercised is an
                // assumption, and this one stands between a command and a hard stop.
                if (pods == null || !(pods[selected] instanceof PositionalPod)) {
                    message = "Pod " + selected + " is not positional.";
                    break;
                }
                PositionalPod pp = (PositionalPod) pods[selected];
                double over = doubleArg(cmd, "over", 30.0);
                setMode(Mode.IDLE);
                double lo = pp.commandRawDegForTest(cals[selected].rawDegAtPos0
                        + (cals[selected].rawDegAtPos1 > cals[selected].rawDegAtPos0 ? -over : over));
                double hi = pp.commandRawDegForTest(cals[selected].rawDegAtPos1
                        + (cals[selected].rawDegAtPos1 > cals[selected].rawDegAtPos0 ? over : -over));
                message = String.format(Locale.US,
                        "Clamp probe: asked %.0f deg past each end, wrote positions %.4f and %.4f "
                                + "(both must be strictly inside 0 and 1).", over, lo, hi);
                break;
            }
            case "setPwmEnable":
                applyPwmEnable(selected, Boolean.parseBoolean(cmd.get("value")));
                break;
            case "setPwmRange": {
                double lower = doubleArg(cmd, "lower", 600);
                double upper = doubleArg(cmd, "upper", 2400);
                double frame = doubleArg(cmd, "frame", PwmControl.PwmRange.usFrameDefault);
                applyPwmRange(lower, upper, frame, "all".equals(cmd.get("scope")));
                break;
            }
            case "recStart":
                recorder.start(cmd.get("label"));
                message = "Recording run " + recorder.runId() + ".";
                break;
            case "recStop":
                recorder.stop();
                message = "Recording stopped: " + recorder.count() + " samples"
                        + (recorder.overflowed() ? " (BUFFER FULL - run truncated)." : ".");
                break;
            case "rawServo": {
                // Open-loop drive at a known power, which is how breakaway, deadband and max slew
                // rate get measured. Reuses the jog path so the same timeout protects it.
                double pow = Utils.clamp(doubleArg(cmd, "pow", 0), -1.0, 1.0);
                double sec = Utils.clamp(doubleArg(cmd, "sec", 0.5), 0.0, 5.0);
                beginJog(pow, sec, String.format(Locale.US,
                        "Pod %d open loop at %.3f for %.2fs.", selected, pow, sec));
                break;
            }
            case "pidStep":
                startPidStep(doubleArg(cmd, "deg", 90), false);
                break;
            case "pidStepAll":
                startPidStep(doubleArg(cmd, "deg", 90), true);
                break;
            case "autoTune":
                startAutoTune();
                break;
            case "headingStep":
                startHeadingStep(doubleArg(cmd, "deg", 90));
                break;
            case "setHeadingPidf":
                headingKp = doubleArg(cmd, "hkp", headingKp);
                headingKd = doubleArg(cmd, "hkd", headingKd);
                headingKf = doubleArg(cmd, "hkf", headingKf);
                message = String.format(Locale.US, "Heading PIDF: kP=%.4f kD=%.4f kF=%.4f",
                        headingKp, headingKd, headingKf);
                break;
            case "pidHold":
                pidHolding = false;
                allStop();
                message = "Pod released.";
                break;
            case "headingGoto": {
                // Commands a heading change of a chosen size and holds it, so rotations of very
                // different magnitudes can be compared with the same controller.
                if (!headingOk) {
                    message = "No heading from the Pinpoint.";
                    break;
                }
                headingTargetRad = Angle.normalize(
                        headingRad + Math.toRadians(doubleArg(cmd, "deg", 90)));
                headingPidf.reset();
                headingHold = true;
                // A goto is demand: it keeps the hold phase machine in ACTIVE (which would
                // otherwise rest at zero input and never execute the rotation) until the
                // target is reached, and it must not be re-adopted away on ACTIVE entry.
                headingGotoActive = true;
                headingPhase = HeadingHoldPhase.ACTIVE;
                headingStickActive = false;
                podsDirty = true;
                driveForward = 0;
                driveStrafe = 0;
                driveTurn = 0;
                lastDriveCmdMs = System.currentTimeMillis();
                clearTrace(selected);
                tracePod = -2;
                // setMode, not a bare assignment: it stops whatever routine was mid-flight and
                // restores scan-forced servo directions. It re-zeroes the drive inputs, which
                // this command already set to zero, so ordering is safe.
                setMode(Mode.DRIVE);
                phaseTimer.reset();
                headingStickTimer.reset();
                message = String.format(Locale.US, "Turning %.0f deg and holding.",
                        doubleArg(cmd, "deg", 90));
                break;
            }
            case "recalibrateImu":
                if (pinpoint == null) {
                    message = "No pinpoint device.";
                    break;
                }
                try {
                    stopForFrameChange();
                    pinpoint.recalibrateIMU();
                    headingStuckSeconds = 0;
                    message = "Pinpoint IMU recalibrating - keep the robot still for a moment.";
                } catch (RuntimeException e) {
                    message = "Recalibrate failed: " + e.getMessage();
                }
                break;
            case "resetImu":
                if (pinpoint == null) {
                    message = "No pinpoint device.";
                    break;
                }
                try {
                    stopForFrameChange();
                    pinpoint.resetPosAndIMU();
                    headingStuckSeconds = 0;
                    // The box lives in the pose frame that was just destroyed. A stale fence
                    // pointing at the wrong patch of floor is worse than no fence.
                    boxValid = false;
                    boxMarked0 = false;
                    if (BOX_FILE.exists()) {
                        BOX_FILE.delete();
                    }
                    message = "Pinpoint position and IMU reset - keep the robot still. The "
                            + "saved box was cleared with the frame it lived in; re-mark it.";
                } catch (RuntimeException e) {
                    message = "Reset failed: " + e.getMessage();
                }
                break;
            case "setHeadingHold":
                headingHold = cmd.get("value") == null
                        ? !headingHold
                        : Boolean.parseBoolean(cmd.get("value"));
                if (headingHold && headingOk) {
                    headingTargetRad = headingRad;
                    headingPidf.reset();
                }
                podsDirty = true;   // zero-power behaviour depends on this
                message = headingHold
                        ? "Right stick steers a heading setpoint; the robot holds it."
                        : "Right stick commands rotation power directly.";
                break;
            case "pedroStart": {
                if (!poseOk) {
                    message = "No pose - the follower needs the Pinpoint.";
                    break;
                }
                if (!boxValid) {
                    message = "No box - the follower bench refuses to run without a fence. "
                            + "Mark the box first.";
                    break;
                }
                String act = cmd.get("activate");
                if (act != null && !"all".equals(act)) {
                    // 2.1.2 could switch individual follower PIDFs off. Foresight has no such
                    // switch, so a single-term request is refused rather than silently run with
                    // every term on.
                    message = "activate=" + act + " is not available on Pedro 3 (Foresight has "
                            + "no per-term switch). Use activate=all.";
                    break;
                }
                act = "all";
                String podsArg = cmd.get("pods") == null ? "shipped" : cmd.get("pods");
                if (!"shipped".equals(podsArg) && !"live".equals(podsArg)) {
                    message = "pedroStart pods= must be shipped or live.";
                    break;
                }
                pedroBreach = false;
                try {
                    // Always a fresh follower: a live-pod build has to pick up every setPidf
                    // since the last one, and no controller state survives between runs.
                    buildPedro("live".equals(podsArg));
                } catch (RuntimeException e) {
                    pedro = null;
                    pedroPods = null;
                    message = "Follower build failed: " + e.getMessage();
                    break;
                }
                setMode(Mode.FOLLOW);
                pedro.hold(pedro.pose());
                pedroJob = "hold";
                message = "Pedro follower active: " + act + ", pods=" + podsArg
                        + (pedroLivePods ? " (this tool's live calibration)."
                                : " (Constants.java as installed).");
                break;
            }
            case "pedroLine": {
                if (mode != Mode.FOLLOW || pedro == null) {
                    message = "pedroStart first.";
                    break;
                }
                double dx = doubleArg(cmd, "dx", 20);
                double dy = doubleArg(cmd, "dy", 0);
                double power = doubleArg(cmd, "power", 0.5);
                com.pedropathing.math.Pose cur = pedro.pose();
                com.pedropathing.math.Pose tgt = new com.pedropathing.math.Pose(
                        cur.x() + dx, cur.y() + dy, cur.heading());
                if (!pedroPointOk(tgt.x(), tgt.y())) {
                    message = String.format(Locale.US,
                            "REFUSED: line target (%.1f, %.1f) is outside the fence minus %.0f "
                                    + "in margin. Nothing moved.",
                            tgt.x(), tgt.y(), PEDRO_TARGET_MARGIN_IN);
                    break;
                }
                String lineRefusal = pedroPathRefusal(
                        new double[][] {{cur.x(), cur.y()}, {tgt.x(), tgt.y()}});
                if (lineRefusal != null) {
                    message = lineRefusal;
                    break;
                }
                com.pedropathing.paths.Path p =
                        com.pedropathing.api.Paths.line(cur, tgt).constant(cur.heading());
                setPedroSpeedCap(Math.max(0.1, Math.min(1.0, power)));
                pedro.follow(p);
                pedroJob = "line";
                message = String.format(Locale.US, "Line %+.1f, %+.1f at %.2f.", dx, dy, power);
                break;
            }
            case "pedroHold": {
                if (mode != Mode.FOLLOW || pedro == null) {
                    message = "pedroStart first.";
                    break;
                }
                double dx = doubleArg(cmd, "dx", 0);
                double dy = doubleArg(cmd, "dy", 0);
                // dh: a heading change in degrees, held at the target point - Foresight's own
                // heading controller doing the turn, which is what validates headingKP.
                double dh = Math.toRadians(doubleArg(cmd, "dh", 0));
                com.pedropathing.math.Pose cur = pedro.pose();
                double hxT = cur.x() + dx;
                double hyT = cur.y() + dy;
                if (!pedroPointOk(hxT, hyT)) {
                    message = String.format(Locale.US,
                            "REFUSED: hold point (%.1f, %.1f) is outside the fence minus %.0f "
                                    + "in margin. Nothing moved.",
                            hxT, hyT, PEDRO_TARGET_MARGIN_IN);
                    break;
                }
                // A hold servoes straight at its point, so the robot sweeps the segment from
                // here to there - checked like a pedroLine, or a hold across a rail gets through.
                String holdRefusal = pedroPathRefusal(
                        new double[][] {{cur.x(), cur.y()}, {hxT, hyT}});
                if (holdRefusal != null) {
                    message = holdRefusal;
                    break;
                }
                pedro.hold(new com.pedropathing.math.Pose(hxT, hyT, cur.heading() + dh));
                pedroJob = "hold";
                message = String.format(Locale.US, "Holding %+.1f, %+.1f, %+.1f deg from here.",
                        dx, dy, Math.toDegrees(dh));
                break;
            }
            case "pedroCurve": {
                if (mode != Mode.FOLLOW || pedro == null) {
                    message = "pedroStart first.";
                    break;
                }
                // The CentripetalTuner's quadratic: forward |d|, then |d| to the left (or right
                // for negative d), built in the robot's current heading frame so the follower's
                // absolute convention never matters.
                double dCurve = doubleArg(cmd, "d", 18);
                double power = doubleArg(cmd, "power", 0.6);
                com.pedropathing.math.Pose cur = pedro.pose();
                double h = cur.heading();
                double fxu = Math.cos(h), fyu = Math.sin(h);
                double lxu = -Math.sin(h), lyu = Math.cos(h);
                double ad = Math.abs(dCurve);
                com.pedropathing.math.Pose c1 = new com.pedropathing.math.Pose(
                        cur.x() + ad * fxu, cur.y() + ad * fyu, h);
                com.pedropathing.math.Pose c2 = new com.pedropathing.math.Pose(
                        cur.x() + ad * fxu + dCurve * lxu,
                        cur.y() + ad * fyu + dCurve * lyu, h);
                // A Bezier stays inside its control points' convex hull, so these checks bound
                // the whole curve.
                if (!pedroPointOk(c1.x(), c1.y()) || !pedroPointOk(c2.x(), c2.y())) {
                    message = String.format(Locale.US,
                            "REFUSED: curve control points (%.1f, %.1f)/(%.1f, %.1f) leave the "
                                    + "box minus %.0f in margin. Nothing moved.",
                            c1.x(), c1.y(), c2.x(), c2.y(), PEDRO_TARGET_MARGIN_IN);
                    break;
                }
                String curveRefusal = pedroPathRefusal(new double[][] {
                        {cur.x(), cur.y()}, {c1.x(), c1.y()}, {c2.x(), c2.y()}});
                if (curveRefusal != null) {
                    message = curveRefusal;
                    break;
                }
                com.pedropathing.paths.Path p =
                        com.pedropathing.api.Paths.curve(cur, c1, c2).constant(h);
                setPedroSpeedCap(Math.max(0.1, Math.min(1.0, power)));
                pedro.follow(p);
                pedroJob = "curve";
                message = String.format(Locale.US, "Curve d %+.1f at %.2f.", dCurve, power);
                break;
            }
            case "pedroPidf": {
                // Follower gains, set on the shared Foresight config and then the follower is
                // thrown away - the next pedroStart rebuilds from clean state, so no stale
                // internal copy survives. These are tuned THROUGH Pedro, which is the point: the
                // tool's own heading hold (setHeadingPidf) is a different control law and its
                // gains do not transfer to Foresight.
                //   hp                   Foresight heading kP (P only)
                //   tp                   all four translational kP (primary = secondary)
                //   tfp tfs tsp tss      forward/strafe primary/secondary kP individually
                //   ti td                translational kI, kD (all four controllers)
                //   fzpa lzpa            natural forward/strafe deceleration, in/s^2
                //   hbl hbq              heading braking (lookahead) linear / quadratic coefficients
                //   fbq sbq              forward/strafe quadratic brake coefficient, PINNED (else it
                //                        follows fzpa/lzpa as 1/(2a), the way Constants derives it)
                ForesightConfig fc = Constants.foresightConfig;
                List<String> ignored = new ArrayList<>();
                for (String k : new String[] {"tf", "dp", "di", "dd", "dt", "df", "cent"}) {
                    if (cmd.get(k) != null) {
                        ignored.add(k);
                    }
                }
                fHeadingKP = doubleArg(cmd, "hp", fHeadingKP);
                double tpAll = doubleArg(cmd, "tp", Double.NaN);
                if (!Double.isNaN(tpAll)) {
                    fFwdPrimKP = fFwdSecKP = fStrPrimKP = fStrSecKP = tpAll;
                }
                fFwdPrimKP = doubleArg(cmd, "tfp", fFwdPrimKP);
                fFwdSecKP = doubleArg(cmd, "tfs", fFwdSecKP);
                fStrPrimKP = doubleArg(cmd, "tsp", fStrPrimKP);
                fStrSecKP = doubleArg(cmd, "tss", fStrSecKP);
                fTransKI = doubleArg(cmd, "ti", fTransKI);
                fTransKD = doubleArg(cmd, "td", fTransKD);
                // 2.1.2's zero-power accelerations were negative; Foresight takes the same
                // coast-down as a positive deceleration.
                double fzpa = Math.abs(doubleArg(cmd, "fzpa", 0));
                if (fzpa > 0) {
                    fFwdDecel = fzpa;
                }
                double lzpa = Math.abs(doubleArg(cmd, "lzpa", 0));
                if (lzpa > 0) {
                    fStrDecel = lzpa;
                }
                fHeadBrakeLin = doubleArg(cmd, "hbl", fHeadBrakeLin);
                fHeadBrakeQuad = doubleArg(cmd, "hbq", fHeadBrakeQuad);
                fFwdBrakeQuad = doubleArg(cmd, "fbq", fFwdBrakeQuad);
                fStrBrakeQuad = doubleArg(cmd, "sbq", fStrBrakeQuad);
                applyFollowerGains();
                if (pedro != null) {
                    pedroBreak();
                    pedro = null;
                }
                if (mode == Mode.FOLLOW) {
                    setMode(Mode.IDLE);
                }
                message = String.format(Locale.US, "Follower gains: heading kP %.3f, "
                                + "translational fwd %.3f/%.3f strafe %.3f/%.3f (primary/secondary)"
                                + " kI %.4f kD %.4f, decel %.1f/%.1f. Follower discarded - "
                                + "pedroStart to rebuild.",
                        fHeadingKP, fFwdPrimKP, fFwdSecKP, fStrPrimKP, fStrSecKP, fTransKI,
                        fTransKD, fFwdDecel, fStrDecel)
                        + String.format(Locale.US, " Heading braking %.4f/%.4f, brake quad %s/%s.", fHeadBrakeLin,
                                fHeadBrakeQuad, Double.isNaN(fFwdBrakeQuad) ? "derived" : String.valueOf(fFwdBrakeQuad),
                                Double.isNaN(fStrBrakeQuad) ? "derived" : String.valueOf(fStrBrakeQuad))
                        + (ignored.isEmpty() ? "" : " IGNORED, no Pedro 3 equivalent: " + ignored);
                break;
            }
            case "pedroReset":
                // Back to exactly what is installed: Foresight gains from Constants, mixer and
                // schedule at the vendored defaults. pedrocheck.py sends it before every A/B arm
                // so no arm inherits the previous one's pedroPidf.
                restoreShippedStatics();
                if (pedro != null) {
                    pedroBreak();
                    pedro = null;
                }
                if (mode == Mode.FOLLOW) {
                    setMode(Mode.IDLE);
                }
                message = "Follower gains, mixer and schedule reset to the installed build.";
                break;
            case "pedroStop":
                setMode(Mode.IDLE);
                message = "Follower stopped.";
                break;
            case "foresight": {
                // stage=headingId|headingBrake|decel|brake|translational axis=forward|strafe p1= p2=
                // pods=shipped|live. Always on a fresh follower, inside the box.
                if (!poseOk) {
                    message = "No pose - the Foresight stages need the Pinpoint.";
                    break;
                }
                if (!boxValid) {
                    message = "No box - the Foresight stages refuse to run without a fence. Mark the box first.";
                    break;
                }
                ForesightBoxTuner.Stage st;
                String sa = cmd.get("stage") == null ? "" : cmd.get("stage");
                switch (sa) {
                    case "headingId": st = ForesightBoxTuner.Stage.HEADING_ID; break;
                    case "headingBrake": st = ForesightBoxTuner.Stage.HEADING_BRAKE; break;
                    case "decel": st = ForesightBoxTuner.Stage.DECEL; break;
                    case "brake": st = ForesightBoxTuner.Stage.BRAKE; break;
                    case "translational": st = ForesightBoxTuner.Stage.TRANSLATIONAL; break;
                    default:
                        message = "foresight stage= must be headingId, headingBrake, decel, brake or translational.";
                        st = null;
                }
                if (st == null) {
                    break;
                }
                String ax = cmd.get("axis") == null ? "forward" : cmd.get("axis");
                if (!"forward".equals(ax) && !"strafe".equals(ax)) {
                    message = "foresight axis= must be forward or strafe.";
                    break;
                }
                String podsArg = cmd.get("pods") == null ? "shipped" : cmd.get("pods");
                if (!"shipped".equals(podsArg) && !"live".equals(podsArg)) {
                    message = "foresight pods= must be shipped or live.";
                    break;
                }
                setMode(Mode.IDLE);
                try {
                    buildPedro("live".equals(podsArg));
                } catch (RuntimeException e) {
                    pedro = null;
                    pedroPods = null;
                    message = "Follower build failed: " + e.getMessage();
                    break;
                }
                pedro.localizer.update();
                String refusal = fTuner.start(st, "forward".equals(ax), doubleArg(cmd, "p1", Double.NaN),
                        doubleArg(cmd, "p2", Double.NaN), foresightIo);
                if (refusal != null) {
                    pedro.drivetrain.stop();
                    message = "foresight REFUSED: " + refusal;
                    break;
                }
                setMode(Mode.FORESIGHT);
                message = fTuner.message();
                break;
            }
            case "foresightStop":
                setMode(Mode.IDLE);
                message = "Foresight stage stopped.";
                break;
            case "zeroTrim": {
                // Rotates every pod's forward reference by the same angle. Exists because a
                // by-eye re-zero can carry a COMMON bias - all four wheels parallel but
                // pointing off true chassis-forward - which no pod-relative measurement can
                // see: the encoders track perfectly while the robot crabs. Measured 2026-08-14
                // as a 16 degree motion-left crab at crawl speed, identical at speed. The trim
                // is derived from odometry (drive straight, measure the crab angle, trim by
                // it) and verified by re-measuring.
                double trimDeg = doubleArg(cmd, "deg", 0);
                for (PodCal c : cals) {
                    c.angleOffsetRad = c.angleOffsetRad + Math.toRadians(trimDeg);
                }
                podsDirty = true;
                saveCalibration();
                message = String.format(Locale.US,
                        "All pod zeros trimmed %+.2f deg and saved. Re-measure the crab.",
                        trimDeg);
                break;
            }
            case "odoConfig": {
                // Live Pinpoint geometry, in Constants.java's own terms so a value moves from
                // here to the TUNED block unchanged (export carries it):
                //   xPodOffset   pinpointXPodOffset - the FORWARD (x) pod's sideways distance
                //                from the robot centre, inches, left positive
                //   yPodOffset   pinpointYPodOffset - the STRAFE (y) pod's forward distance
                //                from the robot centre, inches, forward positive
                //   xPodReversed / yPodReversed   true|false
                //   restore=1    back to Constants.java as installed
                // Anything not given keeps its live value. Legacy 2.x names still work:
                // fy = xPodOffset, sx = yPodOffset, fdir/sdir = forward|reversed.
                // Config and pose frame are inseparable, so this resets the pose and clears
                // the fence.
                if (pinpoint == null) {
                    message = "No pinpoint device.";
                    break;
                }
                try {
                    if (cmd.get("restore") != null) {
                        odoXPodOffset = Constants.pinpointXPodOffset;
                        odoYPodOffset = Constants.pinpointYPodOffset;
                        odoXPodReversed = Constants.pinpointXPodReversed;
                        odoYPodReversed = Constants.pinpointYPodReversed;
                    }
                    odoXPodOffset = doubleArg(cmd, "xPodOffset",
                            doubleArg(cmd, "fy", odoXPodOffset));
                    odoYPodOffset = doubleArg(cmd, "yPodOffset",
                            doubleArg(cmd, "sx", odoYPodOffset));
                    odoXPodReversed = boolArg(cmd, "xPodReversed", cmd.get("fdir") == null
                            ? odoXPodReversed : "reversed".equalsIgnoreCase(cmd.get("fdir")));
                    odoYPodReversed = boolArg(cmd, "yPodReversed", cmd.get("sdir") == null
                            ? odoYPodReversed : "reversed".equalsIgnoreCase(cmd.get("sdir")));
                    stopForFrameChange();
                    applyOdometry();
                    pinpoint.resetPosAndIMU();
                    boxValid = false;
                    boxMarked0 = false;
                    if (BOX_FILE.exists()) {
                        BOX_FILE.delete();
                    }
                    message = String.format(Locale.US, "Pinpoint: x pod offset %.3f in %s, y pod "
                                    + "offset %.3f in %s%s. Pose reset - keep the robot still a "
                                    + "moment. Fence cleared with the old frame.",
                            odoXPodOffset, odoXPodReversed ? "REVERSED" : "FORWARD",
                            odoYPodOffset, odoYPodReversed ? "REVERSED" : "FORWARD",
                            odometryShipped() ? " (= Constants.java)" : " (DIFFERS from "
                                    + "Constants.java)");
                } catch (RuntimeException e) {
                    message = "odoConfig failed: " + e.getMessage();
                }
                break;
            }
            case "boxMark": {
                if (!poseOk) {
                    message = "Cannot mark a corner: no pose from the Pinpoint.";
                    break;
                }
                int corner = intArg(cmd, "corner", boxMarked0 ? 1 : 0);
                if (corner == 0) {
                    boxCorner0X = poseXIn;
                    boxCorner0Y = poseYIn;
                    boxMarked0 = true;
                    message = String.format(Locale.US,
                            "Corner A marked at (%.1f, %.1f). Drive to the opposite corner and "
                                    + "mark B.", poseXIn, poseYIn);
                } else if (!boxMarked0) {
                    message = "Mark corner A first.";
                } else {
                    boxMinX = Math.min(boxCorner0X, poseXIn);
                    boxMaxX = Math.max(boxCorner0X, poseXIn);
                    boxMinY = Math.min(boxCorner0Y, poseYIn);
                    boxMaxY = Math.max(boxCorner0Y, poseYIn);
                    if (boxMaxX - boxMinX < 12 || boxMaxY - boxMinY < 12) {
                        message = String.format(Locale.US,
                                "Box %.0f x %.0f in is too small to drive in (need 12+ each "
                                        + "way). Not saved.",
                                boxMaxX - boxMinX, boxMaxY - boxMinY);
                        boxValid = false;
                    } else {
                        boxValid = true;
                        setMarkedFence();
                        saveBox();
                        message = String.format(Locale.US,
                                "Box saved: x %.1f..%.1f, y %.1f..%.1f (%.0f x %.0f in). Hard "
                                        + "limit armed.",
                                boxMinX, boxMaxX, boxMinY, boxMaxY,
                                boxMaxX - boxMinX, boxMaxY - boxMinY);
                    }
                    boxMarked0 = false;
                }
                break;
            }
            case "boxSet": {
                // Arms a box from explicit coordinates rather than by driving to two corners.
                //
                // The recovery path that pairs with setPose: if a frame was re-origined by
                // something else and then restored, the box that was marked in it is still
                // correct and re-marking corners means driving the robot to two walls with no
                // fence armed - the more dangerous of the two options, not the safer one.
                //
                // Deliberately NOT a shortcut around marking. It writes exactly what it is
                // given, so the operator still has to confirm the fence lines up with the mat.
                double x0 = doubleArg(cmd, "minX", Double.NaN);
                double y0 = doubleArg(cmd, "minY", Double.NaN);
                double x1 = doubleArg(cmd, "maxX", Double.NaN);
                double y1 = doubleArg(cmd, "maxY", Double.NaN);
                if (Double.isNaN(x0) || Double.isNaN(y0) || Double.isNaN(x1) || Double.isNaN(y1)) {
                    message = "boxSet needs minX, minY, maxX and maxY.";
                    break;
                }
                boxMinX = Math.min(x0, x1);
                boxMaxX = Math.max(x0, x1);
                boxMinY = Math.min(y0, y1);
                boxMaxY = Math.max(y0, y1);
                boxValid = true;
                boxMarked0 = false;
                boxNeedsFrameCheck = false;
                setMarkedFence();
                saveBox();
                message = String.format(Locale.US,
                        "Box armed from coordinates: %.2f x %.2f in. VERIFY IT AGAINST THE MAT "
                                + "before driving - nothing here checked it.",
                        boxMaxX - boxMinX, boxMaxY - boxMinY);
                break;
            }

            case "fieldFence":
            case "borderFence": {
                // Arms a PERIMETER fence - the rect is the physical wall and the robot's
                // footprint (heading-aware) must stay inside it - from a pose the operator
                // MEASURED and reports. The pose is written into the Pinpoint, so the fence and
                // the robot agree by construction.
                //
                // fieldFence: the BIOBUZZ field, 0..141.5 in (or minX/minY/maxX/maxY inside it),
                //   plus every keep-out in field/biobuzz_field.json. Pose in the visualizer's
                //   frame: x = 0 the red wall, y = 0 the audience wall.
                // borderFence: just a border, width x height inches, no keep-outs - any floor
                //   with walls or tape. Origin at the corner the operator picks; x runs along
                //   the width, y along the height, so the pose is "inches out from the x = 0
                //   edge, inches out from the y = 0 edge, heading from +x".
                //
                // Like boxSet, it trusts what it is given. The operator confirms the drawn
                // fence matches the floor before anything moves.
                boolean border = "borderFence".equals(action);
                if (pinpoint == null) {
                    message = "No pinpoint device.";
                    break;
                }
                double fx = doubleArg(cmd, "x", Double.NaN);
                double fy = doubleArg(cmd, "y", Double.NaN);
                double fh = doubleArg(cmd, "headingDeg", Double.NaN);
                if (Double.isNaN(fx) || Double.isNaN(fy) || Double.isNaN(fh)) {
                    message = action + " needs the robot's measured pose: x, y (inches from the "
                            + (border ? "x = 0 and y = 0 edges of the border"
                                    : "red wall and the audience wall")
                            + ") and headingDeg.";
                    break;
                }
                List<FenceGeometry.Polygon> loaded = new ArrayList<>();
                double rx0, ry0, rx1, ry1;
                if (border) {
                    double bw = doubleArg(cmd, "width", Double.NaN);
                    double bh = doubleArg(cmd, "height", Double.NaN);
                    if (!(bw > 0) || !(bh > 0)) {
                        message = "borderFence needs width and height, inches.";
                        break;
                    }
                    rx0 = 0;
                    ry0 = 0;
                    rx1 = bw;
                    ry1 = bh;
                } else {
                    double size;
                    try {
                        size = loadFieldKeepOuts(loaded);
                    } catch (IOException | RuntimeException e) {
                        message = "fieldFence: could not load the field: " + e.getMessage();
                        break;
                    }
                    rx0 = doubleArg(cmd, "minX", 0);
                    ry0 = doubleArg(cmd, "minY", 0);
                    rx1 = doubleArg(cmd, "maxX", size);
                    ry1 = doubleArg(cmd, "maxY", size);
                    if (Math.min(rx0, rx1) < 0 || Math.min(ry0, ry1) < 0
                            || Math.max(rx0, rx1) > size || Math.max(ry0, ry1) > size) {
                        message = String.format(Locale.US, "fieldFence: region must lie inside "
                                + "the %.1f in field.", size);
                        break;
                    }
                }
                double rl = doubleArg(cmd, "robotL", fenceRobotL);
                double rw = doubleArg(cmd, "robotW", fenceRobotW);
                if (rl <= 0 || rw <= 0) {
                    message = action + ": the robot size must be positive.";
                    break;
                }
                if (Math.abs(rx1 - rx0) - Math.max(rl, rw) < 12
                        || Math.abs(ry1 - ry0) - Math.max(rl, rw) < 12) {
                    message = action + ": the area leaves under 12 in of travel each way for the "
                            + "robot's footprint. Not armed.";
                    break;
                }
                if (pedro != null) {
                    // The follower carries the old frame; a fresh one is built on pedroStart.
                    pedroBreak();
                    pedro = null;
                }
                setMode(Mode.IDLE);
                allStop();
                pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, fx, fy, AngleUnit.DEGREES, fh));
                pinpoint.update();
                poseXIn = fx;
                poseYIn = fy;
                headingRad = Math.toRadians(fh);
                boxMinX = Math.min(rx0, rx1);
                boxMaxX = Math.max(rx0, rx1);
                boxMinY = Math.min(ry0, ry1);
                boxMaxY = Math.max(ry0, ry1);
                fenceRobotL = rl;
                fenceRobotW = rw;
                boxBoundary = true;
                boxKind = border ? "border" : "field";
                keepOuts.clear();
                keepOuts.addAll(loaded);
                keepOutsJson = buildKeepOutsJson();
                boxValid = true;
                boxMarked0 = false;
                boxNeedsFrameCheck = false;
                saveBox();
                double[] cb = centreBounds(headingRad);
                boolean startInside = fx >= cb[0] && fx <= cb[2] && fy >= cb[1] && fy <= cb[3];
                double startClear = keepOutClearance(fx, fy);
                message = String.format(Locale.US,
                        "%s fence armed: x %.1f..%.1f, y %.1f..%.1f, %d keep-outs, robot %.1f x "
                                + "%.1f in. Pose set to (%.1f, %.1f) at %.1f deg.%s%s VERIFY the "
                                + "drawn fence against the floor before driving.",
                        border ? "Border" : "Field", boxMinX, boxMaxX, boxMinY, boxMaxY,
                        keepOuts.size(), rl, rw, fx, fy, fh,
                        startInside ? "" : " START POSE IS PAST A WALL'S FOOTPRINT LIMIT - only "
                                + "inward motion will pass.",
                        startClear >= 0 ? "" : " START POSE OVERLAPS A KEEP-OUT - only motion "
                                + "away from it will pass.");
                break;
            }

            case "boxClear":
                boxValid = false;
                boxMarked0 = false;
                setMarkedFence();
                if (BOX_FILE.exists() && !BOX_FILE.delete()) {
                    message = "Box disarmed, but the file on the hub would not delete.";
                } else {
                    message = "Box cleared and disarmed.";
                }
                break;
            case "setXLock":
                xLock = cmd.get("value") == null ? !xLock : Boolean.parseBoolean(cmd.get("value"));
                podsDirty = true;
                message = xLock
                        ? "Zero input locks the pods into an X."
                        : "Zero input holds pod headings (easier to read while tuning).";
                break;
            case "drive":
                if (mode != Mode.DRIVE) {
                    // Entering drive from any other mode goes through setMode so a routine that
                    // was mid-flight (wire scan, pulse, autotune) is actually stopped and its
                    // busy flag cleared, instead of leaving routineActive latched true and its
                    // last servo powers held. Assigning mode directly skipped all of that.
                    // setMode also zeroes the drive inputs, so it must run before they are set.
                    setMode(Mode.DRIVE);
                    if (headingOk) {
                        // Adopt the heading we are already at, so enabling drive never snaps.
                        headingTargetRad = headingRad;
                        headingPidf.reset();
                        headingStickTimer.reset();
                    }
                }
                // Clamped to the mixer's range: nothing downstream bounds a host value, and an
                // out-of-range demand would drive past what the fence's margins were sized for.
                driveForward = Utils.clamp(doubleArg(cmd, "f", 0), -1.0, 1.0);
                driveStrafe = Utils.clamp(doubleArg(cmd, "s", 0), -1.0, 1.0);
                driveTurn = Utils.clamp(doubleArg(cmd, "t", 0), -1.0, 1.0);
                driveFieldOriented = doubleArg(cmd, "foc", 0) != 0;
                lastDriveCmdMs = System.currentTimeMillis();
                message = "Drive test active.";
                break;
            case "focRef":
                if (headingOk) {
                    focRefRad = headingRad;
                    message = String.format(Locale.US,
                            "Field forward captured at %.1f deg.", Math.toDegrees(headingRad));
                } else {
                    message = "No heading available - field forward not captured.";
                }
                break;
            case "export": {
                SwerveExport.Live live = liveForExport();
                String nonFinite = SwerveExport.nonFinite(cals, live);
                if (nonFinite != null) {
                    // Cleared, not left holding the previous export, so nothing stale can be
                    // spliced on the back of a refused one.
                    exportText = "";
                    message = "REFUSED export: non-finite values (" + nonFinite + "). Set them "
                            + "with setPidf/pedroPidf first.";
                    break;
                }
                exportText = SwerveExport.generate(cals, live);
                message = "Constants generated - Constants.java declarations; splice with "
                        + "robot.py constants --write.";
                break;
            }
            case "save":
                saveCalibration();
                message = "Calibration saved to " + CAL_FILE.getName();
                break;
            case "reload":
                loadCalibration();
                acquireHardware();
                podsDirty = true;
                message = "Calibration reloaded.";
                break;
            default:
                // A misspelt or imagined action used to fall through here and report nothing, so
                // the caller saw a successful HTTP response for a command that did not exist. That
                // burned a whole staircase run against a "setMode" that was never implemented.
                // Silence is the worst answer available; say so instead.
                message = "UNKNOWN COMMAND: \"" + action + "\" - nothing was done.";
                unknownCommand = action;
                break;
        }
    }

    /** Pedro's factory order: leftFront, rightFront, leftBack, rightBack. */
    /**
     * The non-pod half of an export: the heading gains this tool holds, the follower gains as
     * pedroPidf left them on the shared Foresight config, and the Pinpoint geometry as odoConfig
     * last wrote it.
     */
    private SwerveExport.Live liveForExport() {
        SwerveExport.Live live = new SwerveExport.Live();
        live.label = recorder.label();
        live.headingKP = fHeadingKP;
        live.toolHeadingHoldKP = headingKp;
        live.toolHeadingHoldKD = headingKd;
        live.forwardTranslationalPrimaryKP = fFwdPrimKP;
        live.forwardTranslationalSecondaryKP = fFwdSecKP;
        live.strafeTranslationalPrimaryKP = fStrPrimKP;
        live.strafeTranslationalSecondaryKP = fStrSecKP;
        live.translationalKI = fTransKI;
        live.translationalKD = fTransKD;
        live.naturalForwardDeceleration = fFwdDecel;
        live.naturalStrafeDeceleration = fStrDecel;
        live.epsilonTaper = Swerve.getEpsilonTaper();
        live.demandSlewDegPerSec = Swerve.getDemandSlewDegPerSec();
        live.schedule = CoaxialPod.getScheduleTuning();
        live.pinpointXPodOffset = odoXPodOffset;
        live.pinpointYPodOffset = odoYPodOffset;
        live.pinpointXPodReversed = odoXPodReversed;
        live.pinpointYPodReversed = odoYPodReversed;
        return live;
    }

    private PodCal findByLabel(String label) {
        for (PodCal c : cals) {
            if (label.equalsIgnoreCase(c.label)) {
                return c;
            }
        }
        return null;
    }

    private static int intArg(Map<String, String> cmd, String key, int fallback) {
        try {
            String v = cmd.get(key);
            return v == null ? fallback : Integer.parseInt(v);
        } catch (NumberFormatException e) {
            return fallback;
        }
    }

    private static boolean boolArg(Map<String, String> cmd, String key, boolean fallback) {
        String v = cmd.get(key);
        return v == null ? fallback : Boolean.parseBoolean(v);
    }

    private static double doubleArg(Map<String, String> cmd, String key, double fallback) {
        double d;
        try {
            String v = cmd.get(key);
            if (v == null) {
                return fallback;
            }
            d = Double.parseDouble(v);
        } catch (NumberFormatException e) {
            return fallback;
        }
        // Double.parseDouble accepts "NaN" and "Infinity" (and "1e999"). NaN passes straight
        // through applyBoxLimit - every comparison is false - and a NaN gain was being saved to
        // the calibration file. handleCommand already refuses these up front; this is the
        // backstop for any path that reaches here another way.
        if (Double.isNaN(d) || Double.isInfinite(d)) {
            throw new IllegalArgumentException(key + "=" + cmd.get(key) + " is not a finite number");
        }
        return d;
    }

    /** Double.parseDouble for host-sent text, refusing NaN/Infinity. */
    private static double finiteDouble(String text) {
        double d = Double.parseDouble(text.trim());
        if (Double.isNaN(d) || Double.isInfinite(d)) {
            throw new IllegalArgumentException("\"" + text + "\" is not a finite number");
        }
        return d;
    }

    /** Free-text parameters (pts/head are parsed piecewise with finiteDouble). */
    private static boolean isTextParam(String key) {
        return "action".equals(key) || "label".equals(key) || "pts".equals(key)
                || "head".equals(key);
    }

    /**
     * Throws if any numeric-looking parameter is NaN or infinite, before the command does
     * anything - a half-applied setPidf (some pods changed, then a NaN) is worse than none.
     * drainCommands turns the throw into an ERROR message and IDLE.
     */
    private static void requireFiniteArgs(Map<String, String> cmd) {
        for (Map.Entry<String, String> e : cmd.entrySet()) {
            if (e.getValue() == null || isTextParam(e.getKey())) {
                continue;
            }
            double d;
            try {
                d = Double.parseDouble(e.getValue().trim());
            } catch (NumberFormatException notANumber) {
                continue;   // non-numeric text: doubleArg falls back, as it always has
            }
            if (Double.isNaN(d) || Double.isInfinite(d)) {
                throw new IllegalArgumentException(e.getKey() + "=" + e.getValue()
                        + " is not a finite number");
            }
        }
    }

    // ---------------------------------------------------------------- gamepad fallback

    private void handleGamepad() {
        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;
        if (up && !prevDpadUp) {
            selected = (selected + 1) % POD_COUNT;
        }
        if (down && !prevDpadDown) {
            selected = (selected + POD_COUNT - 1) % POD_COUNT;
        }
        prevDpadUp = up;
        prevDpadDown = down;

        // Zeroing is safe before START because it only reads encoders; motion is not.
        boolean canMove = started;

        if (gamepad1.a && !prevA && canMove) {
            setMode(Mode.MOTOR_PULSE);
        }
        prevA = gamepad1.a;

        if (gamepad1.b && !prevB && canMove) {
            setMode(Mode.WIRE_SCAN);
        }
        prevB = gamepad1.b;

        if (gamepad1.x && !prevX) {
            captureZeros(false);
        }
        prevX = gamepad1.x;

        if (gamepad1.y && !prevY && canMove) {
            setMode(Mode.ENC_SWEEP);
        }
        prevY = gamepad1.y;

        if (gamepad1.start && !prevStart) {
            allStop();
            pidHolding = false;
            setMode(Mode.IDLE);
        }
        prevStart = gamepad1.start;

        // Bumpers jog the selected pod, but must not hijack an automatic routine mid-run.
        // That includes every automatic mode, not just the scans: a bumper press during
        // AUTOTUNE stranded the tuner mid-run, and during CAL_POS it abandoned a guarded walk.
        boolean scanning = mode == Mode.WIRE_SCAN || mode == Mode.ENC_SWEEP
                || mode == Mode.AUTOTUNE || mode == Mode.CAL_POS || mode == Mode.HEADING;
        if ((gamepad1.left_bumper || gamepad1.right_bumper) && canMove && !scanning) {
            beginJog(gamepad1.right_bumper ? NUDGE_POWER : -NUDGE_POWER,
                    NUDGE_SECONDS, "Jogging pod " + selected + ".");
        }
    }

    // ---------------------------------------------------------------- persistence

    private void saveCalibration() {
        FileWriter w = null;
        try {
            w = new FileWriter(CAL_FILE, false);
            w.write("# Swerve bring-up calibration. Generated by SwerveBringUp.\n");
            for (PodCal c : cals) {
                w.write(c.serialize());
                w.write("\n");
            }
        } catch (IOException e) {
            message = "Could not save calibration: " + e.getMessage();
        } finally {
            closeQuietly(w);
        }
    }

    private void loadCalibration() {
        if (!CAL_FILE.exists()) {
            return;
        }
        BufferedReader r = null;
        try {
            r = new BufferedReader(new FileReader(CAL_FILE));
            String line;
            while ((line = r.readLine()) != null) {
                line = line.trim();
                if (line.isEmpty() || line.startsWith("#")) {
                    continue;
                }
                int idx;
                try {
                    idx = Integer.parseInt(line.substring(0, line.indexOf('|')));
                } catch (RuntimeException e) {
                    continue;
                }
                if (idx >= 0 && idx < POD_COUNT) {
                    cals[idx].applySerialized(line);
                }
            }
        } catch (IOException e) {
            // Not added to hwErrors: acquireHardware() clears that list right after this runs.
            message = "Could not read saved calibration: " + e.getMessage();
        } finally {
            closeQuietly(r);
        }
    }

    private static void closeQuietly(java.io.Closeable c) {
        if (c != null) {
            try {
                c.close();
            } catch (IOException ignored) {
                // nothing useful to do during bring-up
            }
        }
    }

    // ---------------------------------------------------------------- commanded state

    /**
     * Reproduces the pod vectors {@code Swerve.arcadeDrive} builds, so the visualizer can draw
     * where each wheel is being told to point alongside where it actually is.
     *
     * <p>Pedro constructs these as {@code Vector(magnitude, theta)}, which flips theta by pi for a
     * negative magnitude. Building them from cartesian components here is equivalent, and lets
     * atan2 resolve direction directly.
     */
    private void computeTargets() {
        for (int i = 0; i < POD_COUNT; i++) {
            targetTheta[i] = Double.NaN;
            targetPower[i] = 0;
        }

        if (mode == Mode.PID && pidHolding) {
            if (pidAllPods) {
                for (int i = 0; i < POD_COUNT; i++) {
                    targetTheta[i] = pidTargetRad;
                }
            } else {
                targetTheta[selected] = pidTargetRad;
            }
            return;
        }
        if (mode != Mode.DRIVE) {
            return;
        }

        double forward = driveForward;
        double strafe = -driveStrafe; // arcadeDrive negates strafe before building the vector
        // The rotation arcadeDrive actually received this loop. Under heading hold that is the
        // heading PID's output, and mirroring the raw stick here instead put targets in the
        // recorder that were never commanded - smooth stick, wild "demand".
        double rotation = appliedTurn;

        double transMag = Math.min(1.0, Math.hypot(strafe, forward));
        double transTheta = Math.atan2(forward, strafe);
        boolean zeroTrans = transMag < SWERVE_EPSILON;
        // Was SWERVE_EPSILON. The mixer moved its rotation wall to 0.015 on 2026-08-15 and this
        // mirror did not follow, so for |rotation| in [0.015, 0.05) the recorder logged a
        // translation-only demand while the pods were given translation PLUS rotation: 4.0% of
        // the samples in mydrive-001, worst disagreement 15.0 deg, in the one column that is
        // supposed to discriminate "the demand is shaking" from "the response is shaking".
        boolean zeroRot = Math.abs(rotation) < SWERVE_ROTATION_EPSILON;

        // Mirror the mixer's X_LOCK engage delay too, so the displayed demand does not lead
        // reality by 0.35 s at every stick release.
        if (!(zeroTrans && zeroRot)) {
            lastActiveTargetInputNano = System.nanoTime();
        }
        boolean xLockRipe = zeroTrans && zeroRot
                && (System.nanoTime() - lastActiveTargetInputNano) / 1.0e9 >= X_LOCK_ENGAGE_DELAY_S;

        if (zeroTrans && zeroRot) {
            // Mirrors rebuildPods: X_LOCK whenever xLock is on (heading hold no longer disables
            // it - the hold phase machine guarantees clean zero rotation at rest).
            if (!xLock || !xLockRipe) {
                // Pods just hold their heading, so there is nothing being commanded to show.
                return;
            }
            // X_LOCK points each pod along its own radius, which is what draws the X.
            for (int i = 0; i < POD_COUNT; i++) {
                targetTheta[i] = Math.atan2(cals[i].podX, -cals[i].podY);
                targetPower[i] = 0;
            }
            return;
        }

        double rotScalar = zeroRot ? 0 : rotation;
        double[] px = new double[POD_COUNT];
        double[] py = new double[POD_COUNT];
        double maxMag = SWERVE_MAX_POWER;

        for (int i = 0; i < POD_COUNT; i++) {
            double tx = zeroTrans ? 0 : transMag * Math.cos(transTheta);
            double ty = zeroTrans ? 0 : transMag * Math.sin(transTheta);

            // Perpendicular to the pod's radius: atan2(x, -y) then rotated a quarter turn.
            double rotTheta = Math.atan2(cals[i].podX, -cals[i].podY) + Math.PI / 2.0;
            px[i] = tx + rotScalar * Math.cos(rotTheta);
            py[i] = ty + rotScalar * Math.sin(rotTheta);
            maxMag = Math.max(maxMag, Math.hypot(px[i], py[i]));
        }

        for (int i = 0; i < POD_COUNT; i++) {
            targetTheta[i] = Math.atan2(py[i], px[i]);
            targetPower[i] = Math.hypot(px[i], py[i]) * SWERVE_MAX_POWER / maxMag;
        }
    }

    // ---------------------------------------------------------------- state publishing

    private void publish() {
        long pubMark = System.nanoTime();
        int fmtAtEntry = fmtCalls;
        StringBuilder sb = new StringBuilder(2048);
        sb.append("{");
        sb.append("\"live\":true");
        sb.append(",\"mode\":\"").append(mode.name()).append('"');
        sb.append(",\"selected\":").append(selected);
        sb.append(",\"loopHz\":").append(fmt(loopHz));
        sb.append(",\"voltage\":").append(fmt(batteryVolts()));
        sb.append(",\"servoMa\":").append(fmt(servoRailMa));
        sb.append(",\"fastCurrent\":").append(fastCurrent);
        sb.append(",\"posCoverage\":")
                .append(Double.isNaN(positionalCoverageDeg)
                        ? "null" : fmt(positionalCoverageDeg));
        sb.append(",\"batteryMa\":").append(fmt(batteryMa));
        sb.append(",\"totalMa\":").append(fmt(totalMa));
        sb.append(",\"busy\":").append(routineActive);
        sb.append(",\"started\":").append(started);
        sb.append(",\"xLock\":").append(xLock);
        sb.append(",\"heading\":{\"ok\":").append(headingOk)
                .append(",\"deg\":").append(fmt(Math.toDegrees(headingRad)))
                .append(",\"targetDeg\":").append(fmt(Math.toDegrees(headingTargetRad)))
                .append(",\"kp\":").append(fmt(headingKp))
                .append(",\"kd\":").append(fmt(headingKd))
                .append(",\"kf\":").append(fmt(headingKf))
                .append(",\"closedLoop\":").append(headingClosedLoop)
                .append(",\"hold\":").append(headingHold)
                .append(",\"correcting\":").append(headingPhase == HeadingHoldPhase.ACTIVE)
                .append(",\"restPhase\":\"").append(headingPhase.name()).append('"')
                .append(",\"rate\":").append(fmt(Math.toDegrees(headingRateRadS)))
                .append(",\"foc\":").append(driveFieldOriented)
                .append(",\"focRefDeg\":").append(fmt(Math.toDegrees(focRefRad)))
                .append('}');
        sb.append(",\"pose\":{\"ok\":").append(poseOk)
                .append(",\"x\":").append(fmt(poseXIn))
                .append(",\"y\":").append(fmt(poseYIn))
                .append(",\"vx\":").append(fmt(poseVxIn))
                .append(",\"vy\":").append(fmt(poseVyIn))
                .append('}');
        // Pinpoint device state, for the heading freeze after app restarts (2026-11-15): the driver keeps the
        // OLD values whenever a read looks corrupt - with LOCAL_TEST that includes "loopTime < 1" on v1/v2
        // firmware - so a stalled device reads as a frozen pose with zero rates.
        if (pinpoint != null) {
            try {
                sb.append(",\"pinpointDev\":{\"status\":\"").append(pinpoint.getDeviceStatus().name())
                        .append("\",\"loopTimeUs\":").append(pinpoint.getLoopTime())
                        .append(",\"hz\":").append(fmt(pinpoint.getFrequency()))
                        .append(",\"version\":").append(pinpoint.getDeviceVersion())
                        .append(",\"encX\":").append(pinpoint.getEncoderX())
                        .append(",\"encY\":").append(pinpoint.getEncoderY())
                        .append(",\"yawScalar\":").append(fmt(pinpoint.getYawScalar()))
                        .append(",\"headingRaw\":").append(Double.toString(podoLastHeading))
                        .append(",\"sameHeadingReads\":").append(podoSameHeadingReads)
                        .append('}');
            } catch (RuntimeException e) {
                sb.append(",\"pinpointDev\":{\"error\":\"").append(esc(String.valueOf(e))).append("\"}");
            }
        }
        sb.append(",\"pedro\":{\"active\":").append(mode == Mode.FOLLOW)
                .append(",\"job\":\"").append(esc(pedroJob)).append('"')
                .append(",\"pods\":\"").append(pedro == null ? "none"
                        : pedroLivePods ? "live" : "shipped").append('"')
                // Is the bench running exactly what is installed? Follower gains untouched by
                // pedroPidf AND mixer/schedule at the vendored defaults. A pods=shipped run with
                // this false is not an acceptance run.
                .append(",\"gainsShipped\":").append(followerGainsShipped())
                .append(",\"staticsShipped\":").append(staticsShipped());
        if (pedro != null) {
            try {
                com.pedropathing.math.Pose pp = pedro.pose();
                sb.append(",\"busy\":").append(pedro.isBusy())
                        .append(",\"x\":").append(fmt(pp.x()))
                        .append(",\"y\":").append(fmt(pp.y()))
                        .append(",\"h\":").append(fmt(Math.toDegrees(pp.heading())))
                        .append(",\"terr\":")
                        .append(fmt(pedroTranslationalError()));
            } catch (RuntimeException e) {
                sb.append(",\"busy\":false");
            }
        } else {
            sb.append(",\"busy\":false");
        }
        sb.append('}');
        fTuner.appendJson(sb);
        sb.append(",\"box\":{\"valid\":").append(boxValid)
                .append(",\"minX\":").append(fmt(boxMinX))
                .append(",\"minY\":").append(fmt(boxMinY))
                .append(",\"maxX\":").append(fmt(boxMaxX))
                .append(",\"maxY\":").append(fmt(boxMaxY))
                .append(",\"marked0\":").append(boxMarked0)
                .append(",\"clamped\":").append(boxClampedNow)
                .append(",\"kind\":\"").append(boxKind).append('"')
                .append(",\"robotL\":").append(fmt(fenceRobotL))
                .append(",\"robotW\":").append(fmt(fenceRobotW))
                .append(",\"keepOutClear\":").append(keepOuts.isEmpty() || !poseOk ? "null"
                        : fmt(keepOutClearance(poseXIn, poseYIn)))
                .append(",\"keepOuts\":").append(keepOutsJson)
                .append('}');
        sb.append(",\"odo\":{\"xPodOffset\":").append(fmt(odoXPodOffset))
                .append(",\"yPodOffset\":").append(fmt(odoYPodOffset))
                .append(",\"xPodReversed\":").append(odoXPodReversed)
                .append(",\"yPodReversed\":").append(odoYPodReversed)
                .append(",\"shipped\":").append(odometryShipped())
                .append('}');
        sb.append(",\"message\":\"").append(esc(message)).append('"');
        sb.append(",\"phase\":").append(fmt(phaseTimer.seconds()));
        sb.append(",\"timing\":{\"encoders\":").append(fmt(msEncoders))
                .append(",\"heading\":").append(fmt(msHeading))
                .append(",\"mode\":").append(fmt(msMode))
                .append(",\"publish\":").append(fmt(msPublish))
                .append(",\"telemetry\":").append(fmt(msTelemetry))
                .append(",\"publishHz\":").append(fmt(publishIntervalS > 0 ? 1 / publishIntervalS : 0))
                // Where publish() actually spends its time. Sections in emission order: the
                // scalar header, the four pods, errors/shipped/notes, the 260-sample trace, the
                // tuner log and tail, and the handoff (sb.toString + the atomic swap).
                .append(",\"pub\":{\"head\":").append(fmt(msPubSection[0]))
                .append(",\"pods\":").append(fmt(msPubSection[1]))
                .append(",\"errs\":").append(fmt(msPubSection[2]))
                .append(",\"trace\":").append(fmt(msPubSection[3]))
                .append(",\"tail\":").append(fmt(msPubSection[4]))
                .append(",\"handoff\":").append(fmt(msPubSection[5]))
                .append(",\"fmtCalls\":").append(publishFmtCalls)
                .append(",\"chars\":").append(publishChars)
                .append(",\"traceLen\":").append(traceCount)
                .append(",\"fastFmt\":").append(fastFmt)
                .append('}')
                .append('}');
        sb.append(",\"rec\":{\"recording\":").append(recorder.recording())
                .append(",\"runId\":").append(recorder.runId())
                .append(",\"samples\":").append(recorder.count())
                .append(",\"overflowed\":").append(recorder.overflowed())
                .append(",\"label\":\"").append(esc(recorder.label()))
                .append("\"}");

        msPubSection[0] = smooth(msPubSection[0], System.nanoTime() - pubMark);
        pubMark = System.nanoTime();

        sb.append(",\"pods\":[");
        for (int i = 0; i < POD_COUNT; i++) {
            if (i > 0) {
                sb.append(',');
            }
            appendPod(sb, i);
        }
        sb.append(']');

        msPubSection[1] = smooth(msPubSection[1], System.nanoTime() - pubMark);
        pubMark = System.nanoTime();

        // Hardware faults first, then any divergence from the shipped gains. The divergence
        // entries are recomputed every publish rather than stored, so they clear themselves the
        // moment the gains match again.
        sb.append(",\"errors\":[");
        boolean firstError = true;
        for (String e : hwErrors) {
            if (!firstError) {
                sb.append(',');
            }
            sb.append('"').append(esc(e)).append('"');
            firstError = false;
        }
        if (unknownCommand != null) {
            if (!firstError) {
                sb.append(',');
            }
            sb.append('"').append(esc("UNKNOWN COMMAND \"" + unknownCommand
                    + "\" was sent and ignored - check the caller for a typo")).append('"');
            firstError = false;
        }
        for (String d : gainDivergences()) {
            if (!firstError) {
                sb.append(',');
            }
            sb.append('"').append(esc(d)).append('"');
            firstError = false;
        }
        sb.append(']');

        // Scalars kept for old readers; the perPod arrays are what the guard actually compares.
        sb.append(",\"shipped\":{\"kp\":").append(fmt(Constants.turnKP))
                .append(",\"kd\":").append(fmt(Constants.turnKD))
                .append(",\"ks\":").append(fmt(Constants.turnKS))
                .append(",\"ksband\":").append(fmt(Constants.turnKSBandDeg))
                .append(",\"cache\":").append(fmt(Constants.turnServoCaching));
        sb.append(",\"perPod\":[");
        for (int i = 0; i < POD_COUNT; i++) {
            if (i > 0) {
                sb.append(',');
            }
            sb.append("{\"kp\":").append(fmt(Constants.turnKPPerPod[i]))
                    .append(",\"kd\":").append(fmt(Constants.turnKDPerPod[i]))
                    .append(",\"ks\":").append(fmt(Constants.turnKSPerPod[i]))
                    .append(",\"ksband\":")
                    .append(fmt(Constants.turnKSBandDegPerPod[i]))
                    .append('}');
        }
        sb.append("]}");

        sb.append(",\"notes\":[");
        for (int i = 0; i < scanNotes.size(); i++) {
            if (i > 0) {
                sb.append(',');
            }
            sb.append('"').append(esc(scanNotes.get(i))).append('"');
        }
        sb.append(']');

        msPubSection[2] = smooth(msPubSection[2], System.nanoTime() - pubMark);
        pubMark = System.nanoTime();

        // Emitted oldest-first; the ring buffer's start walks forward once it has wrapped.
        int start = (traceCount < TRACE_LEN) ? 0 : traceHead;
        sb.append(",\"trace\":{\"pod\":").append(tracePod);
        appendTraceSeries(sb, ",\"t\":[", traceT, start);
        appendTraceSeries(sb, "],\"tgt\":[", traceTarget, start);
        appendTraceSeries(sb, "],\"act\":[", traceActual, start);
        sb.append("]}");

        msPubSection[3] = smooth(msPubSection[3], System.nanoTime() - pubMark);
        pubMark = System.nanoTime();

        sb.append(",\"tune\":{\"running\":").append(tuner.isRunning())
                .append(",\"pod\":").append(tuner.podIndex())
                .append(",\"status\":\"").append(esc(tuner.status()))
                .append("\",\"log\":[");
        List<String> tuneLog = tuner.log();
        int logFrom = Math.max(0, tuneLog.size() - 14);
        for (int i = logFrom; i < tuneLog.size(); i++) {
            if (i > logFrom) {
                sb.append(',');
            }
            sb.append('"').append(esc(tuneLog.get(i))).append('"');
        }
        sb.append("]}");

        sb.append(",\"cmd\":{\"f\":").append(fmt(driveForward))
                .append(",\"s\":").append(fmt(driveStrafe))
                .append(",\"t\":").append(fmt(driveTurn))
                .append('}');

        sb.append(",\"export\":\"").append(esc(exportText)).append('"');
        sb.append('}');

        msPubSection[4] = smooth(msPubSection[4], System.nanoTime() - pubMark);
        pubMark = System.nanoTime();

        String json = sb.toString();
        publishChars = json.length();
        publishFmtCalls = fmtCalls - fmtAtEntry;
        SwerveBench.INSTANCE.publish(json);

        msPubSection[5] = smooth(msPubSection[5], System.nanoTime() - pubMark);
    }

    private void appendPod(StringBuilder sb, int i) {
        PodCal c = cals[i];
        double v = volts[i];
        boolean readable = !Double.isNaN(v);
        sb.append("{\"i\":").append(i);
        sb.append(",\"label\":\"").append(esc(c.label)).append('"');
        sb.append(",\"motor\":\"").append(esc(c.motorName)).append('"');
        sb.append(",\"servo\":\"").append(esc(c.servoName)).append('"');
        sb.append(",\"enc\":\"").append(esc(c.encoderName)).append('"');
        sb.append(",\"hasMotor\":").append(motors[i] != null);
        sb.append(",\"hasServo\":").append(servos[i] != null || posServos[i] != null);
        sb.append(",\"servoType\":\"").append(servos[i] != null ? "CRServo"
                : (posServos[i] != null ? "Servo" : "none")).append('"');
        sb.append(",\"hasEnc\":").append(encoders[i] != null);
        sb.append(",\"volts\":").append(readable ? fmt(v) : "null");
        sb.append(",\"rawDeg\":").append(readable ? fmt(Math.toDegrees(c.rawAngleRad(v))) : "null");
        sb.append(",\"zeroDeg\":")
                .append(readable ? fmt(Math.toDegrees(c.zeroedAngleRad(v))) : "null");

        // Visualizer geometry: where the wheel actually points, and where it is being told to.
        sb.append(",\"wheelDeg\":").append(readable
                ? fmt(Math.toDegrees(c.wheelThetaFromEncoder(c.zeroedAngleRad(v))))
                : "null");
        sb.append(",\"tgtDeg\":").append(Double.isNaN(targetTheta[i])
                ? "null"
                : fmt(Math.toDegrees(normalizeTwoPi(targetTheta[i]))));
        sb.append(",\"cmdPower\":").append(fmt(targetPower[i]));
        sb.append(",\"podX\":").append(fmt(c.podX));
        sb.append(",\"podY\":").append(fmt(c.podY));
        sb.append(",\"offsetDeg\":").append(fmt(Math.toDegrees(c.angleOffsetRad)));
        sb.append(",\"minV\":").append(fmt(c.analogMin));
        sb.append(",\"maxV\":").append(fmt(c.analogMax));
        sb.append(",\"encRev\":").append(c.encoderReversed);
        sb.append(",\"drvRev\":").append(c.driveReversed());
        sb.append(",\"srvRev\":").append(c.servoReversed());
        sb.append(",\"kp\":").append(fmt(c.kP));
        sb.append(",\"ki\":").append(fmt(c.kI));
        sb.append(",\"kd\":").append(fmt(c.kD));
        sb.append(",\"kf\":").append(fmt(c.kF));
        sb.append(",\"cache\":").append(fmt(c.servoCaching));
        sb.append(",\"slew\":").append(fmt(c.servoSlewPerUpdate));
        sb.append(",\"ks\":").append(fmt(c.kS));
        sb.append(",\"ksband\":").append(fmt(c.kSBandDeg));
        sb.append(",\"kilimit\":").append(fmt(c.kILimit));
        sb.append(",\"kiband\":").append(fmt(c.kIBandDeg));
        sb.append(",\"kireset\":").append(fmt(c.kIResetDeg));
        sb.append(",\"dom\":").append(c.derivativeOnMeasurement);
        sb.append(",\"pulsed\":").append(c.pulsed);
        sb.append(",\"pband\":").append(fmt(c.pulseBandDeg));
        sb.append(",\"ptol\":").append(fmt(c.pulseTolDeg));
        sb.append(",\"ppow\":").append(fmt(c.pulsePower));
        sb.append(",\"pms\":").append(fmt(c.pulseMs));
        sb.append(",\"pcoast\":").append(fmt(c.pulseCoastMs));
        sb.append(",\"positional\":").append(c.positional);
        sb.append(",\"posCalibrated\":").append(c.posCalibrated);
        sb.append(",\"posMarked0\":").append(c.posMarked0);
        sb.append(",\"posMarked1\":").append(c.posMarked1);
        sb.append(",\"clampMargin\":").append(fmt(c.clampMarginDeg));
        sb.append(",\"raw0\":").append(fmt(c.rawDegAtPos0));
        sb.append(",\"raw1\":").append(fmt(c.rawDegAtPos1));
        sb.append(",\"pwmLo\":").append(fmt(pwmLower[i]));
        sb.append(",\"pwmHi\":").append(fmt(pwmUpper[i]));
        sb.append(",\"pwmFrame\":").append(fmt(pwmFrame[i]));
        sb.append(",\"discovered\":").append(c.discoveredEncoderIndex);

        // Cached command values, never hardware reads. getPower() on a CRServo or DcMotorEx is a
        // live Lynx transaction, and eight of them per publish was 30+ ms of the 38 ms publish
        // cost - the "publish is slow in DRIVE" mystery from 2026-08-13 in its entirety. In
        // closed-loop modes the pod's own cache is the truth (it writes through its own device
        // object); everywhere else the bench's shadow of its last write is.
        double shownServo = servoCmd[i];
        double shownDrive = motorCmd[i];
        if (podMoved[i] && pods != null && pods[i] instanceof CoaxialPod) {
            shownServo = ((CoaxialPod) pods[i]).getLastTurnPower();
            shownDrive = ((CoaxialPod) pods[i]).getLastDrivePower();
        } else if (podMoved[i] && pods != null && pods[i] instanceof PositionalPod) {
            shownServo = ((PositionalPod) pods[i]).getLastTurnPower();
            shownDrive = ((PositionalPod) pods[i]).getLastDrivePower();
        }
        sb.append(",\"servoPower\":").append(fmt(shownServo));
        sb.append(",\"drivePower\":").append(fmt(shownDrive));
        sb.append('}');
    }

    private void appendTraceSeries(StringBuilder sb, String prefix, double[] values, int start) {
        sb.append(prefix);
        for (int i = 0; i < traceCount; i++) {
            if (i > 0) {
                sb.append(',');
            }
            sb.append(fmt(values[(start + i) % TRACE_LEN]));
        }
    }

    private static double normalizeTwoPi(double radians) {
        double a = radians % (2 * Math.PI);
        return a < 0 ? a + 2 * Math.PI : a;
    }

    /**
     * Last cached battery reading. See {@link #refreshBatteryVolts()} for why this is not read on
     * demand.
     */
    private volatile double cachedVolts;

    /**
     * Reads the battery and caches it. Called only from the slow idle-sensor path.
     *
     * <p>{@code getVoltage()} is a Lynx ADC transaction, not something bulk caching covers. It used
     * to be called inline from {@code record()} - every loop - and again from {@code publish()} and
     * {@code pushTelemetry()}. In IDLE that is nearly free, but in DRIVE the bus is already
     * carrying eight actuator writes per loop and the read queues behind them: publish() measured
     * 37-56 ms in DRIVE against 9-13 ms in IDLE, which dragged the control loop down to 20-27 Hz.
     *
     * <p>Battery voltage does not change at loop rate, so sampling it at the idle-sensor rate costs
     * nothing real. The trade is that the recorder's volts column is now stair-stepped at that
     * rate rather than per-sample, which is ample for sag across a step but too coarse to catch a
     * sub-200 ms transient - measure that deliberately if it is ever the question.
     */
    private void refreshBatteryVolts() {
        if (voltageSensor == null) {
            cachedVolts = 0;
            return;
        }
        try {
            cachedVolts = voltageSensor.getVoltage();
        } catch (RuntimeException e) {
            cachedVolts = 0;
        }
    }

    private double batteryVolts() {
        return cachedVolts;
    }

    /**
     * Number of {@link #fmt} calls since the OpMode started, and how many the last publish made.
     *
     * <p>The publish cost was the last unexplained item in the loop budget. Caching the eight
     * {@code getPower()} transactions took it from 37 ms to something smaller but still unmeasured,
     * and the suspect that remained was this method: {@code String.format} builds a Formatter,
     * parses the pattern and allocates on every call, and one publish makes roughly a thousand of
     * them - about 780 of those from the 260-sample trace alone. Counting the calls turns "publish
     * is slow" into a cost per call that can be checked against the section timers.
     */
    private static int fmtCalls;
    private int publishFmtCalls;
    private int publishChars;

    /**
     * Selects the hand-rolled formatter over {@code String.format}. Runtime-switchable
     * ({@code setFastFmt}) so the two can be A/B'd inside one session against one battery, rather
     * than across a redeploy.
     *
     * <p><b>Measured 2026-08-16</b>, robot stationary in IDLE, 12.72 V, 6 randomised interleaved
     * blocks, n=114 samples per arm, identical 193-call payload in both:
     *
     * <pre>
     *   String.format   11.77 ms  sd 1.01  (9.50-15.10)   61.0 us per call
     *   hand-rolled      1.62 ms  sd 0.19  (1.11- 1.95)    8.4 us per call
     *   delta 10.15 ms, 95% CI [9.96, 10.34], t=105
     * </pre>
     *
     * The two distributions do not overlap - the slowest fast-formatter sample is 1.95 ms, the
     * fastest String.format sample is 9.50. 209 numeric fields were compared between the two
     * arms' snapshots and none differed, so this is a pure cost saving and not a rounding change.
     *
     * <p>At 20 Hz publishing that is 203 ms of every second returned to the control loop. The
     * projection that matters is DRIVE with heading hold, where the 260-sample trace pushes the
     * payload to ~973 calls: 59.4 ms against 8.2 ms. That is the loop's bistability - publish
     * runs off a 50 ms timer, so once it costs more than 50 ms every loop pays it.
     *
     * <p>Defaults ON as of that measurement.
     */
    private static volatile boolean fastFmt = true;

    private static String fmt(double v) {
        fmtCalls++;
        if (Double.isNaN(v) || Double.isInfinite(v)) {
            return "0";
        }
        return fastFmt ? fmtFast(v) : String.format(Locale.US, "%.4f", v);
    }

    /**
     * Four decimal places, assembled by hand. Same output as {@code %.4f} over the range this
     * dashboard publishes (angles, powers, voltages, milliamps, milliseconds); values past
     * +/-1e9 are not representable in the fixed-point path and fall back to the SDK formatter.
     */
    private static String fmtFast(double v) {
        double a = v < 0 ? -v : v;
        if (a >= 1e9) {
            return String.format(Locale.US, "%.4f", v);
        }
        long scaled = (long) (a * 10000.0 + 0.5);
        StringBuilder sb = new StringBuilder(16);
        if (v < 0 && scaled != 0) {
            sb.append('-');
        }
        sb.append(scaled / 10000).append('.');
        long frac = scaled % 10000;
        if (frac < 1000) {
            sb.append('0');
        }
        if (frac < 100) {
            sb.append('0');
        }
        if (frac < 10) {
            sb.append('0');
        }
        return sb.append(frac).toString();
    }

    private static String esc(String s) {
        if (s == null) {
            return "";
        }
        StringBuilder out = new StringBuilder(s.length() + 16);
        for (int i = 0; i < s.length(); i++) {
            char ch = s.charAt(i);
            switch (ch) {
                case '"':
                    out.append("\\\"");
                    break;
                case '\\':
                    out.append("\\\\");
                    break;
                case '\n':
                    out.append("\\n");
                    break;
                case '\r':
                    out.append("\\r");
                    break;
                case '\t':
                    out.append("\\t");
                    break;
                default:
                    if (ch < 0x20) {
                        out.append(String.format(Locale.US, "\\u%04x", (int) ch));
                    } else {
                        out.append(ch);
                    }
                    break;
            }
        }
        return out.toString();
    }

    // ---------------------------------------------------------------- telemetry

    private void pushTelemetry() {
        telemetry.addData("Dashboard", "http://192.168.43.1:8080/swerve");
        telemetry.addData("Mode", "%s%s", mode, routineActive ? " (running)" : "");
        telemetry.addData("Selected Pod", "%d (%s)", selected, cals[selected].label);
        telemetry.addData("Status", message);
        telemetry.addData("Loop Hz", "%.0f", loopHz);
        telemetry.addData("Battery", "%.2f V", batteryVolts());

        for (int i = 0; i < POD_COUNT; i++) {
            PodCal c = cals[i];
            String marker = (i == selected) ? ">" : " ";
            if (Double.isNaN(volts[i])) {
                telemetry.addData(marker + " pod" + i + " " + c.label, "encoder unreadable");
            } else {
                telemetry.addData(marker + " pod" + i + " " + c.label,
                        "%.3fV raw %.1f zero %.1f",
                        volts[i],
                        Math.toDegrees(c.rawAngleRad(volts[i])),
                        Math.toDegrees(c.zeroedAngleRad(volts[i])));
            }
        }

        if (!hwErrors.isEmpty()) {
            telemetry.addData("Hardware Errors", hwErrors.size());
            for (int i = 0; i < Math.min(hwErrors.size(), 4); i++) {
                telemetry.addData("  err" + i, hwErrors.get(i));
            }
        }
        for (int i = 0; i < Math.min(scanNotes.size(), 4); i++) {
            telemetry.addData("  scan" + i, scanNotes.get(i));
        }

        telemetry.addLine("Gamepad: dpad=select  A=pulse motor  B=wire scan  "
                + "X=zero pod  Y=sweep  bumpers=nudge  START=stop");
        telemetry.update();
    }
}
