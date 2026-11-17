package org.firstinspires.ftc.teamcode.diagnostics.swerve;

import com.pedropathing.revhub.drivetrains.CoaxialPod;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Locale;
import java.util.Map;

/**
 * Measurement harness for the COMPETITION drive OpMode.
 *
 * <p>Every steering number this project has published came from {@code SwerveBringUp}, which
 * builds its own drivetrain and runs its own publish path. {@code DriveTeleOp} has neither, so its
 * true loop rate had never been measured - the 75.8 Hz in the logs was {@code mean(1/dt)}, the
 * inflated statistic - and no capture of the shipped path existed at all. This is the smallest
 * thing that fixes both:
 *
 * <ul>
 *   <li>a 1 ms-bin histogram of loop dt, so the honest {@code 1/mean(dt)}, p50, p90, p99, min and
 *       max can be read off the Driver Station without a laptop;
 *   <li>the same {@link PodRecorder} the bring-up tool uses, registered with {@link SwerveBench},
 *       so {@code drivecapture.py} and every host-side scorer work unchanged against the
 *       competition OpMode;
 *   <li>a small {@code /swerve/state} snapshot, because the web routes are registered by
 *       {@code SwerveWebApp}'s {@code @WebHandlerRegistrar} at app start and serve whatever
 *       OpMode last published - they were never specific to bring-up.
 * </ul>
 *
 * <p>Cost in the drive loop: the pod reads are cache hits (the OpMode clears the bulk cache once
 * per loop and Pedro has already read every encoder by the time this runs), the histogram is one
 * array increment, the battery is sampled at 2 Hz off the ADC, and the snapshot is built at 20 Hz.
 * Everything is behind {@link #ENABLED} - one constant to turn the whole thing off.
 *
 * <p>DIAGNOSTIC. Lives here, not in {@code tele/}, so the competition OpMode's own file stays
 * three lines longer rather than two hundred.
 */
public class TeleLoopProbe {

    /** Master switch. False makes every method a no-op. */
    public static final boolean ENABLED = true;

    private static final int POD_COUNT = 4;

    /** 1 ms bins up to 200 ms, plus an overflow bin. */
    private static final int BINS = 201;

    private static final double PUBLISH_INTERVAL_S = 0.05;
    private static final double VOLTS_INTERVAL_S = 0.5;

    private final int[] histogram = new int[BINS];
    private long samples;
    private double dtSum;
    private double dtMin = Double.MAX_VALUE;
    private double dtMax;

    private final PodRecorder recorder = new PodRecorder();
    private final ElapsedTime publishTimer = new ElapsedTime();
    private final ElapsedTime voltsTimer = new ElapsedTime();

    private CoaxialPod[] pods;
    private VoltageSensor voltageSensor;
    private double volts = Double.NaN;
    private String message = "";

    private final double[] podVolts = new double[POD_COUNT];
    private final double[] wheelDeg = new double[POD_COUNT];
    private final double[] targetDeg = new double[POD_COUNT];
    private final double[] cmdTargetDeg = new double[POD_COUNT];
    private final double[] errDeg = new double[POD_COUNT];
    private final double[] power = new double[POD_COUNT];
    private final boolean[] flipped = new boolean[POD_COUNT];

    /** Host commands the OpMode handles itself (TeleDriveProbe: sticks, arm). Return true if handled. */
    public interface CommandHandler {
        boolean handle(String action, Map<String, String> cmd);
    }

    private CommandHandler handler;
    private String source = "DriveTeleOp";
    private String extraJson = "";

    public void setCommandHandler(CommandHandler h) {
        handler = h;
    }

    /** Name in /state "source". */
    public void setSource(String s) {
        source = s;
    }

    /** Extra top-level JSON members for /state, e.g. {@code ,"box":{...}} - must start with a comma. */
    public void setExtraJson(String json) {
        extraJson = json == null ? "" : json;
    }

    public void setMessage(String m) {
        message = m == null ? "" : m;
    }

    /** Mode ordinal reported to host tools. Matches SwerveBringUp.Mode.DRIVE so scorers agree. */
    private static final int MODE_DRIVE = 9;

    public void init(HardwareMap hardwareMap) {
        if (!ENABLED) {
            return;
        }
        pods = Constants.builtPods;
        try {
            voltageSensor = hardwareMap.voltageSensor.iterator().next();
            volts = voltageSensor.getVoltage();
        } catch (RuntimeException e) {
            voltageSensor = null;
        }
        SwerveBench.INSTANCE.clearCommands();
        SwerveBench.INSTANCE.setRecorder(recorder);
        for (int i = 0; i < POD_COUNT; i++) {
            podVolts[i] = Double.NaN;
            // No host-side mirror of the mixer runs here, deliberately: ctgt comes from the pods
            // themselves, so there is nothing that can disagree with what they were commanded.
            targetDeg[i] = Double.NaN;
        }
    }

    /**
     * One loop's worth of measurement. Call after the follower has been updated, so the pods hold
     * this loop's commands.
     *
     * @param dt seconds since the previous loop
     * @param loopHz smoothed rate, for the recorder's loopHz column
     * @param headingDeg robot heading, degrees
     * @param headingTargetDeg heading-hold setpoint, degrees; NaN when the hold is off
     * @param poseX field x, inches
     * @param poseY field y, inches
     * @param cmdF forward command applied this loop
     * @param cmdS strafe command applied this loop
     * @param cmdT rotation command applied this loop
     */
    public void update(double dt, double loopHz, double headingDeg, double headingTargetDeg,
            double poseX, double poseY, double cmdF, double cmdS, double cmdT) {
        update(dt, loopHz, headingDeg, headingTargetDeg, poseX, poseY, cmdF, cmdS, cmdT, null);
    }

    /** As above, plus the recorder's drive columns (PodRecorder.DRIVE_COLS values, or null). */
    public void update(double dt, double loopHz, double headingDeg, double headingTargetDeg,
            double poseX, double poseY, double cmdF, double cmdS, double cmdT, double[] drive) {
        if (!ENABLED) {
            return;
        }

        if (dt > 0) {
            samples++;
            dtSum += dt;
            if (dt < dtMin) {
                dtMin = dt;
            }
            if (dt > dtMax) {
                dtMax = dt;
            }
            int bin = (int) (dt * 1000.0);
            histogram[bin < 0 ? 0 : Math.min(bin, BINS - 1)]++;
        }

        if (voltageSensor != null && voltsTimer.seconds() >= VOLTS_INTERVAL_S) {
            voltsTimer.reset();
            // A Lynx ADC transaction, which bulk caching does not cover. Off the hot path.
            volts = voltageSensor.getVoltage();
        }

        drainCommands();

        if (recorder.recording()) {
            samplePods();
            recorder.add(dt, volts, loopHz, MODE_DRIVE, Double.NaN, Double.NaN,
                    podVolts, wheelDeg, targetDeg, cmdTargetDeg, errDeg, power, flipped,
                    headingDeg, headingTargetDeg, poseX, poseY, cmdF, cmdS, cmdT, null, drive);
        }

        if (publishTimer.seconds() >= PUBLISH_INTERVAL_S) {
            publishTimer.reset();
            publish(loopHz, headingDeg, headingTargetDeg, poseX, poseY, cmdF, cmdS, cmdT);
        }
    }

    private void samplePods() {
        for (int i = 0; i < POD_COUNT; i++) {
            CoaxialPod pod = pods == null ? null : pods[i];
            if (pod == null) {
                wheelDeg[i] = Double.NaN;
                cmdTargetDeg[i] = Double.NaN;
                errDeg[i] = Double.NaN;
                power[i] = Double.NaN;
                flipped[i] = false;
                continue;
            }
            // Encoder-frame angle back to wheel space, the inverse of adjustThetaForEncoder and
            // the same convention PodCal.wheelThetaFromEncoder uses, so both recorders agree.
            double t = pod.getAngle() - Math.PI / 2.0;
            wheelDeg[i] = Math.toDegrees(norm2pi(pod.isEncoderReversed() ? t : -t));
            double ct = pod.getLastTargetWheelRad();
            cmdTargetDeg[i] = Double.isNaN(ct) ? Double.NaN : Math.toDegrees(norm2pi(ct));
            errDeg[i] = Math.toDegrees(pod.getLastErrorRad());
            power[i] = pod.getLastTurnPower();
            flipped[i] = pod.wasLastMoveFlipped();
        }
    }

    /** Drains queued host commands. update() calls it too; call it before the drive code to act this loop. */
    public void drainCommands() {
        if (!ENABLED) {
            return;
        }
        for (int guard = 0; guard < 8; guard++) {
            Map<String, String> cmd = SwerveBench.INSTANCE.poll();
            if (cmd == null) {
                return;
            }
            String action = cmd.get("action");
            if ("recStart".equals(action)) {
                String label = cmd.get("label");
                recorder.start(label == null ? "teleop" : label);
                message = "Recording " + recorder.label();
            } else if ("recStop".equals(action)) {
                recorder.stop();
                message = "Recorded " + recorder.count() + " samples.";
            } else if (handler != null && handler.handle(action, cmd)) {
                // handled by the OpMode
            } else {
                // Everything else belongs to the bring-up tool. Say so rather than swallowing it:
                // a host script pointed at the wrong OpMode otherwise looks like a dead robot.
                message = source + " ignores \"" + action + "\" - only recStart/recStop are "
                        + "handled here. Run Swerve Bring-Up for the rest.";
            }
        }
    }

    /** loop_hz_true, from summed dt. Never mean(1/dt). */
    public double loopHzTrue() {
        return samples > 0 ? samples / dtSum : Double.NaN;
    }

    /** Percentile of the dt histogram, in milliseconds. */
    public double dtPercentileMs(double q) {
        if (samples == 0) {
            return Double.NaN;
        }
        long want = (long) Math.ceil(q * samples);
        long seen = 0;
        for (int i = 0; i < BINS; i++) {
            seen += histogram[i];
            if (seen >= want) {
                // Bin i covers [i, i+1) ms; report the upper edge so a quoted p90 is never
                // optimistic.
                return i + 1.0;
            }
        }
        return BINS;
    }

    public String summary() {
        if (!ENABLED || samples == 0) {
            return "loop: no samples";
        }
        return String.format(Locale.US,
                "loop_hz_true %.1f | dt mean %.1f p50 %.0f p90 %.0f p99 %.0f min %.1f max %.1f ms "
                        + "| n %d",
                loopHzTrue(), 1000.0 * dtSum / samples, dtPercentileMs(0.50),
                dtPercentileMs(0.90), dtPercentileMs(0.99), 1000.0 * dtMin, 1000.0 * dtMax,
                samples);
    }

    public String recStatus() {
        if (!ENABLED) {
            return "probe off";
        }
        return (recorder.recording() ? "REC " : "idle ") + recorder.count() + " " + message;
    }

    private void publish(double loopHz, double headingDeg, double headingTargetDeg,
            double poseX, double poseY, double cmdF, double cmdS, double cmdT) {
        StringBuilder sb = new StringBuilder(1024);
        sb.append("{\"live\":true,\"source\":\"").append(esc(source)).append("\",\"mode\":\"DRIVE\",\"started\":true")
                .append(",\"busy\":false,\"selected\":0")
                .append(",\"loopHz\":").append(f(loopHz))
                .append(",\"voltage\":").append(f(volts))
                .append(",\"message\":\"").append(esc(message)).append('"')
                .append(",\"loop\":{\"hzTrue\":").append(f(loopHzTrue()))
                .append(",\"meanMs\":").append(f(samples > 0 ? 1000.0 * dtSum / samples : 0))
                .append(",\"p50Ms\":").append(f(dtPercentileMs(0.50)))
                .append(",\"p90Ms\":").append(f(dtPercentileMs(0.90)))
                .append(",\"p99Ms\":").append(f(dtPercentileMs(0.99)))
                .append(",\"minMs\":").append(f(1000.0 * dtMin))
                .append(",\"maxMs\":").append(f(1000.0 * dtMax))
                .append(",\"n\":").append(samples).append('}')
                .append(",\"heading\":{\"ok\":true,\"deg\":").append(f(headingDeg))
                .append(",\"targetDeg\":").append(f(headingTargetDeg))
                .append(",\"hold\":").append(!Double.isNaN(headingTargetDeg))
                .append(",\"closedLoop\":").append(!Double.isNaN(headingTargetDeg)).append('}')
                .append(",\"pose\":{\"ok\":true,\"x\":").append(f(poseX))
                .append(",\"y\":").append(f(poseY)).append(",\"vx\":0,\"vy\":0}")
                .append(",\"cmd\":{\"f\":").append(f(cmdF))
                .append(",\"s\":").append(f(cmdS))
                .append(",\"t\":").append(f(cmdT)).append('}')
                .append(",\"rec\":{\"recording\":").append(recorder.recording())
                .append(",\"runId\":").append(recorder.runId())
                .append(",\"samples\":").append(recorder.count())
                .append(",\"overflowed\":").append(recorder.overflowed())
                .append(",\"label\":\"").append(esc(recorder.label())).append("\"}");

        sb.append(extraJson);
        sb.append(",\"pods\":[");
        for (int i = 0; i < POD_COUNT; i++) {
            CoaxialPod pod = pods == null ? null : pods[i];
            if (i > 0) {
                sb.append(',');
            }
            sb.append("{\"i\":").append(i)
                    .append(",\"label\":\"ss").append(i).append('"')
                    .append(",\"kp\":").append(f(Constants.turnKPPerPod[i]))
                    .append(",\"kd\":").append(f(Constants.turnKDPerPod[i]))
                    .append(",\"ks\":").append(f(Constants.turnKSPerPod[i]))
                    .append(",\"ksband\":")
                    .append(f(Constants.turnKSBandDegPerPod[i]))
                    .append(",\"wheelDeg\":").append(f(pod == null ? Double.NaN : wheelDeg[i]))
                    .append(",\"tgtDeg\":").append(f(pod == null ? Double.NaN : cmdTargetDeg[i]))
                    .append(",\"servoPower\":")
                    .append(f(pod == null ? Double.NaN : pod.getLastTurnPower()))
                    .append(",\"drivePower\":")
                    .append(f(pod == null ? Double.NaN : pod.getLastDrivePower()))
                    .append('}');
        }
        sb.append("],\"errors\":[],\"notes\":[]}");
        SwerveBench.INSTANCE.publish(sb.toString());
    }

    private static double norm2pi(double radians) {
        double a = radians % (2 * Math.PI);
        return a < 0 ? a + 2 * Math.PI : a;
    }

    private static String f(double v) {
        if (Double.isNaN(v) || Double.isInfinite(v)) {
            return "0";
        }
        long scaled = (long) (Math.abs(v) * 10000.0 + 0.5);
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
        StringBuilder out = new StringBuilder(s.length() + 8);
        for (int i = 0; i < s.length(); i++) {
            char c = s.charAt(i);
            if (c == '"' || c == '\\') {
                out.append('\\').append(c);
            } else if (c >= ' ') {
                out.append(c);
            } else {
                out.append(' ');
            }
        }
        return out.toString();
    }
}
