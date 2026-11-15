package org.firstinspires.ftc.teamcode.diagnostics.swerve;

import java.util.Locale;

/**
 * Loop-rate recorder for all four pods at once.
 *
 * <p>The dashboard's {@code trace} covers one pod and 260 samples, which is enough to eyeball a
 * chart and not enough to score one. The acceptance criteria for pod rotation are written in
 * milliseconds and tenths of a degree - a 200 ms settling time sampled over HTTP at 20 Hz is four
 * points - and {@code pidStepAll} is only trustworthy if all four pods are measured, not just the
 * selected one. So this records every pod every loop into a ring buffer, and renders it as CSV for
 * a host-side scorer to pull once the run is over.
 *
 * <h2>Threading</h2>
 *
 * <p>{@link #add} is called only from the OpMode loop. {@link #toCsv()} is called only from a web
 * worker thread, and only once {@link #recording()} is false - at which point the buffer is no
 * longer written, so the read needs no lock. {@code count} and {@code recording} are volatile so
 * the web thread sees the final values.
 */
public class PodRecorder {

    /**
     * Columns shared by the whole robot. Grew 7 -> 14 on 2026-08-14 for driver-session
     * recording: heading and its hold target, field pose, and the drive command actually
     * applied - so a wobble or drift episode in a human session can be lined up against what
     * the sticks were doing at loop rate, in one synchronized stream.
     */
    private static final int GLOBAL_COLS = 14;

    /**
     * Columns recorded per pod. Grew 6 -> 7 on 2026-08-16: {@code ctgt}, the demand read back out
     * of the pod itself ({@code CoaxialPod.getLastTargetWheelRad}). {@code tgt} is a host-side
     * mirror of the mixer and mirrors drift - it disagreed with the real demand on 4.0% of the
     * samples in mydrive-001. Both are kept: {@code tgt} so the whole archive stays comparable,
     * {@code ctgt} because it cannot be wrong.
     */
    private static final int POD_COLS = 7;

    private static final int POD_COUNT = 4;

    /**
     * Follower columns, appended AFTER the pod block on 2026-10-01 so every existing column keeps
     * its position: {@code fterr} (Foresight translational error, in), {@code fherr} (heading
     * error, deg), {@code fcx,fcy,fch} (closest point on the path and its target heading),
     * {@code fvel} (target velocity, in/s), {@code fbusy}. Empty outside FOLLOW.
     */
    public static final int FOLLOW_COLS = 7;

    /**
     * Drive columns, appended AFTER the follower block on 2026-11-15 (jolt diagnosis) so every existing
     * column keeps its position: {@code p0..p3_dpwr} (drive-motor power each pod was commanded - times
     * {@code volts} it is the motor voltage), {@code dcf,dcs,dct} (the forward / strafe / turn command the
     * drivetrain was handed - the follower's or a Foresight stage's DrivePowers, else the tool's own mixer),
     * {@code bvx,bvy} (body-frame velocity, in/s) and {@code bw} (yaw rate, rad/s) from the bring-up's pose.
     */
    public static final int DRIVE_COLS = 10;

    private static final int COLS = GLOBAL_COLS + POD_COUNT * POD_COLS + FOLLOW_COLS + DRIVE_COLS;

    /**
     * Roughly 30 s at 100 Hz. Floats rather than doubles: the widest value recorded is an angle in
     * degrees, where a float still resolves better than 0.001 - far finer than the encoder's own
     * 0.11 degree read granularity - and it halves a buffer that has to live on the hub.
     */
    private static final int CAPACITY = 3000;

    private final float[] data = new float[CAPACITY * COLS];

    private volatile int count;
    private volatile boolean recording;
    private volatile int runId;
    private volatile String label = "";

    /** Wall-clock reference for the {@code t} column, so t starts at 0 for each run. */
    private long startNano;

    /** Set when the buffer filled up, so a truncated run is never mistaken for a complete one. */
    private volatile boolean overflowed;

    /** Begins a new run, discarding whatever was recorded before. */
    public void start(String runLabel) {
        recording = false;      // stop writes before resetting, in case a run was already active
        count = 0;
        overflowed = false;
        label = runLabel == null ? "" : runLabel;
        runId++;
        startNano = System.nanoTime();
        recording = true;
    }

    public void stop() {
        recording = false;
    }

    public boolean recording() {
        return recording;
    }

    public int count() {
        return count;
    }

    public int runId() {
        return runId;
    }

    public String label() {
        return label;
    }

    public boolean overflowed() {
        return overflowed;
    }

    /**
     * Appends one sample. Called from the OpMode loop only.
     *
     * <p>Recording stops rather than wrapping when the buffer fills. A step response is scored from
     * its beginning, so keeping the oldest samples and dropping the newest is the useful choice; a
     * ring that overwrote the step onset would silently ruin the run.
     *
     * @param dt seconds since the previous loop
     * @param volts battery voltage
     * @param loopHz smoothed loop rate
     * @param mode ordinal of the OpMode's current mode
     * @param batteryMa hub battery-side current, milliamps
     * @param servoMa hub servo rail current, milliamps; sampled slower than the loop
     * @param podVolts raw encoder voltage per pod
     * @param wheelDeg measured wheel heading per pod, degrees
     * @param targetDeg commanded wheel heading per pod, degrees; NaN when nothing is commanded
     * @param cmdTargetDeg the demand read back out of the pod, degrees; NaN when it never moved
     * @param errDeg signed post-flip pod error per pod, degrees; NaN when the loop is not closed
     * @param power turn servo power actually written per pod
     * @param flipped whether the pod took the 180 degree flip this loop
     * @param headingDeg robot heading, degrees; NaN when the Pinpoint is unreadable
     * @param headingTgtDeg heading-hold target, degrees; NaN when hold is off
     * @param poseX field x, inches; NaN without a pose
     * @param poseY field y, inches; NaN without a pose
     * @param cmdF forward command applied this loop
     * @param cmdS strafe command applied this loop
     * @param cmdT rotation actually applied (heading-hold output, not the raw stick)
     * @param follow the {@link #FOLLOW_COLS} follower values, or null outside FOLLOW
     * @param drive the {@link #DRIVE_COLS} drive values, or null
     */
    public void add(double dt, double volts, double loopHz, int mode, double servoMa,
            double batteryMa, double[] podVolts, double[] wheelDeg, double[] targetDeg,
            double[] cmdTargetDeg, double[] errDeg, double[] power, boolean[] flipped,
            double headingDeg, double headingTgtDeg, double poseX, double poseY,
            double cmdF, double cmdS, double cmdT, double[] follow, double[] drive) {
        if (!recording) {
            return;
        }
        if (count >= CAPACITY) {
            overflowed = true;
            recording = false;
            return;
        }

        int at = count * COLS;
        data[at] = (float) ((System.nanoTime() - startNano) / 1e9);
        data[at + 1] = (float) dt;
        data[at + 2] = (float) volts;
        data[at + 3] = (float) loopHz;
        data[at + 4] = mode;
        data[at + 5] = (float) servoMa;
        data[at + 6] = (float) batteryMa;
        data[at + 7] = (float) headingDeg;
        data[at + 8] = (float) headingTgtDeg;
        data[at + 9] = (float) poseX;
        data[at + 10] = (float) poseY;
        data[at + 11] = (float) cmdF;
        data[at + 12] = (float) cmdS;
        data[at + 13] = (float) cmdT;

        for (int i = 0; i < POD_COUNT; i++) {
            int p = at + GLOBAL_COLS + i * POD_COLS;
            data[p] = (float) podVolts[i];
            data[p + 1] = (float) wheelDeg[i];
            data[p + 2] = (float) targetDeg[i];
            data[p + 3] = (float) errDeg[i];
            data[p + 4] = (float) power[i];
            data[p + 5] = flipped[i] ? 1 : 0;
            data[p + 6] = (float) cmdTargetDeg[i];
        }
        int f = at + GLOBAL_COLS + POD_COUNT * POD_COLS;
        for (int k = 0; k < FOLLOW_COLS; k++) {
            data[f + k] = follow == null ? Float.NaN : (float) follow[k];
        }
        int d = f + FOLLOW_COLS;
        for (int k = 0; k < DRIVE_COLS; k++) {
            data[d + k] = drive == null ? Float.NaN : (float) drive[k];
        }

        count++;
    }

    /**
     * Renders the run as CSV, with a comment header carrying the run metadata.
     *
     * <p>Safe to call from a web thread once recording has stopped. Calling it mid-run returns
     * whatever had been written when the sample count was read, which is consistent but partial.
     */
    public String toCsv() {
        int rows = count;
        StringBuilder sb = new StringBuilder(rows * COLS * 8 + 512);

        sb.append("# runId=").append(runId)
                .append(" label=").append(label.replace('\n', ' '))
                .append(" samples=").append(rows)
                .append(" overflowed=").append(overflowed)
                .append(" recording=").append(recording)
                .append('\n');

        sb.append("t,dt,volts,loopHz,mode,servoMa,batteryMa,heading,htgt,px,py,cf,cs,ct");
        for (int i = 0; i < POD_COUNT; i++) {
            sb.append(",p").append(i).append("_v")
                    .append(",p").append(i).append("_wheel")
                    .append(",p").append(i).append("_tgt")
                    .append(",p").append(i).append("_err")
                    .append(",p").append(i).append("_pwr")
                    .append(",p").append(i).append("_flip")
                    .append(",p").append(i).append("_ctgt");
        }
        sb.append(",fterr,fherr,fcx,fcy,fch,fvel,fbusy");
        sb.append(",p0_dpwr,p1_dpwr,p2_dpwr,p3_dpwr,dcf,dcs,dct,bvx,bvy,bw");
        sb.append('\n');

        for (int r = 0; r < rows; r++) {
            int at = r * COLS;
            for (int c = 0; c < COLS; c++) {
                if (c > 0) {
                    sb.append(',');
                }
                float v = data[at + c];
                if (Float.isNaN(v)) {
                    // Left empty rather than written as "NaN": pandas and numpy both read an empty
                    // field as NaN, and "NaN" as a string in some locales does not round-trip.
                    continue;
                }
                sb.append(fmt(v));
            }
            sb.append('\n');
        }

        return sb.toString();
    }

    /** Six significant figures, trailing zeros trimmed - enough for angles, small enough to move. */
    private static String fmt(float v) {
        if (v == (long) v && Math.abs(v) < 1e7) {
            return Long.toString((long) v);
        }
        String s = String.format(Locale.US, "%.6g", v);
        if (s.indexOf('.') >= 0 && s.indexOf('e') < 0 && s.indexOf('E') < 0) {
            int end = s.length();
            while (end > 0 && s.charAt(end - 1) == '0') {
                end--;
            }
            if (end > 0 && s.charAt(end - 1) == '.') {
                end--;
            }
            s = s.substring(0, end);
        }
        return s;
    }
}
