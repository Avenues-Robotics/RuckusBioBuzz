package org.firstinspires.ftc.teamcode.diagnostics.swerve;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.drivetrains.CoaxialPod;
import com.pedropathing.revhub.drivetrains.Swerve;
import com.pedropathing.revhub.drivetrains.SwerveConfig;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.SystemConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.tele.TeleOp;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.Locale;
import java.util.Map;

/**
 * The competition TeleOp's drive code, driven from the laptop and recorded.
 *
 * <p><b>DIAGNOSTIC - never ships.</b> Every drivetrain number before 2026-11-17 came from the bring-up
 * tool's own DRIVE / FOLLOW paths, and the tune accepted there was judged bad by the driver in TeleOp. This
 * OpMode IS {@link TeleOp}: it extends it and only swaps where the drive sticks come from
 * ({@link TeleOp#driveSticks()}) - the heading hold, shooter-lock stage, field-centric transform,
 * {@code follower.manual}, the vendored Swerve mixer and the pods are TeleOp's own code. Differences, all
 * deliberate:
 * <ul>
 *   <li>Sticks come from host {@code sticks f l t} commands (TeleOp's convention: forward, left, CCW turn
 *       positive), with a 400 ms watchdog to zero. Driver 1's real gamepad still owns relocalize / re-zero.</li>
 *   <li>Mechanisms (shooter, intake, scoopula) and driver 2 are OFF - no flywheel spins in a drive test. The
 *       loop therefore lacks their hardware writes and is NOT the competition loop rate.</li>
 *   <li>Stop area: no motion until {@code arm hx hy} (a field-axis rectangle centred on the pose at arm time).
 *       Near an edge, translation that points out is ZEROED - a stop, never a taper (margin 4 in +
 *       0.30 s x outward speed, the bring-up fence's). Zeroed samples show as cmd 0 in the recording; scorers
 *       drop them. Turning is never blocked.</li>
 *   <li>The recorder / {@code /swerve/state} of {@link TeleLoopProbe} (mode DRIVE = 9, so host scorers work).</li>
 *   <li>Live A/B setters for the steering and TeleOp tunables; {@code reset} restores the shipped values.</li>
 * </ul>
 */
public class TeleDriveProbe extends TeleOp {

    private static final double WATCHDOG_S = 0.40;
    private static final double MARGIN_IN = 4.0;
    private static final double MARGIN_S_PER_IN_S = 0.30;
    private static final double VEL_TAU_S = 0.05;

    private final Gamepad scripted = new Gamepad();
    private final TeleLoopProbe probe = new TeleLoopProbe();

    private volatile double stickF, stickL, stickT;
    private long sticksAtNs;

    private boolean armed;
    private double minX, maxX, minY, maxY;
    private int guardStops;
    private boolean guardActive;

    private long lastNs;
    private double loopHzEma = Double.NaN;
    private double prevX = Double.NaN, prevY = Double.NaN, prevH = Double.NaN;
    private double vx, vy, omega;

    /** Shipped values captured at init, for {@code reset}. */
    private double[] schedShipped, schedHighShipped, ffShipped;
    private double slewShipped, rolloutShipped;
    private boolean taperShipped;
    private SwerveConfig.ZeroPowerBehavior zeroShipped;
    private final Map<String, Object> teleShipped = new java.util.HashMap<>();

    @Override
    protected Gamepad driveSticks() {
        return scripted;
    }

    @Override
    protected boolean mechanismsEnabled() {
        return false;
    }

    @Override
    public void init() {
        super.init();
        schedShipped = CoaxialPod.getScheduleTuning();
        schedHighShipped = CoaxialPod.getScheduleHighSpeed();
        ffShipped = CoaxialPod.getDemandRateFeedforward();
        slewShipped = Swerve.getDemandSlewDegPerSec();
        rolloutShipped = Swerve.getSmallTurnRolloutS();
        taperShipped = Swerve.getEpsilonTaper();
        zeroShipped = Constants.swerveConfig.zeroPowerBehavior.get();
        for (Field f : SystemConstants.TeleOp.class.getFields()) {
            if (Modifier.isStatic(f.getModifiers()) && !Modifier.isFinal(f.getModifiers())) {
                try {
                    teleShipped.put(f.getName(), f.get(null));
                } catch (IllegalAccessException ignored) {
                    // public field: cannot happen
                }
            }
        }
        probe.setSource("TeleDriveProbe");
        probe.init(hardwareMap);
        probe.setCommandHandler(this::handle);
    }

    @Override
    public void loop() {
        long now = System.nanoTime();
        double dt = lastNs == 0 ? 0 : (now - lastNs) / 1e9;
        lastNs = now;
        if (dt > 0) {
            loopHzEma = Double.isNaN(loopHzEma) ? 1 / dt : loopHzEma + 0.1 * (1 / dt - loopHzEma);
        }
        Follower follower = drivetrainFollower();
        Pose pose = follower.pose();
        estimateVelocity(pose, dt);

        probe.drainCommands();
        double f = stickF, l = stickL, t = stickT;
        if (!armed || (now - sticksAtNs) / 1e9 > WATCHDOG_S) {
            f = 0;
            l = 0;
            t = 0;
        }
        boolean blocked = blocksTranslation(pose, f, l);
        if (blocked && !guardActive) {
            guardStops++;
            probe.setMessage(String.format(Locale.US, "STOP AREA: translation zeroed at (%.1f, %.1f)",
                    pose.x(), pose.y()));
        }
        guardActive = blocked;
        if (blocked) {
            f = 0;
            l = 0;
        }
        // TeleOp's convention: forward = -left_stick_y, left = -left_stick_x, turn = -right_stick_x.
        scripted.left_stick_y = (float) -f;
        scripted.left_stick_x = (float) -l;
        scripted.right_stick_x = (float) -t;

        super.loop();

        double[] cmd = lastDriveCommand();
        double h = pose.heading();
        double c = Math.cos(h), s = Math.sin(h);
        double[] drive = new double[PodRecorder.DRIVE_COLS];
        for (int i = 0; i < 4; i++) {
            CoaxialPod pod = Constants.builtPods[i];
            drive[i] = pod == null ? Double.NaN : pod.getLastDrivePower();
        }
        drive[4] = cmd[0];
        drive[5] = cmd[1];
        drive[6] = cmd[2];
        drive[7] = vx * c + vy * s;      // body forward, in/s
        drive[8] = -vx * s + vy * c;     // body left, in/s
        drive[9] = omega;                // rad/s
        double hold = headingHoldTargetRad();
        probe.setExtraJson(extraJson());
        probe.update(dt, loopHzEma, Math.toDegrees(h), Double.isNaN(hold) ? Double.NaN : Math.toDegrees(hold),
                pose.x(), pose.y(), cmd[0], cmd[1], cmd[2], drive);
        telemetry.addData("probe", probe.summary());
        telemetry.addData("probe rec", probe.recStatus());
        telemetry.addData("stop area", armed ? String.format(Locale.US, "x %.1f..%.1f y %.1f..%.1f, stops %d",
                minX, maxX, minY, maxY, guardStops) : "NOT ARMED - no motion");
    }

    @Override
    public void stop() {
        stickF = stickL = stickT = 0;
        armed = false;
        restoreShipped();
        super.stop();
        SwerveBench.INSTANCE.markStopped();
    }

    private void estimateVelocity(Pose pose, double dt) {
        if (dt > 0 && !Double.isNaN(prevX)) {
            double a = Math.min(1.0, dt / VEL_TAU_S);
            vx += a * ((pose.x() - prevX) / dt - vx);
            vy += a * ((pose.y() - prevY) / dt - vy);
            double dh = Math.atan2(Math.sin(pose.heading() - prevH), Math.cos(pose.heading() - prevH));
            omega += a * (dh / dt - omega);
        }
        prevX = pose.x();
        prevY = pose.y();
        prevH = pose.heading();
    }

    /** True when translation would carry the robot out of the stop area. */
    private boolean blocksTranslation(Pose p, double f, double l) {
        if (!armed || Math.hypot(f, l) < 1e-3) {
            return false;
        }
        double fx, fy;
        if (SystemConstants.TeleOp.FIELD_CENTRIC) {
            // The drivers' frame depends on TeleOp's private re-zero: be conservative - any translation near an edge.
            fx = Double.NaN;
            fy = Double.NaN;
        } else {
            double h = p.heading();
            fx = f * Math.cos(h) - l * Math.sin(h);
            fy = f * Math.sin(h) + l * Math.cos(h);
        }
        return outward(p.x(), vx, fx, minX, maxX) || outward(p.y(), vy, fy, minY, maxY);
    }

    private static boolean outward(double pos, double vel, double dir, double lo, double hi) {
        double mHi = MARGIN_IN + MARGIN_S_PER_IN_S * Math.max(0, vel);
        double mLo = MARGIN_IN + MARGIN_S_PER_IN_S * Math.max(0, -vel);
        boolean unknown = Double.isNaN(dir);
        if (pos > hi - mHi && (unknown || dir > 0.02)) {
            return true;
        }
        return pos < lo + mLo && (unknown || dir < -0.02);
    }

    // ---- host commands -----------------------------------------------------------------------------------

    private boolean handle(String action, Map<String, String> cmd) {
        try {
            switch (action) {
                case "sticks":
                    stickF = clamp1(num(cmd, "f", 0));
                    stickL = clamp1(num(cmd, "l", 0));
                    stickT = clamp1(num(cmd, "t", 0));
                    sticksAtNs = System.nanoTime();
                    return true;
                case "stop":
                    stickF = stickL = stickT = 0;
                    probe.setMessage("Stopped.");
                    return true;
                case "arm": {
                    double hx = num(cmd, "hx", Double.NaN), hy = num(cmd, "hy", Double.NaN);
                    if (!(hx > 6 && hy > 6 && hx < 80 && hy < 80)) {
                        probe.setMessage("REFUSED arm: hx, hy must be 6..80 in (half extents of the centre's travel)");
                        return true;
                    }
                    Pose p = drivetrainFollower().pose();
                    minX = p.x() - hx;
                    maxX = p.x() + hx;
                    minY = p.y() - hy;
                    maxY = p.y() + hy;
                    armed = true;
                    guardStops = 0;
                    probe.setMessage(String.format(Locale.US, "Stop area armed around (%.1f, %.1f): +/-%.1f x +/-%.1f in",
                            p.x(), p.y(), hx, hy));
                    return true;
                }
                case "disarm":
                    armed = false;
                    stickF = stickL = stickT = 0;
                    probe.setMessage("Stop area disarmed - no motion.");
                    return true;
                case "setTele":
                    probe.setMessage(setTele(cmd.get("name"), cmd.get("value")));
                    return true;
                case "podGains":
                    probe.setMessage(podGains(cmd));
                    return true;
                case "pulse":
                    probe.setMessage(pulse(cmd));
                    return true;
                case "sched":
                    CoaxialPod.setScheduleTuning(num(cmd, "floor", Double.NaN),
                            Math.toRadians(num(cmd, "velStartDeg", Double.NaN)), Math.toRadians(num(cmd, "gateDeg", Double.NaN)));
                    CoaxialPod.setScheduleRamp(num(cmd, "ramp", Double.NaN));
                    CoaxialPod.setScheduleHighSpeed(num(cmd, "floorHigh", Double.NaN), num(cmd, "highStart", Double.NaN),
                            num(cmd, "highFull", Double.NaN));
                    probe.setMessage("sched set: " + tuneJson());
                    return true;
                case "ff":
                    CoaxialPod.setDemandRateFeedforward(num(cmd, "kv", Double.NaN), num(cmd, "tau", Double.NaN),
                            Math.toRadians(num(cmd, "maxDeg", Double.NaN)));
                    probe.setMessage("ff set: " + tuneJson());
                    return true;
                case "mixer": {
                    double slew = num(cmd, "slew", Double.NaN), roll = num(cmd, "rollout", Double.NaN);
                    if (!Double.isNaN(slew)) Swerve.setDemandSlewDegPerSec(slew);
                    if (!Double.isNaN(roll)) Swerve.setSmallTurnRolloutS(roll);
                    if (cmd.containsKey("taper")) Swerve.setEpsilonTaper(Boolean.parseBoolean(cmd.get("taper")));
                    if (cmd.containsKey("xlock")) {
                        // Constants.swerveConfig is the live config the Swerve reads every call. Static: reset/stop restore it.
                        Constants.swerveConfig.zeroPowerBehavior.set(Boolean.parseBoolean(cmd.get("xlock"))
                                ? SwerveConfig.ZeroPowerBehavior.X_LOCK : SwerveConfig.ZeroPowerBehavior.IGNORE_ANGLE_CHANGES);
                    }
                    probe.setMessage("mixer set: " + tuneJson());
                    return true;
                }
                case "reset":
                    restoreShipped();
                    probe.setMessage("Reset to shipped values.");
                    return true;
                default:
                    return false;
            }
        } catch (RuntimeException e) {
            probe.setMessage("REFUSED " + action + ": " + e);
            return true;
        }
    }

    private String setTele(String name, String value) {
        if (name == null || value == null) {
            return "REFUSED setTele: name=, value=";
        }
        try {
            Field f = SystemConstants.TeleOp.class.getField(name);
            if (!Modifier.isStatic(f.getModifiers()) || Modifier.isFinal(f.getModifiers())) {
                return "REFUSED setTele: " + name + " is not a tunable static";
            }
            Class<?> type = f.getType();
            if (type == double.class) f.setDouble(null, Double.parseDouble(value));
            else if (type == boolean.class) f.setBoolean(null, Boolean.parseBoolean(value));
            else if (type == int.class) f.setInt(null, Integer.parseInt(value));
            else return "REFUSED setTele: " + name + " has type " + type.getSimpleName();
            return "TeleOp." + name + " = " + f.get(null);
        } catch (NoSuchFieldException e) {
            return "REFUSED setTele: no field SystemConstants.TeleOp." + name;
        } catch (IllegalAccessException e) {
            return "REFUSED setTele: " + e;
        }
    }

    private String podGains(Map<String, String> cmd) {
        int only = (int) num(cmd, "pod", -1);
        for (int i = 0; i < 4; i++) {
            CoaxialPod pod = Constants.builtPods[i];
            if (pod == null || (only >= 0 && only != i)) {
                continue;
            }
            PIDFController c = pod.getTurnPIDF();
            PIDFCoefficients k = c.getCoefficients();
            double kp = num(cmd, "kp", k.P), kd = num(cmd, "kd", k.D);
            c.setCoefficients(new PIDFCoefficients(kp, 0, kd, 0));
            double ks = num(cmd, "ks", pod.getStaticFrictionPower());
            double band = cmd.containsKey("ksband") ? Math.toRadians(num(cmd, "ksband", 0)) : pod.getStaticFrictionBandRad();
            pod.setStaticFriction(ks, band);
        }
        return "podGains set: " + tuneJson();
    }

    private String pulse(Map<String, String> cmd) {
        boolean on = !cmd.containsKey("on") || Boolean.parseBoolean(cmd.get("on"));
        double band = num(cmd, "bandDeg", Constants.pulseBandDeg), tol = num(cmd, "tolDeg", Constants.pulseToleranceDeg);
        double pw = num(cmd, "power", Constants.pulsePower), sec = num(cmd, "sec", Constants.pulseSeconds);
        for (CoaxialPod pod : Constants.builtPods) {
            if (pod != null) {
                pod.setPulsedApproach(on, Math.toRadians(band), Math.toRadians(tol), pw, sec,
                        Math.toRadians(Constants.pulseStationaryDegPerSec), Constants.pulseCoastSeconds);
            }
        }
        return String.format(Locale.US, "pulse %s band %.1f tol %.1f power %.3f sec %.3f", on ? "ON" : "OFF", band, tol, pw, sec);
    }

    /** Everything back to the build's values: pod gains + pulse from Constants, statics as captured at init. */
    private void restoreShipped() {
        for (int i = 0; i < 4; i++) {
            CoaxialPod pod = Constants.builtPods[i];
            if (pod == null) {
                continue;
            }
            pod.getTurnPIDF().setCoefficients(new PIDFCoefficients(Constants.turnKPPerPod[i], 0, Constants.turnKDPerPod[i], 0));
            pod.setStaticFriction(Constants.turnKSPerPod[i], Math.toRadians(Constants.turnKSBandDegPerPod[i]));
            pod.setPulsedApproach(true, Math.toRadians(Constants.pulseBandDeg), Math.toRadians(Constants.pulseToleranceDeg),
                    Constants.pulsePower, Constants.pulseSeconds, Math.toRadians(Constants.pulseStationaryDegPerSec),
                    Constants.pulseCoastSeconds);
        }
        if (schedShipped != null) {
            CoaxialPod.setScheduleTuning(schedShipped[0], schedShipped[1], schedShipped[2]);
            CoaxialPod.setScheduleHighSpeed(schedHighShipped[0], schedHighShipped[1], schedHighShipped[2]);
            CoaxialPod.setDemandRateFeedforward(ffShipped[0], ffShipped[1], ffShipped[2]);
            Swerve.setDemandSlewDegPerSec(slewShipped);
            Swerve.setSmallTurnRolloutS(rolloutShipped);
            Swerve.setEpsilonTaper(taperShipped);
            Constants.swerveConfig.zeroPowerBehavior.set(zeroShipped);
        }
        for (Map.Entry<String, Object> e : teleShipped.entrySet()) {
            try {
                SystemConstants.TeleOp.class.getField(e.getKey()).set(null, e.getValue());
            } catch (ReflectiveOperationException ignored) {
                // captured from the same class at init
            }
        }
    }

    private String extraJson() {
        StringBuilder sb = new StringBuilder(512);
        sb.append(",\"box\":{\"valid\":").append(armed)
                .append(",\"minX\":").append(r(minX)).append(",\"maxX\":").append(r(maxX))
                .append(",\"minY\":").append(r(minY)).append(",\"maxY\":").append(r(maxY))
                .append(",\"kind\":\"probeStopArea\",\"stops\":").append(guardStops)
                .append(",\"clamped\":").append(guardActive).append('}')
                .append(",\"vel\":{\"vx\":").append(r(vx)).append(",\"vy\":").append(r(vy))
                .append(",\"omegaDeg\":").append(r(Math.toDegrees(omega))).append('}')
                .append(",\"sticks\":{\"f\":").append(r(stickF)).append(",\"l\":").append(r(stickL))
                .append(",\"t\":").append(r(stickT)).append(",\"ageS\":")
                .append(r(sticksAtNs == 0 ? -1 : (System.nanoTime() - sticksAtNs) / 1e9)).append('}')
                .append(",\"tune\":").append(tuneJson());
        return sb.toString();
    }

    private String tuneJson() {
        double[] sc = CoaxialPod.getScheduleTuning(), hi = CoaxialPod.getScheduleHighSpeed(), ff = CoaxialPod.getDemandRateFeedforward();
        StringBuilder sb = new StringBuilder(256);
        sb.append("{\"floor\":").append(r(sc[0])).append(",\"velStartDeg\":").append(r(Math.toDegrees(sc[1])))
                .append(",\"gateDeg\":").append(r(Math.toDegrees(sc[2])))
                .append(",\"floorHigh\":").append(r(hi[0])).append(",\"highStart\":").append(r(hi[1]))
                .append(",\"highFull\":").append(r(hi[2]))
                .append(",\"ffKv\":").append(r(ff[0])).append(",\"ffTau\":").append(r(ff[1]))
                .append(",\"ffMaxDeg\":").append(r(Math.toDegrees(ff[2])))
                .append(",\"slew\":").append(r(Swerve.getDemandSlewDegPerSec()))
                .append(",\"taper\":").append(Swerve.getEpsilonTaper())
                .append(",\"xlock\":").append(Constants.swerveConfig.zeroPowerBehavior.get() == SwerveConfig.ZeroPowerBehavior.X_LOCK)
                .append(",\"rollout\":").append(r(Swerve.getSmallTurnRolloutS()))
                .append(",\"hold\":").append(SystemConstants.TeleOp.HEADING_HOLD)
                .append(",\"holdKp\":").append(r(SystemConstants.TeleOp.HOLD_KP))
                .append(",\"translationPriority\":").append(r(SystemConstants.TeleOp.TRANSLATION_PRIORITY))
                .append(",\"pods\":[");
        for (int i = 0; i < 4; i++) {
            CoaxialPod pod = Constants.builtPods[i];
            if (i > 0) sb.append(',');
            if (pod == null) {
                sb.append("null");
                continue;
            }
            PIDFCoefficients k = pod.getTurnPIDF().getCoefficients();
            sb.append("{\"kp\":").append(r(k.P)).append(",\"kd\":").append(r(k.D))
                    .append(",\"ks\":").append(r(pod.getStaticFrictionPower()))
                    .append(",\"ksband\":").append(r(Math.toDegrees(pod.getStaticFrictionBandRad()))).append('}');
        }
        return sb.append("]}").toString();
    }

    private static double num(Map<String, String> cmd, String key, double dflt) {
        String v = cmd.get(key);
        return v == null || v.isEmpty() ? dflt : Double.parseDouble(v);
    }

    private static double clamp1(double v) {
        return Double.isNaN(v) ? 0 : Math.max(-1, Math.min(1, v));
    }

    /** Hand-rolled: String.format cost 61 us/call in publish (FINDINGS 2026-08-16). */
    private static String r(double v) {
        if (Double.isNaN(v) || Double.isInfinite(v)) {
            return "null";
        }
        return Double.toString(Math.round(v * 1000.0) / 1000.0);
    }
}
