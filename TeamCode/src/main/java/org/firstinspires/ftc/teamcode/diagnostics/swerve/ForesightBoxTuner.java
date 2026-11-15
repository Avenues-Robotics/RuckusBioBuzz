package org.firstinspires.ftc.teamcode.diagnostics.swerve;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;
import com.pedropathing.utils.Utils;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * DIAGNOSTIC ONLY - never ships. Pedro 3's Foresight Tuner, ported into Swerve Bring-Up (FORESIGHT mode)
 * so it runs inside the safe-area fence on a 2x2 of tiles.
 *
 * <p>Source: {@code pedroPathing/procedures/ForesightTuner.java} as of {@code cf5dcee9^} (deleted with Sloth on
 * 2026-11-01). Every stage keeps that tuner's drive commands and its fits - {@link Utils#linearFit} /
 * {@link Utils#quadraticFit} are the same vendored functions it called - and result names, so the host
 * output imports with {@code robot.py foresight --partial}. What changed, and only this:
 * <ul>
 *   <li><b>Room.</b> The tuner assumed 36-48 in of open floor. Each linear stage here drives toward
 *       whichever way along its axis has more room inside the fence, and ends its drive early - on room, not
 *       on a fixed distance - so the coast or brake that follows stops short of the fence (zero command
 *       cannot be clamped: the fence cannot stop a coasting robot). DECEL and TRANSLATIONAL lower their
 *       speed / power up front when the box is too short for the tuner's defaults, and say so.</li>
 *   <li><b>Heading hold in BRAKE</b> clamps symmetrically, +/-0.3 before the /2; the tuner clamped
 *       [-0.3, +1.0], which looks like a typo and only weakens the hold one way.</li>
 *   <li>Max velocity (full power over 48 in) is NOT here - it needs the long strip. TRANSLATIONAL reports
 *       its K (in/s per unit power) as an extrapolated estimate under a name nothing imports.</li>
 * </ul>
 */
final class ForesightBoxTuner {

    /** What the stages need from the OpMode. Body velocities are the localizer's twist (robot frame). */
    interface Io {
        void drive(DrivePowers powers);

        /** Zero power, FLOAT, pods released: a coast (the tuner's drivetrain.stop(false)). */
        void coast();

        Pose pose();

        double forwardVel();

        double strafeVel();

        double omega();

        /** Inches the robot centre can travel from here along field unit vector (fx, fy) before the fence. */
        double room(double fx, double fy);

        double headingKpDefault();

        double headingBrakeLinearDefault();

        double headingBrakeQuadraticDefault();
    }

    enum Stage { HEADING_ID, HEADING_BRAKE, DECEL, BRAKE, TRANSLATIONAL }

    // Planning guesses - deliberately pessimistic, only used to decide where to stop driving.
    private static final double MARGIN_IN = 3.0;
    private static final double ACCEL_GUESS = 60.0;      // in/s^2 at full power (low = plans a longer run-up)
    private static final double DECEL_GUESS = 30.0;      // in/s^2 coasting (measured 40 on the old pods)

    private final Map<String, Double> results = new LinkedHashMap<>();
    private final Map<String, Double> extras = new LinkedHashMap<>();
    private final List<double[]> points = new ArrayList<>();
    private final List<Double> times = new ArrayList<>();
    private final List<Double> vels = new ArrayList<>();

    private Stage stage;
    private boolean forwardAxis = true;
    private boolean running;
    private String phase = "idle";
    private String message = "";
    private long startNs, phaseNs;
    private double param1, param2;   // stage parameters (see start)
    private int trial, trials;
    private double sign;             // +1 / -1 along the stage's axis
    private double[] powers;
    private double headingRef, totalHeading, prevHeading, brakeStartHeading, measuredV;
    private Pose brakeStartPose;
    private double prevV;
    private long prevNs;
    private final List<Double> accels = new ArrayList<>();
    private boolean cutShort;

    boolean running() {
        return running;
    }

    String message() {
        return message;
    }

    void abort(String why) {
        if (running) {
            running = false;
            phase = "aborted";
            message = stageName() + " ABORTED: " + why;
        }
    }

    /**
     * Starts a stage. {@code p1}/{@code p2} are NaN for the tuner's defaults:
     * HEADING_ID power / runtime s; HEADING_BRAKE max power / trials; DECEL target in/s / power;
     * BRAKE max power / trials; TRANSLATIONAL power / runtime s. Returns null, or why it refused.
     */
    String start(Stage s, boolean forward, double p1, double p2, Io io) {
        stage = s;
        forwardAxis = forward;
        extras.clear();
        points.clear();
        times.clear();
        vels.clear();
        accels.clear();
        cutShort = false;
        trial = 0;
        trials = 1;
        message = "";
        startNs = phaseNs = System.nanoTime();
        Pose p = io.pose();
        headingRef = p.heading();
        prevHeading = p.heading();
        totalHeading = 0;
        switch (s) {
            case HEADING_ID:
                param1 = def(p1, 0.4);
                param2 = def(p2, 1.2);
                phase = "spin";
                break;
            case HEADING_BRAKE:
                param1 = def(p1, 1.0);
                trials = (int) Math.round(def(p2, 12));
                powers = biasedGradient(trials, param1, 0.2, 1.5);
                phase = "drive";
                break;
            case DECEL: {
                double want = def(p1, 30.0);
                param2 = def(p2, 1.0);
                double room = pickDirection(io);
                double perV2 = 1 / (2 * ACCEL_GUESS) + 1 / (2 * DECEL_GUESS);
                double fit = Math.sqrt(Math.max(0, room - MARGIN_IN) / perV2);
                param1 = Math.min(want, fit);
                extras.put("roomIn", room);
                if (param1 < 10) {
                    return "only " + r1(room) + " in of room along the " + axisName()
                            + " axis - DECEL needs about 10 in/s. Re-centre the robot in the box.";
                }
                if (param1 < want) {
                    message = "DECEL target lowered " + r1(want) + " -> " + r1(param1) + " in/s to fit "
                            + r1(room) + " in of room.";
                }
                phase = "drive";
                break;
            }
            case BRAKE:
                param1 = def(p1, 0.7);
                trials = (int) Math.round(def(p2, 5));
                powers = biasedGradient(trials, param1, 0.3, 1.5);
                pickDirection(io);
                phase = "drive";
                break;
            case TRANSLATIONAL: {
                // The tuner's 0.4 / 1.2 s as given - NOT scaled down: at low power the drive sits in its
                // static-friction deadband (heading: 0.16 rad/s at 0.2 vs 2.6 at 0.4, 2026-11-15), so K from a
                // 0.2 run is not the drivetrain's. Safety is the live room cut in translational(); a run cut
                // before its plateau FAILS instead of fitting a rise.
                param1 = def(p1, 0.4);
                param2 = def(p2, 1.2);
                double room = pickDirection(io);
                extras.put("roomIn", room);
                if (room < 10) {
                    return "only " + r1(room) + " in of room along the " + axisName()
                            + " axis - start TRANSLATIONAL from one end of the box.";
                }
                phase = "drive";
                break;
            }
        }
        running = true;
        if (message.isEmpty()) {
            message = stageName() + " running.";
        }
        return null;
    }

    void update(Io io) {
        if (!running) {
            return;
        }
        long now = System.nanoTime();
        double t = (now - phaseNs) / 1e9;
        Pose p = io.pose();
        totalHeading += Angle.normalizeSigned(p.heading() - prevHeading);
        prevHeading = p.heading();
        try {
            switch (stage) {
                case HEADING_ID:
                    headingId(io, t, now);
                    break;
                case HEADING_BRAKE:
                    headingBrake(io, t, now);
                    break;
                case DECEL:
                    decel(io, t, now);
                    break;
                case BRAKE:
                    brake(io, t, now);
                    break;
                case TRANSLATIONAL:
                    translational(io, t, now);
                    break;
            }
        } catch (IllegalStateException e) {
            io.coast();
            running = false;
            phase = "failed";
            message = stageName() + " FAILED: " + e.getMessage();
        }
    }

    // ---------------------------------------------------------------- stages

    /** HeadingTuner: spin at 0.4 for 1.2 s, identify K and tau, kP = tau * alpha^2 / K (alpha 18.25). */
    private void headingId(Io io, double t, long now) {
        if ("spin".equals(phase)) {
            io.drive(new DrivePowers(0, 0, param1));
            times.add(t);
            vels.add(Math.abs(io.omega()));
            if (t >= param2) {
                double[] kt = identify(param1);
                extras.put("K", kt[0]);
                extras.put("tau", kt[1]);
                results.put("heading kP", kt[1] * 18.25 * 18.25 / kt[0]);
                next("settle", now);
            }
        } else {
            settle(io, t, Math.abs(io.omega()) < 0.05, "heading kP " + r4(results.get("heading kP")));
        }
    }

    /** HeadingBraking: spin 2 s at each power (1.0 -> 0.2), brake at -0.001, fit distance(omega). */
    private void headingBrake(Io io, double t, long now) {
        double dir = trial % 2 == 0 ? 1 : -1;
        if ("drive".equals(phase)) {
            if (t > 2) {
                brakeStartHeading = totalHeading;
                measuredV = Math.abs(io.omega());
                io.drive(new DrivePowers(0, 0, -0.001 * dir));
                next("brake", now);
                return;
            }
            io.drive(new DrivePowers(0, 0, powers[trial] * dir));
        } else if ("brake".equals(phase)) {
            if (Math.abs(io.omega()) > 0.001 && t < 3) {
                io.drive(new DrivePowers(0, 0, -0.001 * dir));
                return;
            }
            points.add(new double[] {measuredV, Math.abs(totalHeading - brakeStartHeading)});
            trial++;
            if (trial >= trials) {
                double[] c = Utils.quadraticFit(points);
                results.put("headingBrakingLinearCoefficient", c[0]);
                results.put("headingBrakingQuadraticCoefficient", c[1]);
                next("settle", now);
            } else {
                next("drive", now);
            }
        } else {
            settle(io, t, Math.abs(io.omega()) < 0.05, "heading braking " + r4(c(0)) + " / " + r4(c(1)));
        }
    }

    /** Forward/StrafeDeceleration: full power to the target speed, coast, mean of dv/dt down to 1 in/s. */
    private void decel(Io io, double t, long now) {
        double v = sign * bodyVel(io);
        if ("drive".equals(phase)) {
            double remaining = roomAhead(io);
            boolean atSpeed = v > param1;
            boolean outOfRoom = v > 0 && remaining <= v * v / (2 * DECEL_GUESS) + MARGIN_IN;
            if (atSpeed || outOfRoom) {
                cutShort = !atSpeed;
                extras.put("vCut", v);
                brakeStartPose = io.pose();
                prevV = v;
                prevNs = now;
                io.coast();
                next("coast", now);
                return;
            }
            if (t > 3) {
                throw new IllegalStateException("never reached " + r1(param1) + " in/s in 3 s (at " + r1(v) + ")");
            }
            io.drive(axisPowers(sign * param2, 0));
        } else if ("coast".equals(phase)) {
            io.coast();
            double dt = (now - prevNs) / 1e9;
            if (dt > 0) {
                accels.add((v - prevV) / dt);
            }
            prevV = v;
            prevNs = now;
            if (Math.abs(v) <= 1 || t > 5) {
                if (accels.isEmpty()) {
                    throw new IllegalStateException("no coast samples");
                }
                double sum = 0;
                for (double a : accels) {
                    sum += a;
                }
                double decel = Math.abs(sum / accels.size());
                double dist = distance(brakeStartPose, io.pose());
                extras.put("coastIn", dist);
                extras.put("decelFromDistance", extras.get("vCut") * extras.get("vCut") / (2 * Math.max(dist, 1e-6)));
                extras.put("cutShort", cutShort ? 1.0 : 0.0);
                results.put(forwardAxis ? "naturalForwardDeceleration" : "naturalStrafeDeceleration", decel);
                running = false;
                phase = "done";
                message = stageName() + ": " + r2(decel) + " in/s^2 from " + r1(extras.get("vCut")) + " in/s"
                        + (cutShort ? " (drive cut on room before the target)" : "") + ", coast " + r1(dist) + " in.";
            }
        }
    }

    /** Forward/StrafeBraking: back and forth at 0.7 -> 0.3, brake at -0.001, fit distance(speed). */
    private void brake(Io io, double t, long now) {
        double dir = trial % 2 == 0 ? sign : -sign;
        double head = headingPower(io);
        if ("drive".equals(phase)) {
            double v = dir * bodyVel(io);
            double remaining = room(io, dir);
            if ((v > 0 && remaining <= v * v / (2 * decelFor()) + MARGIN_IN) || t > 4) {
                brakeStartPose = io.pose();
                measuredV = Math.hypot(io.forwardVel(), io.strafeVel());
                io.drive(brakePowers(dir, head));
                next("brake", now);
                return;
            }
            io.drive(axisPowers(powers[trial] * dir, head));
        } else if ("brake".equals(phase)) {
            if (Math.hypot(io.forwardVel(), io.strafeVel()) > 0.25 && t < 7) {
                io.drive(brakePowers(dir, head));
                return;
            }
            points.add(new double[] {measuredV, distance(brakeStartPose, io.pose())});
            trial++;
            if (trial >= trials) {
                double[] c = Utils.quadraticFit(points);
                String ax = forwardAxis ? "forward" : "strafe";
                results.put(ax + "BrakingLinearCoefficient", c[0]);
                results.put(ax + "BrakingQuadraticCoefficient", c[1]);
                next("settle", now);
            } else {
                next("wait", now);
            }
        } else if ("wait".equals(phase)) {
            io.coast();
            if (t > 1) {
                next("drive", now);
            }
        } else {
            settle(io, t, Math.hypot(io.forwardVel(), io.strafeVel()) < 1, "braking " + r4(c(0)) + " / " + r4(c(1)));
        }
    }

    /** Forward/StrafeTranslational: 0.4 for 1.2 s (scaled to the room), identify K and tau. */
    private void translational(Io io, double t, long now) {
        double v = sign * bodyVel(io);
        if ("drive".equals(phase)) {
            io.drive(axisPowers(sign * param1, 0));
            times.add(t);
            vels.add(Math.abs(v));
            boolean outOfRoom = v > 0 && roomAhead(io) <= v * v / (2 * decelFor()) + MARGIN_IN;
            if (t >= param2 || outOfRoom) {
                cutShort = outOfRoom && t < param2;
                io.coast();
                double spread = plateauSpread();
                if (cutShort && !(Math.abs(spread) <= 0.05)) {
                    throw new IllegalStateException("cut on room at " + r2(t) + " s, before a plateau (last-15 vs"
                            + " previous-15 spread " + r2(spread) + ") - not identifiable in this room at power "
                            + r2(param1));
                }
                double[] kt = identify(param1);
                double kLarge = kt[1] * 10.2 * 10.2 / kt[0];
                double kSmall = kt[1] * 6.2 * 6.2 / kt[0];
                String ax = forwardAxis ? "forwardTranslational" : "strafeTranslational";
                results.put(ax + " Primary kP", kLarge);
                results.put(ax + " Secondary kP", kSmall);
                if (forwardAxis) {
                    results.put("coast kV", 1 / kt[0]);
                    results.put("brake kV", 0.85 / kt[0]);
                }
                extras.put("K", kt[0]);
                extras.put("tau", kt[1]);
                extras.put("power", param1);
                extras.put("runtime", t);
                extras.put("cutShort", cutShort ? 1.0 : 0.0);
                extras.put("plateauSpread", plateauSpread());
                next("settle", now);
            }
        } else {
            io.coast();
            settle(io, t, Math.abs(v) < 1, "K " + r1(extras.get("K")) + " in/s per power, tau "
                    + r2(extras.get("tau")) + " s" + (cutShort ? " (cut on room)" : ""));
        }
    }

    // ---------------------------------------------------------------- helpers

    /** The tuner's systemIdentification: K = mean of the last 15 samples / power; tau from ln(K - v). */
    private double[] identify(double power) {
        int n = vels.size();
        if (n < 4) {
            throw new IllegalStateException("only " + n + " samples");
        }
        int from = Math.max(0, n - 15);
        double sum = 0;
        for (int i = from; i < n; i++) {
            sum += vels.get(i);
        }
        double k = sum / (n - from) / power;
        List<Double> xs = new ArrayList<>(), ys = new ArrayList<>();
        for (int i = 0; i < n; i++) {
            double vel = vels.get(i) / power;
            if (vel > 0.8 * k || vel < 0.1 * k) {
                continue;
            }
            ys.add(Math.log(k - vel));
            xs.add(times.get(i));
        }
        if (xs.size() < 3) {
            throw new IllegalStateException("only " + xs.size() + " samples on the rise (loop too slow?)");
        }
        double[] fit = Utils.linearFit(xs.toArray(new Double[0]), ys.toArray(new Double[0]));
        if (fit[1] >= 0) {
            throw new IllegalStateException("rise does not decay (slope " + fit[1] + ")");
        }
        return new double[] {k, -1.0 / fit[1]};
    }

    /** Relative change between the last 15 samples and the 15 before - a plateau check for identify(). */
    private double plateauSpread() {
        int n = vels.size();
        if (n < 30) {
            return Double.NaN;
        }
        double a = 0, b = 0;
        for (int i = n - 30; i < n - 15; i++) {
            a += vels.get(i);
        }
        for (int i = n - 15; i < n; i++) {
            b += vels.get(i);
        }
        return b == 0 ? Double.NaN : (b - a) / b;
    }

    /** The tuner's getHeadingPower, holding the stage's start heading, clamped symmetrically. */
    private double headingPower(Io io) {
        double kp = results.containsKey("heading kP") ? results.get("heading kP") : io.headingKpDefault();
        double hl = results.containsKey("headingBrakingLinearCoefficient")
                ? results.get("headingBrakingLinearCoefficient") : io.headingBrakeLinearDefault();
        double hq = results.containsKey("headingBrakingQuadraticCoefficient")
                ? results.get("headingBrakingQuadraticCoefficient") : io.headingBrakeQuadraticDefault();
        double w = io.omega();
        double brakeDist = hl * w + hq * w * w * Math.signum(w);
        double error = Angle.normalizeSigned(headingRef - io.pose().heading()) - brakeDist;
        return Utils.clamp(kp * error, -0.3, 0.3) / 2;
    }

    /** The tuner's brake(): -0.001 along the axis, but never weaker than the heading command. */
    private DrivePowers brakePowers(double dir, double head) {
        double b = -0.001 * dir;
        double min = Math.abs(head) + 0.001;
        b = dir > 0 ? Math.min(b, -min) : Math.max(b, min);
        return axisPowers(b, head);
    }

    private DrivePowers axisPowers(double along, double turn) {
        return forwardAxis ? new DrivePowers(along, 0, turn) : new DrivePowers(0, along, turn);
    }

    private double bodyVel(Io io) {
        return forwardAxis ? io.forwardVel() : io.strafeVel();
    }

    /** Field unit vector of the robot's +axis at the current heading (strafe = +y body = left). */
    private double[] axisDir(Io io) {
        double h = io.pose().heading();
        return forwardAxis ? new double[] {Math.cos(h), Math.sin(h)} : new double[] {-Math.sin(h), Math.cos(h)};
    }

    private double room(Io io, double dir) {
        double[] d = axisDir(io);
        return io.room(dir * d[0], dir * d[1]);
    }

    private double roomAhead(Io io) {
        return room(io, sign);
    }

    /** Points the stage along whichever way of its axis has more room; returns that room. */
    private double pickDirection(Io io) {
        double plus = room(io, 1), minus = room(io, -1);
        sign = plus >= minus ? 1 : -1;
        return Math.max(plus, minus);
    }

    private double decelFor() {
        Double d = results.get(forwardAxis ? "naturalForwardDeceleration" : "naturalStrafeDeceleration");
        return d != null && d > 5 ? Math.min(d, DECEL_GUESS * 2) : DECEL_GUESS;
    }

    private void settle(Io io, double t, boolean still, String summary) {
        if (stage == Stage.HEADING_ID || stage == Stage.HEADING_BRAKE) {
            io.drive(new DrivePowers(0, 0, 0));
        } else {
            io.coast();
        }
        if (still || t > 3) {
            io.coast();
            running = false;
            phase = "done";
            message = stageName() + " done: " + summary + ".";
        }
    }

    private void next(String p, long now) {
        phase = p;
        phaseNs = now;
    }

    private double c(int i) {
        String ax = stage == Stage.HEADING_BRAKE ? "heading" : forwardAxis ? "forward" : "strafe";
        Double v = results.get(ax + "Braking" + (i == 0 ? "Linear" : "Quadratic") + "Coefficient");
        return v == null ? Double.NaN : v;
    }

    private static double distance(Pose a, Pose b) {
        return Math.hypot(b.x() - a.x(), b.y() - a.y());
    }

    private static double[] biasedGradient(int count, double max, double min, double bias) {
        if (count < 2) {
            return new double[] {max};
        }
        double[] values = new double[count];
        for (int i = 0; i < count; i++) {
            double t = (double) i / (count - 1);
            values[i] = min + (1 - Math.pow(t, bias)) * (max - min);
        }
        return values;
    }

    private static double def(double v, double d) {
        return Double.isNaN(v) ? d : v;
    }

    private String axisName() {
        return forwardAxis ? "forward" : "strafe";
    }

    private String stageName() {
        return stage == null ? "Foresight" : stageNameOf(stage) + (usesAxis(stage) ? " " + axisName() : "");
    }

    private static String stageNameOf(Stage s) {
        return s.name();
    }

    private static boolean usesAxis(Stage s) {
        return s == Stage.DECEL || s == Stage.BRAKE || s == Stage.TRANSLATIONAL;
    }

    private static String r1(Double v) {
        return v == null ? "?" : String.valueOf(Math.round(v * 10) / 10.0);
    }

    private static String r2(Double v) {
        return v == null ? "?" : String.valueOf(Math.round(v * 100) / 100.0);
    }

    private static String r4(Double v) {
        return v == null ? "?" : String.valueOf(Math.round(v * 1e4) / 1e4);
    }

    // ---------------------------------------------------------------- /state

    void appendJson(StringBuilder sb) {
        sb.append(",\"foresight\":{\"running\":").append(running)
                .append(",\"stage\":\"").append(stage == null ? "" : stage.name()).append('"')
                .append(",\"axis\":\"").append(axisName()).append('"')
                .append(",\"phase\":\"").append(phase).append('"')
                .append(",\"trial\":").append(trial)
                .append(",\"trials\":").append(trials)
                .append(",\"sign\":").append(num(sign))
                .append(",\"message\":\"").append(message.replace("\\", "\\\\").replace("\"", "'")).append('"');
        appendMap(sb, "results", results);
        appendMap(sb, "extras", extras);
        sb.append(",\"points\":[");
        for (int i = 0; i < points.size(); i++) {
            sb.append(i == 0 ? "" : ",").append('[').append(num(points.get(i)[0])).append(',')
                    .append(num(points.get(i)[1])).append(']');
        }
        sb.append("],\"t\":[");
        for (int i = 0; i < times.size(); i++) {
            sb.append(i == 0 ? "" : ",").append(num(times.get(i)));
        }
        sb.append("],\"v\":[");
        for (int i = 0; i < vels.size(); i++) {
            sb.append(i == 0 ? "" : ",").append(num(vels.get(i)));
        }
        sb.append("]}");
    }

    private static void appendMap(StringBuilder sb, String name, Map<String, Double> m) {
        sb.append(",\"").append(name).append("\":{");
        boolean first = true;
        for (Map.Entry<String, Double> e : m.entrySet()) {
            sb.append(first ? "" : ",").append('"').append(e.getKey()).append("\":").append(num(e.getValue()));
            first = false;
        }
        sb.append('}');
    }

    private static String num(double v) {
        return Double.isFinite(v) ? String.valueOf(Math.round(v * 1e6) / 1e6) : "null";
    }
}
