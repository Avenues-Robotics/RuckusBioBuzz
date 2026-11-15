// HOST CHECK for diagnostics/swerve/ForesightBoxTuner.java (no robot): every stage against a simulated swerve with
// known constants inside a 30 x 30 in centre box. Expect the fits to recover K, tau, the coast deceleration and
// heading K/tau within a few percent, and "worst excursion outside the box: 0.00 in". From the repo root:
//   J="/Applications/Android Studio.app/Contents/jbr/Contents/Home/bin"; O=$(mktemp -d); P=$O/src/org/firstinspires/ftc/teamcode/diagnostics/swerve
//   mkdir -p $P && cp tools/swervetune/ForesightBoxTunerCheck.java TeamCode/src/main/java/org/firstinspires/ftc/teamcode/diagnostics/swerve/ForesightBoxTuner.java $P/
//   "$J/javac" -nowarn -d $O/out -sourcepath "$O/src:$PWD/third_party/PedroPathing/core/src/main/java" $P/ForesightBoxTunerCheck.java
//   "$J/java" -cp $O/out org.firstinspires.ftc.teamcode.diagnostics.swerve.ForesightBoxTunerCheck
// Result 2026-11-15: heading kP 8.04 (truth 8.33), decel 34.9/35.7 (35), K 79.7/79.9 (80), tau 0.174/0.181 (0.18),
// worst excursion 0.00 in. Runs at real time (the tuner reads System.nanoTime) - about a minute.
package org.firstinspires.ftc.teamcode.diagnostics.swerve;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.math.Pose;
import java.util.Random;

/** Host check: ForesightBoxTuner against a simulated swerve with known constants, inside a 30 x 30 in centre box. */
public class ForesightBoxTunerCheck {
    static final double K = 80, TAU = 0.18, DECEL = 35, KW = 6, TAUW = 0.15, DT = 0.01;
    static final double MINX = 0, MAXX = 30, MINY = 0, MAXY = 30;
    double x = 15, y = 15, h = 0, vf, vs, w;            // body velocities
    double cmdF, cmdS, cmdT; boolean coasting; double zeroSince = -1, t;
    final Random rnd = new Random(1);
    double worstOut = 0;

    void step() {
        boolean zero = coasting || (Math.hypot(cmdF, cmdS) < 0.05 && Math.abs(cmdT) < 0.015);
        if (zero) { if (zeroSince < 0) zeroSince = t; } else zeroSince = -1;
        boolean xlock = !coasting && zero && t - zeroSince >= 0.35;
        // translation
        if (zero) {
            double d = (xlock ? 3 * DECEL : DECEL) * DT;
            double sp = Math.hypot(vf, vs);
            double k = sp > d ? (sp - d) / sp : 0;
            vf *= k; vs *= k;
        } else {
            vf += (K * cmdF - vf) * DT / TAU;
            vs += (K * cmdS - vs) * DT / TAU;
        }
        // rotation
        if (zero) {
            double d = (xlock ? 15 : 4) * DT;
            w = Math.abs(w) > d ? w - Math.signum(w) * d : 0;
        } else {
            w += (KW * cmdT - w) * DT / TAUW;
        }
        double c = Math.cos(h), s = Math.sin(h);
        x += (vf * c - vs * s) * DT;
        y += (vf * s + vs * c) * DT;
        h += w * DT;
        t += DT;
        worstOut = Math.max(worstOut, Math.max(Math.max(MINX - x, x - MAXX), Math.max(MINY - y, y - MAXY)));
    }

    double room(double fx, double fy) {
        double r = Double.POSITIVE_INFINITY;
        if (fx > 1e-9) r = Math.min(r, (MAXX - x) / fx);
        if (fx < -1e-9) r = Math.min(r, (MINX - x) / fx);
        if (fy > 1e-9) r = Math.min(r, (MAXY - y) / fy);
        if (fy < -1e-9) r = Math.min(r, (MINY - y) / fy);
        return Math.max(0, r);
    }

    ForesightBoxTuner.Io io() {
        return new ForesightBoxTuner.Io() {
            public void drive(DrivePowers p) { coasting = false; cmdF = p.forward(); cmdS = p.strafe(); cmdT = p.turn(); }
            public void coast() { coasting = true; cmdF = cmdS = cmdT = 0; }
            public Pose pose() { return new Pose(x, y, h); }
            public double forwardVel() { return vf + rnd.nextGaussian() * 0.3; }
            public double strafeVel() { return vs + rnd.nextGaussian() * 0.3; }
            public double omega() { return w + rnd.nextGaussian() * 0.005; }
            public double room(double fx, double fy) { return ForesightBoxTunerCheck.this.room(fx, fy); }
            public double headingKpDefault() { return 1.2; }
            public double headingBrakeLinearDefault() { return 0; }
            public double headingBrakeQuadraticDefault() { return 0; }
        };
    }

    void run(ForesightBoxTuner tuner, ForesightBoxTuner.Stage st, boolean fwd) {
        ForesightBoxTuner.Io io = io();
        String refusal = tuner.start(st, fwd, Double.NaN, Double.NaN, io);
        if (refusal != null) { System.out.println(st + " REFUSED: " + refusal); return; }
        double t0 = t;
        while (tuner.running() && t - t0 < 120) {
            if (x < MINX || x > MAXX || y < MINY || y > MAXY) { tuner.abort("left the fence"); break; }
            tuner.update(io);
            // real clock: the tuner reads System.nanoTime, so pace the sim to it
            long until = System.nanoTime() + (long) (DT * 1e9);
            while (System.nanoTime() < until) { }
            step();
        }
        StringBuilder sb = new StringBuilder();
        tuner.appendJson(sb);
        String js = sb.toString();
        int cut = js.indexOf(",\"t\":[");
        System.out.printf("%-14s %-7s %5.1fs pose (%.1f, %.1f) worstOut %.2f  %s%n", st, fwd ? "fwd" : "strafe",
                t - t0, x, y, worstOut, tuner.message());
        System.out.println("    " + js.substring(js.indexOf("\"results\""), cut));
    }

    public static void main(String[] a) {
        ForesightBoxTunerCheck sim = new ForesightBoxTunerCheck();
        ForesightBoxTuner tuner = new ForesightBoxTuner();
        sim.run(tuner, ForesightBoxTuner.Stage.HEADING_ID, true);
        sim.run(tuner, ForesightBoxTuner.Stage.HEADING_BRAKE, true);
        sim.run(tuner, ForesightBoxTuner.Stage.DECEL, true);
        sim.run(tuner, ForesightBoxTuner.Stage.DECEL, false);
        sim.run(tuner, ForesightBoxTuner.Stage.TRANSLATIONAL, true);
        sim.run(tuner, ForesightBoxTuner.Stage.TRANSLATIONAL, false);
        sim.run(tuner, ForesightBoxTuner.Stage.BRAKE, true);
        sim.run(tuner, ForesightBoxTuner.Stage.BRAKE, false);
        System.out.printf("truth: K %.0f in/s/power -> kV %.5f, tau %.2f s; coast decel %.0f in/s^2; Kw %.1f rad/s/power, tauw %.2f s -> heading kP %.3f; translational kP_large %.4f kP_small %.4f%n",
                K, 1 / K, TAU, DECEL, KW, TAUW, TAUW * 18.25 * 18.25 / KW, TAU * 10.2 * 10.2 / K, TAU * 6.2 * 6.2 / K);
        System.out.printf("worst excursion outside the 30x30 centre box: %.2f in%n", sim.worstOut);
    }
}
