package org.firstinspires.ftc.teamcode.diagnostics.autosim;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

import java.util.function.Supplier;

/**
 * DIAGNOSTIC ONLY (Auto Dry Run). A kinematic stand-in for the Pinpoint: the robot moves the way the
 * follower's last commanded DrivePowers say, so an auto can run on blocks while its pose travels the field.
 *
 * <p>Model: the commanded powers go through the same limits as the vendored Swerve mixer - translation
 * clipped to magnitude 1, each pod vector = translation + turn x (unit tangent at that pod), and all of them
 * scaled by 1 / max(1, largest pod magnitude). Robot-frame velocity = scaled translation x vMax, yaw rate =
 * scaled turn x vMax / podRadius, each with a first-order lag tau, integrated in the field frame. vMax is the
 * max velocity Foresight plans with. It is a model, not a measurement: no slip, no breakaway, no battery.
 *
 * <p>The clip is load-bearing: Foresight's braking term grows with velocity squared, so unclipped powers fed
 * back into an ideal integrator run away (first dry run 2026-11-15: pose 1e135 in within 1.5 s, NaN reached
 * the turret servo and the SDK stopped the OpMode). A non-finite pose is refused anyway: {@link #diverged()}.
 */
final class SimLocalizer implements Localizer {
    private final Supplier<SimDrive> drive;
    private final double vMax;
    private final double podRadius;
    private final double tau;
    private double x, y, h;
    private double vxR, vyR, w;
    private long lastNs;
    private volatile boolean diverged;

    SimLocalizer(Supplier<SimDrive> drive, double vMax, double podRadius, double tau) {
        this.drive = drive;
        this.vMax = vMax;
        this.podRadius = podRadius;
        this.tau = tau;
    }

    @Override
    public void setPose(Pose pose) {
        x = pose.x();
        y = pose.y();
        h = pose.heading();
        vxR = vyR = w = 0;
    }

    @Override
    public MotionState state() {
        double c = Math.cos(h), s = Math.sin(h);
        return MotionState.ofVelocity(new Pose(x, y, h), new Velocity(vxR * c - vyR * s, vxR * s + vyR * c, w));
    }

    @Override
    public void update() {
        long now = System.nanoTime();
        double dt = lastNs == 0 ? 0 : Math.min(0.1, (now - lastNs) / 1e9);
        lastNs = now;
        SimDrive d = drive.get();
        DrivePowers p = d == null ? DrivePowers.zero() : d.last;
        double fwd = finite(p.forward()), str = finite(p.strafe()), turn = finite(p.turn());
        double mag = Math.hypot(fwd, str);
        if (mag > 1) {
            fwd /= mag;
            str /= mag;
        }
        double maxPod = 1;
        for (int i = 0; i < 4; i++) {
            double tangent = Math.PI / 4 + i * Math.PI / 2 + Math.PI / 2;
            maxPod = Math.max(maxPod, Math.hypot(fwd + turn * Math.cos(tangent), str + turn * Math.sin(tangent)));
        }
        double scale = 1 / maxPod;
        double a = tau > 0 ? Math.min(1, dt / tau) : 1;
        vxR += (fwd * scale * vMax - vxR) * a;
        vyR += (str * scale * vMax - vyR) * a;
        w += (turn * scale * vMax / podRadius - w) * a;
        double c = Math.cos(h), s = Math.sin(h);
        double nx = x + (vxR * c - vyR * s) * dt;
        double ny = y + (vxR * s + vyR * c) * dt;
        double nh = Math.atan2(Math.sin(h + w * dt), Math.cos(h + w * dt));
        if (Double.isFinite(nx) && Double.isFinite(ny) && Double.isFinite(nh)) {
            x = nx;
            y = ny;
            h = nh;
        } else {
            vxR = vyR = w = 0;
            diverged = true;
        }
    }

    /** True once an update produced a non-finite pose (refused; the pose stayed at its last finite value). */
    boolean diverged() {
        return diverged;
    }

    private static double finite(double v) {
        return Double.isFinite(v) ? v : 0;
    }

    @Override
    public void reset() {
        vxR = vyR = w = 0;
    }
}
