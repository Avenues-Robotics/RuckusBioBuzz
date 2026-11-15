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
 * <p>Model: robot-frame velocity = (forward, strafe) x vMax, yaw rate = turn x vMax / podRadius (the swerve
 * gives every pod a tangential share equal to the turn power), each with a first-order lag tau; integrated in
 * the field frame. vMax is the same max velocity Foresight plans with, so the follower's intent is honoured.
 * It is a model, not a measurement: no slip, no breakaway, no battery.
 */
final class SimLocalizer implements Localizer {
    private final Supplier<SimDrive> drive;
    private final double vMax;
    private final double podRadius;
    private final double tau;
    private double x, y, h;
    private double vxR, vyR, w;
    private long lastNs;

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
        double a = tau > 0 ? Math.min(1, dt / tau) : 1;
        vxR += (p.forward() * vMax - vxR) * a;
        vyR += (p.strafe() * vMax - vyR) * a;
        w += (p.turn() * vMax / podRadius - w) * a;
        double c = Math.cos(h), s = Math.sin(h);
        x += (vxR * c - vyR * s) * dt;
        y += (vxR * s + vyR * c) * dt;
        h = Math.atan2(Math.sin(h + w * dt), Math.cos(h + w * dt));
    }

    @Override
    public void reset() {
        vxR = vyR = w = 0;
    }
}
