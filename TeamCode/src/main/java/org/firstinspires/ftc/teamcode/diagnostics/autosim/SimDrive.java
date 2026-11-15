package org.firstinspires.ftc.teamcode.diagnostics.autosim;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;

import java.util.Map;

/**
 * DIAGNOSTIC ONLY (Auto Dry Run). Wraps the competition swerve: records the last DrivePowers the follower
 * asked for (the simulated localizer integrates them) and, when {@code passThrough}, still drives the real
 * pods - on blocks the wheels steer and spin in the air exactly as they would on the field.
 */
final class SimDrive implements Drivetrain {
    private final Drivetrain real;
    private final boolean passThrough;
    volatile DrivePowers last = DrivePowers.zero();

    SimDrive(Drivetrain real, boolean passThrough) {
        this.real = real;
        this.passThrough = passThrough;
    }

    @Override
    public void drive(DrivePowers powers, boolean manual) {
        last = powers;
        if (passThrough) {
            real.drive(powers, manual);
        } else {
            real.stop();
        }
    }

    @Override
    public double maxScaling(DrivePowers current, DrivePowers delta) {
        return real.maxScaling(current, delta);
    }

    @Override
    public void stop() {
        last = DrivePowers.zero();
        real.stop();
    }

    @Override
    public void stop(boolean brake) {
        last = DrivePowers.zero();
        real.stop(brake);
    }

    @Override
    public Map<String, Object> debug() {
        return real.debug();
    }

    @Override
    public double interpolateAcceleration(double xRadius, double yRadius, double theta) {
        return real.interpolateAcceleration(xRadius, yRadius, theta);
    }

    @Override
    public double interpolateVelocity(double xRadius, double yRadius, double theta) {
        return real.interpolateVelocity(xRadius, yRadius, theta);
    }
}
