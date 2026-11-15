package org.firstinspires.ftc.teamcode.helpers;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.SystemConstants;

/**
 * Driver 2's aim corrections in TeleOp: a field-frame shift of the aim point and a flywheel speed offset.
 *
 * <p>Why a field-frame shift: odometry drift is a field-frame error, so moving the goal by a fixed vector is what
 * cancels it - the correction stays right wherever the robot drives next, and nobody has to relocalize.
 *
 * <p>How a "2 RIGHT" call becomes a vector: each press moves the aim point {@code AIM_STEP_IN} inches sideways to
 * the line from the LAST SHOT position to the hive, toward the coach's right. The coach stands behind their own
 * alliance wall looking across the field - red from x = 0 toward +x, blue from the far wall toward -x (visualizer
 * frame: x = 0 red wall, y = 0 audience wall). A robot shooting away from its wall is seen from behind, so the
 * coach's right is the shot's right; a robot beyond the hive shooting back toward its own wall is seen from the
 * front, so the coach's right is the shot's left (the reversal on the opposite side of the field).
 *
 * <p>Pure math, no hardware: tools/swervetune/AimOffsetsCheck.java runs it on the host.
 */
public final class AimOffsets {
    private double goalDx;
    private double goalDy;
    private double speedOffset;
    private int lateralClicks;
    private int speedClicks;

    /** One d-pad left (-1) or right (+1) press, judged from where the robot last shot (null: from {@code now}). */
    public void nudgeLateral(int direction, Pose lastShot, Pose now, Alliance alliance) {
        Pose from = lastShot != null ? lastShot : now;
        double[] right = coachRight(from, HivePosition.target(from, alliance), alliance);
        double step = Math.signum(direction) * SystemConstants.TeleOp.AIM_STEP_IN;
        goalDx += step * right[0];
        goalDy += step * right[1];
        lateralClicks += (int) Math.signum(direction);
    }

    /** One d-pad up (+1) or down (-1) press: the flywheel target's y-intercept, ticks/s. */
    public void nudgeSpeed(int direction) {
        speedOffset += Math.signum(direction) * SystemConstants.TeleOp.SPEED_STEP;
        speedClicks += (int) Math.signum(direction);
    }

    public void reset() {
        goalDx = 0;
        goalDy = 0;
        speedOffset = 0;
        lateralClicks = 0;
        speedClicks = 0;
    }

    public double goalDx() { return goalDx; }
    public double goalDy() { return goalDy; }
    public double speedOffset() { return speedOffset; }
    /** Net presses, + = right / faster - for driver telemetry. */
    public int lateralClicks() { return lateralClicks; }
    public int speedClicks() { return speedClicks; }

    /**
     * Unit vector (field frame) for "the coach's right", sideways to the shot from {@code from} to {@code hive}.
     * Robot on the hive: the coach's own right (red -y, blue +y).
     */
    public static double[] coachRight(Pose from, Pose hive, Alliance alliance) {
        double dx = hive.x() - from.x();
        double dy = hive.y() - from.y();
        double n = Math.hypot(dx, dy);
        if (n < 1e-6) {
            return new double[] {0, alliance == Alliance.BLUE ? 1 : -1};
        }
        dx /= n;
        dy /= n;
        double rx = dy;   // right of the shot, seen from behind the shooter
        double ry = -dx;
        double awayFromCoach = alliance == Alliance.BLUE ? -dx : dx;
        if (awayFromCoach < 0) {   // shooting back toward the drivers: seen from the front, left and right swap
            rx = -rx;
            ry = -ry;
        }
        return new double[] {rx, ry};
    }
}
