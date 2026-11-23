package org.firstinspires.ftc.teamcode.modules;

import org.firstinspires.ftc.teamcode.SystemConstants;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

public class PredictiveAiming {


    private final Follower follower;

    public PredictiveAiming(Follower follower) {
        this.follower = follower;
    }
    public Pose predictPose() {
        return predictPose(SystemConstants.PredictiveAiming.PREDICT_MS);
    }
    public Pose predictPose(double ms) {
        return follower.pose().exp(follower.twist(), ms / 1000.0);
    }

    /** Measured turn rate, rad/s (CCW +), from the localizer. */
    public double headingRate() {
        return follower.velocity().omega;
    }

    public Pose leadPose() {
        return leadPose(SystemConstants.PredictiveAiming.CENTER);
    }

    public Pose leadPose(double[] offset) {
        return leadPose(offset, SystemConstants.PredictiveAiming.PREDICT_MS / 1000.0);
    }

    /** The point at robot-frame {@code offset}, moved along its own field velocity (spin included) for {@code t} s. */
    public Pose leadPose(double[] offset, double t) {
        Pose robot = follower.pose();
        Velocity v = follower.velocity();
        double c = Math.cos(robot.heading());
        double s = Math.sin(robot.heading());
        double rx = offset[0] * c - offset[1] * s;
        double ry = offset[0] * s + offset[1] * c;
        double vx = v.vx - v.omega * ry;
        double vy = v.vy + v.omega * rx;
        return new Pose(robot.x() + rx + vx * t, robot.y() + ry + vy * t, robot.heading());
    }
}
