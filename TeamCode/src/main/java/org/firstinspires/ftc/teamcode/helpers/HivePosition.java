package org.firstinspires.ftc.teamcode.helpers;

import org.firstinspires.ftc.teamcode.SystemConstants;

import com.pedropathing.math.Pose;

public class HivePosition {


    public HivePosition() {}

    /** The bare rule, no hysteresis: the upper end (UP_POS) at y >= HALF_Y. Turret uses {@link #upper(Pose, Boolean)}. */
    public static Pose target(Pose robot, Alliance alliance) {
        return target(alliance, robot.y() >= SystemConstants.HivePosition.HALF_Y);
    }

    public static Pose target(Alliance alliance, boolean upper) {
        double x = alliance == Alliance.RED ? SystemConstants.HivePosition.xred : SystemConstants.HivePosition.xblue;
        return new Pose(x, upper ? SystemConstants.HivePosition.UP_POS : SystemConstants.HivePosition.DOWN_POS);
    }

    /**
     * The upper end or not, with hysteresis: {@code previous} (null = no previous choice: the bare rule) is kept
     * until the robot is more than HALF_Y_HYSTERESIS_IN past HALF_Y on the other side.
     */
    public static boolean upper(Pose robot, Boolean previous) {
        double y = robot.y(), half = SystemConstants.HivePosition.HALF_Y;
        double band = Math.max(0, SystemConstants.HivePosition.HALF_Y_HYSTERESIS_IN);
        if (previous == null) return y >= half;
        if (previous && y < half - band) return false;
        if (!previous && y > half + band) return true;
        return previous;
    }
}
