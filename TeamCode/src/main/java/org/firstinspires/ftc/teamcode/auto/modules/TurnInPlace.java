package org.firstinspires.ftc.teamcode.auto.modules;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.AutoConstants;
import org.firstinspires.ftc.teamcode.SystemConstants;

/**
 * Where an auto's turn-in-place step (runTurnStep, the visualizer's "Turn first") turns. A robot turning about its
 * centre sweeps a circle of its half-diagonal: 18 x 15 in is 11.7 in, but pressed on a wall (the camera pick ends
 * with the centre 9 in off it) the centre is only half the LENGTH away, so the corners would scrape the wall and
 * push or bind the robot. The turn point is moved straight off any field wall until that circle (plus
 * TURN_WALL_MARGIN_IN) clears it; anywhere else it is the robot's own position. Obstacles are not considered.
 */
public final class TurnInPlace {
    private TurnInPlace() {}

    public static Pose clearOfWalls(Pose here, double headingRad) {
        double r = Math.hypot(AutoConstants.Autonomous.ROBOT_LENGTH_IN / 2.0, AutoConstants.Autonomous.ROBOT_WIDTH_IN / 2.0)
                + AutoConstants.Autonomous.TURN_WALL_MARGIN_IN;
        double field = SystemConstants.Zone.FIELD_SIZE;
        double x = Math.max(r, Math.min(field - r, here.x()));
        double y = Math.max(r, Math.min(field - r, here.y()));
        return new Pose(x, y, headingRad);
    }
}
