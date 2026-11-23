package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;

public final class AutoConstants {
    private AutoConstants() {}

    @Config
    @Configurable
    public static class Autonomous {
        // A Shoot step ends once the gate has been open SHOOT_OPEN_TIME in total, or after SHOOT_TIME at most
        // (AutoMechanisms.eventDone). It was a fixed 1500 ms: a turret unwind (~1.0-1.35 s, gate shut) at the
        // start of the step ate the shot - Auto Dry Run 2026-11-15, 2/4 Shoot steps fired, the gate open
        // 1.18 / 1.23 s in the two that did (SHOOT_OPEN_TIME keeps that).
        public static double SHOOT_TIME = 3000; //ms, cap
        public static double SHOOT_OPEN_TIME = 1200; //ms of gate open
        // While a path is followed, aim the turret from the path's END pose (fixed robot-relative angle)
        // instead of tracking the live pose: no unwinds in transit, arrives aimed. Host model of the RED
        // auto (ideal following): 5 -> 3 unwinds. AutoMechanisms.update.
        public static boolean PRE_AIM = true;
        public static double SCOOPULA_TRANSITION_TIME = 300; //ms
        // Camera pick on path 5 (auto/modules/CameraPick, operator 2026-11-23). RED frame, inches; blue mirrors.
        // At PICK_Y on the way down, one frame's clumps are projected onto the audience wall and the robot is
        // re-routed into the biggest, ending pressed against the wall at (pick x, WALL_Y) heading 270, then shoots.
        public static boolean CAMERA_PICK = true;
        /** Where on path 5 (x ~58, heading 270) the frame is taken. Robot was drawn at y ~59 on the visualizer. */
        public static double PICK_Y = 59;
        /** Robot centre against the audience wall: half the 18 in length. Lower pushes into the wall harder. */
        public static double WALL_Y = 9.0;
        /** Clumps are assumed against the wall, centred this far out (a 2.8 in ball's radius). */
        public static double BALL_LINE_Y = 1.4;
        /** Allowed drive-to x along the wall: red wall + half the 15 in width + 1; audience flower (x 90.7) - 9.7. */
        public static double PICK_X_MIN = 8.5;
        public static double PICK_X_MAX = 81.0;
        // Turn in place before a path marked "turn first" (after the camera-pick shot: to the next path's end
        // heading, away from the turret wrap). Done within the tolerance, or after the timeout regardless.
        public static double TURN_TOLERANCE_DEG = 4.0;
        public static double TURN_TIMEOUT_MS = 1200;
        // The turn point is moved off any wall until the robot's half-diagonal + margin clears it (TurnInPlace):
        // 18 x 15 in (operator 2026-11-23) sweeps 11.7 in, and the camera pick leaves the centre 9 in off the wall.
        public static double ROBOT_LENGTH_IN = 18;
        public static double ROBOT_WIDTH_IN = 15;
        public static double TURN_WALL_MARGIN_IN = 0.75;
    }
}
