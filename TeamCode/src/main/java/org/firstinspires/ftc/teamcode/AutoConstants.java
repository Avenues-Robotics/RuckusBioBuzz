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
        /** Allowed drive-to x along the wall. MIN: red wall + half the 15 in width + 1. */
        public static double PICK_X_MIN = 8.5;
        /**
         * MAX: the AUTO midline (x 70.75, G402 - the whole ROBOT stays on its own half). Pressed on the wall the
         * robot's side is at x + 7.5, but the turn in place after the pick sweeps its corners out to its half-
         * diagonal, 11.7 in: 70.75 - 11.72 - 1.0 margin = 58.0. Was 81.0 (the audience flower) until 2026-11-23,
         * which let a pick drive the robot - and its turn - across the midline. Blue mirrors: x >= 83.5.
         */
        public static double PICK_X_MAX = 58.0;
        // Turn in place before a path marked "turn first" (after the camera-pick shot: to the next path's end
        // heading, away from the turret wrap). Done within the tolerance, or after the timeout regardless.
        public static double TURN_TOLERANCE_DEG = 4.0;
        public static double TURN_TIMEOUT_MS = 1200;
        // The turn point is moved off any wall until the robot's half-diagonal + margin clears it (TurnInPlace):
        // 18 x 15 in (operator 2026-11-23) sweeps 11.7 in, and the camera pick leaves the centre 9 in off the wall.
        public static double ROBOT_LENGTH_IN = 18;
        public static double ROBOT_WIDTH_IN = 15;
        public static double TURN_WALL_MARGIN_IN = 0.75;
        // Phase 1 of the turn: back off (heading held) until within this of the clear point, or give up waiting
        // after the timeout and turn anyway. A back-off shorter than the tolerance is skipped.
        public static double TURN_BACKOFF_TOLERANCE_IN = 0.5;
        public static double TURN_BACKOFF_TIMEOUT_MS = 800;
        // Path stall guard (followPathStep): within NEAR_IN of a path's end and moving < MOVE_IN over S seconds ->
        // hold there and go on. 2026-11-24 blue auto: the corner end sat 2.7 in into the far wall and the
        // follower pushed for 17 s.
        public static double PATH_STALL_NEAR_IN = 6.0;
        public static double PATH_STALL_MOVE_IN = 0.5;
        public static double PATH_STALL_S = 0.8;
        /**
         * Foresight heading kP for the autos (AutoMechanisms applies it at init; NaN = Constants.headingKP 7.7173).
         * 2026-11-24: kP 7.72 was accepted on in-place turns and holds in Bring-Up (100+ Hz loop). In the auto
         * (25.7 Hz loop) blue run 4 had pod |error| median 25 / p90 60 deg on path 3 AND on the straight return,
         * yaw rate reversing 2.4-3.3 /s, heading error only 2-2.5 deg median - the TeleOp fishtail of 11-17
         * again (hold kP 3.0 -> 1.0 fixed it there). 1.5 is an ESTIMATE (loop gain ~6.5 rad/s per power x kP
         * kept near ~10 rad/s for ~40-80 ms of loop delay), on trial in the next run - not accepted.
         */
        // 1.5 (run 5) -> 0.8 (run 16 trial): with the braking projection off, run 15's constant-heading legs still
        // hunt (heading error +/-6-10 deg, sign changes ~2.5-3.4 /s, yaw +/-90 deg/s). Rotation answers the command
        // ~170 ms late while driving (run 11); at kP 1.5 x ~6.5 rad/s per power the crossover (~9.8 rad/s) loses
        // ~95 deg to that delay. 0.8 is an ESTIMATE for margin; on trial.
        public static double HEADING_KP = 0.8;
        /**
         * Foresight translational kP for the autos, all four (forward / strafe, primary / secondary) set to it; NaN =
         * Constants (0.26, kD 0). 2026-11-24 blue run 7: at 25-35 in/s the pods' commanded direction swings together
         * by +/-25-30 deg at ~2-3 Hz with the pods ~90 ms behind - a cross-track loop at 0.26 power/in (~18 deg of
         * direction per inch at cruise, ~12 rad/s crossover against ~0.12 s of pod + loop delay), accepted on holds in
         * the 2x2 only. 0.12 is an ESTIMATE (crossover ~5-6 rad/s); on trial, not accepted. Cost to watch: holds stop
         * further short (0.19 power breakaway / 0.12 ~ 1.6 in band).
         */
        public static double TRANSLATIONAL_KP = 0.12;
        /**
         * Foresight headingDeviationTolerance for the autos, degrees (NaN = Pedro's 11.25). Past it the heading
         * correction is allocated before translation. Blue run 9 (2026-11-24): 37% of path 3 (a 172 deg turn while
         * driving) sat past 11.25 at heading kP 1.5, translation command 0.91 -> 0.59 and speed 30 -> 20 in/s there.
         * 30 is an ESTIMATE; on trial, not accepted. The heading still converges - it just stops preempting driving.
         */
        public static double HEADING_TOLERANCE_DEG = 30;
        /**
         * Pod steering gain-schedule HIGH-SPEED floor for the autos (CoaxialPod.setScheduleHighSpeed; NaN =
         * Constants.turnScheduleFloorHigh 0.80). Blue run 10 (2026-11-24), straight return leg at 0.85-0.95 drive
         * power: commanded pod targets steady (80-100 deg) while the wheels swing +/-35-45 deg around them at ~3 Hz,
         * turn power +/-0.5-0.6, yaw rate +/-80 deg/s in step - the pod loops limit-cycling. 0.80 was accepted
         * 2026-11-24 in the TeleOp Drive Probe (mechanisms off, 48-75 Hz loop); the auto loop is 30-35 Hz.
         * 0.30 = the high-speed leg off (same as the low floor): a hypothesis test, not a tune.
         */
        public static double STEER_FLOOR_HIGH = 0.30;
        /**
         * Scale on Foresight's heading braking coefficients for the autos (1 = Constants 0.0505 / 0.0199, measured
         * on in-place spins 2026-11-15). Foresight aims at heading + braking(omega), so the term is rate feedback.
         * Blue run 11 (2026-11-24), path 3 (172 deg turn while driving): the rotation command tracks kP x PROJECTED
         * error (corr 0.86 vs 0.61 for the raw error), the projection term is as large as the error term (sd 0.11
         * vs 0.13), and omega answers the command ~160-190 ms later - rate feedback through that delay is a ~3 Hz
         * cycle, the jerky turning the operator sees. 0 = projection off: a hypothesis test, not a tune.
         */
        public static double HEADING_BRAKE_SCALE = 0.0;
        /**
         * Swerve demand slew limit for the autos, deg/s (Swerve.setDemandSlewDegPerSec; NaN = Constants, 300). Blue
         * run 12 (2026-11-24), path 3: pod targets moved at a median 155 / p90 329 deg/s while the robot turned
         * ~41 deg/s - loop-to-loop command jitter - and pod error grows with it (10 deg median below 50 deg/s, 24
         * above 300). 150 filters the jitter and still passes the real demand. Hypothesis test, not a tune.
         */
        // Run 13: 150 cut target rates (median 155 -> 131, p90 329 -> 210 deg/s) but yaw jitter rose 33 -> 52 deg/s
        // (the lag hurt the heading loop). Back to Constants.
        public static double DEMAND_SLEW_DEG_S = Double.NaN;
        /**
         * Autos: strafe braking = the MEASURED forward braking (Constants forwardBrake*), instead of the derived strafe
         * pair (linear 0, quadratic 1/(2 x 95) = 0.0053, 2.2x forward's 0.00237). Foresight projects the pose ahead by
         * these per robot axis; on path 3 (turning while driving) much of the motion is robot-frame strafe, where
         * the larger coefficient turns lateral-velocity wobble into projected-pose jumps (est. ~2 in, ~0.25 command,
         * for 5 in/s at 40 in/s) - straight forward legs are smooth, turning ones are not. Swerve brakes the same in
         * every direction (CLAUDE.md). Hypothesis test: false = Constants.
         */
        public static boolean ISOTROPIC_BRAKING = true;
        /**
         * Pod demand-rate feed-forward kV for the autos, power per rad/s of target motion (CoaxialPod
         * setDemandRateFeedforward; NaN = Constants.turnDemandRateKV 0.045, same tau / cap). Run 14 (2026-11-24):
         * pod error is tracking lag - 110-120 ms on path 3, 176 ms on the path 1-2 curves (r -0.86, error median 24 /
         * p90 60 deg). Following a moving target takes ~0.108 power per rad/s (pods ~530 deg/s at full power); 0.045
         * covers ~40% and the P term makes up the rest by lagging. 0.09 is an ESTIMATE (~85%); on trial.
         */
        // Run 15 (0.09): operator "prob better" on the curves, but the second garden clear got worse vs run 14 (heading
        // err rms 3.0 -> 4.6 deg, yaw p90 40 -> 71 deg/s, pod err median 13 -> 21) and path 3 pod error 10 -> 24. Back
        // to Constants (0.045).
        public static double DEMAND_RATE_KV = Double.NaN;
        /**
         * Auto flywheel offset, ticks/s added to the regression target (Shooter.setAimOffsets). Operator 2026-11-24:
         * auto shots aimed but too powerful - POLLEN_COEFFS (2026-11-23) predate the third flywheel weight and P
         * 45 -> 450. QUICK FIX until a Regression-tab re-fit: -90 = ~-8% at the auto's 36-45 in (~1160 ticks/s),
         * an estimate (range ~ v^2, so ~-15% range). TeleOp unaffected.
         */
        public static double FLYWHEEL_OFFSET = -90;
    }
}
