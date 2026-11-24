package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.api.Paths;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathSegment;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.ArrayList;
import java.util.List;


import org.firstinspires.ftc.teamcode.AutoConstants;
import org.firstinspires.ftc.teamcode.auto.modules.AutoMechanisms;
import org.firstinspires.ftc.teamcode.auto.modules.PathStep;
import org.firstinspires.ftc.teamcode.auto.modules.CameraPick;
import org.firstinspires.ftc.teamcode.auto.modules.TurnInPlace;
import org.firstinspires.ftc.teamcode.helpers.Alliance;
import org.firstinspires.ftc.teamcode.helpers.PinpointGuard;
import org.firstinspires.ftc.teamcode.helpers.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@Autonomous(name = "Redpollenauto", group = "Auto")
public class RedSoloAuto extends OpMode {
    // PathStep fields are editable from FTC Dashboard; paths are built in init(), so edits apply on the next init.
    public static PathStep START_STEP = new PathStep(56.000, 7.600, 180.000);   // flush on the audience wall (15 in wide: 7.5 + 0.1); was 9.196 (1.7 in gap) until 2026-11-24
    public static PathStep POINT_2 = new PathStep(10.222, 13.299, 270.000);   // 3 in off the audience wall (mirror of blue, 2026-11-24)
    public static PathStep POINT_3 = new PathStep(46.480, 125.857, 90.000);
    public static PathStep POINT_4 = new PathStep(58.812, 115.566, 270.000);
    public static PathStep POINT_5 = new PathStep(57.943, 10.419, 270.000);
    public static PathStep POINT_6 = new PathStep(15.442, 47.215, 180.000);
    public static PathStep POINT_7 = new PathStep(45.308, 11.717, 180.000);
    public static PathStep POINT_8 = new PathStep(10.466, 101.157, 180.000);
    public static PathStep POINT_9 = new PathStep(17.155, 22.330, 225.000);

    private Follower follower;
    private AutoMechanisms mechanisms;
    private PinpointGuard pinpointGuard;
    private AutoPath chain1;
    private AutoPath chain2;
    private AutoPath chain3;
    private AutoPath chain4;
    private AutoPath chain5;
    private AutoPath chain6;
    private AutoPath chain7;
    
    // Camera pick on path 5 (visualizer: the path's "Camera pick"; auto/modules/CameraPick).
    private CameraPick cameraPick;
    private AutoPath chain4Pick;
    private boolean chain4Switched;
    // Turn first (runTurnStep): where the robot turns, and whether it is still backing off a wall to get there.
    private Pose turnClearPose;
    private boolean turnBackingOff;

    private AutoPath activePath;
    private int sequenceIndex;
    private long stepStartTime;
    private boolean stepStarted;
    private boolean pathFinished;
    // Step durations from AutoConstants.Autonomous, read at init so Dashboard edits apply on the next init.
    private long shootTimeMs;
    private long scoopulaTransitionMs;
    private static final int REPEAT_LOOP_CAPACITY = 1;
    private final int[] repeatLoopIterations = new int[REPEAT_LOOP_CAPACITY];
    private final int[] repeatLoopPathIndexes = new int[REPEAT_LOOP_CAPACITY];
    private static final int PARALLEL_EVENT_CAPACITY = 5;
    private final String[] activeParallelEventNames = new String[PARALLEL_EVENT_CAPACITY];
    private final long[] activeParallelEventStartTimes = new long[PARALLEL_EVENT_CAPACITY];
    private final long[] activeParallelEventDurations = new long[PARALLEL_EVENT_CAPACITY];

    @Override
    public void init() {
        // Pedro 3 follows paths with Foresight, whose braking model and gains must be measured
        // on the robot first. Refuse to build a path-following OpMode on placeholders.
        Constants.requireForesightMeasured();

        PoseStorage.clear();
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(START_STEP.toPose());
        // Hand-wired (re-apply after a visualizer export): reset Pinpoint position + IMU while the robot stands
        // still at init - the heading has frozen after app restarts - and set the start pose again once done.
        pinpointGuard = new PinpointGuard(hardwareMap);
        pinpointGuard.begin();
        mechanisms = new AutoMechanisms(hardwareMap, follower, Alliance.RED);
        cameraPick = new CameraPick(hardwareMap, Alliance.RED);

        shootTimeMs = Math.round(AutoConstants.Autonomous.SHOOT_TIME);
        scoopulaTransitionMs = Math.round(AutoConstants.Autonomous.SCOOPULA_TRANSITION_TIME);

        buildPaths();
        updateTelemetry("Initialized");
    }

    @Override
    public void init_loop() {
        // Pedro 3's update() with no path stops the drivetrain, writing every servo each loop.
        // Refresh the localizer alone so init stays actuator-silent.
        follower.localizer.update();
        if (pinpointGuard.update()) {
            follower.setPose(START_STEP.toPose());
        }
        telemetry.addData("Pinpoint", pinpointGuard.status());
        updateTelemetry("Ready");
    }

    @Override
    public void start() {
        sequenceIndex = 0;
        stepStarted = false;
        pathFinished = false;
        activePath = null;
        resetRepeatLoops();
        resetParallelEvents();

        pinpointGuard.finish();
        follower.setPose(START_STEP.toPose());
        chain4Pick = null;
        chain4Switched = false;
        chain5 = buildChain5(POINT_5.toPose());
    }

    @Override
    public void loop() {
        mechanisms.clearCache();
        follower.update();
        // Path callbacks run right after the follower update, where Pedro 2 ran them.
        if (activePath != null) {
            activePath.update(follower);
        }
        updateParallelEvents();

        runSequence();
        mechanisms.update();
        // Recording (2026-11-24): pose on /field (TeleOp's FieldView, with the zone) at its own 20 Hz throttle.
        if (org.firstinspires.ftc.teamcode.Config.telemetryEnabled()) {
            org.firstinspires.ftc.teamcode.fieldview.FieldView.publish(follower.pose(), follower.pose(), Alliance.RED);
        }

        updateTelemetry(pathFinished ? "Done" : "Running");
    }

    @Override
    public void stop() {
        if (cameraPick != null) {
            cameraPick.close();
        }
        finishAllParallelEvents();
        if (mechanisms != null) {
            mechanisms.stop();
        }

        if (follower == null) {
            return;
        }

        follower.manual(DrivePowers.zero());
        follower.update();
        PoseStorage.pose = follower.pose();
    }

    private void buildPaths() {
        // Path 1 + corner loop "back": ONE pass into the corner, the come-back-down approach only (operator
        // 2026-11-23: the straight run along the wall into the corner, then out and back in, collected twice and
        // cost ~1.2 s). Start -> the loop's turn-around point (control point set so the two curves join
        // tangentially, no kink to slow at) -> down into the corner, chained without a stop. Intake on from the start.
        chain1 = new AutoPath(
            Paths.path(
                Paths.curve(
                    START_STEP.toPose(),
                    new Pose(36.000, 24.000),
                    POINT_9.toPose()
                ).heading(
                    headingInterpolator(
                        Math.toRadians(180.000),
                        Math.toRadians(225.000),
                        1.000
                    )
                ),
                Paths.curve(
                    POINT_9.toPose(),
                    new Pose(15.361, 22.120),
                    new Pose(11.204, 17.548),
                    POINT_2.toPose()
                ).heading(
                    headingInterpolator(
                        Math.toRadians(225.000),
                        Math.toRadians(270.000),
                        1.000
                    )
                )
            )
        ).addParametricCallback(0, 0.000, () ->
            startParallelEvent("Intake On", 0)   // collecting from path 1 on (operator 2026-11-23)
        );

        chain2 = new AutoPath(
            Paths.curve(
                POINT_2.toPose(),
                new Pose(20.787, 41.881),
                new Pose(24.377, 92.484),
                new Pose(48.724, 111.429),
                POINT_3.toPose()
            ).heading(
                headingInterpolator(
                    Math.toRadians(270.000),
                    Math.toRadians(90.000),   // face the far-wall flower: the scoopula is on the front (was 0, 2026-11-15, for the turret wrap)
                    0.700
                )
            )
        ).addParametricCallback(0, 0.380, () ->
            startParallelEvent("Scoopula Down", scoopulaTransitionMs)
        ).addParametricCallback(0, 0.380, () ->
            startParallelEvent("Intake On", 0)   // the scoopula feeds the intake
        );

        chain3 = new AutoPath(
            Paths.line(POINT_3.toPose(), POINT_4.toPose()).heading(
                headingInterpolator(
                    Math.toRadians(90.000),
                    Math.toRadians(270.000),
                    2.000   // turn late: leave the far-wall flower before swinging (a corner passed 1.0 in from it at 1.0)
                )
            )
        ).addParametricCallback(0, 0.500, () ->
            startParallelEvent("Scoopula Up", scoopulaTransitionMs)
        );

        // Path 5 "Down the field": heading 270 the whole way (operator 2026-11-23, was 270 -> 180) so the intake
        // faces the stray balls on the way down; intake on from the start. Camera pick: at CameraPick.pickY() the
        // latest frame picks the biggest clump against the audience wall and the rest of the path is replaced by
        // a drive into it (followPickStep). Nothing picked: the path ends at POINT_5 as planned.
        chain4 = new AutoPath(
            Paths.curve(
                POINT_4.toPose(),
                new Pose(61.964, 56.009),
                POINT_5.toPose()
            ).heading(
                headingInterpolator(
                    Math.toRadians(270.000),
                    Math.toRadians(270.000),
                    1.000
                )
            )
        ).addParametricCallback(0, 0.000, () ->
            startParallelEvent("Intake On", 0)
        ).addPoseCallback(0, new Pose(POINT_5.x, cameraPick.pickY()), this::decideChain4Pick, 0.5);

        chain5 = buildChain5(POINT_5.toPose());

        chain6 = new AutoPath(
            Paths.line(POINT_6.toPose(), POINT_7.toPose()).heading(
                headingInterpolator(
                    Math.toRadians(180.000),
                    Math.toRadians(180.000),
                    1.000
                )
            )
        ).addParametricCallback(0, 0.370, () ->
            startParallelEvent("Scoopula Up", scoopulaTransitionMs)
        );

        chain7 = new AutoPath(
            Paths.curve(
                POINT_7.toPose(),
                new Pose(11.313, 54.838),
                POINT_8.toPose()
            ).heading(
                headingInterpolator(
                    Math.toRadians(180.000),
                    Math.toRadians(180.000),
                    1.000
                )
            )
        );
    }

    /**
     * Path 6 "To the flower", from wherever path 5 ended (POINT_5, or the wall spot the camera picked); its control
     * point stays where it was planned. The robot has already turned to 180 in place (runTurnStep), so the
     * heading holds 180 (operator 2026-11-23: turn right after the shot, away from the turret wrap).
     */
    private AutoPath buildChain5(Pose start) {
        return new AutoPath(
            Paths.curve(
                start,
                new Pose(49.769, 42.297),
                POINT_6.toPose()
            ).heading(
                headingInterpolator(
                    Math.toRadians(180.000),
                    Math.toRadians(180.000),
                    1.000
                )
            )
        ).addParametricCallback(0, 0.280, () ->
            startParallelEvent("Scoopula Down", scoopulaTransitionMs)
        ).addParametricCallback(0, 0.280, () ->
            startParallelEvent("Intake On", 0)   // the scoopula feeds the intake
        );
    }

    /** Path 5's camera pick: when the path passes CameraPick.pickY(). Builds the drive into the clump, if any. */
    private void decideChain4Pick() {
        Pose end = cameraPick.decide(follower.pose());
        if (end == null) {
            return;
        }
        chain4Pick = new AutoPath(CameraPick.slamPath(follower.pose(), end));
        chain5 = buildChain5(end);
    }

    /** Path 5, switched mid-path to the camera-picked drive once decideChain4Pick() has built it. */
    private void followPickStep(AutoPath path, AutoPath pick, double pathSpeed) {
        if (!stepStarted) {
            chain4Switched = false;
            startPath(path, pathSpeed);
            stepStarted = true;
            resetStallWatch();
        }
        if (pick != null && !chain4Switched) {
            startPath(pick, pathSpeed);
            chain4Switched = true;
            resetStallWatch();
        }
        if (!follower.isBusy() || stalledNearEnd(chain4Switched && pick != null ? pick : path)) {
            startEvent("Intake Off");
            advanceSequence();
        }
    }

    /**
     * Turn first: turn in place to {@code headingDeg} (the next path's end heading) before driving it. Two phases,
     * because Foresight's hold moves and turns AT ONCE - one hold to the backed-off pose at the new heading (until
     * 2026-11-23) started turning while the front corners were still on the wall: (1) back straight off any wall,
     * holding the current heading, until the corners' sweep clears it (TurnInPlace.clearOfWalls) - within
     * TURN_BACKOFF_TOLERANCE_IN or after TURN_BACKOFF_TIMEOUT_MS; (2) turn there - within TURN_TOLERANCE_DEG or after
     * TURN_TIMEOUT_MS. Then {@code rebuild} gets the pose reached, so the next path starts where the robot is.
     */
    private void runTurnStep(double headingDeg, java.util.function.Consumer<Pose> rebuild) {
        long now = System.currentTimeMillis();
        if (!stepStarted) {
            Pose here = follower.pose();
            turnClearPose = TurnInPlace.clearOfWalls(here, Math.toRadians(headingDeg));
            turnBackingOff = Math.hypot(turnClearPose.x() - here.x(), turnClearPose.y() - here.y())
                    > AutoConstants.Autonomous.TURN_BACKOFF_TOLERANCE_IN;
            follower.hold(turnBackingOff
                    ? new Pose(turnClearPose.x(), turnClearPose.y(), here.heading())
                    : turnClearPose);
            stepStartTime = now;
            stepStarted = true;
        }
        Pose pose = follower.pose();
        if (turnBackingOff) {
            boolean clear = Math.hypot(turnClearPose.x() - pose.x(), turnClearPose.y() - pose.y())
                    <= AutoConstants.Autonomous.TURN_BACKOFF_TOLERANCE_IN;
            if (clear || now - stepStartTime >= AutoConstants.Autonomous.TURN_BACKOFF_TIMEOUT_MS) {
                turnBackingOff = false;
                follower.hold(turnClearPose);
                stepStartTime = now;
            }
            return;
        }
        double error = Math.abs(normalizeRadians(Math.toRadians(headingDeg) - pose.heading()));
        if (error <= Math.toRadians(AutoConstants.Autonomous.TURN_TOLERANCE_DEG)
                || now - stepStartTime >= AutoConstants.Autonomous.TURN_TIMEOUT_MS) {
            rebuild.accept(new Pose(pose.x(), pose.y()));
            advanceSequence();
        }
    }

    private void runSequence() {
        if (pathFinished) {
            return;
        }

        switch (sequenceIndex) {
      case 0:
        runTimedEventStep("Shoot", shootTimeMs);
        break;
      case 1:
        followPathStep(chain1, 1.000);
        break;
      case 2:
        runWaitStep(500L);
        break;
      case 3:
        followPathStep(chain2, 1.000);
        break;
      case 4:
        runTimedEventStep("Shoot", shootTimeMs);
        break;
      case 5:
        followPathStep(chain3, 1.000);
        break;
      case 6:
        runWaitStep(2000L);
        break;
      case 7:
        followPickStep(chain4, chain4Pick, 1.000);
        break;
      case 8:
        // Back off the wall and turn BEFORE the shot (operator 2026-11-23): facing the wall, the hive is
        // straight behind the robot - the turret's wrap - so the wall shot could hang unaimed or unwind.
        runTurnStep(180.000, start -> chain5 = buildChain5(start));
        break;
      case 9:
        runTimedEventStep("Shoot", shootTimeMs);
        break;
      case 10:
        followPathStep(chain5, 1.000);
        break;
      case 11:
        followPathStep(chain6, 1.000);
        break;
      case 12:
        runTimedEventStep("Shoot", shootTimeMs);
        break;
      case 13:
        followPathStep(chain7, 1.000);
        break;
            default:
                pathFinished = true;
                activePath = null;
                finishAllParallelEvents();
                follower.manual(DrivePowers.zero());
                break;
        }
    }

    /**
     * Heading from start to end on the curve parameter t, shaped by t^curve - exactly what the
     * visualizer draws, and Pedro 2's setLinearHeadingInterpolation at curve 1. Pedro 3's
     * Path.linear() interpolates on distance fraction instead, which differs on most Beziers.
     */
    private static Interpolator headingInterpolator(
        double startHeading,
        double endHeading,
        double curve
    ) {
        double clampedCurve = Math.max(0.25, Math.min(4.0, curve));
        double deltaHeading = headingDelta(startHeading, endHeading);
        return (pathCurve, t) ->
            normalizeRadians(
                startHeading + deltaHeading * Math.pow(Math.max(0.0, Math.min(1.0, t)), clampedCurve)
            );
    }

    /**
     * The turn from start to end, the way the visualizer draws it (math.ts getAngularDifference): the shorter way,
     * and an exact half turn goes clockwise when the end's 0-360 angle is below the start's (270 -> 90 turns
     * through 180), counter-clockwise otherwise. normalizeRadians alone sent every half turn counter-clockwise,
     * so 270 -> 90 drove through 0 while the editor showed it through 180 (fixed 2026-11-23).
     */
    private static double headingDelta(double startHeading, double endHeading) {
        double s = ((Math.toDegrees(startHeading) % 360.0) + 360.0) % 360.0;
        double e = ((Math.toDegrees(endHeading) % 360.0) + 360.0) % 360.0;
        double diff = Math.round((e - s) * 1e6) / 1e6;   // 270.0000000001 vs 90 must still read as -180
        if (diff > 180.0) diff -= 360.0;
        else if (diff < -180.0) diff += 360.0;
        return Math.toRadians(diff);
    }

    private static double normalizeRadians(double angle) {
        while (angle <= -Math.PI) {
            angle += 2.0 * Math.PI;
        }
        while (angle > Math.PI) {
            angle -= 2.0 * Math.PI;
        }
        return angle;
    }

    private void followPathStep(AutoPath path, double pathSpeed) {
        if (!stepStarted) {
            startPath(path, pathSpeed);
            stepStarted = true;
            resetStallWatch();
        }

        if (!follower.isBusy() || stalledNearEnd(path)) {
            advanceSequence();
        }
    }

    // Stall guard (2026-11-24, blue auto on the field): the follower stays busy until it REACHES the path end,
    // so an end it cannot reach (a wall, a game piece) ate 17 s of AUTO pushing into the far wall. Near the end
    // (AutoConstants.Autonomous.PATH_STALL_NEAR_IN) and not moving (< PATH_STALL_MOVE_IN over PATH_STALL_S), hold
    // where the robot is and move on.
    private Pose stallAnchor;
    private long stallAnchorMs;

    private void resetStallWatch() {
        stallAnchor = null;
    }

    private boolean stalledNearEnd(AutoPath path) {
        Pose now = follower.pose();
        Pose end = path.path.endPose();
        if (Math.hypot(end.x() - now.x(), end.y() - now.y()) > AutoConstants.Autonomous.PATH_STALL_NEAR_IN) {
            stallAnchor = null;
            return false;
        }
        long t = System.currentTimeMillis();
        if (stallAnchor == null
                || Math.hypot(now.x() - stallAnchor.x(), now.y() - stallAnchor.y()) > AutoConstants.Autonomous.PATH_STALL_MOVE_IN) {
            stallAnchor = now;
            stallAnchorMs = t;
            return false;
        }
        if (t - stallAnchorMs < AutoConstants.Autonomous.PATH_STALL_S * 1000) {
            return false;
        }
        com.qualcomm.robotcore.util.RobotLog.ww(getClass().getSimpleName(), String.format(java.util.Locale.US,
                "path stalled %.1f in short of its end at (%.1f, %.1f) - holding here and moving on",
                Math.hypot(end.x() - now.x(), end.y() - now.y()), now.x(), now.y()));
        follower.hold(now);
        return true;
    }

    /** Pedro 2's followPath(path, maxPower, holdEnd = true), callbacks re-armed as it did. */
    private void startPath(AutoPath path, double pathSpeed) {
        path.arm();
        activePath = path;
        follower.holdEnd.set(true);
        follower.follow(withPathSpeed(path.path, clampPathSpeed(pathSpeed)));
    }

    /**
     * Pedro 2 capped drive POWER per path; Pedro 3 has no power cap. The nearest thing is
     * Foresight's maxPathSpeed - a fraction of max achievable VELOCITY - applied only while
     * this path is followed. At 1.0 nothing is applied.
     */
    private Path withPathSpeed(Path path, double pathSpeed) {
        if (pathSpeed >= 1.0 || !(follower.algorithm() instanceof Foresight)) {
            return path;
        }
        return path.with(((Foresight) follower.algorithm()).config.maxPathSpeed.at(pathSpeed));
    }

    private void followRepeatStep(
        AutoPath[] repeatPaths,
        double[] repeatPathSpeeds,
        long[] repeatHoldMs,
        String[] repeatHoldEvents,
        int repeatCount,
        int repeatSlot
    ) {
        if (
            repeatSlot < 0 ||
            repeatSlot >= REPEAT_LOOP_CAPACITY ||
            repeatPaths == null ||
            repeatPaths.length == 0 ||
            repeatCount <= 0
        ) {
            advanceSequence();
            return;
        }

        int pathIndex = Math.max(0, Math.min(repeatPaths.length - 1, repeatLoopPathIndexes[repeatSlot]));
        AutoPath path = repeatPaths[pathIndex];
        double pathSpeed =
            repeatPathSpeeds != null && pathIndex < repeatPathSpeeds.length
                ? repeatPathSpeeds[pathIndex]
                : 1.0;

        // A null chain marks a wait or an event sitting between two paths: the
        // slot holds a duration instead of something to drive.
        if (path == null) {
            long holdMs =
                repeatHoldMs != null && pathIndex < repeatHoldMs.length
                    ? repeatHoldMs[pathIndex]
                    : 0L;

            if (!stepStarted) {
                stepStartTime = System.currentTimeMillis();
                stepStarted = true;

                String eventName =
                    repeatHoldEvents != null && pathIndex < repeatHoldEvents.length
                        ? repeatHoldEvents[pathIndex]
                        : null;
                if (eventName != null) {
                    startParallelEvent(eventName, holdMs);
                }
            }

            if (System.currentTimeMillis() - stepStartTime < holdMs) {
                return;
            }

            stepStarted = false;
            repeatLoopPathIndexes[repeatSlot]++;

            if (repeatLoopPathIndexes[repeatSlot] >= repeatPaths.length) {
                repeatLoopPathIndexes[repeatSlot] = 0;
                repeatLoopIterations[repeatSlot]++;
            }

            if (repeatLoopIterations[repeatSlot] >= repeatCount) {
                repeatLoopIterations[repeatSlot] = 0;
                repeatLoopPathIndexes[repeatSlot] = 0;
                advanceSequence();
            }
            return;
        }

        if (!stepStarted) {
            startPath(path, pathSpeed);
            stepStarted = true;
        }

        if (follower.isBusy()) {
            return;
        }

        stepStarted = false;
        repeatLoopPathIndexes[repeatSlot]++;

        if (repeatLoopPathIndexes[repeatSlot] >= repeatPaths.length) {
            repeatLoopPathIndexes[repeatSlot] = 0;
            repeatLoopIterations[repeatSlot]++;
        }

        if (repeatLoopIterations[repeatSlot] >= repeatCount) {
            repeatLoopIterations[repeatSlot] = 0;
            repeatLoopPathIndexes[repeatSlot] = 0;
            advanceSequence();
        }
    }

    private double clampPathSpeed(double pathSpeed) {
        return Math.max(0.05, Math.min(1.0, pathSpeed));
    }

    private void runWaitStep(long durationMs) {
        if (!stepStarted) {
            stepStartTime = System.currentTimeMillis();
            stepStarted = true;
        }

        if (System.currentTimeMillis() - stepStartTime >= durationMs) {
            advanceSequence();
        }
    }

    private void runTimedEventStep(String eventName, long durationMs) {
        if (!stepStarted) {
            stepStartTime = System.currentTimeMillis();
            startEvent(eventName);
            stepStarted = true;
        }

        // Hand-wired to AutoMechanisms (a Shoot step waits for the shot) - re-apply after a visualizer export.
        long elapsedMs = System.currentTimeMillis() - stepStartTime;
        if (mechanisms != null ? mechanisms.eventDone(eventName, elapsedMs, durationMs) : elapsedMs >= durationMs) {
            finishEvent(eventName);
            advanceSequence();
        }
    }

    private void startParallelEvent(String eventName, long durationMs) {
        startEvent(eventName);

        long clampedDurationMs = Math.max(0L, durationMs);
        if (clampedDurationMs == 0L) {
            // A zero duration means "fire once" -- finish immediately instead of leaving the
            // event permanently active, since updateParallelEvents() never finishes an event
            // with a non-positive duration.
            finishEvent(eventName);
            return;
        }

        long now = System.currentTimeMillis();
        int slot = -1;

        for (int i = 0; i < activeParallelEventNames.length; i++) {
            if (eventName.equals(activeParallelEventNames[i])) {
                slot = i;
                break;
            }
        }

        if (slot < 0) {
            for (int i = 0; i < activeParallelEventNames.length; i++) {
                if (activeParallelEventNames[i] == null) {
                    slot = i;
                    break;
                }
            }
        }

        if (slot < 0) {
            return;
        }

        activeParallelEventNames[slot] = eventName;
        activeParallelEventStartTimes[slot] = now;
        activeParallelEventDurations[slot] = clampedDurationMs;
    }

    private void updateParallelEvents() {
        long now = System.currentTimeMillis();

        for (int i = 0; i < activeParallelEventNames.length; i++) {
            String eventName = activeParallelEventNames[i];
            if (eventName == null || activeParallelEventDurations[i] <= 0L) {
                continue;
            }

            if (now - activeParallelEventStartTimes[i] >= activeParallelEventDurations[i]) {
                finishEvent(eventName);
                clearParallelEvent(i);
            }
        }
    }

    private void finishAllParallelEvents() {
        for (int i = 0; i < activeParallelEventNames.length; i++) {
            String eventName = activeParallelEventNames[i];
            if (eventName != null) {
                finishEvent(eventName);
                clearParallelEvent(i);
            }
        }
    }

    private void resetParallelEvents() {
        for (int i = 0; i < activeParallelEventNames.length; i++) {
            clearParallelEvent(i);
        }
    }

    private void resetRepeatLoops() {
        for (int i = 0; i < REPEAT_LOOP_CAPACITY; i++) {
            repeatLoopIterations[i] = 0;
            repeatLoopPathIndexes[i] = 0;
        }
    }

    private void clearParallelEvent(int index) {
        activeParallelEventNames[index] = null;
        activeParallelEventStartTimes[index] = 0L;
        activeParallelEventDurations[index] = 0L;
    }

    private void startEvent(String eventName) {
        if (mechanisms != null) {
            mechanisms.start(eventName);
        }
    }

    private void finishEvent(String eventName) {
        if (mechanisms != null) {
            mechanisms.finish(eventName);
        }
    }

    

    private void advanceSequence() {
        sequenceIndex++;
        stepStarted = false;
        // Recording (2026-11-24): one logcat line per step - the run's timeline (tag = the class name).
        Pose p = follower.pose();
        com.qualcomm.robotcore.util.RobotLog.ii(getClass().getSimpleName(), String.format(java.util.Locale.US,
                "step %d at (%.1f, %.1f, %.0f deg) %s", sequenceIndex, p.x(), p.y(), Math.toDegrees(p.heading()),
                mechanisms != null ? mechanisms.stateLine() : ""));
    }

    private void updateTelemetry(String state) {
        if (!org.firstinspires.ftc.teamcode.Config.telemetryEnabled()) {
            return;
        }
        Pose pose = follower.pose();

        telemetry.addData("State", state);
        telemetry.addData("Sequence", sequenceIndex);
        telemetry.addData("X", "%.2f", pose.x());
        telemetry.addData("Y", "%.2f", pose.y());
        telemetry.addData("Heading", "%.2f", Math.toDegrees(pose.heading()));
        if (cameraPick != null) {
            telemetry.addData("Camera pick", cameraPick.note() + " · " + cameraPick.status());
        }
        if (mechanisms != null) {
            mechanisms.addTelemetry(telemetry);
        }
        telemetry.update();
    }

    /**
     * A Pedro 3 path plus the per-path callbacks Pedro 2's PathBuilder had and Pedro 3 dropped.
     *
     * <p>Segment indexes count the paths of a chain (Paths.path(...)) from 0, as Pedro 2
     * attached a callback to the path it was added after. Each trigger keeps Pedro 2's rule:
     * <ul>
     *   <li>parametric - once the segment's DISTANCE completion reaches the value (Pedro 2's
     *       getPathCompletion()), at the parametric end, or once the follower has left it;
     *   <li>temporal - that many milliseconds after the segment began; dropped if the segment
     *       ends first, as Pedro 2 dropped it with its path;
     *   <li>pose - once the follower's curve parameter passes the point on the segment closest
     *       to the pose, or once the follower has left it.
     * </ul>
     */
    private static final class AutoPath {
        private static final int PARAMETRIC = 0;
        private static final int TEMPORAL = 1;
        private static final int POSE = 2;

        final Path path;
        private final List<PathSegment> segments;
        private final List<Trigger> triggers = new ArrayList<>();
        private int segmentIndex;
        private long segmentStartMs;

        AutoPath(Path path) {
            this.path = path;
            this.segments = path.getSegments();
        }

        AutoPath addParametricCallback(int segment, double completion, Runnable action) {
            triggers.add(new Trigger(PARAMETRIC, segment, completion, action));
            return this;
        }

        AutoPath addTemporalCallback(int segment, long delayMs, Runnable action) {
            triggers.add(new Trigger(TEMPORAL, segment, delayMs, action));
            return this;
        }

        AutoPath addPoseCallback(int segment, Pose pose, Runnable action, double initialGuess) {
            double t = segments.get(segment).curve.closestParameter(pose.toVector2D(), initialGuess);
            triggers.add(new Trigger(POSE, segment, t, action));
            return this;
        }

        /** Call as the path is handed to the follower; Pedro 2 re-armed callbacks on every followPath. */
        void arm() {
            segmentIndex = 0;
            segmentStartMs = System.currentTimeMillis();
            for (Trigger trigger : triggers) {
                trigger.fired = false;
            }
        }

        /** Call once per loop, after follower.update(). */
        void update(Follower follower) {
            long now = System.currentTimeMillis();
            // Once the follower stops following this path, every segment counts as left.
            int index = follower.following() ? follower.pathIndex() : segments.size();
            if (index != segmentIndex) {
                segmentIndex = index;
                segmentStartMs = now;
            }

            PathSegment current = index < segments.size() ? follower.currentSegment() : null;
            double parameter = current != null
                ? Math.max(0.0, Math.min(1.0, follower.parametricCompletion()))
                : 1.0;
            double completion = current != null ? current.curve.pathCompletion(parameter) : 1.0;
            boolean atEnd = follower.atParametricEnd();

            for (Trigger trigger : triggers) {
                if (trigger.fired) {
                    continue;
                }
                boolean left = trigger.segment < segmentIndex;
                boolean here = trigger.segment == segmentIndex;
                boolean ready;
                if (trigger.kind == TEMPORAL) {
                    if (left) {
                        trigger.fired = true;
                        continue;
                    }
                    ready = here && now - segmentStartMs >= trigger.value;
                } else if (trigger.kind == POSE) {
                    ready = left || (here && parameter >= trigger.value);
                } else {
                    ready = left || (here && (atEnd || completion >= trigger.value));
                }
                if (ready) {
                    trigger.fired = true;
                    trigger.action.run();
                }
            }
        }

        private static final class Trigger {
            final int kind;
            final int segment;
            final double value;
            final Runnable action;
            boolean fired;

            Trigger(int kind, int segment, double value, Runnable action) {
                this.kind = kind;
                this.segment = segment;
                this.value = value;
                this.action = action;
            }
        }
    }
}