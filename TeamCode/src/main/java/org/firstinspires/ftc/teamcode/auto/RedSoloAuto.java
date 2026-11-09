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
import org.firstinspires.ftc.teamcode.helpers.Alliance;
import org.firstinspires.ftc.teamcode.helpers.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@Autonomous(name = "Redpollenauto", group = "Auto")
public class RedSoloAuto extends OpMode {
    // PathStep fields are editable from FTC Dashboard; paths are built in init(), so edits apply on the next init.
    public static PathStep START_STEP = new PathStep(56.000, 8.561, 180);
    public static PathStep POINT_1 = new PathStep(9.908, 8.180, 180);
    public static PathStep POINT_2 = new PathStep(10.222, 10.299, 270);
    public static PathStep POINT_3 = new PathStep(46.480, 125.857, 90);
    public static PathStep POINT_4 = new PathStep(58.812, 115.566, 270);
    public static PathStep POINT_5 = new PathStep(57.943, 10.419, 270);
    public static PathStep POINT_6 = new PathStep(15.442, 47.215, 180);
    public static PathStep POINT_7 = new PathStep(45.308, 11.717, 180);
    public static PathStep POINT_8 = new PathStep(10.466, 101.157, 180);

    private Follower follower;
    private AutoMechanisms mechanisms;
    private AutoPath chain1;
    private AutoPath chain2;
    private AutoPath chain3;
    private AutoPath chain4;
    private AutoPath chain5;
    private AutoPath chain6;
    private AutoPath chain7;
    private AutoPath chain8;
    
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
    private static final int PARALLEL_EVENT_CAPACITY = 4;
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
        mechanisms = new AutoMechanisms(hardwareMap, follower, Alliance.RED);

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

        follower.setPose(START_STEP.toPose());
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

        updateTelemetry(pathFinished ? "Done" : "Running");
    }

    @Override
    public void stop() {
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
      chain1 = new AutoPath(
            Paths.line(
              START_STEP.toPose(),
              POINT_1.toPose()
            ).heading(headingInterpolator(Math.toRadians(180), Math.toRadians(180), 1.000))
          );

      chain2 = new AutoPath(
            Paths.curve(
              POINT_1.toPose(),
              new Pose(26.850, 28.589),
              new Pose(12.186, 24.797),
              POINT_2.toPose()
            ).heading(headingInterpolator(Math.toRadians(180.000), Math.toRadians(270), 1.000))
          );

      chain3 = new AutoPath(
            Paths.curve(
              POINT_2.toPose(),
              new Pose(20.787, 41.881),
              new Pose(24.377, 92.484),
              new Pose(48.724, 111.429),
              POINT_3.toPose()
            ).heading(headingInterpolator(Math.toRadians(270.000), Math.toRadians(90), 0.700))
          )
          .addParametricCallback(0, 0.380, () -> startParallelEvent("Scoopula Down", scoopulaTransitionMs));

      chain4 = new AutoPath(
            Paths.line(
              POINT_3.toPose(),
              POINT_4.toPose()
            ).heading(headingInterpolator(Math.toRadians(90.000), Math.toRadians(270), 1.000))
          )
          .addParametricCallback(0, 0.500, () -> startParallelEvent("Scoopula Up", scoopulaTransitionMs));

      chain5 = new AutoPath(
            Paths.line(
              POINT_4.toPose(),
              POINT_5.toPose()
            ).heading(headingInterpolator(Math.toRadians(270.000), Math.toRadians(270), 1.000))
          );

      chain6 = new AutoPath(
            Paths.curve(
              POINT_5.toPose(),
              new Pose(49.769, 42.297),
              POINT_6.toPose()
            ).heading(headingInterpolator(Math.toRadians(270.000), Math.toRadians(180), 0.600))
          )
          .addParametricCallback(0, 0.280, () -> startParallelEvent("Scoopula Down", scoopulaTransitionMs));

      chain7 = new AutoPath(
            Paths.line(
              POINT_6.toPose(),
              POINT_7.toPose()
            ).heading(headingInterpolator(Math.toRadians(180.000), Math.toRadians(180), 1.000))
          )
          .addParametricCallback(0, 0.370, () -> startParallelEvent("Scoopula Up", scoopulaTransitionMs));

      chain8 = new AutoPath(
            Paths.curve(
              POINT_7.toPose(),
              new Pose(11.313, 54.838),
              POINT_8.toPose()
            ).heading(headingInterpolator(Math.toRadians(180.000), Math.toRadians(180), 1.000))
          );

      
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
        followPathStep(chain2, 1.000);
        break;
      case 3:
        runWaitStep(500L);
        break;
      case 4:
        followPathStep(chain3, 1.000);
        break;
      case 5:
        runTimedEventStep("Shoot", shootTimeMs);
        break;
      case 6:
        followPathStep(chain4, 1.000);
        break;
      case 7:
        runWaitStep(2000L);
        break;
      case 8:
        followPathStep(chain5, 1.000);
        break;
      case 9:
        runTimedEventStep("Shoot", shootTimeMs);
        break;
      case 10:
        followPathStep(chain6, 1.000);
        break;
      case 11:
        followPathStep(chain7, 1.000);
        break;
      case 12:
        runTimedEventStep("Shoot", shootTimeMs);
        break;
      case 13:
        followPathStep(chain8, 1.000);
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
        double deltaHeading = normalizeRadians(endHeading - startHeading);
        return (pathCurve, t) ->
            normalizeRadians(
                startHeading + deltaHeading * Math.pow(Math.max(0.0, Math.min(1.0, t)), clampedCurve)
            );
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
        }

        if (!follower.isBusy()) {
            advanceSequence();
        }
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

        if (System.currentTimeMillis() - stepStartTime >= durationMs) {
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