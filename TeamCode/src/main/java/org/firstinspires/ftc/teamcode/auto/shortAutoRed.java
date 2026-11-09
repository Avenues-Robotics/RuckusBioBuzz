package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
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


import org.firstinspires.ftc.teamcode.helpers.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@Configurable
@Autonomous(name = "Short RED auto", group = "Auto")
public class shortAutoRed extends OpMode {
    // PathStep fields are editable from FTC Dashboard; paths are built in init(), so edits apply on the next init.
    private static final double NUMBER_INTAKE_TIME = 1500; // These constants will probably carry over to other Autos. Consider making an AutoConstants class to hold these.
    private static final double NUMBER_SHOOT_TIME = 2000;
    private static final double NUMBER_SCOOPULA_WAIT_TIME = 2000;
    public static PathStep START_STEP = new PathStep(56.000, 8.000, 180);
    public static PathStep POINT_1 = new PathStep(8.537, 9.096, 270);
    public static PathStep POINT_2 = new PathStep(47.429, 125.030, 90);
    public static PathStep POINT_3 = new PathStep(47.444, 129.059, 0.000);
    public static PathStep POINT_4 = new PathStep(13.232, 117.092, 0.000);

    /** One state per step of the auto. Each case in runSequence() handles exactly one of these. */
    private enum AutoState {
        DRIVE_TO_CORNER_INTAKE,   // chain1: shoot at start, intake at 53% of the path
        DRIVE_TO_GARDEN,          // chain2: shoot at 75% of the path
        LOWER_SCOOPULA,           // "scoopula" event, timed
        DRIVE_FORWARD_TO_FLOWER,  // chain3
        INTAKE_FROM_FLOWER,       // wait for intake time
        SHOOT,                    // "Shoot" event, timed
        PARK,                     // chain4
        DONE
    }

    private Follower follower;
    private AutoPath chain1;
    private AutoPath chain2;
    private AutoPath chain3;
    private AutoPath chain4;

    private AutoPath activePath;
    private AutoState state;
    private long stepStartTime;
    private boolean stepStarted;
    private boolean stepDone;
    private boolean pathFinished;
    private static final int PARALLEL_EVENT_CAPACITY = 3;
    private final String[] activeParallelEventNames = new String[PARALLEL_EVENT_CAPACITY];
    private final long[] activeParallelEventStartTimes = new long[PARALLEL_EVENT_CAPACITY];
    private final long[] activeParallelEventDurations = new long[PARALLEL_EVENT_CAPACITY];

    @Override
    public void init() {
        // Before anything that can throw: a pose left by an earlier auto must not reach TeleOp.
        PoseStorage.clear();
        // Pedro 3 follows paths with Foresight, whose braking model and gains must be measured
        // on the robot first. Refuse to build a path-following OpMode on placeholders.
        Constants.requireForesightMeasured();

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(START_STEP.toPose());

        buildPaths();
        state = AutoState.DRIVE_TO_CORNER_INTAKE;
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
        pathFinished = false;
        activePath = null;
        resetParallelEvents();
        setState(AutoState.DRIVE_TO_CORNER_INTAKE);

        follower.setPose(START_STEP.toPose());
    }

    @Override
    public void loop() {
        follower.update();
        // Path callbacks run right after the follower update, where Pedro 2 ran them.
        if (activePath != null) {
            activePath.update(follower);
        }
        updateParallelEvents();

        runSequence();

        updateTelemetry(pathFinished ? "Done" : "Running");
    }

    @Override
    public void stop() {
        finishAllParallelEvents();

        if (follower == null) {
            return;
        }

        follower.manual(DrivePowers.zero());
        follower.update();
        PoseStorage.pose = follower.pose();
    }

    private void buildPaths() {
        chain1 = new AutoPath(
                Paths.curve(
                        START_STEP.toPose(),
                        new Pose(32.855, 37.609),
                        POINT_1.toPose()
                ).heading(headingInterpolator(Math.toRadians(180), Math.toRadians(270), 1.000))
        )
                .addParametricCallback(0, 0.000, () -> startParallelEvent("shoot", Math.round(NUMBER_SHOOT_TIME)))
                .addParametricCallback(0, 0.530, () -> startParallelEvent("intake", Math.round(NUMBER_INTAKE_TIME)));

        chain2 = new AutoPath(
                Paths.curve(
                        POINT_1.toPose(),
                        new Pose(41.433, 47.710),
                        new Pose(9.138, 117.000),
                        POINT_2.toPose()
                ).heading(headingInterpolator(Math.toRadians(180.000), Math.toRadians(90), 1.450))
        )
                .addParametricCallback(0, 0.75, () -> startParallelEvent("shoot", Math.round(NUMBER_SHOOT_TIME)));

        chain3 = new AutoPath(
                Paths.line(
                        POINT_2.toPose(),
                        POINT_3.toPose()
                ).tangent()
        );

        chain4 = new AutoPath(
                Paths.curve(
                        POINT_3.toPose(),
                        new Pose(53.785, 111.397),
                        POINT_4.toPose()
                ).heading(headingInterpolator(Math.toRadians(89.783), Math.toRadians(0.000), 4.000))
        );
    }

    private void runSequence() {
        if (pathFinished) {
            return;
        }

        switch (state) {
            case DRIVE_TO_CORNER_INTAKE:
                followPathStep(chain1, 0.500);
                if (stepDone) setState(AutoState.DRIVE_TO_GARDEN);
                break;
            case DRIVE_TO_GARDEN:
                followPathStep(chain2, 1.000);
                if (stepDone) setState(AutoState.LOWER_SCOOPULA);
                break;
            case LOWER_SCOOPULA:
                runTimedEventStep("scoopula", Math.round(NUMBER_SCOOPULA_WAIT_TIME));
                if (stepDone) setState(AutoState.DRIVE_FORWARD_TO_FLOWER);
                break;
            case DRIVE_FORWARD_TO_FLOWER:
                followPathStep(chain3, 1.000);
                if (stepDone) setState(AutoState.INTAKE_FROM_FLOWER);
                break;
            case INTAKE_FROM_FLOWER:
                runWaitStep(Math.round(NUMBER_INTAKE_TIME));
                if (stepDone) setState(AutoState.SHOOT);
                break;
            case SHOOT:
                runTimedEventStep("Shoot", Math.round(NUMBER_SHOOT_TIME));
                if (stepDone) setState(AutoState.PARK);
                break;
            case PARK:
                followPathStep(chain4, 1.000);
                if (stepDone) setState(AutoState.DONE);
                break;
            case DONE:
            default:
                pathFinished = true;
                activePath = null;
                finishAllParallelEvents();
                follower.manual(DrivePowers.zero());
                break;
        }
    }

    private void setState(AutoState next) {
        state = next;
        stepStarted = false;
        stepDone = false;
    }

    private void finishStep() {
        stepDone = true;
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
            finishStep();
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

    private double clampPathSpeed(double pathSpeed) {
        return Math.max(0.05, Math.min(1.0, pathSpeed));
    }

    private void runWaitStep(long durationMs) {
        if (!stepStarted) {
            stepStartTime = System.currentTimeMillis();
            stepStarted = true;
        }

        if (System.currentTimeMillis() - stepStartTime >= durationMs) {
            finishStep();
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
            finishStep();
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

    private void clearParallelEvent(int index) {
        activeParallelEventNames[index] = null;
        activeParallelEventStartTimes[index] = 0L;
        activeParallelEventDurations[index] = 0L;
    }

    private void startEvent(String eventName) {
        switch (eventName) {
            case "scoopula":
                startScoopula();
                break;
            case "Shoot":
                startShoot();
                break;
            case "shoot":
                startShoot2();
                break;
            case "intake":
                startIntake();
                break;
            default:
                break;
        }
    }

    private void finishEvent(String eventName) {
        switch (eventName) {
            case "scoopula":
                finishScoopula();
                break;
            case "Shoot":
                finishShoot();
                break;
            case "shoot":
                finishShoot2();
                break;
            case "intake":
                finishIntake();
                break;
            default:
                break;
        }
    }

    private void startScoopula() {
        // TODO: start scoopula mechanism here.
    }

    private void finishScoopula() {
        // TODO: stop scoopula mechanism here.
    }

    private void startShoot() {
        // TODO: start Shoot mechanism here.
    }

    private void finishShoot() {
        // TODO: stop Shoot mechanism here.
    }

    private void startShoot2() {
        // TODO: start shoot mechanism here.
    }

    private void finishShoot2() {
        // TODO: stop shoot mechanism here.
    }

    private void startIntake() {
        // TODO: start intake mechanism here.
    }

    private void finishIntake() {
        // TODO: stop intake mechanism here.
    }

    private void updateTelemetry(String status) {
        Pose pose = follower.pose();

        telemetry.addData("Status", status);
        telemetry.addData("State", state);
        telemetry.addData("X", "%.2f", pose.x());
        telemetry.addData("Y", "%.2f", pose.y());
        telemetry.addData("Heading", "%.2f", Math.toDegrees(pose.heading()));
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