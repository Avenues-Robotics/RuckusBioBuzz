import { TURN_SETTLE_SECONDS, TURN_TIMEOUT_SECONDS } from "./cameraPick";
import type {
  Point,
  BasePoint,
  ChainProfile,
  Line,
  Settings,
  TimePrediction,
  TimelineEvent,
  SequenceItem,
  SequenceWaitItem,
  Variable,
} from "../types";
import {
  getCurvePoint,
  getLineStartHeading,
  getLineEndHeading,
  getAngularDifference,
  goalHeadingAt,
  headingCatchUpFraction,
} from "./math";
import { buildChainRuns, buildRoute } from "./sequence";
import {
  buildMotionProfile,
  curveParameterAtDistance,
  profileDistanceAtTime,
  profilePointsFromPolyline,
  profileTimeAtDistance,
  profileVelocityAtDistance,
} from "./motionProfile";
import { buildExpressionScope } from "./numberExpressions";
import { planPollenStep } from "./pollenVision";
import { isEnabled } from "./variables";

export interface TravelLineTimingMeta {
  executionIndex: number;
  lineIndex: number;
  /** 0-based repeat iteration; always 0 outside a repeat loop. */
  iteration: number;
  /** The PathChain this path is driven as part of, when there is one. */
  chain?: ChainProfile;
  line: Line;
  startPoint: BasePoint;
  length: number;
  duration: number;
  startTime: number;
  endTime: number;
  maxVelocity: number;
  maxAcceleration: number;
  maxDeceleration: number;
  hasMotionProfile: boolean;
}

export interface EventTimingWindow {
  name: string;
  lineIndex: number;
  lineName: string;
  markerIndex: number;
  triggerType: "parametric" | "temporal" | "pose";
  startTime: number;
  endTime: number;
  durationMs: number;
  startPercent: number;
  endPercent: number;
  triggerPathPercent: number;
}

/**
 * Samples per path used to build the arc-length table.
 *
 * The table is inverted to turn a distance back into a curve parameter, and it
 * is the *slope* of that inversion the animation rides. A Bezier with bunched
 * control points can cover ground twenty times faster at one end than the
 * other, so a coarse table makes the speed step at every entry — enough of them
 * and the robot visibly stutters along an otherwise smooth path.
 */
const CHAIN_PROFILE_SAMPLES = 200;
/** Spacing, in inches, of the evenly spread samples the profile is built on. */
const CHAIN_SAMPLE_STEP = 0.5;
const MAX_CHAIN_SAMPLES = 2000;
const DEFAULT_MIN_CORNER_RADIUS = 8;

function clampNumber(value: number, min: number, max: number) {
  if (!Number.isFinite(value)) return min;
  return Math.max(min, Math.min(max, value));
}

/**
 * Calculate the length of a curve by sampling points
 */
export function calculateCurveLength(
  start: BasePoint,
  controlPoints: BasePoint[],
  end: BasePoint,
  samples: number = 100,
): number {
  let length = 0;
  let prevPoint: BasePoint = start;

  for (let i = 1; i <= samples; i++) {
    const t = i / samples;
    const point = getCurvePoint(t, [start, ...controlPoints, end]);
    const dx = point.x - prevPoint.x;
    const dy = point.y - prevPoint.y;
    length += Math.sqrt(dx * dx + dy * dy);
    prevPoint = point;
  }

  return length;
}

/**
 * Calculate time for a motion profile (trapezoidal or triangular)
 */
export function calculateMotionProfileTime(
  distance: number,
  maxVel: number,
  maxAcc: number,
  maxDec?: number,
): number {
  const totalDistance = Math.max(0, Number(distance) || 0);
  const velocity = Math.max(0, Number(maxVel) || 0);
  const acceleration = Math.max(0, Number(maxAcc) || 0);
  const deceleration = Math.max(0, Number(maxDec ?? maxAcc) || 0);

  // Matches the degenerate-input contract of calculateMotionProfileDistanceAtTime /
  // calculateMotionProfileTimeAtDistance: a zero distance/velocity/acceleration/deceleration
  // would otherwise divide by zero (NaN/Infinity), which then poisons every downstream
  // accumulated timeline value.
  if (
    totalDistance <= 0 ||
    velocity <= 0 ||
    acceleration <= 0 ||
    deceleration <= 0
  ) {
    return 0;
  }

  const accDist = (velocity * velocity) / (2 * acceleration);
  const decDist = (velocity * velocity) / (2 * deceleration);

  if (totalDistance >= accDist + decDist) {
    const accTime = velocity / acceleration;
    const decTime = velocity / deceleration;
    const constDist = totalDistance - accDist - decDist;
    const constTime = constDist / velocity;

    return accTime + constTime + decTime;
  } else {
    const vPeak = Math.sqrt(
      (2 * totalDistance * acceleration * deceleration) / (acceleration + deceleration),
    );
    const accTime = vPeak / acceleration;
    const decTime = vPeak / deceleration;

    return accTime + decTime;
  }
}

export function calculateMotionProfileDistanceAtTime(
  elapsedTime: number,
  distance: number,
  maxVel: number,
  maxAcc: number,
  maxDec?: number,
): number {
  const totalDistance = Math.max(0, Number(distance) || 0);
  const velocity = Math.max(0, Number(maxVel) || 0);
  const acceleration = Math.max(0, Number(maxAcc) || 0);
  const deceleration = Math.max(0, Number(maxDec ?? maxAcc) || 0);

  if (
    totalDistance <= 0 ||
    velocity <= 0 ||
    acceleration <= 0 ||
    deceleration <= 0
  ) {
    return 0;
  }

  const t = Math.max(0, Number(elapsedTime) || 0);
  const accDist = (velocity * velocity) / (2 * acceleration);
  const decDist = (velocity * velocity) / (2 * deceleration);

  if (totalDistance >= accDist + decDist) {
    const accTime = velocity / acceleration;
    const decTime = velocity / deceleration;
    const constDist = totalDistance - accDist - decDist;
    const constTime = constDist / velocity;
    const totalTime = accTime + constTime + decTime;
    const clampedT = Math.min(t, totalTime);

    if (clampedT <= accTime) {
      return 0.5 * acceleration * clampedT * clampedT;
    }

    if (clampedT <= accTime + constTime) {
      return accDist + velocity * (clampedT - accTime);
    }

    const decT = clampedT - accTime - constTime;
    return Math.min(
      totalDistance,
      accDist + constDist + velocity * decT - 0.5 * deceleration * decT * decT,
    );
  }

  const vPeak = Math.sqrt(
    (2 * totalDistance * acceleration * deceleration) /
      (acceleration + deceleration),
  );
  const accTime = vPeak / acceleration;
  const decTime = vPeak / deceleration;
  const totalTime = accTime + decTime;
  const clampedT = Math.min(t, totalTime);

  if (clampedT <= accTime) {
    return 0.5 * acceleration * clampedT * clampedT;
  }

  const decT = clampedT - accTime;
  const peakDistance = 0.5 * acceleration * accTime * accTime;
  return Math.min(
    totalDistance,
    peakDistance + vPeak * decT - 0.5 * deceleration * decT * decT,
  );
}

export function calculateMotionProfileTimeAtDistance(
  distanceTraveled: number,
  distance: number,
  maxVel: number,
  maxAcc: number,
  maxDec?: number,
): number {
  const totalDistance = Math.max(0, Number(distance) || 0);
  const traveled = clampNumber(Number(distanceTraveled) || 0, 0, totalDistance);
  const velocity = Math.max(0, Number(maxVel) || 0);
  const acceleration = Math.max(0, Number(maxAcc) || 0);
  const deceleration = Math.max(0, Number(maxDec ?? maxAcc) || 0);

  if (
    totalDistance <= 0 ||
    velocity <= 0 ||
    acceleration <= 0 ||
    deceleration <= 0
  ) {
    return 0;
  }

  const accDist = (velocity * velocity) / (2 * acceleration);
  const decDist = (velocity * velocity) / (2 * deceleration);

  if (totalDistance >= accDist + decDist) {
    const accTime = velocity / acceleration;
    const decTime = velocity / deceleration;
    const constDist = totalDistance - accDist - decDist;
    const constTime = constDist / velocity;
    const totalTime = accTime + constTime + decTime;

    if (traveled <= accDist) {
      return Math.sqrt((2 * traveled) / acceleration);
    }

    if (traveled <= accDist + constDist) {
      return accTime + (traveled - accDist) / velocity;
    }

    const remainingDistance = Math.max(0, totalDistance - traveled);
    return totalTime - Math.sqrt((2 * remainingDistance) / deceleration);
  }

  const vPeak = Math.sqrt(
    (2 * totalDistance * acceleration * deceleration) /
      (acceleration + deceleration),
  );
  const accTime = vPeak / acceleration;
  const decTime = vPeak / deceleration;
  const totalTime = accTime + decTime;
  const peakDistance = 0.5 * acceleration * accTime * accTime;

  if (traveled <= peakDistance) {
    return Math.sqrt((2 * traveled) / acceleration);
  }

  const remainingDistance = Math.max(0, totalDistance - traveled);
  return totalTime - Math.sqrt((2 * remainingDistance) / deceleration);
}

export function calculateMotionProfileVelocityAtDistance(
  distanceTraveled: number,
  distance: number,
  maxVel: number,
  maxAcc: number,
  maxDec?: number,
): number {
  const totalDistance = Math.max(0, Number(distance) || 0);
  const velocity = Math.max(0, Number(maxVel) || 0);
  const acceleration = Math.max(0, Number(maxAcc) || 0);
  const deceleration = Math.max(0, Number(maxDec ?? maxAcc) || 0);

  if (
    totalDistance <= 0 ||
    velocity <= 0 ||
    acceleration <= 0 ||
    deceleration <= 0
  ) {
    return 0;
  }

  const traveled = Math.max(0, Math.min(totalDistance, Number(distanceTraveled) || 0));
  const accDist = (velocity * velocity) / (2 * acceleration);
  const decDist = (velocity * velocity) / (2 * deceleration);

  if (totalDistance >= accDist + decDist) {
    if (traveled <= accDist) {
      return Math.sqrt(2 * acceleration * traveled);
    }

    if (traveled <= totalDistance - decDist) {
      return velocity;
    }

    return Math.sqrt(2 * deceleration * Math.max(0, totalDistance - traveled));
  }

  const vPeak = Math.sqrt(
    (2 * totalDistance * acceleration * deceleration) /
      (acceleration + deceleration),
  );
  const peakDistance = (vPeak * vPeak) / (2 * acceleration);

  if (traveled <= peakDistance) {
    return Math.sqrt(2 * acceleration * traveled);
  }

  return Math.sqrt(2 * deceleration * Math.max(0, totalDistance - traveled));
}

export function getPathSpeed(line: Line): number {
  const speed = Number(line.speed ?? 1);
  if (!Number.isFinite(speed)) return 1;
  return Math.max(0.05, Math.min(1, speed));
}

function sampleLineCurve(
  sourceStartPoint: BasePoint,
  line: Line,
  samples = 100,
) {
  const sampleCount = Math.max(1, Math.round(samples));
  const curvePoints = [sourceStartPoint, ...line.controlPoints, line.endPoint];
  const sampledPoints = [
    {
      point: { x: sourceStartPoint.x, y: sourceStartPoint.y },
      distance: 0,
      t: 0,
    },
  ];
  let distance = 0;
  let previousPoint = sourceStartPoint;

  for (let i = 1; i <= sampleCount; i++) {
    const t = i / sampleCount;
    const point = getCurvePoint(t, curvePoints);
    const dx = point.x - previousPoint.x;
    const dy = point.y - previousPoint.y;
    distance += Math.sqrt(dx * dx + dy * dy);
    sampledPoints.push({
      point: { x: point.x, y: point.y },
      distance,
      t,
    });
    previousPoint = point;
  }

  return sampledPoints;
}

function getLineDistanceAtT(
  sourceStartPoint: BasePoint,
  line: Line,
  position: number,
): number {
  const targetT = clampNumber(position, 0, 1);
  const samples = sampleLineCurve(sourceStartPoint, line);
  for (let i = 1; i < samples.length; i++) {
    const previous = samples[i - 1];
    const current = samples[i];
    if (current.t < targetT) continue;

    const tSpan = current.t - previous.t;
    const ratio = tSpan <= 0 ? 0 : (targetT - previous.t) / tSpan;
    return previous.distance + (current.distance - previous.distance) * ratio;
  }
  return samples[samples.length - 1]?.distance || 0;
}

function getNearestDistanceOnLine(
  sourceStartPoint: BasePoint,
  line: Line,
  fieldPoint: BasePoint,
): number {
  const samples = sampleLineCurve(sourceStartPoint, line);
  let bestDistance = 0;
  let bestDistanceSq = Number.POSITIVE_INFINITY;

  samples.forEach((sample) => {
    const dx = sample.point.x - fieldPoint.x;
    const dy = sample.point.y - fieldPoint.y;
    const distanceSq = dx * dx + dy * dy;
    if (distanceSq < bestDistanceSq) {
      bestDistanceSq = distanceSq;
      bestDistance = sample.distance;
    }
  });

  return bestDistance;
}

function getLineMotionValues(line: Line, settings: Settings) {
  const pathSpeed = getPathSpeed(line);
  return {
    maxVelocity: Math.max(0, Number(settings.maxVelocity) || 0) * pathSpeed,
    maxAcceleration:
      Math.max(0, Number(settings.maxAcceleration) || 0) * pathSpeed,
    maxDeceleration:
      Math.max(
        0,
        Number(settings.maxDeceleration ?? settings.maxAcceleration) || 0,
      ) * pathSpeed,
  };
}

export function calculatePathTime(
  startPoint: Point,
  lines: Line[],
  settings: Settings,
  sequence?: SequenceItem[],
  variables: Variable[] = [],
): TimePrediction {
  const msToSeconds = (value?: number | string) => {
    const numeric = Number(value);
    if (!Number.isFinite(numeric) || numeric <= 0) return 0;
    return numeric / 1000;
  };

  const useMotionProfile =
    settings.maxVelocity !== undefined &&
    settings.maxAcceleration !== undefined;
  // A zero/negative angular velocity would make every rotation take forever
  // (Infinity), which poisons the whole timeline.
  const angularVelocity =
    Number(settings.aVelocity) > 0 ? Number(settings.aVelocity) : 0;
  // How much turning eats into driving speed; 1 when unset, matching a
  // drivetrain with no power to spare.
  const turnCoupling = Number.isFinite(Number(settings.turnCoupling))
    ? Math.max(0, Math.min(1, Number(settings.turnCoupling)))
    : 1;
  // Sideways grip caps speed through a curve. Falling back to the forward limit
  // assumes the robot corners as hard as it accelerates.
  const lateralAcceleration = Math.max(
    0,
    Number(settings.maxLateralAcceleration ?? settings.maxAcceleration) || 0,
  );
  // Sampled geometry reports a near-zero radius at a hard join between paths;
  // half the robot is the tightest turn worth modelling as cornering.
  const minCorneringRadius = Math.max(
    1,
    (Number(settings.rWidth) || DEFAULT_MIN_CORNER_RADIUS * 2) / 2,
  );

  const segmentLengths: number[] = [];
  const segmentTimes: number[] = [];
  const timeline: TimelineEvent[] = [];

  let currentTime = 0;
  let currentHeading = 0;

  // One shared walk of the sequence: repeats expanded, only the taken `if`
  // branch included, disabled steps dropped, and every path carrying the start
  // point the robot will really be at.
  const { steps } = buildRoute(startPoint, lines, sequence, variables);

  /**
   * The robot starts at the heading the start point declares, and turning from
   * there to whatever the first path needs costs real time — so it is timed like
   * any other rotation instead of being assumed away.
   *
   * A tangential start point stores no heading of its own; it is defined as
   * facing the way the first path needs, which is what the start point editor
   * shows, so it never produces a phantom turn.
   */
  const firstRouteLine = steps.find((step) => step.kind === "path")?.line;
  if (startPoint.heading === "linear") {
    currentHeading = Number(startPoint.startDeg) || 0;
  } else if (startPoint.heading === "constant") {
    currentHeading = Number(startPoint.degrees) || 0;
  } else {
    currentHeading = firstRouteLine
      ? getLineStartHeading(firstRouteLine, startPoint)
      : 0;
  }

  let lastPoint: Point = startPoint;

  /**
   * Consecutive paths are driven as one PathChain, which decelerates only on its
   * last path, so the profile spans the whole chain instead of braking to a stop
   * on every waypoint. Each path still gets its own travel event, timed as its
   * slice of the chain's profile.
   */
  const chainRuns = buildChainRuns(steps);
  let runCursor = 0;
  let stepIndex = 0;

  while (stepIndex < steps.length) {
    const step = steps[stepIndex];

    // A Pollen Pickup drives legs of its own. They are charged at their
    // expected length - the search at its expected time, not its timeout - and
    // replayed as maneuvers, so the animation shows the approach and the
    // return instead of a robot standing still.
    if (step.kind === "pollen") {
      const plan = planPollenStep(
        step.item,
        { x: step.atPoint.x, y: step.atPoint.y, heading: currentHeading },
        {
          settings,
          followedByPaths: steps.slice(stepIndex + 1).some((next) => next.kind === "path"),
        },
      );
      for (const leg of plan.legs) {
        // The search leg is always emitted, even at zero length: it is how the
        // editor finds the pose a step starts from.
        if (leg.seconds <= 0 && leg.phase !== "search") continue;
        timeline.push({
          type: "maneuver",
          name: step.item.name,
          itemId: step.item.id,
          maneuverPhase: leg.phase,
          duration: leg.seconds,
          startTime: currentTime,
          endTime: currentTime + leg.seconds,
          atPoint: { x: leg.from.x, y: leg.from.y },
          toPoint: { x: leg.to.x, y: leg.to.y },
          startHeading: leg.from.heading,
          targetHeading: leg.to.heading,
        });
        currentTime += leg.seconds;
      }
      // Returning puts the robot back on its planned heading; without the
      // return it is left facing the POLLEN.
      if (!step.item.returnToStart) {
        currentHeading = plan.intakeEndPose.heading;
      }
      stepIndex++;
      continue;
    }

    // Turn first: the robot turns in place to the next path's end heading
    // (TeamCode runTurnStep), timed from the angular velocity, plus a short
    // settle for the hold to converge; capped by the Java step's timeout.
    const turnToDeg = step.kind === "wait" ? (step.item as SequenceWaitItem).turnToDeg : undefined;
    if (step.kind === "wait" && turnToDeg !== undefined && Number.isFinite(Number(turnToDeg))) {
      const target = Number(turnToDeg);
      const sweep = Math.abs(getAngularDifference(currentHeading, target));
      const turnSeconds =
        sweep <= 0.5
          ? 0
          : Math.min(
              TURN_TIMEOUT_SECONDS,
              (angularVelocity > 0 ? (sweep * Math.PI) / 180 / angularVelocity : 0) +
                TURN_SETTLE_SECONDS,
            );
      if (turnSeconds > 0) {
        timeline.push({
          type: "wait",
          name: step.item.name || "Turn",
          itemId: step.item.id,
          duration: turnSeconds,
          startTime: currentTime,
          endTime: currentTime + turnSeconds,
          startHeading: currentHeading,
          targetHeading: target,
          atPoint: lastPoint,
        });
        currentTime += turnSeconds;
      }
      currentHeading = target;
      stepIndex++;
      continue;
    }

    if (step.kind !== "path") {
      const waitSeconds = msToSeconds(step.item.durationMs);
      if (waitSeconds > 0) {
        timeline.push({
          type: "wait",
          name: step.item.name,
          itemId: step.item.id,
          duration: waitSeconds,
          startTime: currentTime,
          endTime: currentTime + waitSeconds,
          startHeading: currentHeading,
          targetHeading: currentHeading,
          atPoint: lastPoint,
        });
        currentTime += waitSeconds;
      }
      stepIndex++;
      continue;
    }

    // Runs cover the path steps in order, so the next one starts here.
    const run = chainRuns[runCursor++];
    if (!run || run.steps.length === 0) {
      stepIndex++;
      continue;
    }

    const firstMember = run.steps[0];

    // --- TRAVEL: one profile across the whole chain ---
    // Sample the chain's geometry end to end. The profile needs the shape, not
    // just the length: a corner caps speed however long the chain is.
    const memberLengths: number[] = [];
    // Distance reached at each evenly spaced curve parameter, per member. The
    // animation needs it to turn a distance back into a curve parameter.
    const memberArcLengths: Float32Array[] = [];

    run.steps.forEach((member, memberIndex) => {
      const curvePoints = [
        member.startPoint,
        ...member.line.controlPoints,
        member.line.endPoint,
      ];
      const arcLengths = new Float32Array(CHAIN_PROFILE_SAMPLES + 1);
      let memberLength = 0;
      let previous: BasePoint = member.startPoint;

      for (let i = 1; i <= CHAIN_PROFILE_SAMPLES; i++) {
        const point = getCurvePoint(i / CHAIN_PROFILE_SAMPLES, curvePoints);
        memberLength += Math.hypot(point.x - previous.x, point.y - previous.y);
        arcLengths[i] = memberLength;
        previous = point;
      }

      memberLengths.push(memberLength);
      memberArcLengths.push(arcLengths);
    });

    // Re-sample the chain at an even spacing along the curve itself.
    //
    // The samples above sit at evenly spaced curve parameters, which on a
    // Bezier bunch up wherever the control points do. Curvature read across
    // three nearly-touching points is mostly rounding error, and reading it
    // across a wide gap smears a corner out — either way the speed cap ends up
    // oscillating. Walking the arc-length table and evaluating the real curve
    // at each step gives points that are both evenly spread and actually on the
    // path, which is what makes the speed curve come out smooth.
    const evenPoints: BasePoint[] = [];
    // Heading goal at each sample, so the profile can see how fast the robot
    // has to spin per inch travelled.
    const evenHeadings: number[] = [];
    // Where each sample sits along the chain, on the same ruler the chain
    // offsets use.
    const evenDistances: number[] = [];
    let travelledSoFar = 0;
    run.steps.forEach((member, memberIndex) => {
      const curvePoints = [
        member.startPoint,
        ...member.line.controlPoints,
        member.line.endPoint,
      ];
      const arcLengths = memberArcLengths[memberIndex];
      const memberLength = memberLengths[memberIndex];
      const steps = Math.max(
        1,
        Math.min(MAX_CHAIN_SAMPLES, Math.ceil(memberLength / CHAIN_SAMPLE_STEP)),
      );

      if (memberIndex === 0) {
        evenPoints.push({ x: member.startPoint.x, y: member.startPoint.y });
        // The heading the robot arrives with, so a goal that does not pick up
        // where the last one left off is paid for like any other rotation.
        evenHeadings.push(currentHeading);
        evenDistances.push(0);
      }
      for (let i = 1; i <= steps; i++) {
        const intoMember = (memberLength * i) / steps;
        const t = curveParameterAtDistance(arcLengths, intoMember);
        const point = getCurvePoint(t, curvePoints);
        evenPoints.push({ x: point.x, y: point.y });
        evenHeadings.push(goalHeadingAt(member.line, curvePoints, t));
        evenDistances.push(travelledSoFar + intoMember);
      }
      travelledSoFar += memberLength;
    });

    const chainLength = memberLengths.reduce((sum, value) => sum + value, 0);

    // The speed is uniform across a run, so any member gives the chain's motion.
    const pathSpeed = run.speed;
    const chainMotion = {
      maxVelocity: useMotionProfile
        ? Math.max(0, Number(settings.maxVelocity) || 0) * pathSpeed
        : 0,
      maxAcceleration: useMotionProfile
        ? Math.max(0, Number(settings.maxAcceleration) || 0) * pathSpeed
        : 0,
      maxDeceleration: useMotionProfile
        ? Math.max(
            0,
            Number(settings.maxDeceleration ?? settings.maxAcceleration) || 0,
          ) * pathSpeed
        : 0,
    };
    const hasProfile =
      chainLength > 0 &&
      chainMotion.maxVelocity > 0 &&
      chainMotion.maxAcceleration > 0 &&
      chainMotion.maxDeceleration > 0;
    // Guard the fallback: a zero average velocity would make the chain take
    // Infinity seconds and turn every later timestamp into NaN.
    const averageVelocity =
      ((Number(settings.xVelocity) + Number(settings.yVelocity)) / 2) * pathSpeed;

    const chainProfile = hasProfile
      ? buildMotionProfile(
          profilePointsFromPolyline(
            evenPoints,
            evenHeadings,
            minCorneringRadius,
            evenDistances,
          ),
          {
            ...chainMotion,
            // Grip and spin rate are properties of the robot, not of the power
            // the path asks for, so the path speed scale does not touch them.
            maxLateralAcceleration: lateralAcceleration,
            maxAngularVelocity: angularVelocity,
            turnCoupling,
            minRadius: minCorneringRadius,
          },
        )
      : null;

    const timeAtChainDistance = (distance: number): number => {
      const clamped = clampNumber(distance, 0, chainLength);
      if (chainProfile) return profileTimeAtDistance(chainProfile, clamped);
      return averageVelocity > 0 ? clamped / averageVelocity : 0;
    };

    const chainStartTime = currentTime;
    let travelled = 0;
    // Wall-clock cursor: turning stretches a path past its time on the chain
    // profile, so the two clocks drift apart as the chain goes on.
    let elapsed = 0;

    run.steps.forEach((member, memberIndex) => {
      const length = memberLengths[memberIndex];
      const enterTime = timeAtChainDistance(travelled);
      travelled += length;
      const exitTime = timeAtChainDistance(travelled);
      const translationTime = Math.max(0, exitTime - enterTime);

      // The robot never stops to turn — not even at the head of a chain, where
      // the follower starts driving and correcting heading in the same command.
      // The cost of turning is already in the chain's speed curve, charged
      // where the turning actually happens, so the path takes exactly as long
      // as the profile says. Stretching it per path instead would make the
      // speed jump at every boundary between paths that turn by different
      // amounts.
      const entryHeading = currentHeading;
      const goalHeading = getLineStartHeading(member.line, member.startPoint);
      const segmentTime = translationTime;

      // How much of the path is spent picking the heading goal up, measured
      // against the time the path really takes.
      const catchUp = headingCatchUpFraction(
        entryHeading,
        goalHeading,
        segmentTime,
        angularVelocity,
      );

      segmentLengths.push(length);
      segmentTimes.push(segmentTime);
      timeline.push({
        type: "travel",
        duration: segmentTime,
        startTime: chainStartTime + elapsed,
        endTime: chainStartTime + elapsed + segmentTime,
        lineIndex: member.lineIndex,
        startPoint: member.startPoint,
        arcLengths: memberArcLengths[memberIndex],
        startHeading: entryHeading,
        targetHeading: goalHeading,
        headingCatchUp: catchUp,
        chain: {
          index: run.index,
          startTime: chainStartTime,
          length: chainLength,
          offset: travelled - length,
          enterTime,
          translationDuration: translationTime,
          maxVelocity: chainMotion.maxVelocity,
          maxAcceleration: chainMotion.maxAcceleration,
          maxDeceleration: chainMotion.maxDeceleration,
          profile: chainProfile ?? undefined,
        },
      });

      elapsed += segmentTime;
      currentHeading = getLineEndHeading(member.line, member.startPoint);
      lastPoint = member.line.endPoint as Point;
    });

    currentTime = chainStartTime + elapsed;
    stepIndex += run.steps.length;
  }

  const totalTime = currentTime;
  const totalDistance = segmentLengths.reduce((sum, length) => sum + length, 0);

  return {
    totalTime,
    segmentTimes,
    totalDistance,
    timeline,
  };
}

export function buildTravelLineTimingMetas(
  startPoint: Point,
  lines: Line[],
  timePrediction: TimePrediction | null | undefined,
  settings: Settings,
  sequence?: SequenceItem[],
  variables: Variable[] = [],
): TravelLineTimingMeta[] {
  // The same route walk the timeline was built from, so entry N here is
  // travel event N there.
  const expandedPaths = buildRoute(
    startPoint,
    lines,
    sequence,
    variables,
  ).steps.filter((step): step is Extract<typeof step, { kind: "path" }> =>
    step.kind === "path",
  );

  const travelEvents = (timePrediction?.timeline || []).filter(
    (event) => event.type === "travel" && Number.isFinite(event.duration),
  );

  return expandedPaths.map((entry, executionIndex) => {
    const travelEvent = travelEvents[executionIndex];
    const length = calculateCurveLength(
      entry.startPoint,
      entry.line.controlPoints,
      entry.line.endPoint,
    );
    const motion = getLineMotionValues(entry.line, settings);
    const hasMotionProfile =
      length > 0 &&
      motion.maxVelocity > 0 &&
      motion.maxAcceleration > 0 &&
      motion.maxDeceleration > 0;
    const fallbackDuration = hasMotionProfile
      ? calculateMotionProfileTime(
          length,
          motion.maxVelocity,
          motion.maxAcceleration,
          motion.maxDeceleration,
        )
      : 0;
    const duration = Number(travelEvent?.duration) || fallbackDuration;
    const startTime = Number(travelEvent?.startTime) || 0;

    return {
      executionIndex,
      lineIndex: entry.lineIndex,
      iteration: entry.iteration,
      chain: travelEvent?.chain,
      line: entry.line,
      startPoint: entry.startPoint,
      length,
      duration,
      startTime,
      endTime: startTime + duration,
      maxVelocity: motion.maxVelocity,
      maxAcceleration: motion.maxAcceleration,
      maxDeceleration: motion.maxDeceleration,
      hasMotionProfile,
    };
  });
}

/**
 * How far into a travel segment the robot is at a time within it.
 *
 * A path driven as part of a PathChain has no profile of its own — it is a slice
 * of the chain's single accelerate/cruise/decelerate curve, so the robot can be
 * moving at full speed at both ends of it. Reading the position from a per-path
 * profile would show it braking to a stop on a waypoint it drives straight
 * through.
 */
export function travelDistanceAtLocalTime(
  meta: TravelLineTimingMeta,
  localTime: number,
): number {
  const clampedTime = clampNumber(localTime, 0, meta.duration);
  const chain = meta.chain;

  if (chain && chain.length > 0) {
    // Wall-clock time within the path maps onto the chain's own profile in
    // proportion, since turning stretches the path evenly rather than changing
    // where along the chain it sits.
    const progress = meta.duration > 0 ? clampedTime / meta.duration : 1;
    const timeIntoChain = chain.enterTime + progress * chain.translationDuration;
    const distanceAlongChain = chain.profile
      ? profileDistanceAtTime(chain.profile, timeIntoChain)
      : chain.offset + meta.length * progress;
    return clampNumber(distanceAlongChain - chain.offset, 0, meta.length);
  }

  if (meta.hasMotionProfile) {
    return calculateMotionProfileDistanceAtTime(
      clampedTime,
      meta.length,
      meta.maxVelocity,
      meta.maxAcceleration,
      meta.maxDeceleration,
    );
  }

  return meta.length * (meta.duration > 0 ? clampedTime / meta.duration : 1);
}

/** The inverse of {@link travelDistanceAtLocalTime}. */
export function travelLocalTimeAtDistance(
  meta: TravelLineTimingMeta,
  distance: number,
): number {
  const clampedDistance = clampNumber(distance, 0, meta.length);
  const chain = meta.chain;

  if (chain && chain.length > 0) {
    if (chain.profile && chain.translationDuration > 0) {
      const timeIntoChain = profileTimeAtDistance(
        chain.profile,
        chain.offset + clampedDistance,
      );
      const progress = clampNumber(
        (timeIntoChain - chain.enterTime) / chain.translationDuration,
        0,
        1,
      );
      return progress * meta.duration;
    }
    return meta.duration * (meta.length > 0 ? clampedDistance / meta.length : 0);
  }

  if (meta.hasMotionProfile) {
    return calculateMotionProfileTimeAtDistance(
      clampedDistance,
      meta.length,
      meta.maxVelocity,
      meta.maxAcceleration,
      meta.maxDeceleration,
    );
  }

  return meta.duration * (meta.length > 0 ? clampedDistance / meta.length : 0);
}

export function buildEventTimingWindows(
  startPoint: Point,
  lines: Line[],
  timePrediction: TimePrediction | null | undefined,
  settings: Settings,
  sequence?: SequenceItem[],
  variables: Variable[] = [],
): EventTimingWindow[] {
  const metas = buildTravelLineTimingMetas(
    startPoint,
    lines,
    timePrediction,
    settings,
    sequence,
    variables,
  );
  const totalTimelineTime = Math.max(
    timePrediction?.totalTime || 0,
    ...metas.map((meta) => meta.endTime),
  );
  const windows: EventTimingWindow[] = [];

  const scope = buildExpressionScope(variables);

  metas.forEach((meta) => {
    (meta.line.eventMarkers || []).forEach((marker, markerIndex) => {
      // A marker switched off by its own condition is skipped in generated
      // code, so it must not show up in the estimate either.
      if (!isEnabled(marker, variables, scope)) return;

      const triggerType =
        marker.triggerType === "temporal" || marker.triggerType === "pose"
          ? marker.triggerType
          : "parametric";
      const position = clampNumber(Number(marker.position ?? 0.5), 0, 1);
      let triggerDistance = 0;
      let localTriggerTime = 0;

      if (
        triggerType === "pose" &&
        Number.isFinite(Number(marker.poseX)) &&
        Number.isFinite(Number(marker.poseY))
      ) {
        triggerDistance = getNearestDistanceOnLine(meta.startPoint, meta.line, {
          x: Number(marker.poseX),
          y: Number(marker.poseY),
        });
        localTriggerTime = travelLocalTimeAtDistance(meta, triggerDistance);
      } else if (triggerType === "temporal") {
        localTriggerTime = clampNumber(
          Math.max(0, Number(marker.triggerMs ?? 0) || 0) / 1000,
          0,
          meta.duration,
        );
        triggerDistance = travelDistanceAtLocalTime(meta, localTriggerTime);
      } else {
        triggerDistance = getLineDistanceAtT(meta.startPoint, meta.line, position);
        localTriggerTime = travelLocalTimeAtDistance(meta, triggerDistance);
      }

      const durationMs = Math.max(0, Math.round(Number(marker.durationMs ?? 0) || 0));
      const startTime = meta.startTime + localTriggerTime;
      // A zero duration fires once: the generated `startParallelEvent` finishes
      // the event immediately, so it must not read as "active until the end of
      // auto" here either.
      const endTime = startTime + durationMs / 1000;

      windows.push({
        name: marker.name?.trim() || `Event ${markerIndex + 1}`,
        lineIndex: meta.lineIndex,
        lineName: meta.line.name?.trim() || `Path ${meta.lineIndex + 1}`,
        markerIndex,
        triggerType,
        startTime,
        endTime,
        durationMs,
        startPercent:
          totalTimelineTime > 0
            ? clampNumber((startTime / totalTimelineTime) * 100, 0, 100)
            : 0,
        endPercent:
          totalTimelineTime > 0
            ? clampNumber((endTime / totalTimelineTime) * 100, 0, 100)
            : 0,
        triggerPathPercent:
          meta.length > 0 ? clampNumber((triggerDistance / meta.length) * 100, 0, 100) : 0,
      });
    });
  });

  return windows.sort((a, b) => a.startTime - b.startTime);
}

export function formatTime(totalSeconds: number): string {
  if (totalSeconds <= 0) return "0.0s";
  const minutes = Math.floor(totalSeconds / 60);
  const seconds = totalSeconds % 60;
  if (minutes > 0) {
    return `${minutes}:${seconds.toFixed(1).padStart(4, "0")}s`;
  }
  return `${seconds.toFixed(1)}s`;
}

export function getAnimationDuration(
  totalTime: number,
  speedFactor: number = 1.0,
): number {
  return (totalTime * 1000) / speedFactor;
}
