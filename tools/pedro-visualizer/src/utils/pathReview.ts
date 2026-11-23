/**
 * Reviewing a route without opening the editor.
 *
 * Everything here is measured with the modules the editor itself draws and
 * checks with - `calculatePathTime`, `buildRoute`, `checkClearance` - so a
 * review handed to someone (or to a chat, through `mcp/`) says what the
 * visualizer would say. If a number here ever disagrees with the editor, that
 * is a bug in one of them, not a difference of opinion.
 */
import type { Line, Point, SequenceItem, Settings, Shape, Variable } from "../types";
import { FIELD_SIZE } from "../config/defaults";
import {
  checkClearance,
  clearanceTargetName,
  TOUCH_TOLERANCE,
  describeClearanceSpan,
  footprintFromSettings,
  midlineRuleFromSettings,
  type ClearanceReport,
  type ClearanceSpan,
  type MidlineRule,
  type StationaryPose,
} from "./clearance";
import {
  getAngularDifference,
  getLineEndHeading,
  getLineStartHeading,
} from "./math";
import { buildRoute } from "./sequence";
import { expandRouteEdits } from "./cameraPick";
import {
  calculateCurveLength,
  calculatePathTime,
  getPathSpeed,
} from "./timeCalculator";
import type { LoadedProject } from "./projectFile";
import {
  planPollenSteps,
  pollenLegsOf,
  visionOf,
  type PollenStepPlan,
} from "./pollenVision";

/** The AUTO period is 30 seconds; a route that does not fit does not run. */
export const AUTO_PERIOD_SECONDS = 30;

export type FindingLevel = "error" | "warning" | "note";

export interface ReviewFinding {
  level: FindingLevel;
  message: string;
}

export interface PathTiming {
  index: number;
  name: string;
  lengthInches: number;
  seconds: number;
  speed: number;
  /** Where this path starts on the route, following the sequence. */
  start: { x: number; y: number };
  end: { x: number; y: number };
}

export interface PathReview {
  field: {
    map: string;
    sizeInches: number;
    obstacleCount: number;
    obstacleSource: "file" | "field preset";
    obstacleNames: string[];
    midline: MidlineRule | undefined;
  };
  robot: {
    widthInches: number;
    heightInches: number;
    fromCad: boolean;
    marginInches: number;
  };
  time: {
    totalSeconds: number;
    autoLimitSeconds: number;
    /** Seconds left of AUTO, negative when the route does not fit. */
    headroomSeconds: number;
    drivingSeconds: number;
    waitingSeconds: number;
  };
  motion: {
    pathCount: number;
    executedPathCount: number;
    totalDistanceInches: number;
    paths: PathTiming[];
  };
  clearance: ClearanceReport;
  /** Every Pollen Pickup the route runs, with its plan. */
  pollen: {
    id: string;
    name: string;
    plan: PollenStepPlan;
  }[];
  /** Seconds the route takes if every pickup searches to its timeout. */
  worstCaseSeconds: number;
  cameraMeasured: boolean;
  findings: ReviewFinding[];
}

export interface ReviewOptions {
  /** Overrides the file's own safety margin. */
  marginInches?: number;
  /** Overrides the file's own AUTO midline setting. */
  midline?: MidlineRule | "off";
  /** Footprint sample spacing along the route, inches. */
  stepInches?: number;
  fieldSize?: number;
  autoSeconds?: number;
}

function outsideField(point: { x: number; y: number }, size: number): boolean {
  return point.x < 0 || point.x > size || point.y < 0 || point.y > size;
}

/**
 * Poses the robot holds without driving, from the timeline's wait events. Same
 * derivation the editor uses, deduplicated by sequence item because a wait
 * inside a repeat loop runs many times at the same pose.
 */
export function stationaryPosesFrom(
  timeline: { type: string; name?: string; itemId?: string; startTime: number; duration: number; atPoint?: { x: number; y: number }; startHeading?: number }[],
): StationaryPose[] {
  const seen = new Set<string>();
  const poses: StationaryPose[] = [];
  for (const event of timeline || []) {
    if (event.type !== "wait" || !event.atPoint) continue;
    const id = event.itemId || `${event.name || "wait"}@${event.startTime}`;
    if (seen.has(id)) continue;
    seen.add(id);
    poses.push({
      id,
      name: event.name,
      x: event.atPoint.x,
      y: event.atPoint.y,
      heading: Number(event.startHeading) || 0,
      seconds: event.duration,
    });
  }
  return poses;
}

export function reviewProject(
  project: LoadedProject,
  options: ReviewOptions = {},
): PathReview {
  const {
    startPoint,
    variables,
    shapes,
    settings,
    usedFieldObstaclePreset,
  } = project;

  const fieldSize = options.fieldSize ?? FIELD_SIZE;
  const autoLimit = options.autoSeconds ?? AUTO_PERIOD_SECONDS;
  const margin =
    options.marginInches !== undefined
      ? options.marginInches
      : Number(settings.safetyMargin) || 0;
  const midline =
    options.midline === "off"
      ? undefined
      : options.midline ?? midlineRuleFromSettings(settings);

  // Reviewed as driven: Camera pick (at its preview clump) and Turn first applied,
  // so the MCP review times and checks what the editor shows.
  const edited = expandRouteEdits(startPoint, project.lines, project.sequence);
  const lines = edited.lines;
  const sequence = edited.sequence;
  const prediction = calculatePathTime(startPoint, lines, settings, sequence, variables);
  const route = buildRoute(startPoint, lines, sequence, variables);
  const footprint = footprintFromSettings(settings);
  const pollenPlans = planPollenSteps(
    prediction.timeline as any,
    sequence,
    route.steps,
    startPoint,
    settings,
  );

  const clearance = checkClearance(
    {
      startPoint,
      lines,
      footprint,
      obstacles: shapes,
      lineStartPoints: route.startPoints,
      headingTransitions: headingTransitionsOf(prediction, lines),
      stationaryPoses: stationaryPosesFrom(prediction.timeline as any),
      maneuverLegs: pollenLegsOf(pollenPlans, sequence),
    },
    { fieldSize, margin, midline, step: options.stepInches },
  );

  // --- timing, per path as the route runs it ---------------------------------
  const paths: PathTiming[] = [];
  let totalDistance = 0;
  const travelEvents = prediction.timeline.filter((event) => event.type === "travel");
  route.steps
    .filter((step): step is Extract<typeof step, { kind: "path" }> => step.kind === "path")
    .forEach((step, executionIndex) => {
      const length = calculateCurveLength(
        step.startPoint,
        step.line.controlPoints,
        step.line.endPoint,
      );
      totalDistance += length;
      paths.push({
        index: step.lineIndex,
        name: step.line.name?.trim() || `Path ${step.lineIndex + 1}`,
        lengthInches: length,
        seconds: travelEvents[executionIndex]?.duration ?? 0,
        speed: getPathSpeed(step.line),
        start: { x: step.startPoint.x, y: step.startPoint.y },
        end: { x: step.line.endPoint.x, y: step.line.endPoint.y },
      });
    });

  const waitingSeconds = prediction.timeline
    .filter((event) => event.type === "wait")
    .reduce((sum, event) => sum + (Number(event.duration) || 0), 0);
  const drivingSeconds = Math.max(0, prediction.totalTime - waitingSeconds);

  // --- findings --------------------------------------------------------------
  const findings: ReviewFinding[] = [];

  if (!lines.length) {
    findings.push({ level: "error", message: "The file has no paths." });
  }

  if (prediction.totalTime > autoLimit) {
    findings.push({
      level: "error",
      message: `The route takes ${prediction.totalTime.toFixed(1)}s, which is ${(
        prediction.totalTime - autoLimit
      ).toFixed(1)}s over the ${autoLimit}s AUTO period — it will be cut off partway.`,
    });
  } else if (autoLimit - prediction.totalTime < 2) {
    findings.push({
      level: "warning",
      message: `The route takes ${prediction.totalTime.toFixed(1)}s of the ${autoLimit}s AUTO period, leaving ${(
        autoLimit - prediction.totalTime
      ).toFixed(1)}s — thin, once a real robot's settling is in it.`,
    });
  }

  if (clearance.startPose && clearance.startPose.clearance < -TOUCH_TOLERANCE) {
    const staged =
      clearance.startPose.kind === "midline"
        ? "staged across the midline"
        : clearance.startPose.kind === "wall"
          ? "staged over the field wall"
          : `staged inside ${clearance.startPose.obstacleName?.trim() || "an obstacle"}`;
    findings.push({
      level: "error",
      message: `The robot is ${staged} — no path can fix that, move the starting point.`,
    });
  }

  for (const span of clearance.spans) {
    const line = lines[span.lineIndex];
    const name = line?.name?.trim() || `Path ${span.lineIndex + 1}`;
    // A path can begin in trouble rather than drive into it - the robot was put
    // there by whatever came before, and "crosses at 0.8in" hides that.
    // Waits and pickup legs are not paths: they name themselves.
    const ownRow = Boolean(span.stationary || span.maneuver);
    const fromTheStart =
      !ownRow &&
      span.severity === "hit" &&
      (span.contactDistance ?? span.startDistance) <= clearance.step + 1e-6;

    findings.push({
      level: span.severity === "hit" ? "error" : "warning",
      message: fromTheStart
        ? `${name} starts with the robot already ${
            span.kind === "midline"
              ? "across the midline"
              : `inside ${clearanceTargetName(span)}`
          }.`
        : `${describeClearanceSpan(span)}${ownRow ? "" : ` on ${name}`}.`,
    });
  }

  // Heading the robot arrives with vs the heading the next path wants.
  let previousEndHeading: number | null = null;
  route.steps.forEach((step) => {
    if (step.kind !== "path") {
      previousEndHeading = null;
      return;
    }
    const entry = previousEndHeading;
    const goal = getLineStartHeading(step.line, step.startPoint);
    previousEndHeading = getLineEndHeading(step.line, step.startPoint);
    if (entry === null) return;
    const jump = Math.abs(getAngularDifference(entry, goal));
    if (jump < 20) return;
    findings.push({
      level: "warning",
      message: `${
        step.line.name?.trim() || `Path ${step.lineIndex + 1}`
      } starts ${Math.round(jump)}° away from the heading the robot arrives with, so it turns while driving.`,
    });
  });

  if (outsideField(startPoint, fieldSize)) {
    findings.push({ level: "warning", message: "The starting point is outside the field." });
  }

  lines.forEach((line, index) => {
    const name = line.name?.trim() || `Path ${index + 1}`;
    if (outsideField(line.endPoint, fieldSize)) {
      findings.push({ level: "warning", message: `${name} ends outside the field.` });
    }
    line.controlPoints.forEach((point, controlIndex) => {
      if (outsideField(point, fieldSize)) {
        findings.push({
          level: "warning",
          message: `${name} control point ${controlIndex + 1} is outside the field.`,
        });
      }
    });
    const start =
      route.startPoints.get(line.id || "") ??
      (index === 0 ? startPoint : lines[index - 1]?.endPoint);
    if (
      start &&
      line.controlPoints.length === 0 &&
      Number(start.x) === Number(line.endPoint.x) &&
      Number(start.y) === Number(line.endPoint.y)
    ) {
      findings.push({ level: "warning", message: `${name} has zero length.` });
    }
    line.eventMarkers?.forEach((marker, markerIndex) => {
      if (Number(marker.durationMs ?? 0) <= 0) {
        findings.push({
          level: "note",
          message: `Event "${
            marker.name || `#${markerIndex + 1}`
          }" on ${name} has no duration, so it starts and finishes instantly.`,
        });
      }
    });
  });

  const firstPath = sequence.findIndex(
    (item) => item.kind === "path" || item.kind === "repeat" || item.kind === "conditional",
  );
  if (firstPath > 0) {
    findings.push({
      level: "note",
      message: "The sequence starts with a wait or an event before the first path.",
    });
  }

  // Pollen Pickups: their own findings, and what happens to the AUTO budget
  // when every search runs to its timeout instead of its expected length.
  const pollen: PathReview["pollen"] = [];
  let worstCaseSeconds = prediction.totalTime;
  pollenPlans.forEach((plan, id) => {
    const item = sequence.find((candidate) => candidate.kind === "pollen" && candidate.id === id);
    const name = item && item.kind === "pollen" ? item.name : "Pollen Pickup";
    pollen.push({ id, name, plan });
    worstCaseSeconds += plan.worstSeconds - plan.expectedSeconds;
    for (const finding of plan.findings) {
      findings.push({ level: finding.level, message: `${name}: ${finding.message}` });
    }
  });
  if (pollen.length && worstCaseSeconds > autoLimit && prediction.totalTime <= autoLimit) {
    findings.push({
      level: "warning",
      message: `If every Pollen Pickup searches to its timeout the route takes ${worstCaseSeconds.toFixed(
        1,
      )}s — ${(worstCaseSeconds - autoLimit).toFixed(1)}s past the ${autoLimit}s AUTO period.`,
    });
  }

  if (clearance.startPose?.againstWall) {
    findings.push({
      level: "note",
      message: "Starts touching the perimeter wall, as G304 requires, and drives straight off it.",
    });
  }

  if (usedFieldObstaclePreset) {
    findings.push({
      level: "note",
      message:
        "The file carries no obstacles of its own, so it was measured against the field map's preset.",
    });
  }
  if (!midline) {
    findings.push({
      level: "note",
      message:
        "The AUTO midline rule is off, so nothing here says whether the route crosses into the opposing half.",
    });
  }
  if (clearance.coarse) {
    findings.push({
      level: "note",
      message: `The route is long enough that the footprint was sampled every ${clearance.step.toFixed(
        2,
      )}in rather than the usual spacing.`,
    });
  }

  return {
    field: {
      map: settings.fieldMap || "unknown",
      sizeInches: fieldSize,
      obstacleCount: shapes.length,
      obstacleSource: usedFieldObstaclePreset ? "field preset" : "file",
      obstacleNames: shapes.map((shape: Shape) => shape.name || shape.id),
      midline,
    },
    robot: {
      widthInches: Number(settings.rWidth) || 0,
      heightInches: Number(settings.rHeight) || 0,
      fromCad: Boolean(settings.robotOutline?.points?.length),
      marginInches: margin,
    },
    time: {
      totalSeconds: prediction.totalTime,
      autoLimitSeconds: autoLimit,
      headroomSeconds: autoLimit - prediction.totalTime,
      drivingSeconds,
      waitingSeconds,
    },
    motion: {
      pathCount: lines.length,
      executedPathCount: paths.length,
      totalDistanceInches: totalDistance,
      paths,
    },
    clearance,
    pollen,
    worstCaseSeconds,
    cameraMeasured: visionOf(settings).measured,
    findings,
  };
}

/** Same map the editor builds: how each path picks up its heading goal. */
function headingTransitionsOf(
  prediction: ReturnType<typeof calculatePathTime>,
  lines: Line[],
): Map<string, { entryHeading: number; catchUp: number }> {
  const transitions = new Map<string, { entryHeading: number; catchUp: number }>();
  prediction.timeline.forEach((event) => {
    if (event.type !== "travel" || event.lineIndex === undefined) return;
    const lineId = lines[event.lineIndex]?.id;
    if (!lineId || transitions.has(lineId)) return;
    transitions.set(lineId, {
      entryHeading: Number(event.startHeading) || 0,
      catchUp: Number(event.headingCatchUp) || 0,
    });
  });
  return transitions;
}

const LEVEL_MARK: Record<FindingLevel, string> = {
  error: "✗",
  warning: "!",
  note: "·",
};

/** The review as text, which is how a chat reads it. */
export function formatPathReview(review: PathReview, title?: string): string {
  const out: string[] = [];
  const n = (value: number, digits = 1) => value.toFixed(digits);

  out.push(`# Path review${title ? `: ${title}` : ""}`);

  const errors = review.findings.filter((f) => f.level === "error").length;
  const warnings = review.findings.filter((f) => f.level === "warning").length;
  out.push(
    errors === 0 && warnings === 0
      ? "**Clear.** Nothing hits anything, and the route fits AUTO."
      : `**${errors} problem${errors === 1 ? "" : "s"}, ${warnings} warning${
          warnings === 1 ? "" : "s"
        }.**`,
  );

  out.push("", "## Time");
  out.push(
    `- ${n(review.time.totalSeconds)}s total (${n(review.time.drivingSeconds)}s driving, ${n(
      review.time.waitingSeconds,
    )}s waiting) of a ${review.time.autoLimitSeconds}s AUTO period.`,
  );
  out.push(
    review.time.headroomSeconds >= 0
      ? `- ${n(review.time.headroomSeconds)}s of headroom left.`
      : `- **${n(-review.time.headroomSeconds)}s over the AUTO period.**`,
  );
  out.push(
    `- ${review.motion.executedPathCount} path${
      review.motion.executedPathCount === 1 ? "" : "s"
    } driven, ${n(review.motion.totalDistanceInches)}in of travel.`,
  );

  out.push("", "## Setup");
  out.push(
    `- Field: ${review.field.map}, ${n(review.field.sizeInches)}in square. ${
      review.field.obstacleCount
    } obstacle${review.field.obstacleCount === 1 ? "" : "s"} from the ${review.field.obstacleSource}${
      review.field.obstacleNames.length
        ? `: ${review.field.obstacleNames.join(", ")}`
        : ""
    }.`,
  );
  out.push(
    `- Robot: ${n(review.robot.widthInches)}in forward × ${n(
      review.robot.heightInches,
    )}in across${review.robot.fromCad ? " (CAD outline)" : ""}, ${n(
      review.robot.marginInches,
    )}in safety margin.`,
  );
  out.push(
    `- AUTO midline: ${
      review.field.midline
        ? `stay on the ${review.field.midline.side} side`
        : "not checked"
    }.`,
  );

  if (review.findings.length) {
    out.push("", "## Findings");
    for (const level of ["error", "warning", "note"] as FindingLevel[]) {
      for (const finding of review.findings.filter((f) => f.level === level)) {
        out.push(`- ${LEVEL_MARK[level]} ${finding.message}`);
      }
    }
  }

  if (review.pollen.length) {
    out.push("", "## Pollen pickups");
    out.push(
      `- Camera mount: ${review.cameraMeasured ? "measured" : "**not measured** — every position below is a guess until it is"}.`,
    );
    out.push(
      `- Worst case, every search to its timeout: ${n(review.worstCaseSeconds)}s of the ${review.time.autoLimitSeconds}s AUTO period.`,
    );
    for (const { name, plan } of review.pollen) {
      const view = plan.targetPixel?.inImage
        ? `in view at (${Math.round(plan.targetPixel.u)}, ${Math.round(plan.targetPixel.v)})px, one ball ~${Math.round(plan.targetPixelArea)}px`
        : "out of the camera's view";
      out.push(
        `- **${name}**: looks from (${n(plan.searchPose.x)}, ${n(plan.searchPose.y)}) heading ${plan.searchPose.heading.toFixed(0)}° for POLLEN at (${n(plan.target.x)}, ${n(plan.target.y)}) — ${view}. ${n(plan.expectedSeconds)}s expected, ${n(plan.worstSeconds)}s worst.`,
      );
    }
  }

  if (review.clearance.spans.length) {
    out.push("", "## Clearance detail");
    for (const span of review.clearance.spans) {
      out.push(`- ${clearanceDetail(span, review)}`);
    }
  }

  if (review.motion.paths.length) {
    out.push("", "## Paths");
    out.push("| # | path | length | time | speed | from | to |");
    out.push("|---|------|--------|------|-------|------|----|");
    review.motion.paths.forEach((path, order) => {
      out.push(
        `| ${order + 1} | ${path.name} | ${n(path.lengthInches)}in | ${n(
          path.seconds,
        )}s | ${n(path.speed, 2)} | (${n(path.start.x)}, ${n(path.start.y)}) | (${n(
          path.end.x,
        )}, ${n(path.end.y)}) |`,
      );
    });
  }

  out.push(
    "",
    `Measured with the visualizer's own modules: footprint sampled every ${review.clearance.step.toFixed(
      2,
    )}in at the heading the robot holds.`,
  );

  return out.join("\n");
}

function clearanceDetail(span: ClearanceSpan, review: PathReview): string {
  const line = review.motion.paths.find((path) => path.index === span.lineIndex);
  const where = span.stationary
    ? `wait "${span.stationary.name || span.stationary.id}"`
    : span.maneuver
      ? `${span.maneuver.name?.trim() || "Pollen Pickup"} (${span.maneuver.phase} leg)`
      : line?.name || `Path ${span.lineIndex + 1}`;
  const pose = span.contactPose ?? span.worstPose;
  const at = span.stationary
    ? ""
    : `, first at ${(span.contactDistance ?? span.startDistance).toFixed(1)}in into ${
        span.maneuver ? "the leg" : "it"
      }`;
  const worst =
    Math.abs(span.worstClearance) < TOUCH_TOLERANCE
      ? "just touching"
      : `worst ${Math.abs(span.worstClearance).toFixed(2)}in ${
          span.worstClearance < 0 ? "of overlap" : "of clearance"
        }`;
  return `${where}: ${describeClearanceSpan(span)}${at} — robot at (${pose.x.toFixed(
    1,
  )}, ${pose.y.toFixed(1)}) heading ${pose.heading.toFixed(0)}°, ${worst}.`;
}
