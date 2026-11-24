import prettier from "prettier";
import type {
  Point,
  Line,
  BasePoint,
  PathChain,
  SequenceEventItem,
  SequenceGroupItem,
  SequenceItem,
  SequencePollenItem,
  SequenceWaitItem,
  PoseVariable,
  EventMarker,
  Settings,
  Variable,
} from "../types";
import { DEFAULT_SETTINGS, FIELD_SIZE } from "../config/defaults";
import {
  INTAKE_CREEP_SPEED,
  POLLEN_RADIUS_INCHES,
  normalizePollenItem,
  visionOf,
} from "./pollenVision";
import { calculatePathTime } from "./timeCalculator";
import {
  buildJavaExpression,
  buildJavaIdentifierMap,
  evaluateExpression,
  expressionAliases,
  expressionIdentifiers,
  type JavaIdentifierMap,
} from "./numberExpressions";
import { poseVariablesOf } from "./variables";
import {
  activeConditionalIds,
  buildChainRuns,
  buildLineStartPoints,
  buildRoute,
  conditionalChainPredecessors,
  groupMembers,
  type ChainRun,
  type RoutePathStep,
} from "./sequence";
import { getCurvePoint, getLineEndHeading, getLineStartHeading } from "./math";
import { activeCameraPick, holdEndHeading } from "./cameraPick";

// Lazy-load Prettier's Java plugin; fall back gracefully if unavailable
let cachedJavaPlugin: any | null = null;
async function loadJavaPlugin() {
  if (cachedJavaPlugin !== null) return cachedJavaPlugin;
  const candidates = [() => import("prettier-plugin-java")];
  for (const loadPlugin of candidates) {
    try {
      const mod = await loadPlugin();
      cachedJavaPlugin = (mod as any).default ?? mod;
      return cachedJavaPlugin;
    } catch (err) {
      // ignore and try next
    }
  }
  cachedJavaPlugin = null;
  return null;
}

/**
 * Generate Java code from path data
 */
function sanitizeIdentifier(
  input: string | undefined,
  fallback: string,
): string {
  const cleaned = (input || "").replace(/[^a-zA-Z0-9]/g, "");
  if (!cleaned) return fallback;
  if (/^[0-9]/.test(cleaned)) return `${fallback}${cleaned}`;
  return cleaned;
}

function sanitizeJavaConstantName(
  input: string | undefined,
  fallback: string,
): string {
  const cleaned = (input || "")
    .replace(/([a-z])([A-Z])/g, "$1_$2")
    .replace(/[^a-zA-Z0-9]+/g, "_")
    .replace(/^_+|_+$/g, "")
    .toUpperCase();

  if (!cleaned) return fallback;
  if (/^[0-9]/.test(cleaned)) return `${fallback}_${cleaned}`;
  return cleaned;
}

function uniqueJavaConstantName(
  baseName: string,
  usedNames: Set<string>,
): string {
  let candidate = baseName;
  let duplicateIndex = 2;

  while (usedNames.has(candidate)) {
    candidate = `${baseName}_${duplicateIndex}`;
    duplicateIndex++;
  }

  usedNames.add(candidate);
  return candidate;
}

function sanitizeClassName(
  input: string | undefined,
  fallback: string,
): string {
  const cleaned = sanitizeIdentifier(input, fallback);
  return cleaned.charAt(0).toUpperCase() + cleaned.slice(1);
}

function fixed(value: number): string {
  return value.toFixed(3);
}

function pathSpeedValue(line: Line | undefined): number {
  const speed = Number(line?.speed ?? 1);
  if (!Number.isFinite(speed)) return 1;
  return Math.max(0.05, Math.min(1, speed));
}

function buildScalarVariableCode(
  name: string,
  variable: Variable,
  identifiers: JavaIdentifierMap,
): string {
  if (variable.type === "boolean") {
    const rendered = buildJavaExpression(
      variable.valueExpression,
      variable.value ? "true" : "false",
      identifiers,
    );
    return `private static final boolean ${name} = ${rendered};`;
  }

  const value = Number((variable as { value?: number }).value);
  const rendered = buildJavaExpression(
    (variable as { valueExpression?: string }).valueExpression,
    fixed(Number.isFinite(value) ? value : 0),
    identifiers,
  );
  return `private static final double ${name} = ${rendered};`;
}

type NumberExpressionType = "double" | "int" | "long" | "position";

type NormalizedEventMarker = {
  id: string;
  name: string;
  triggerType: "parametric" | "temporal" | "pose";
  position: number;
  positionExpression?: string;
  triggerMs: number;
  triggerMsExpression?: string;
  poseX: number;
  poseXExpression?: string;
  poseY: number;
  poseYExpression?: string;
  durationMs: number;
  durationExpression?: string;
  enabledExpression?: string;
};

function normalizeEventMarkers(
  line: Line,
  pathIndex = 0,
): NormalizedEventMarker[] {
  return (line.eventMarkers || []).map((marker, markerIndex) => {
    const position = Number(marker.position);
    const triggerMs = Number(marker.triggerMs ?? 0);
    const durationMs = Number(marker.durationMs ?? 0);
    const triggerType =
      marker.triggerType === "temporal" || marker.triggerType === "pose"
        ? marker.triggerType
        : "parametric";

    return {
      id: marker.id || `path-${pathIndex + 1}-event-${markerIndex + 1}`,
      name:
        marker.name?.trim() || `Path ${pathIndex + 1} Event ${markerIndex + 1}`,
      triggerType,
      position: Number.isFinite(position)
        ? Math.max(0, Math.min(1, position))
        : 0.5,
      positionExpression: marker.positionExpression,
      triggerMs: Number.isFinite(triggerMs)
        ? Math.max(0, Math.round(triggerMs))
        : 0,
      triggerMsExpression: marker.triggerMsExpression,
      poseX: Number.isFinite(Number(marker.poseX))
        ? Number(marker.poseX)
        : Number(line.endPoint.x) || 0,
      poseXExpression: marker.poseXExpression,
      poseY: Number.isFinite(Number(marker.poseY))
        ? Number(marker.poseY)
        : Number(line.endPoint.y) || 0,
      poseYExpression: marker.poseYExpression,
      durationMs: Number.isFinite(durationMs)
        ? Math.max(0, Math.round(durationMs))
        : 0,
      durationExpression: marker.durationExpression,
      enabledExpression: marker.enabledExpression,
    };
  });
}

type NumberExpressionRenderer = (
  expression: string | undefined,
  fallbackExpression: string,
  expressionType: NumberExpressionType,
) => string;

/**
 * One event marker as a callback on the generated `AutoPath`.
 *
 * Pedro 3 dropped PathBuilder's callbacks, so the generated auto carries its own
 * small runtime (`AutoPath`) that keeps Pedro 2's trigger rules. `segmentIndex`
 * is the path's position inside its chain — Pedro 2 attached a callback to the
 * path it was added after, and a chain is now one `Paths.path(...)`.
 */
function buildTeamCodeCallback(
  marker: NormalizedEventMarker,
  segmentIndex: number,
  numberExpression: NumberExpressionRenderer,
  booleanExpression: (expression: string | undefined) => string | null = () => null,
): string {
  const start = `startParallelEvent(${javaStringLiteral(marker.name)}, ${numberExpression(marker.durationExpression, `${marker.durationMs}L`, "long")})`;
  // A builder chain cannot hold an `if`, so the guard goes inside the lambda.
  const condition = booleanExpression(marker.enabledExpression);
  const action = condition
    ? `() -> { if (${condition}) { ${start}; } }`
    : `() -> ${start}`;

  if (marker.triggerType === "temporal") {
    return `.addTemporalCallback(${segmentIndex}, ${numberExpression(marker.triggerMsExpression, `${marker.triggerMs}L`, "long")}, ${action})`;
  }

  if (marker.triggerType === "pose") {
    return `.addPoseCallback(${segmentIndex}, new Pose(${numberExpression(marker.poseXExpression, fixed(marker.poseX), "double")}, ${numberExpression(marker.poseYExpression, fixed(marker.poseY), "double")}), ${action}, ${numberExpression(marker.positionExpression, fixed(marker.position), "position")})`;
  }

  return `.addParametricCallback(${segmentIndex}, ${numberExpression(marker.positionExpression, fixed(marker.position), "position")}, ${action})`;
}

/**
 * Heading along a path on the curve parameter t, shaped by t^curve.
 *
 * This is what the visualizer draws (`goalHeadingAt` is fed the curve
 * parameter) and what Pedro 2's `setLinearHeadingInterpolation` did at curve 1.
 * Pedro 3's `Path.linear()` interpolates on distance fraction instead, which
 * differs on nearly every Bezier, so the generated code carries this helper
 * rather than let the robot disagree with the preview.
 */
const HEADING_HELPER_JAVA = `/**
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
    }`;

function headingCurve(line: Line): number {
  if (line.endPoint.heading !== "linear") return 1;
  const curve = Number(line.endPoint.headingCurve ?? 1);
  if (!Number.isFinite(curve)) return 1;
  return Math.max(0.25, Math.min(4, curve));
}

function pathStepHeadingDegrees(
  point: Point,
  pointRole: "start" | "end",
  tangentialHeading = 0,
): number {
  if (point.heading === "constant") {
    return point.degrees ?? 0;
  }

  if (point.heading === "linear") {
    return pointRole === "start" ? (point.startDeg ?? 0) : (point.endDeg ?? 0);
  }

  // A tangential point faces along its path. For the starting pose that has to
  // be the real direction, otherwise the generated auto tells the follower the
  // robot is placed at 0 degrees while the visualizer shows it facing the first
  // path.
  return tangentialHeading;
}

function buildPathStepCode(
  name: string,
  point: Point,
  pointRole: "start" | "end",
  identifiers: JavaIdentifierMap,
  tangentialHeading = 0,
): string {
  const xExpression = buildJavaExpression(
    point.xExpression,
    fixed(point.x),
    identifiers,
  );
  const yExpression = buildJavaExpression(
    point.yExpression,
    fixed(point.y),
    identifiers,
  );
  const headingFallback = fixed(
    pathStepHeadingDegrees(point, pointRole, tangentialHeading),
  );
  const headingExpression = point.heading === "constant"
    ? buildJavaExpression(point.degreesExpression, headingFallback, identifiers)
    : point.heading === "linear"
      ? buildJavaExpression(
          pointRole === "start" ? point.startDegExpression : point.endDegExpression,
          headingFallback,
          identifiers,
        )
      : headingFallback;
  return `public static PathStep ${name} = new PathStep(${xExpression}, ${yExpression}, ${headingExpression});`;
}

function buildPoseVariablePathStepCode(
  name: string,
  variable: PoseVariable,
  identifiers: JavaIdentifierMap,
): string {
  const xExpression = buildJavaExpression(
    variable.xExpression,
    fixed(Number(variable.x) || 0),
    identifiers,
  );
  const yExpression = buildJavaExpression(
    variable.yExpression,
    fixed(Number(variable.y) || 0),
    identifiers,
  );
  const headingExpression = buildJavaExpression(
    variable.headingExpression,
    fixed(Number(variable.heading) || 0),
    identifiers,
  );
  return `public static PathStep ${name} = new PathStep(${xExpression}, ${yExpression}, ${headingExpression});`;
}

/**
 * One path as a Pedro 3 `Path` expression, for the upstream-style exports.
 *
 * `Paths.line` / `Paths.curve` build it; the heading is set on the result.
 * Linear headings go through the generated `headingInterpolator` helper so they
 * follow the curve parameter like the preview does (see HEADING_HELPER_JAVA).
 */
function buildPathSegmentCode(
  line: Line,
  startExpression: string,
  identifiers: JavaIdentifierMap,
): string {
  const controlPoints = line.controlPoints
    .map((point) => `new Pose(${point.x.toFixed(3)}, ${point.y.toFixed(3)})`)
    .join(",\n            ");

  const factory = line.controlPoints.length === 0 ? "Paths.line" : "Paths.curve";

  const endXExpression = buildJavaExpression(
    line.endPoint.xExpression,
    fixed(line.endPoint.x),
    identifiers,
  );
  const endYExpression = buildJavaExpression(
    line.endPoint.yExpression,
    fixed(line.endPoint.y),
    identifiers,
  );

  const allPoints = controlPoints
    ? `${startExpression},\n            ${controlPoints},\n            new Pose(${endXExpression}, ${endYExpression})`
    : `${startExpression},\n            new Pose(${endXExpression}, ${endYExpression})`;

  const degrees = (expression: string | undefined, fallback: number | undefined) =>
    `Math.toRadians(${buildJavaExpression(expression, fixed(fallback ?? 0), identifiers)})`;

  // Reverse only exists on a tangential heading - the editor offers it nowhere
  // else, and the preview ignores a stale flag left on a linear or constant one.
  const headingCall =
    line.endPoint.heading === "constant"
      ? `.constant(${degrees(line.endPoint.degreesExpression, line.endPoint.degrees)})`
      : line.endPoint.heading === "linear"
        ? `.heading(headingInterpolator(${degrees(line.endPoint.startDegExpression, line.endPoint.startDeg)}, ${degrees(line.endPoint.endDegExpression, line.endPoint.endDeg)}, ${buildJavaExpression(line.endPoint.headingCurveExpression, fixed(headingCurve(line)), identifiers)}))`
        : line.endPoint.reverse
          ? ".reverseTangent()"
          : ".tangent()";

  return `${factory}(
            ${allPoints}
          )${headingCall}`;
}

function buildPoseExpression(
  point: { x: number; y: number; xExpression?: string; yExpression?: string },
  identifiers: JavaIdentifierMap,
): string {
  const xExpression = buildJavaExpression(
    point.xExpression,
    fixed(point.x),
    identifiers,
  );
  const yExpression = buildJavaExpression(
    point.yExpression,
    fixed(point.y),
    identifiers,
  );
  return `new Pose(${xExpression}, ${yExpression})`;
}

/**
 * One path of a TeamCode auto as a Pedro 3 `Path` expression.
 *
 * Callbacks are not attached here: in Pedro 3 they live on the generated
 * `AutoPath` that wraps the whole chain, keyed by this path's index in it.
 */
function buildTeamCodePathSegmentCode(
  line: Line,
  startExpression: string,
  endExpression: string,
  numberExpression: NumberExpressionRenderer,
  booleanExpression: (expression: string | undefined) => string | null = () => null,
): string {
  // Control points and headings honour their expressions just like endpoints do;
  // emitting only the resolved literal would freeze them at export time and let
  // the generated auto drift away from the visualizer.
  const controlPoints = line.controlPoints
    .map(
      (point) =>
        `new Pose(${numberExpression(point.xExpression, fixed(point.x), "double")}, ${numberExpression(point.yExpression, fixed(point.y), "double")})`,
    )
    .join(",\n              ");

  const factory = line.controlPoints.length === 0 ? "Paths.line" : "Paths.curve";
  const allPoints = controlPoints
    ? `${startExpression},\n              ${controlPoints},\n              ${endExpression}`
    : `${startExpression},\n              ${endExpression}`;

  const degrees = () =>
    numberExpression(
      line.endPoint.degreesExpression,
      fixed(line.endPoint.degrees ?? 0),
      "double",
    );
  const startDeg = () =>
    numberExpression(
      line.endPoint.startDegExpression,
      fixed(line.endPoint.startDeg ?? 0),
      "double",
    );
  const endDeg = () =>
    numberExpression(
      line.endPoint.endDegExpression,
      fixed(line.endPoint.endDeg ?? 0),
      "double",
    );

  // A linear heading always goes through the shaped helper: at curve 1 it is
  // Pedro 2's linear interpolation on the curve parameter, which is what the
  // preview draws. Reverse only exists on a tangential heading (the editor
  // offers it nowhere else), and an expression driving it stays live in Java.
  const reverseCondition = booleanExpression(line.endPoint.reverseExpression);
  const headingCall =
    line.endPoint.heading === "constant"
      ? `.constant(Math.toRadians(${degrees()}))`
      : line.endPoint.heading === "linear"
        ? `.heading(headingInterpolator(Math.toRadians(${startDeg()}), Math.toRadians(${endDeg()}), ${numberExpression(line.endPoint.headingCurveExpression, fixed(headingCurve(line)), "double")}))`
        : reverseCondition
          ? `.heading(${reverseCondition} ? Interpolator.tangent.reverse() : Interpolator.tangent)`
          : line.endPoint.reverse
            ? ".reverseTangent()"
            : ".tangent()";

  return `${factory}(
              ${allPoints}
            )${headingCall}`;
}

function javaStringLiteral(value: string): string {
  return JSON.stringify(value);
}

function sanitizeJavaMethodSuffix(input: string, fallback: string): string {
  const words = (input || "")
    .replace(/[^a-zA-Z0-9]+/g, " ")
    .trim()
    .split(/\s+/)
    .filter(Boolean);

  const suffix = words
    .map((word) => word.charAt(0).toUpperCase() + word.slice(1))
    .join("");

  const cleaned = suffix.replace(/[^a-zA-Z0-9]/g, "");
  if (!cleaned) return fallback;
  return /^[0-9]/.test(cleaned) ? `${fallback}${cleaned}` : cleaned;
}

export async function generateTeamCodeAutoCode(
  startPoint: Point,
  lines: Line[],
  pathChains: PathChain[] = [],
  className = "GeneratedSwerveAuto",
  sequence: SequenceItem[] = [],
  variables: Variable[] = [],
  settings: Settings = DEFAULT_SETTINGS,
): Promise<string> {
  const autoClassName = sanitizeClassName(className, "GeneratedSwerveAuto");
  const linesWithIds = lines.map((line, idx) => ({
    ...line,
    id: line.id || `line-${idx + 1}`,
  }));
  const lineById = new Map(linesWithIds.map((line) => [line.id!, line]));
  void pathChains; // Chains are organisational only; `if` blocks drive branching.

  const poseVariables = poseVariablesOf(variables);
  const poseVariablesById = new Map(
    poseVariables.map((variable) => [variable.id, variable]),
  );

  // Every variable becomes a named constant so the generated auto stays as
  // editable as the visualizer: change the constant, change the whole path.
  const usedConstantNames = new Set<string>();
  const constantById = new Map<string, string>();
  const poseVariableConstantById = new Map<string, string>();

  const CONSTANT_PREFIX: Record<Variable["type"], string> = {
    number: "NUMBER_",
    boolean: "FLAG_",
    pose: "POSE_",
    path: "PATH_",
  };

  variables.forEach((variable, idx) => {
    if (variable.type === "path") return; // Path variables have no Java constant.

    const suffix = variable.type === "pose" ? "_STEP" : "";
    const baseName = `${CONSTANT_PREFIX[variable.type]}${sanitizeJavaConstantName(
      variable.name,
      `VARIABLE_${idx + 1}`,
    )}${suffix}`;
    const constantName = uniqueJavaConstantName(baseName, usedConstantNames);

    constantById.set(variable.id, constantName);
    if (variable.type === "pose") {
      poseVariableConstantById.set(variable.id, constantName);
    }
  });

  const identifiers = buildJavaIdentifierMap(variables, constantById);

  /**
   * Renders an expression as Java, coercing to the shape the call site needs.
   * Falls back to the resolved literal when there is no expression.
   */
  const numberExpression = (
    expression: string | undefined,
    fallbackExpression: string,
    expressionType: NumberExpressionType,
  ): string => {
    if (!expression?.trim()) return fallbackExpression;

    const rendered = buildJavaExpression(expression, "", identifiers);
    if (!rendered) return fallbackExpression;

    if (expressionType === "position") {
      // Values above 1 are authored as percentages; scale them in Java too.
      const value = evaluateExpression(expression, variables);
      return typeof value === "number" && value > 1
        ? `(${rendered} / 100.0)`
        : rendered;
    }
    if (expressionType === "int") return `(int) Math.round(${rendered})`;
    if (expressionType === "long") return `Math.round(${rendered})`;
    return rendered;
  };

  /** Renders a boolean guard, or null when there is nothing to guard on. */
  const booleanExpression = (expression: string | undefined): string | null => {
    if (!expression?.trim()) return null;
    return buildJavaExpression(expression, "", identifiers) || null;
  };

  /** ANDs a chain's condition with a step's own condition. */
  const combineConditions = (
    ...expressions: (string | undefined)[]
  ): string | undefined => {
    const present = expressions
      .map((expression) => expression?.trim())
      .filter((expression): expression is string => Boolean(expression));

    if (present.length === 0) return undefined;
    if (present.length === 1) return present[0];
    return present.map((expression) => `(${expression})`).join(" && ");
  };

  /** Emits variable constants in dependency order so Java compiles. */
  const buildVariableDeclarations = (): string[] => {
    const aliasToVariable = new Map<string, Variable>();
    variables.forEach((variable) => {
      expressionAliases(variable.name).forEach((alias) => {
        if (!aliasToVariable.has(alias)) aliasToVariable.set(alias, variable);
      });
    });

    const declarations: string[] = [];
    const emitted = new Set<string>();
    const visiting = new Set<string>();

    const dependenciesOf = (variable: Variable): string[] => {
      const expressions =
        variable.type === "pose"
          ? [variable.xExpression, variable.yExpression, variable.headingExpression]
          : variable.type === "path"
            ? []
            : [variable.valueExpression];

      return expressions.flatMap((expression) =>
        expressionIdentifiers(expression),
      );
    };

    const emit = (variable: Variable) => {
      if (variable.type === "path") return;
      if (emitted.has(variable.id)) return;
      if (visiting.has(variable.id)) return; // Reference cycle; break out.

      visiting.add(variable.id);
      dependenciesOf(variable).forEach((alias) => {
        const dependency = aliasToVariable.get(alias);
        if (dependency && dependency.id !== variable.id) emit(dependency);
      });
      visiting.delete(variable.id);

      const constantName = constantById.get(variable.id);
      if (!constantName) return;

      declarations.push(
        variable.type === "pose"
          ? buildPoseVariablePathStepCode(constantName, variable, identifiers)
          : buildScalarVariableCode(constantName, variable, identifiers),
      );
      emitted.add(variable.id);
    };

    variables.forEach(emit);
    return declarations;
  };

  const pathSpeedExpression = (line: Line | undefined): string =>
    numberExpression(
      line?.speedExpression,
      fixed(pathSpeedValue(line)),
      "double",
    );

  const pointStepName = (point: Point, fallbackName: string) =>
    point.poseVariableId
      ? poseVariableConstantById.get(point.poseVariableId) || fallbackName
      : fallbackName;

  const pointStepExpression = (point: Point, fallbackName: string) =>
    `${pointStepName(point, fallbackName)}.toPose()`;

  const branchPredecessors = conditionalChainPredecessors(
    sequence.length > 0 ? sequence : [],
  );

  const sequenceItems =
    sequence.length > 0
      ? sequence
      : linesWithIds.map(
          (line) => ({ kind: "path", lineId: line.id! }) as SequenceItem,
        );

  // The route walk: start points come from the sequence, so each branch of an
  // `if` begins where the block begins rather than where the previous branch
  // ended.
  const route = buildRoute(startPoint, linesWithIds, sequenceItems, variables);
  const lineStartPoints = route.startPoints;

  // A tangential start point faces along the first path it actually drives.
  const firstRouteStep = route.steps.find((step) => step.kind === "path");
  const startTangentialHeading =
    startPoint.heading === "tangential" && firstRouteStep?.kind === "path"
      ? getLineStartHeading(firstRouteStep.line, startPoint)
      : 0;

  const pathStepDeclarations: string[] = buildVariableDeclarations();

  if (pointStepName(startPoint, "START_STEP") === "START_STEP") {
    pathStepDeclarations.push(
      buildPathStepCode(
        "START_STEP",
        startPoint,
        "start",
        identifiers,
        startTangentialHeading,
      ),
    );
  }

  linesWithIds.forEach((line, idx) => {
    const fallbackName = `POINT_${idx + 1}`;
    if (pointStepName(line.endPoint, fallbackName) === fallbackName) {
      pathStepDeclarations.push(
        buildPathStepCode(fallbackName, line.endPoint, "end", identifiers),
      );
    }
  });

  const pathStepDeclarationBlock = pathStepDeclarations.join("\n    ");

  const stepNameForPoint = (point: Point): string => {
    if (point === startPoint) return "START_STEP";
    const ownerIndex = linesWithIds.findIndex((line) => line.endPoint === point);
    return ownerIndex >= 0 ? `POINT_${ownerIndex + 1}` : "START_STEP";
  };

  /**
   * Consecutive paths become one `Paths.path(...)`, wrapped in an `AutoPath`
   * that carries their event markers.
   *
   * Foresight brakes only for the end of a chain (it skips to the next path
   * rather than braking for an interior one), so merging is what keeps the
   * robot from braking to a stop and settling on every waypoint. The grouping
   * rules live in `buildChainRuns`, shared with the time estimate, so the auto
   * stops exactly where the editor says it will.
   */
  const chainFields: string[] = [];
  /** Renders a chain's `new AutoPath(...)`; `startOverride` replaces its first path's start (camera pick). */
  const chainRenderers = new Map<string, (startOverride?: string) => string>();
  /** The original start expression of each chain's first path. */
  const chainStartExpressions = new Map<string, string>();
  /** Extra callbacks appended to a chain (the camera pick's pose callback). */
  const chainExtraCallbacks = new Map<string, string>();

  /** The chains one run of paths compiles to, with the run behind each. */
  type DeclaredChain = { field: string; run: ChainRun };

  const declareChains = (
    lineIds: string[],
    groupId?: string,
  ): DeclaredChain[] => {
    const steps: RoutePathStep[] = [];
    lineIds.forEach((lineId) => {
      const line = lineById.get(lineId);
      if (!line) return;
      steps.push({
        kind: "path",
        lineId,
        line,
        lineIndex: linesWithIds.findIndex((item) => item.id === lineId),
        startPoint: (lineStartPoints.get(lineId) || startPoint) as Point,
        iteration: 0,
        groupId,
      });
    });

    return buildChainRuns(steps).map((run) => {
      const field = `chain${chainFields.length + 1}`;
      chainFields.push(field);

      const firstStart = run.steps[0].startPoint as Point;
      chainStartExpressions.set(
        field,
        pointStepExpression(firstStart, stepNameForPoint(firstStart)),
      );
      const renderSegments = (startOverride?: string) =>
        run.steps.map((member, memberIndex) => {
          // The first path of a chain starts where the route puts it; the rest
          // continue from the previous path's endpoint.
          const previous = run.steps[memberIndex - 1];
          const resolvedStart = previous
            ? (previous.line.endPoint as Point)
            : (member.startPoint as Point);
          const startExpression =
            !previous && startOverride
              ? startOverride
              : pointStepExpression(
                  resolvedStart,
                  previous
                    ? `POINT_${previous.lineIndex + 1}`
                    : stepNameForPoint(resolvedStart),
                );
          const endExpression = pointStepExpression(
            member.line.endPoint,
            `POINT_${member.lineIndex + 1}`,
          );

          // Turn first: the robot has already turned to the end heading in place
          // (runTurnStep), so the path holds it.
          const segmentLine = member.line.turnFirst ? holdEndHeading(member.line) : member.line;
          return buildTeamCodePathSegmentCode(
            segmentLine,
            startExpression,
            endExpression,
            numberExpression,
            booleanExpression,
          );
        });

      // A one-path chain is just that path; Paths.path(...) over one child
      // would add a CompoundPath for nothing.
      const pathExpressionFor = (startOverride?: string) => {
        const segments = renderSegments(startOverride);
        return segments.length === 1
          ? segments[0]
          : `Paths.path(
              ${segments.join(",\n              ")}
            )`;
      };

      // Each marker is keyed by its path's index in this chain, as Pedro 2
      // attached a callback to the path it was added after.
      const callbacks = run.steps.flatMap((member, segmentIndex) =>
        normalizeEventMarkers(member.line, member.lineIndex).map(
          (marker) =>
            `\n          ${buildTeamCodeCallback(marker, segmentIndex, numberExpression, booleanExpression)}`,
        ),
      );

      chainRenderers.set(
        field,
        (startOverride?: string) => `new AutoPath(
            ${pathExpressionFor(startOverride)}
          )${callbacks.join("")}${chainExtraCallbacks.get(field) ?? ""}`,
      );

      return { field, run };
    });
  };

  /** One switch case in the generated state machine. */
  type ExportStep =
    | {
        kind: "chain";
        field: string;
        speedExpression: string;
        enabledExpression?: string;
        /** Camera pick on this chain: followed with followPickStep. */
        pick?: boolean;
      }
    | { kind: "turn"; degrees: string; field: string }
    | { kind: "wait"; item: SequenceWaitItem }
    | { kind: "event"; item: SequenceEventItem }
    | { kind: "pollen"; item: SequencePollenItem; field: string }
    | { kind: "group"; item: SequenceGroupItem; slot: number };

  const exportSteps: ExportStep[] = [];
  const groupChainFields: string[][] = [];
  const groupChainSpeeds: string[][] = [];
  /** Per member slot: how long a hold lasts, `0` for a path. */
  const groupHoldMs: string[][] = [];
  /** Per member slot: the event a hold fires, `null` for a path or a plain wait. */
  const groupHoldEvents: string[][] = [];
  /** Event names used only inside a group, registered with the rest below. */
  const groupEventNames: string[] = [];

  /** The route's camera pick: the chain it is on and the chain after it (rebuilt from the picked spot). */
  type CameraPickChain = {
    field: string;
    line: Line;
    segmentIndex: number;
    endXExpression: string;
    alliance: "RED" | "BLUE";
    nextField: string | null;
  };
  // A holder object: assigned inside the chain callbacks, which TypeScript's
  // narrowing of a plain `let` cannot see.
  const pickState: { chain: CameraPickChain | null } = { chain: null };
  let pendingTopLevelLineIds: string[] = [];
  let pollenStepCount = 0;
  const flushTopLevelPaths = () => {
    if (!pendingTopLevelLineIds.length) return;
    const lineIds = pendingTopLevelLineIds;
    pendingTopLevelLineIds = [];

    declareChains(lineIds).forEach(({ field, run }) => {
      // A path that can be switched off is always alone in its chain, so the
      // chain simply carries that path's guard.
      const enabledExpression =
        run.steps.length === 1 ? run.steps[0].line.enabledExpression : undefined;

      // Turn first: turn in place to the path's end heading, as its own step - right after the previous
      // movement, BEFORE any waits / events between it and this path (operator 2026-11-23: back off the wall,
      // turn, THEN shoot - shooting at the wall put the turret at its wrap).
      const first = run.steps[0];
      if (first.line.turnFirst) {
        let at = exportSteps.length;
        while (at > 0 && (exportSteps[at - 1].kind === "wait" || exportSteps[at - 1].kind === "event")) at--;
        exportSteps.splice(at, 0, {
          kind: "turn",
          degrees: getLineEndHeading(first.line, first.startPoint as Point).toFixed(3),
          field,
        });
      }

      // Camera pick: only the first one in the route (CameraPick holds one set of constants).
      const pickIndex = pickState.chain
        ? -1
        : run.steps.findIndex((member) => activeCameraPick(member.line));
      if (pickIndex >= 0) {
        const member = run.steps[pickIndex];
        pickState.chain = {
          field,
          line: member.line,
          segmentIndex: pickIndex,
          endXExpression: `${pointStepName(member.line.endPoint, `POINT_${member.lineIndex + 1}`)}.x`,
          alliance: activeCameraPick(member.line)!.alliance,
          nextField: null,
        };
      } else if (pickState.chain && pickState.chain.nextField === null) {
        pickState.chain.nextField = field;
      }

      exportSteps.push({
        kind: "chain",
        field,
        speedExpression: pathSpeedExpression(run.steps[0].line),
        enabledExpression,
        pick: pickIndex >= 0,
      });
    });
  };

  sequenceItems.forEach((item) => {
    if (item.kind === "path") {
      if (lineById.has(item.lineId)) pendingTopLevelLineIds.push(item.lineId);
      return;
    }

    flushTopLevelPaths();

    if (item.kind === "wait") {
      exportSteps.push({ kind: "wait", item });
      return;
    }

    if (item.kind === "event") {
      exportSteps.push({ kind: "event", item });
      return;
    }

    if (item.kind === "pollen") {
      exportSteps.push({
        kind: "pollen",
        item: normalizePollenItem(item),
        field: `POLLEN_PICKUP_${pollenStepCount + 1}`,
      });
      pollenStepCount++;
      return;
    }

    // Repeat loops and `if` blocks both wrap a group of paths, so they share the
    // same AutoPath[] fields and the same followRepeatStep runtime helper —
    // an `if` block is simply a group run once, behind a guard.
    // A group runs its members in order, and a wait or an event between two
    // paths is the reason for putting it inside rather than beside. Each member
    // becomes one slot the generated loop steps through; a hold has no chain, so
    // its slot carries a duration instead.
    const members = groupMembers(item).filter((member) =>
      member.kind === "path" ? lineById.has(member.lineId) : true,
    );
    if (members.length === 0) return;
    if (!members.some((member) => member.kind === "path")) return;

    const fields: string[] = [];
    const speeds: string[] = [];
    const holdMs: string[] = [];
    const holdEvents: string[] = [];

    members.forEach((member) => {
      if (member.kind === "path") {
        // One member at a time so a hold between two paths is not chained past.
        const declared = declareChains([member.lineId], item.id);
        declared.forEach((entry) => {
          fields.push(entry.field);
          speeds.push(pathSpeedExpression(entry.run.steps[0].line));
          holdMs.push("0");
          holdEvents.push("null");
        });
        return;
      }

      fields.push("null");
      speeds.push("1.0");
      holdMs.push(
        numberExpression(
          member.durationExpression,
          String(Math.max(0, Math.round(Number(member.durationMs) || 0))),
          "long",
        ),
      );

      if (member.kind === "event") {
        const eventName = member.name?.trim() || "Event";
        groupEventNames.push(eventName);
        holdEvents.push(JSON.stringify(eventName));
      } else {
        holdEvents.push("null");
      }
    });

    const slot = groupChainFields.length;
    groupChainFields.push(fields);
    groupChainSpeeds.push(speeds);
    groupHoldMs.push(holdMs);
    groupHoldEvents.push(holdEvents);
    exportSteps.push({ kind: "group", item, slot });
  });
  flushTopLevelPaths();

  const pick: CameraPickChain | null = pickState.chain;
  const pickCap = pick ? pick.field.charAt(0).toUpperCase() + pick.field.slice(1) : "";
  if (pick) {
    chainExtraCallbacks.set(
      pick.field,
      `.addPoseCallback(${pick.segmentIndex}, new Pose(${pick.endXExpression}, cameraPick.pickY()), this::decide${pickCap}Pick, 0.5)`,
    );
  }
  const rebuiltField = pick?.nextField ?? null;
  const rebuiltCap = rebuiltField ? rebuiltField.charAt(0).toUpperCase() + rebuiltField.slice(1) : "";
  const capOf = (field: string) => field.charAt(0).toUpperCase() + field.slice(1);
  // Chains built from a runtime start: the one after a camera pick (starts where the pick ended) and every
  // Turn first chain (starts where the robot is once it has backed off the wall and turned).
  const turnFields = exportSteps.flatMap((step) => (step.kind === "turn" ? [step.field] : []));
  const builtFields = chainFields.filter((field) => field === rebuiltField || turnFields.includes(field));
  const chainAssignmentBlocks = chainFields.map((field) =>
    builtFields.includes(field)
      ? `${field} = build${capOf(field)}(${chainStartExpressions.get(field)});`
      : `${field} = ${chainRenderers.get(field)!()};`,
  );
  const hasTurn = exportSteps.some((step) => step.kind === "turn");

  const chainFieldDeclarations = chainFields
    .map((field) => `private AutoPath ${field};`)
    .join("\n    ");
  const chainAssignments = chainAssignmentBlocks.join("\n\n      ");

  // Camera pick + Turn first runtime (TeamCode auto/modules/CameraPick, AutoConstants.Autonomous).
  const cameraPickImports = pick
    ? `
import org.firstinspires.ftc.teamcode.auto.modules.CameraPick;
import org.firstinspires.ftc.teamcode.helpers.Alliance;`
    : "";
  // AutoConstants: always (followPathStep's stall guard reads it); TurnInPlace only with a Turn first.
  const autoConstantsImport = `
import org.firstinspires.ftc.teamcode.AutoConstants;` + (hasTurn ? `
import org.firstinspires.ftc.teamcode.auto.modules.TurnInPlace;` : "");
  const cameraPickDeclarations = pick
    ? `
    // Camera pick on ${pick.line.name?.trim() || "a path"} (visualizer: the path's "Camera pick"; auto/modules/CameraPick).
    private CameraPick cameraPick;
    private AutoPath ${pick.field}Pick;
    private boolean ${pick.field}Switched;
`
    : "";
  const turnDeclarations = hasTurn
    ? `
    // Turn first (runTurnStep): where the robot turns, and whether it is still backing off a wall to get there.
    private Pose turnClearPose;
    private boolean turnBackingOff;
`
    : "";
  const cameraPickInit = pick
    ? `
        cameraPick = new CameraPick(hardwareMap, Alliance.${pick.alliance});`
    : "";
  const cameraPickStart = pick
    ? `
        ${pick.field}Pick = null;
        ${pick.field}Switched = false;${rebuiltField ? `
        ${rebuiltField} = build${rebuiltCap}(${chainStartExpressions.get(rebuiltField)});` : ""}`
    : "";
  const cameraPickStop = pick
    ? `        if (cameraPick != null) {
            cameraPick.close();
        }
`
    : "";
  const cameraPickTelemetry = pick
    ? `
        if (cameraPick != null) {
            telemetry.addData("Camera pick", cameraPick.note() + " · " + cameraPick.status());
        }`
    : "";
  const cameraPickMethods = [
    ...builtFields.map(
      (field) => `/**
     * ${field} from a start known only at runtime - ${field === rebuiltField ? "where the camera-pick path ended (its planned end, or the wall spot the camera picked)" : ""}${field === rebuiltField && turnFields.includes(field) ? ", then " : ""}${turnFields.includes(field) ? "where the robot is after its Turn first (backed off the wall, turned)" : ""}; its control points stay where they were planned.
     */
    private AutoPath build${capOf(field)}(Pose start) {
        return ${chainRenderers.get(field)!("start")};
    }`,
    ),
    pick
      ? `/** The camera pick: when ${pick.field} passes CameraPick.pickY(). Builds the drive into the clump, if any. */
    private void decide${pickCap}Pick() {
        Pose end = cameraPick.decide(follower.pose());
        if (end == null) {
            return;
        }
        ${pick.field}Pick = new AutoPath(CameraPick.slamPath(follower.pose(), end));${rebuiltField ? `
        ${rebuiltField} = build${rebuiltCap}(end);` : ""}
    }

    /** ${pick.field}, switched mid-path to the camera-picked drive once decide${pickCap}Pick() has built it. */
    private void followPickStep(AutoPath path, AutoPath pick, double pathSpeed) {
        if (!stepStarted) {
            ${pick.field}Switched = false;
            startPath(path, pathSpeed);
            stepStarted = true;
            resetStallWatch();
        }
        if (pick != null && !${pick.field}Switched) {
            startPath(pick, pathSpeed);
            ${pick.field}Switched = true;
            resetStallWatch();
        }
        if (!follower.isBusy() || stalledNearEnd(${pick.field}Switched && pick != null ? pick : path)) {
            startEvent("Intake Off");
            advanceSequence();
        }
    }`
      : "",
    hasTurn
      ? `/**
     * Turn first: turn in place to {@code headingDeg} (the next path's end heading) before driving it. Two phases,
     * because Foresight's hold moves and turns AT ONCE: (1) back straight off any wall, holding the current
     * heading, until the corners' sweep clears it (TurnInPlace.clearOfWalls) - within TURN_BACKOFF_TOLERANCE_IN or
     * after TURN_BACKOFF_TIMEOUT_MS; (2) turn there - within TURN_TOLERANCE_DEG or after TURN_TIMEOUT_MS. Then
     * {@code rebuild} gets the pose reached, so the next path starts where the robot actually is.
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
    }`
      : "",
  ]
    .filter(Boolean)
    .join("\n\n    ");

  const repeatFieldDeclarations = groupChainFields
    .map(
      (_, slot) => `private AutoPath[] repeat${slot + 1}Paths;
    private double[] repeat${slot + 1}PathSpeeds;
    private long[] repeat${slot + 1}HoldMs;
    private String[] repeat${slot + 1}HoldEvents;`,
    )
    .join("\n    ");
  const repeatAssignments = groupChainFields
    .map(
      (fields, slot) => `repeat${slot + 1}Paths = new AutoPath[] { ${fields.join(", ")} };
      repeat${slot + 1}PathSpeeds = new double[] { ${groupChainSpeeds[slot].join(", ")} };
      repeat${slot + 1}HoldMs = new long[] { ${groupHoldMs[slot].join(", ")} };
      repeat${slot + 1}HoldEvents = new String[] { ${groupHoldEvents[slot].join(", ")} };`,
    )
    .join("\n\n      ");
  const eventItems = sequenceItems.filter((item) => item.kind === "event");
  const eventMethods = new Map<string, { name: string; suffix: string }>();
  const pathEventMarkers = linesWithIds.flatMap((line, lineIndex) =>
    normalizeEventMarkers(line, lineIndex),
  );
  const parallelEventCapacity = Math.max(1, pathEventMarkers.length);

  function registerEventMethod(eventName: string, fallbackSuffix: string) {
    if (eventMethods.has(eventName)) {
      return;
    }

    const baseSuffix = sanitizeJavaMethodSuffix(eventName, fallbackSuffix);
    let suffix = baseSuffix;
    let duplicateIndex = 2;
    while (
      [...eventMethods.values()].some((event) => event.suffix === suffix)
    ) {
      suffix = `${baseSuffix}${duplicateIndex}`;
      duplicateIndex++;
    }
    eventMethods.set(eventName, { name: eventName, suffix });
  }

  eventItems.forEach((item, idx) => {
    const eventName = item.name?.trim() || `Event ${idx + 1}`;
    registerEventMethod(eventName, `Event${idx + 1}`);
  });

  // A Pollen Pickup runs its intake through the same event methods.
  exportSteps.forEach((step, idx) => {
    if (step.kind !== "pollen") return;
    const intakeEvent = step.item.intakeEvent.trim();
    if (intakeEvent) registerEventMethod(intakeEvent, `PollenIntake${idx + 1}`);
  });

  // An event inside a loop needs the same generated method as one beside it.
  groupEventNames.forEach((eventName, idx) => {
    registerEventMethod(eventName, `GroupEvent${idx + 1}`);
  });

  pathEventMarkers.forEach((marker, idx) => {
    registerEventMethod(marker.name, `PathEvent${idx + 1}`);
  });

  /**
   * Wraps a step in its `enabledExpression` guard so a boolean variable can
   * switch whole steps on and off at runtime rather than at export time.
   */
  const guardedCase = (
    idx: number,
    body: string,
    enabledExpression: string | undefined,
  ): string => {
    if (!enabledExpression?.trim()) {
      return `case ${idx}:
        ${body}
        break;`;
    }

    const condition = booleanExpression(enabledExpression);
    if (!condition) {
      return `case ${idx}:
        ${body}
        break;`;
    }

    return `case ${idx}:
        if (${condition}) {
          ${body}
        } else {
          advanceSequence();
        }
        break;`;
  };

  const sequenceCases = exportSteps
    .map((step, idx) => {
      if (step.kind === "chain") {
        return guardedCase(
          idx,
          step.pick
            ? `followPickStep(${step.field}, ${step.field}Pick, ${step.speedExpression});`
            : `followPathStep(${step.field}, ${step.speedExpression});`,
          step.enabledExpression,
        );
      }

      if (step.kind === "turn") {
        return guardedCase(
          idx,
          `runTurnStep(${step.degrees}, start -> ${step.field} = build${capOf(step.field)}(start));`,
          undefined,
        );
      }

      if (step.kind === "wait") {
        return guardedCase(
          idx,
          `runWaitStep(${numberExpression(step.item.durationExpression, `${Math.max(0, Number(step.item.durationMs) || 0)}L`, "long")});`,
          step.item.enabledExpression,
        );
      }

      if (step.kind === "event") {
        return guardedCase(
          idx,
          `runTimedEventStep(${javaStringLiteral(step.item.name || "Event")}, ${numberExpression(step.item.durationExpression, `${Math.max(0, Number(step.item.durationMs) || 0)}L`, "long")});`,
          step.item.enabledExpression,
        );
      }

      if (step.kind === "pollen") {
        return guardedCase(
          idx,
          `runPollenPickupStep(${step.field});`,
          step.item.enabledExpression,
        );
      }

      const item = step.item;
      const fieldName = `repeat${step.slot + 1}`;

      if (item.kind === "repeat") {
        return guardedCase(
          idx,
          `followRepeatStep(${fieldName}Paths, ${fieldName}PathSpeeds, ${fieldName}HoldMs, ${fieldName}HoldEvents, ${numberExpression(item.countExpression, `${Math.max(1, Math.min(20, Math.round(Number(item.count) || 1)))}`, "int")}, ${step.slot});`,
          item.enabledExpression,
        );
      }

      // An `if` block: run its group once, guarded by the condition. Adjacent
      // blocks form an if / else-if chain, so a later branch also requires
      // every earlier condition in its chain to be false.
      // Skip a predecessor whose negation the branch already states itself,
      // so the common "isRed / !isRed" pair stays readable. Whitespace-only
      // comparison keeps this from ever dropping a guard that matters.
      const squash = (text: string) => text.replace(/\s+/g, "");
      const ownCondition = squash(item.condition || "");
      const earlier = (branchPredecessors.get(item.id) || [])
        .filter((condition) => {
          const predecessor = squash(condition);
          return (
            ownCondition !== `!${predecessor}` &&
            ownCondition !== `!(${predecessor})`
          );
        })
        .map((condition) => `!(${condition})`);

      return guardedCase(
        idx,
        `followRepeatStep(${fieldName}Paths, ${fieldName}PathSpeeds, ${fieldName}HoldMs, ${fieldName}HoldEvents, 1, ${step.slot});`,
        combineConditions(...earlier, item.condition),
      );
    })
    .join("\n      ");
  const startEventCases = [...eventMethods.values()]
    .map(
      (event) => `case ${javaStringLiteral(event.name)}:
        start${event.suffix}();
        break;`,
    )
    .join("\n      ");
  const finishEventCases = [...eventMethods.values()]
    .map(
      (event) => `case ${javaStringLiteral(event.name)}:
        finish${event.suffix}();
        break;`,
    )
    .join("\n      ");
  const eventMethodStubs = [...eventMethods.values()]
    .map(
      (event) => `private void start${event.suffix}() {
        // TODO: start ${event.name} mechanism here.
    }

    private void finish${event.suffix}() {
        // TODO: stop ${event.name} mechanism here.
    }`,
    )
    .join("\n\n    ");

  // --- Pollen Pickup -----------------------------------------------------------
  // Search poses come from the same route and time model the editor draws, with
  // every pickup forced on: a step switched off at export time is still switched
  // by its guard at runtime, so it still needs a real pose to return to.
  const pollenSteps = exportSteps.filter(
    (step): step is Extract<ExportStep, { kind: "pollen" }> => step.kind === "pollen",
  );
  const hasPollen = pollenSteps.length > 0;
  const vision = visionOf(settings);
  const pollenSearchPoses = new Map<string, { x: number; y: number; heading: number }>();
  if (hasPollen) {
    const forcedOn = sequenceItems.map((item) =>
      item.kind === "pollen" ? { ...item, enabledExpression: undefined } : item,
    );
    const timeline = calculatePathTime(startPoint, linesWithIds, settings, forcedOn, variables).timeline;
    timeline.forEach((event) => {
      if (event.type !== "maneuver" || !event.itemId || pollenSearchPoses.has(event.itemId)) return;
      pollenSearchPoses.set(event.itemId, {
        x: event.atPoint?.x ?? startPoint.x,
        y: event.atPoint?.y ?? startPoint.y,
        heading: Number(event.startHeading) || 0,
      });
    });
  }
  const midlineSide =
    settings.autoMidline === "red" ? 1 : settings.autoMidline === "blue" ? 2 : 0;
  const javaDouble = (value: number) => {
    const text = Number(value).toFixed(4).replace(/0+$/, "").replace(/\.$/, ".0");
    return text.includes(".") ? text : `${text}.0`;
  };
  const javaTriple = (values: [number, number, number]) =>
    `{${values.map((value) => javaDouble(value)).join(", ")}}`;

  const pollenImports = hasPollen
    ? `
import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pipelines.PollenDetectionPipeline;
import org.firstinspires.ftc.vision.VisionPortal;`
    : "";

  const pollenStepConstants = pollenSteps
    .map(({ item, field }) => {
      const pose = pollenSearchPoses.get(item.id) ?? { x: item.targetX, y: item.targetY, heading: 0 };
      return `// ${item.name.replace(/\*\//g, "* /")}
    private static final PollenPickup ${field} = new PollenPickup(
        ${javaDouble(pose.x)}, ${javaDouble(pose.y)}, ${javaDouble(pose.heading)}, // search pose: where the route has the robot
        ${javaDouble(item.targetX)}, ${javaDouble(item.targetY)}, ${javaDouble(item.zoneRadius)}, // expected POLLEN and zone radius
        ${item.minBalls}, ${item.timeoutMs}L, ${item.onTimeout === "blind"},
        ${javaDouble(item.standoffInches)}, ${javaDouble(item.retargetInches)}, ${javaDouble(item.approachSpeed)},
        ${javaDouble(item.intakeDriveInches)}, ${item.intakeMs}L, ${javaStringLiteral(item.intakeEvent.trim())},
        ${item.returnToStart}
    );`;
    })
    .join("\n\n    ");

  const pollenDeclarations = hasPollen
    ? `
    // --- Pollen Pickup ------------------------------------------------------------
    // Camera model from the visualizer's Settings -> Vision.${
      vision.measured
        ? ""
        : `
    // NOT MEASURED: these are placeholders. Every field position the robot drives to
    // comes from them - measure the mount before trusting a pickup.`
    }
    private static final String CAMERA_NAME = ${javaStringLiteral(vision.cameraName)};
    private static final int CAMERA_WIDTH = ${vision.imageWidth};
    private static final int CAMERA_HEIGHT = ${vision.imageHeight};
    private static final double CAMERA_HFOV_DEG = ${javaDouble(vision.horizontalFovDeg)};
    private static final double CAMERA_VFOV_DEG = ${javaDouble(vision.verticalFovDeg)};
    private static final double CAMERA_FORWARD_IN = ${javaDouble(vision.mountForwardInches)};
    private static final double CAMERA_LEFT_IN = ${javaDouble(vision.mountLeftInches)};
    private static final double CAMERA_HEIGHT_IN = ${javaDouble(vision.mountHeightInches)};
    private static final double CAMERA_PITCH_DEG = ${javaDouble(vision.mountPitchDeg)};
    private static final double CAMERA_YAW_DEG = ${javaDouble(vision.mountYawDeg)};
    private static final double CAMERA_MAX_RANGE_IN = ${javaDouble(vision.maxRangeInches)};
    private static final double POLLEN_RADIUS_IN = ${javaDouble(POLLEN_RADIUS_INCHES)};
    private static final double POLLEN_INTAKE_CREEP_SPEED = ${javaDouble(INTAKE_CREEP_SPEED)};
    /** AUTO midline (BIOBUZZ G402): 0 unchecked, 1 stay on red (x <= mid), 2 stay on blue. */
    private static final int POLLEN_MIDLINE_SIDE = ${midlineSide};
    private static final double FIELD_MIDLINE_X = ${javaDouble(FIELD_SIZE / 2)};

    private static final int POLLEN_SEARCH = 0;
    private static final int POLLEN_APPROACH = 1;
    private static final int POLLEN_INTAKE = 2;
    private static final int POLLEN_RETURN = 3;

    private VisionPortal visionPortal;
    private PollenDetectionPipeline pollenPipeline;
    private int pollenPhase;
    private long pollenPhaseStart;
    private double pollenTargetX;
    private double pollenTargetY;
    private boolean pollenIntakeRunning;
    private String pollenIntakeEvent = "";

    /** One Pollen Pickup step, as the visualizer planned it. */
    private static final class PollenPickup {
        final double searchX, searchY, searchHeadingDeg;
        final double targetX, targetY, zoneRadius;
        final int minBalls;
        final long timeoutMs;
        final boolean blindOnTimeout;
        final double standoffInches, retargetInches, approachSpeed;
        final double intakeDriveInches;
        final long intakeMs;
        final String intakeEvent;
        final boolean returnToStart;

        PollenPickup(
            double searchX, double searchY, double searchHeadingDeg,
            double targetX, double targetY, double zoneRadius,
            int minBalls, long timeoutMs, boolean blindOnTimeout,
            double standoffInches, double retargetInches, double approachSpeed,
            double intakeDriveInches, long intakeMs, String intakeEvent,
            boolean returnToStart
        ) {
            this.searchX = searchX;
            this.searchY = searchY;
            this.searchHeadingDeg = searchHeadingDeg;
            this.targetX = targetX;
            this.targetY = targetY;
            this.zoneRadius = zoneRadius;
            this.minBalls = minBalls;
            this.timeoutMs = timeoutMs;
            this.blindOnTimeout = blindOnTimeout;
            this.standoffInches = standoffInches;
            this.retargetInches = retargetInches;
            this.approachSpeed = approachSpeed;
            this.intakeDriveInches = intakeDriveInches;
            this.intakeMs = intakeMs;
            this.intakeEvent = intakeEvent;
            this.returnToStart = returnToStart;
        }
    }

    ${pollenStepConstants}
`
    : "";

  const pollenInit = hasPollen
    ? `

        // Pollen Pickup: PollenDetectionPipeline tuned from Settings -> Vision.
        PollenDetectionPipeline.Config pollenConfig = new PollenDetectionPipeline.Config();
        pollenConfig.hsvLowA = new double[] ${javaTriple(vision.pipeline.hsvLowA)};
        pollenConfig.hsvHighA = new double[] ${javaTriple(vision.pipeline.hsvHighA)};
        pollenConfig.hsvLowB = new double[] ${javaTriple(vision.pipeline.hsvLowB)};
        pollenConfig.hsvHighB = new double[] ${javaTriple(vision.pipeline.hsvHighB)};
        pollenConfig.openRadius = ${vision.pipeline.openRadius};
        pollenConfig.closeHGap = ${vision.pipeline.closeHGap};
        pollenConfig.closeVRadius = ${vision.pipeline.closeVRadius};
        pollenConfig.minArea = ${javaDouble(vision.pipeline.minArea)};
        pollenConfig.maxArea = ${javaDouble(vision.pipeline.maxArea)};
        pollenConfig.maxAspect = ${javaDouble(vision.pipeline.maxAspect)};
        pollenConfig.clumpMergeGap = ${vision.pipeline.clumpMergeGap};
        pollenConfig.singleBallAreaPx = ${javaDouble(vision.pipeline.singleBallAreaPx)};
        pollenPipeline = new PollenDetectionPipeline(null, pollenConfig);
        visionPortal = new VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName.class, CAMERA_NAME))
            .setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT))
            .addProcessor(pollenPipeline)
            .build();`
    : "";

  const pollenStop = hasPollen
    ? `
        if (pollenIntakeRunning) {
            finishEvent(pollenIntakeEvent);
            pollenIntakeRunning = false;
        }
        if (visionPortal != null) {
            visionPortal.close();
            visionPortal = null;
        }
`
    : "";

  const pollenMethods = hasPollen
    ? `/**
     * A Pollen Pickup: look for POLLEN, drive to the best clump in the zone, run the intake,
     * and come back to where the route has the robot, so every later path starts where it
     * was planned from.
     *
     * The approach re-plans whenever the estimate moves more than retargetInches, which is
     * what makes a camera mount measured to the nearest inch good enough: the field
     * position of a clump is least certain far away, and the step keeps refining it as it
     * closes in. Once the POLLEN drops below the bottom of the image it keeps the last aim.
     */
    private void runPollenPickupStep(PollenPickup step) {
        long now = System.currentTimeMillis();
        if (!stepStarted) {
            stepStarted = true;
            stepStartTime = now;
            pollenPhase = POLLEN_SEARCH;
            pollenPhaseStart = now;
            activePath = null;
        }

        switch (pollenPhase) {
            case POLLEN_SEARCH: {
                double[] seen = bestPollenInZone(step);
                if (seen != null) {
                    pollenTargetX = seen[0];
                    pollenTargetY = seen[1];
                    startPollenApproach(step);
                    return;
                }
                if (now - stepStartTime < step.timeoutMs) {
                    return;
                }
                if (step.blindOnTimeout) {
                    pollenTargetX = step.targetX;
                    pollenTargetY = step.targetY;
                    startPollenApproach(step);
                    return;
                }
                // Nothing in the zone: leave the robot where the route has it.
                advanceSequence();
                return;
            }
            case POLLEN_APPROACH: {
                double[] seen = bestPollenInZone(step);
                if (
                    seen != null &&
                    Math.hypot(seen[0] - pollenTargetX, seen[1] - pollenTargetY) > step.retargetInches
                ) {
                    pollenTargetX = seen[0];
                    pollenTargetY = seen[1];
                    startPollenApproach(step);
                    return;
                }
                if (!follower.isBusy()) {
                    startPollenIntake(step);
                }
                return;
            }
            case POLLEN_INTAKE: {
                if (follower.isBusy() || now - pollenPhaseStart < step.intakeMs) {
                    return;
                }
                if (pollenIntakeRunning) {
                    finishEvent(pollenIntakeEvent);
                    pollenIntakeRunning = false;
                }
                if (step.returnToStart) {
                    startPollenReturn(step);
                } else {
                    advanceSequence();
                }
                return;
            }
            default: {
                if (!follower.isBusy()) {
                    advanceSequence();
                }
            }
        }
    }

    private void startPollenApproach(PollenPickup step) {
        Pose pose = follower.pose();
        double dx = pollenTargetX - pose.x();
        double dy = pollenTargetY - pose.y();
        double distance = Math.hypot(dx, dy);
        double facing = distance > 1e-6 ? Math.atan2(dy, dx) : pose.heading();
        double travel = Math.max(0.0, distance - step.standoffInches);

        pollenPhase = POLLEN_APPROACH;
        pollenPhaseStart = System.currentTimeMillis();
        if (travel < 0.5) {
            // Already inside the standoff: nothing to approach.
            startPollenIntake(step);
            return;
        }
        followPollenLeg(
            pose.x(), pose.y(), pose.heading(),
            pose.x() + Math.cos(facing) * travel, pose.y() + Math.sin(facing) * travel, facing,
            step.approachSpeed
        );
    }

    private void startPollenIntake(PollenPickup step) {
        Pose pose = follower.pose();
        pollenPhase = POLLEN_INTAKE;
        pollenPhaseStart = System.currentTimeMillis();

        if (!step.intakeEvent.isEmpty() && !pollenIntakeRunning) {
            pollenIntakeEvent = step.intakeEvent;
            startEvent(pollenIntakeEvent);
            pollenIntakeRunning = true;
        }

        double dx = pollenTargetX - pose.x();
        double dy = pollenTargetY - pose.y();
        double facing = Math.hypot(dx, dy) > 1e-6 ? Math.atan2(dy, dx) : pose.heading();
        if (step.intakeDriveInches > 0.25) {
            followPollenLeg(
                pose.x(), pose.y(), pose.heading(),
                pose.x() + Math.cos(facing) * step.intakeDriveInches,
                pose.y() + Math.sin(facing) * step.intakeDriveInches,
                facing,
                POLLEN_INTAKE_CREEP_SPEED
            );
        }
    }

    private void startPollenReturn(PollenPickup step) {
        Pose pose = follower.pose();
        pollenPhase = POLLEN_RETURN;
        pollenPhaseStart = System.currentTimeMillis();
        followPollenLeg(
            pose.x(), pose.y(), pose.heading(),
            step.searchX, step.searchY, Math.toRadians(step.searchHeadingDeg),
            1.0
        );
    }

    /** A straight leg built at runtime, turning the short way from one heading to the other. */
    private void followPollenLeg(
        double fromX, double fromY, double fromHeading,
        double toX, double toY, double toHeading,
        double speed
    ) {
        activePath = null;
        if (Math.hypot(toX - fromX, toY - fromY) < 0.5) {
            // A zero-length line has no tangent; there is nowhere to go.
            return;
        }
        Path leg = Paths.line(new Pose(fromX, fromY), new Pose(toX, toY))
            .heading(headingInterpolator(fromHeading, toHeading, 1.0));
        follower.holdEnd.set(true);
        follower.follow(withPathSpeed(leg, clampPathSpeed(speed)));
    }

    /**
     * The best clump this frame that is inside the step's zone and on the allowed side of
     * the midline, as a field position: most estimated balls first, nearest on a tie. Null
     * when there is none.
     */
    private double[] bestPollenInZone(PollenPickup step) {
        if (pollenPipeline == null) {
            return null;
        }
        Pose pose = follower.pose();
        double cos = Math.cos(pose.heading());
        double sin = Math.sin(pose.heading());
        int frameWidth = Math.max(1, pollenPipeline.getFrameWidth());
        int frameHeight = Math.max(1, pollenPipeline.getFrameHeight());

        double[] best = null;
        int bestCount = -1;
        double bestDistance = Double.MAX_VALUE;
        for (PollenDetectionPipeline.Clump clump : pollenPipeline.getAllClumps()) {
            if (clump.estimatedBallCount < step.minBalls) {
                continue;
            }
            // The camera model is for CAMERA_WIDTH x CAMERA_HEIGHT; scale if the stream differs.
            double[] robot = pollenPixelToRobot(
                clump.centerX * CAMERA_WIDTH / frameWidth,
                clump.centerY * CAMERA_HEIGHT / frameHeight
            );
            if (robot == null) {
                continue;
            }
            if (Math.hypot(robot[0] - CAMERA_FORWARD_IN, robot[1] - CAMERA_LEFT_IN) > CAMERA_MAX_RANGE_IN) {
                continue;
            }
            double fieldX = pose.x() + robot[0] * cos - robot[1] * sin;
            double fieldY = pose.y() + robot[0] * sin + robot[1] * cos;
            if (Math.hypot(fieldX - step.targetX, fieldY - step.targetY) > step.zoneRadius) {
                continue;
            }
            if (POLLEN_MIDLINE_SIDE == 1 && fieldX > FIELD_MIDLINE_X) {
                continue;
            }
            if (POLLEN_MIDLINE_SIDE == 2 && fieldX < FIELD_MIDLINE_X) {
                continue;
            }
            double distance = Math.hypot(fieldX - pose.x(), fieldY - pose.y());
            if (
                clump.estimatedBallCount > bestCount ||
                (clump.estimatedBallCount == bestCount && distance < bestDistance)
            ) {
                best = new double[] { fieldX, fieldY };
                bestCount = clump.estimatedBallCount;
                bestDistance = distance;
            }
        }
        return best;
    }

    /**
     * Where a pixel lands on the TILES, in the robot frame (+x forward, +y left), for a
     * POLLEN centre POLLEN_RADIUS_IN above them. Null above the horizon. Mirrors
     * pixelToRobot in the visualizer's src/utils/pollenVision.ts.
     */
    private static double[] pollenPixelToRobot(double u, double v) {
        double pitch = Math.toRadians(CAMERA_PITCH_DEG);
        double yaw = Math.toRadians(CAMERA_YAW_DEG);
        double cosPitch = Math.cos(pitch);
        double sinPitch = Math.sin(pitch);
        double fx = CAMERA_WIDTH / 2.0 / Math.tan(Math.toRadians(CAMERA_HFOV_DEG) / 2.0);
        double fy = CAMERA_HEIGHT / 2.0 / Math.tan(Math.toRadians(CAMERA_VFOV_DEG) / 2.0);
        double xn = (u - CAMERA_WIDTH / 2.0) / fx;
        double yn = (v - CAMERA_HEIGHT / 2.0) / fy;

        // ray = xn * right + yn * down + forward, with right = (0, -1, 0),
        // down = (-sin pitch, 0, -cos pitch), forward = (cos pitch, 0, -sin pitch), then yawed.
        double rayX = -yn * sinPitch + cosPitch;
        double rayY = -xn;
        double rayZ = -yn * cosPitch - sinPitch;
        double yawedX = rayX * Math.cos(yaw) - rayY * Math.sin(yaw);
        double yawedY = rayX * Math.sin(yaw) + rayY * Math.cos(yaw);

        double drop = CAMERA_HEIGHT_IN - POLLEN_RADIUS_IN;
        if (rayZ >= -1e-9 || drop <= 0.0) {
            return null;
        }
        double t = drop / -rayZ;
        return new double[] { CAMERA_FORWARD_IN + t * yawedX, CAMERA_LEFT_IN + t * yawedY };
    }`
    : "";

  const file = `package org.firstinspires.ftc.teamcode.auto;

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
${pollenImports}
${autoConstantsImport}${cameraPickImports}
import org.firstinspires.ftc.teamcode.auto.modules.PathStep;
import org.firstinspires.ftc.teamcode.helpers.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@Autonomous(name = "${autoClassName}", group = "Auto")
public class ${autoClassName} extends OpMode {
    // PathStep fields are editable from FTC Dashboard; paths are built in init(), so edits apply on the next init.
    ${pathStepDeclarationBlock}

    private Follower follower;
    ${chainFieldDeclarations}
    ${repeatFieldDeclarations}
    private AutoPath activePath;
    private int sequenceIndex;
    private long stepStartTime;
    private boolean stepStarted;
    private boolean pathFinished;
    private static final int REPEAT_LOOP_CAPACITY = ${Math.max(1, groupChainFields.length)};
    private final int[] repeatLoopIterations = new int[REPEAT_LOOP_CAPACITY];
    private final int[] repeatLoopPathIndexes = new int[REPEAT_LOOP_CAPACITY];
    private static final int PARALLEL_EVENT_CAPACITY = ${parallelEventCapacity};
    private final String[] activeParallelEventNames = new String[PARALLEL_EVENT_CAPACITY];
    private final long[] activeParallelEventStartTimes = new long[PARALLEL_EVENT_CAPACITY];
    private final long[] activeParallelEventDurations = new long[PARALLEL_EVENT_CAPACITY];
${pollenDeclarations}${cameraPickDeclarations}${turnDeclarations}
    @Override
    public void init() {
        // Pedro 3 follows paths with Foresight, whose braking model and gains must be measured
        // on the robot first. Refuse to build a path-following OpMode on placeholders.
        Constants.requireForesightMeasured();

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(${pointStepExpression(startPoint, "START_STEP")});${cameraPickInit}

        buildPaths();${pollenInit}
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

        follower.setPose(${pointStepExpression(startPoint, "START_STEP")});${cameraPickStart}
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
${cameraPickStop}        finishAllParallelEvents();
${pollenStop}
        if (follower == null) {
            return;
        }

        follower.manual(DrivePowers.zero());
        follower.update();
        PoseStorage.pose = follower.pose();
    }

    private void buildPaths() {
      ${chainAssignments}

      ${repeatAssignments}
    }

    ${cameraPickMethods}

    private void runSequence() {
        if (pathFinished) {
            return;
        }

        switch (sequenceIndex) {
      ${sequenceCases}
            default:
                pathFinished = true;
                activePath = null;
                finishAllParallelEvents();
                follower.manual(DrivePowers.zero());
                break;
        }
    }

    ${HEADING_HELPER_JAVA}

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
        switch (eventName) {
      ${startEventCases}
            default:
                break;
        }
    }

    private void finishEvent(String eventName) {
        switch (eventName) {
      ${finishEventCases}
            default:
                break;
        }
    }

    ${eventMethodStubs}

    ${pollenMethods}

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
        telemetry.addData("Heading", "%.2f", Math.toDegrees(pose.heading()));${cameraPickTelemetry}
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
}`;

  try {
    const javaPlugin = await loadJavaPlugin();
    const formattedCode = await prettier.format(file, {
      parser: "java",
      plugins: javaPlugin ? [javaPlugin] : [],
    });
    return formattedCode;
  } catch (error) {
    console.error("Code formatting error:", error);
    return file;
  }
}

export async function generateJavaCode(
  startPoint: Point,
  lines: Line[],
  exportMode: "full" | "class" | "coordinates" = "class",
  pathChains: PathChain[] = [],
  variables: Variable[] = [],
  sequence: SequenceItem[] = [],
): Promise<string> {
  const linesWithIds = lines.map((line, idx) => ({
    ...line,
    id: line.id || `line-${idx + 1}`,
  }));
  const lineById = new Map(linesWithIds.map((line) => [line.id!, line]));
  const routeStartPoints = buildLineStartPoints(
    startPoint,
    linesWithIds,
    sequence,
    variables,
  );

  const inputChains =
    pathChains.length > 0
      ? pathChains
      : linesWithIds.map((line, idx) => ({
          id: line.id!,
          name: line.name || `Path ${idx + 1}`,
          color: "#22c55e",
          lineIds: [line.id!],
        }));

  const normalizedChains: PathChain[] = inputChains
    .map((chain, idx) => ({
      ...chain,
      id: chain.id || `chain-${idx + 1}`,
      name: chain.name || `PathChain${idx + 1}`,
      lineIds: (chain.lineIds || []).filter((id) => lineById.has(id)),
    }))
    .filter((chain) => chain.lineIds.length > 0);

  // This export has no constants block, so variables are inlined as literals.
  const identifiers: JavaIdentifierMap = {
    scalars: new Map(),
    poses: new Map(),
  };
  variables.forEach((variable) => {
    if (variable.type === "number") {
      const value = Number(variable.value);
      const literal = fixed(Number.isFinite(value) ? value : 0);
      expressionAliases(variable.name).forEach((alias) => {
        if (!identifiers.scalars.has(alias)) identifiers.scalars.set(alias, literal);
      });
    } else if (variable.type === "boolean") {
      const literal = variable.value ? "true" : "false";
      expressionAliases(variable.name).forEach((alias) => {
        if (!identifiers.scalars.has(alias)) identifiers.scalars.set(alias, literal);
      });
    }
  });

  const fieldDeclarations = normalizedChains
    .map((chain, idx) => {
      const variableName = sanitizeIdentifier(
        chain.name,
        `pathChain${idx + 1}`,
      );
      return `public Path ${variableName};`;
    })
    .join("\n    ");

  const pathAssignments = normalizedChains
    .map((chain, chainIdx) => {
      const variableName = sanitizeIdentifier(
        chain.name,
        `pathChain${chainIdx + 1}`,
      );

      const segmentSnippets = chain.lineIds
        .map((lineId, chainLineIndex) => {
          const line = lineById.get(lineId);
          if (!line) return null;

          // A chain runs its own paths in order, so a segment continues from
          // the previous path *in the chain* — the previous entry in `lines`
          // may not even belong to this chain. The first path of a chain starts
          // wherever the route puts it.
          const previousInChain =
            chainLineIndex > 0
              ? lineById.get(chain.lineIds[chainLineIndex - 1])
              : undefined;
          const startPointForLine = previousInChain
            ? previousInChain.endPoint
            : routeStartPoints.get(line.id!) || startPoint;
          const startExpression = buildPoseExpression(
            startPointForLine,
            identifiers,
          );

          return buildPathSegmentCode(line, startExpression, identifiers);
        })
        .filter((segment): segment is string => Boolean(segment));

      // A chain is one Paths.path(...); a single path needs no wrapper.
      return segmentSnippets.length === 1
        ? `${variableName} = ${segmentSnippets[0]};`
        : `${variableName} = Paths.path(
          ${segmentSnippets.join(",\n          ")}
        );`;
    })
    .join("\n\n      ");

  // If coordinates-only mode, return just the path assignments
  if (exportMode === "coordinates") {
    return pathAssignments;
  }

  // Named AutoPaths, not Paths: a class called Paths would shadow Pedro 3's
  // com.pedropathing.api.Paths, which every assignment below calls. Pedro 3
  // paths are built without a Follower, so the constructor takes none.
  const pathsClass = `public static class AutoPaths {
    ${fieldDeclarations}

    public AutoPaths() {
      ${pathAssignments}
    }

    ${HEADING_HELPER_JAVA}
  }`;

  let file = "";
  if (exportMode === "class") {
    file = pathsClass;
  } else {
    file = `package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class PedroAutonomous extends OpMode {
  private TelemetryManager panelsTelemetry; // Panels Telemetry instance
  public Follower follower; // Pedro Pathing follower instance
  private int pathState; // Current autonomous path state (state machine)
  private AutoPaths paths; // Paths defined in the AutoPaths class

  @Override
  public void init() {
    panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    follower = Constants.createFollower(hardwareMap);
    follower.setPose(new Pose(72, 8, Math.toRadians(90)));

    paths = new AutoPaths(); // Build paths

    panelsTelemetry.debug("Status", "Initialized");
    panelsTelemetry.update(telemetry);
  }

  @Override
  public void loop() {
    follower.update(); // Update Pedro Pathing
    pathState = autonomousPathUpdate(); // Update autonomous state machine

    // Log values to Panels and Driver Station
    panelsTelemetry.debug("Path State", pathState);
    panelsTelemetry.debug("X", follower.pose().x());
    panelsTelemetry.debug("Y", follower.pose().y());
    panelsTelemetry.debug("Heading", follower.pose().heading());
    panelsTelemetry.update(telemetry);
  }

  ${pathsClass}

  public int autonomousPathUpdate() {
    // Add your state machine Here
    // Access paths with paths.pathName
    // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
    return 0;
  }
}`;
  }

  try {
    const javaPlugin = await loadJavaPlugin();
    const formattedCode = await prettier.format(file, {
      parser: "java",
      plugins: javaPlugin ? [javaPlugin] : [],
    });
    return formattedCode;
  } catch (error) {
    console.error("Code formatting error:", error);
    return file;
  }
}

/**
 * Generate an array of waypoints (not sampled points) along the path
 */
export function generatePointsArray(startPoint: Point, lines: Line[]): string {
  const points: BasePoint[] = [];

  // Add start point
  points.push(startPoint);

  // Add all waypoints (end points and control points)
  lines.forEach((line) => {
    // Add control points for this line
    line.controlPoints.forEach((controlPoint) => {
      points.push(controlPoint);
    });

    // Add end point of this line
    points.push(line.endPoint);
  });

  // Format as string array, removing decimal places for whole numbers
  const pointsString = points
    .map((point) => {
      const x = Number.isInteger(point.x)
        ? point.x.toFixed(1)
        : point.x.toFixed(3);
      const y = Number.isInteger(point.y)
        ? point.y.toFixed(1)
        : point.y.toFixed(3);
      return `(${x}, ${y})`;
    })
    .join(", ");

  return `[${pointsString}]`;
}

/**
 * Generate Sequential Command code
 */
export async function generateSequentialCommandCode(
  startPoint: Point,
  lines: Line[],
  fileName: string | null = null,
  sequence?: SequenceItem[],
  variables: Variable[] = [],
): Promise<string> {
  // Determine class name from file name or use default
  let className = "AutoPath";
  if (fileName) {
    const baseName = fileName.split(/[\\/]/).pop() || "";
    className = baseName.replace(".pp", "").replace(/[^a-zA-Z0-9]/g, "_");
    if (!className) className = "AutoPath";
  }

  // Collect all pose names including control points
  const allPoseDeclarations: string[] = [];
  const allPoseInitializations: string[] = [];

  /**
   * Java field names have to be unique, so two paths named the same thing (or
   * two names that sanitize to the same identifier) must not both claim it —
   * that produced code that would not compile.
   */
  const usedPoseNames = new Set<string>(["startPoint"]);
  const poseNameByLineId = new Map<string, string>();

  const uniquePoseName = (base: string, fallback: string): string => {
    const cleaned = sanitizeIdentifier(base, fallback);
    let candidate = cleaned;
    let suffix = 2;
    while (usedPoseNames.has(candidate)) {
      candidate = `${cleaned}${suffix}`;
      suffix++;
    }
    usedPoseNames.add(candidate);
    return candidate;
  };

  // Add start point
  allPoseDeclarations.push("  private Pose startPoint;");
  allPoseInitializations.push('    startPoint = pp.get("startPoint");');

  // Process each line
  lines.forEach((line, lineIdx) => {
    const endPointName = uniquePoseName(
      line.name || `point${lineIdx + 1}`,
      `point${lineIdx + 1}`,
    );
    poseNameByLineId.set(line.id || `line-${lineIdx + 1}`, endPointName);

    // Add end point declaration
    allPoseDeclarations.push(`  private Pose ${endPointName};`);
    allPoseInitializations.push(
      `    ${endPointName} = pp.get(\"${endPointName}\");`,
    );

    // Add control points if they exist
    if (line.controlPoints && line.controlPoints.length > 0) {
      line.controlPoints.forEach((_, controlIdx) => {
        const controlPointName = `${endPointName}_control${controlIdx + 1}`;
        allPoseDeclarations.push(`  private Pose ${controlPointName};`);
        allPoseInitializations.push(
          `    ${controlPointName} = pp.get(\"${controlPointName}\");`,
        );
      });
    }
  });

  /** The declared pose field for a line's endpoint. */
  const poseNameFor = (lineIdx: number): string =>
    poseNameByLineId.get(lines[lineIdx]?.id || `line-${lineIdx + 1}`) ||
    `point${lineIdx + 1}`;

  /** The pose a path starts from, following the lines array. */
  const startPoseNameFor = (lineIdx: number): string =>
    lineIdx <= 0 ? "startPoint" : poseNameFor(lineIdx - 1);

  const usedPathNames = new Set<string>();
  const pathNameByLineIdx = new Map<number, string>();
  lines.forEach((_, lineIdx) => {
    const base = `${startPoseNameFor(lineIdx)}TO${poseNameFor(lineIdx)}`;
    let candidate = base;
    let suffix = 2;
    while (usedPathNames.has(candidate)) {
      candidate = `${base}${suffix}`;
      suffix++;
    }
    usedPathNames.add(candidate);
    pathNameByLineIdx.set(lineIdx, candidate);
  });

  // Generate path declarations
  const pathChainDeclarations = lines
    .map((_, idx) => `  private Path ${pathNameByLineIdx.get(idx)};`)
    .join("\n");

  // Generate ProgressTracker field
  const progressTrackerField = `  private final ProgressTracker progressTracker;`;

  // Generate addCommands calls with event handling; iterate sequence if provided
  const commands: string[] = [];

  const defaultSequence: SequenceItem[] = lines.map((ln, idx) => ({
    kind: "path",
    lineId: ln.id || `line-${idx + 1}`,
  }));
  const rawSeq = sequence && sequence.length ? sequence : defaultSequence;
  const seq: SequenceItem[] = [];
  rawSeq.forEach((item) => {
    // This format has no branching, so an `if` block contributes its paths
    // only when its condition currently holds.
    if (item.kind === "conditional") {
      if (!activeConditionalIds(rawSeq, variables).has(item.id)) return;
      (item.lineIds || []).forEach((lineId) => {
        seq.push({ kind: "path", lineId });
      });
      return;
    }

    if (item.kind !== "repeat") {
      seq.push(item);
      return;
    }
    const repeatCount = Math.max(
      1,
      Math.min(20, Math.round(Number(item.count) || 1)),
    );
    for (let repeatIndex = 0; repeatIndex < repeatCount; repeatIndex++) {
      (item.lineIds || []).forEach((lineId) => {
        seq.push({ kind: "path", lineId });
      });
    }
  });

  seq.forEach((item, idx) => {
    if (item.kind === "wait" || item.kind === "event") {
      commands.push(`        new WaitCommand(${(item as any).durationMs})`);
      return;
    }
    const lineIdx = lines.findIndex((l) => l.id === (item as any).lineId);
    if (lineIdx < 0) {
      return; // skip if sequence references a missing line
    }
    const line = lines[lineIdx];
    if (!line) {
      return;
    }
    const pathName = pathNameByLineIdx.get(lineIdx)!;
    const pathDisplayName = pathName;

    if (line.eventMarkers && line.eventMarkers.length > 0) {
      // Path has event markers - use reg.java style structure
      // First: InstantCommand to set up tracker
      commands.push(`        new InstantCommand(
            () -> {
              progressTracker.setCurrentChain(${pathName});
              progressTracker.setCurrentPathName("${pathDisplayName}");`);

      // Add event registrations
      line.eventMarkers.forEach((event) => {
        commands[commands.length - 1] += `
              progressTracker.registerEvent("${event.name}", ${event.position.toFixed(3)});`;
      });

      commands[commands.length - 1] += `
            })`;

      // Second: ParallelRaceGroup for following path with event handling
      commands.push(`        new ParallelRaceGroup(
            follow(${pathName}),
            new SequentialCommandGroup(`);

      // Add WaitUntilCommand for each event
      line.eventMarkers.forEach((event, eventIdx) => {
        if (eventIdx > 0) commands[commands.length - 1] += ",";
        commands[commands.length - 1] += `
                new WaitUntilCommand(() -> progressTracker.shouldTriggerEvent("${event.name}")),
                new InstantCommand(
                    () -> {
                      progressTracker.executeEvent("${event.name}");
                    })`;
      });

      commands[commands.length - 1] += `
            ))`;
    } else {
      // No event markers - simple InstantCommand + FollowPathCommand
      commands.push(`        new InstantCommand(
            () -> {
              progressTracker.setCurrentChain(${pathName});
              progressTracker.setCurrentPathName("${pathDisplayName}");
            }),
        follow(${pathName})`);
    }
  });

  // Generate path building
  const pathBuilders = lines
    .map((line, idx) => {
      const startPoseName = startPoseNameFor(idx);
      const endPoseName = poseNameFor(idx);
      const pathName = pathNameByLineIdx.get(idx)!;

      const isCurve = line.controlPoints.length > 0;
      const factory = isCurve ? "Paths.curve" : "Paths.line";

      // Build control points string
      let controlPointsStr = "";
      if (isCurve) {
        const controlPoints: string[] = [];
        line.controlPoints.forEach((_, cpIdx) => {
          const controlPointName = `${endPoseName}_control${cpIdx + 1}`;
          controlPoints.push(controlPointName);
        });
        controlPointsStr = controlPoints.join(", ") + ", ";
      }

      // Determine heading interpolation. Linear goes through the generated
      // helper so it follows the curve parameter, as the preview draws it;
      // reverse only exists on a tangential heading.
      let headingConfig = "";
      if (line.endPoint.heading === "constant") {
        headingConfig = `.constant(${endPoseName}.heading())`;
      } else if (line.endPoint.heading === "linear") {
        headingConfig = `.heading(headingInterpolator(${startPoseName}.heading(), ${endPoseName}.heading(), ${fixed(headingCurve(line))}))`;
      } else {
        headingConfig = line.endPoint.reverse ? ".reverseTangent()" : ".tangent()";
      }

      return `${pathName} =
        ${factory}(${startPoseName}, ${controlPointsStr}${endPoseName})${headingConfig};`;
    })
    .join("\n\n    ");

  const sequentialCommandCode = `
package org.firstinspires.ftc.teamcode.Commands.AutoCommands;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.FunctionalCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utils.Pathing.ProgressTracker;
import java.io.IOException;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Utils.PedroPathReader;

public class ${className} extends SequentialCommandGroup {

  private final Follower follower;
  ${progressTrackerField}

  // Poses
${allPoseDeclarations.join("\n")}

  // Paths
${pathChainDeclarations}

  public ${className}(final Drivetrain drive, HardwareMap hw, Telemetry telemetry) throws IOException {
    this.follower = drive.getFollower();
    this.progressTracker = new ProgressTracker(follower, telemetry);

    PedroPathReader pp = new PedroPathReader("${fileName ? fileName.split(/[\\/]/).pop() + ".pp" || "AutoPath.pp" : "AutoPath.pp"}", hw.appContext);

    // Load poses
${allPoseInitializations.join("\n")}

    follower.setPose(startPoint);

    buildPaths();

    addCommands(
${commands.join(",\n")});
  }

  public void buildPaths() {
    ${pathBuilders}
  }

  /**
   * Follows one Pedro 3 path until the follower settles at its end. SolversLib's
   * FollowPathCommand takes Pedro 2's PathChain, so this is the same command on Pedro 3's
   * Follower.follow / isBusy. The follower still has to be updated every loop, as before.
   */
  private Command follow(Path path) {
    return new FunctionalCommand(
        () -> follower.follow(path),
        () -> {},
        interrupted -> {},
        () -> !follower.isBusy());
  }

  ${HEADING_HELPER_JAVA}
}
`;

  try {
    const javaPlugin = await loadJavaPlugin();
    const formattedCode = await prettier.format(sequentialCommandCode, {
      parser: "java",
      plugins: javaPlugin ? [javaPlugin] : [],
    });
    return formattedCode;
  } catch (error) {
    console.error("Code formatting error:", error);
    return sequentialCommandCode;
  }
}
