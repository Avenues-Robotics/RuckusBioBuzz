/**
 * Reading a saved project (`.pp`) into the shape the rest of the app works in.
 *
 * This is the ONE place a file is normalized: ids filled in, legacy fields
 * migrated, expressions resolved against the variables, pose variables bound.
 * The editor loads a file through here, and so does the MCP server in `mcp/`,
 * which is the point - a review of a path outside the editor has to be the
 * same path the editor would draw, or its numbers are worth nothing.
 */
import { normalizeCameraPick } from "./cameraPick";
import type {
  EventTriggerType,
  Line,
  PathChain,
  Point,
  PoseVariable,
  SequenceItem,
  Settings,
  Shape,
  Variable,
} from "../types";
import { DEFAULT_SETTINGS, getDefaultShapes, getDefaultStartPoint } from "../config/defaults";
import { getRandomColor } from "./draw";
import {
  buildExpressionScope,
  resolvePointExpressions,
  resolveVariableValues,
} from "./numberExpressions";
import { migrateSequenceGroups } from "./sequence";
import {
  migrateLine,
  migrateSequenceItem,
  migrateVariables,
  poseVariablesOf,
  resolveLineExpressions,
  resolveSequenceItemExpressions,
} from "./variables";

export const makeChainId = () =>
  `chain-${Date.now().toString(36)}-${Math.random().toString(36).slice(2, 8)}`;

export const defaultPathChainName = "Main Chain";

export const createDefaultPathChain = (sourceLines: Line[]): PathChain => ({
  id: makeChainId(),
  name: defaultPathChainName,
  color: getRandomColor(),
  lineIds: sourceLines.map((ln) => ln.id!).filter(Boolean),
});

export function normalizeEventMarkers(markers: Line["eventMarkers"] = []) {
  return (markers || []).map((marker, index) => {
    const position = Number(marker.position ?? 0.5);
    const durationMs = Number(marker.durationMs ?? 0);
    const triggerMs = Number(marker.triggerMs ?? 0);
    const triggerType: EventTriggerType =
      marker.triggerType === "temporal" || marker.triggerType === "pose"
        ? marker.triggerType
        : "parametric";

    return {
      ...marker,
      id: marker.id || `event-${Math.random().toString(36).slice(2)}`,
      name: (marker.name || "").trim() || `Event ${index + 1}`,
      triggerType,
      position: Number.isFinite(position)
        ? Math.max(0, Math.min(1, position))
        : 0.5,
      triggerMs: Number.isFinite(triggerMs)
        ? Math.max(0, Math.round(triggerMs))
        : 0,
      ...(Number.isFinite(Number(marker.poseX))
        ? { poseX: Number(marker.poseX) }
        : {}),
      ...(Number.isFinite(Number(marker.poseY))
        ? { poseY: Number(marker.poseY) }
        : {}),
      durationMs: Number.isFinite(durationMs)
        ? Math.max(0, Math.round(durationMs))
        : 0,
    };
  });
}

export function normalizeLines(input: Line[]): Line[] {
  return (input || []).map((line) => ({
    ...line,
    id: line.id || `line-${Math.random().toString(36).slice(2)}`,
    controlPoints: line.controlPoints || [],
    color: line.color || getRandomColor(),
    name: line.name || "",
    speed: Math.max(0.05, Math.min(1, Number(line.speed ?? 1) || 1)),
    waitBeforeMs: Math.max(
      0,
      Number(line.waitBeforeMs ?? line.waitBefore?.durationMs ?? 0),
    ),
    waitAfterMs: Math.max(
      0,
      Number(line.waitAfterMs ?? line.waitAfter?.durationMs ?? 0),
    ),
    waitBeforeName: line.waitBeforeName ?? line.waitBefore?.name ?? "",
    waitAfterName: line.waitAfterName ?? line.waitAfter?.name ?? "",
    eventMarkers: normalizeEventMarkers(line.eventMarkers),
    cameraPick: line.cameraPick ? normalizeCameraPick(line.cameraPick, line.endPoint) : undefined,
    turnFirst: line.turnFirst ? true : undefined,
  }));
}

/** Fills in missing ids/names so every variable is addressable. */
export function normalizeVariable(variable: Variable, index: number): Variable {
  const id = variable.id || `var-${Math.random().toString(36).slice(2)}`;
  const name = (variable.name || "").trim() || `value${index + 1}`;

  if (variable.type === "pose") {
    return {
      ...variable,
      id,
      name,
      x: Number.isFinite(Number(variable.x)) ? Number(variable.x) : 0,
      y: Number.isFinite(Number(variable.y)) ? Number(variable.y) : 0,
      heading: Number.isFinite(Number(variable.heading))
        ? Number(variable.heading)
        : 0,
    };
  }

  if (variable.type === "number") {
    const value = Number(variable.value);
    return { ...variable, id, name, value: Number.isFinite(value) ? value : 0 };
  }

  if (variable.type === "boolean") {
    return { ...variable, id, name, value: Boolean(variable.value) };
  }

  return { ...variable, id, name };
}

/**
 * Builds the unified variable list from a save file, accepting both the
 * current `variables` field and the legacy per-type arrays.
 */
export function normalizeVariables(source: {
  variables?: Variable[];
  numberVariables?: any[];
  poseVariables?: any[];
  pathVariables?: any[];
}): Variable[] {
  const migrated = migrateVariables(source).map(normalizeVariable);
  const resolved = resolveVariableValues(migrated);
  const poseVariables = poseVariablesOf(resolved);

  // Path variables hold their own lines, so normalize those too.
  return resolved.map((variable) => {
    if (variable.type !== "path") return variable;

    const normalizedLines = applyVariablesToLines(
      normalizeLines(variable.lines || []),
      resolved,
    );
    const normalizedPath = applyPoseVariablesToPath(
      resolvePointExpressions(
        variable.startPoint || getDefaultStartPoint(),
        resolved,
      ),
      normalizedLines,
      poseVariables,
    );

    return {
      ...variable,
      startPoint: normalizedPath.startPoint,
      lines: normalizedPath.lines,
    };
  });
}

export function applyVariablesToLines(
  sourceLines: Line[],
  sourceVariables: Variable[],
): Line[] {
  const scope = buildExpressionScope(sourceVariables);
  return sourceLines.map((line) =>
    resolveLineExpressions(migrateLine(line, sourceVariables), sourceVariables, scope),
  );
}

export function applyVariablesToSequence(
  sourceSequence: SequenceItem[],
  sourceVariables: Variable[],
): SequenceItem[] {
  const scope = buildExpressionScope(sourceVariables);
  // Groups used to hold a bare list of path ids; convert once on the way in so
  // nothing downstream has to know that.
  return migrateSequenceGroups(sourceSequence).map((item) =>
    resolveSequenceItemExpressions(
      migrateSequenceItem(item, sourceVariables),
      sourceVariables,
      scope,
    ),
  );
}

export function bindPointToPoseVariable(
  point: Point,
  variable: PoseVariable,
  forcePoseHeading = false,
): Point {
  const targetHeading = Number.isFinite(Number(variable.heading))
    ? Number(variable.heading)
    : 0;
  const basePoint = {
    x: Number(variable.x) || 0,
    y: Number(variable.y) || 0,
    locked: point.locked,
    poseVariableId: variable.id,
  };

  // The start point keeps its heading in `startDeg`/`degrees`, never `endDeg`,
  // so writing the pose heading into the linear branch would leave the real
  // start heading behind. Give it a definite heading instead.
  if (forcePoseHeading) {
    return {
      ...basePoint,
      heading: "constant",
      degrees: targetHeading,
    };
  }

  if (point.heading === "linear") {
    return {
      ...basePoint,
      heading: "linear",
      startDeg: Number.isFinite(Number(point.startDeg))
        ? Number(point.startDeg)
        : targetHeading,
      endDeg: targetHeading,
      headingCurve: point.headingCurve ?? 1,
    };
  }

  if (point.heading === "tangential") {
    return {
      ...basePoint,
      heading: "tangential",
      reverse: point.reverse ?? false,
    };
  }

  return {
    ...basePoint,
    heading: "constant",
    degrees: targetHeading,
  };
}

export function clearMissingPoseVariable(
  point: Point,
  variablesById: Map<string, PoseVariable>,
  isStartPoint = false,
): Point {
  if (!point.poseVariableId) return point;

  const variable = variablesById.get(point.poseVariableId);
  if (variable) {
    return bindPointToPoseVariable(point, variable, isStartPoint);
  }

  const { poseVariableId, ...nextPoint } = point;
  return nextPoint as Point;
}

export function applyPoseVariablesToPath(
  sourceStartPoint: Point,
  sourceLines: Line[],
  sourcePoseVariables: PoseVariable[],
): { startPoint: Point; lines: Line[] } {
  const variablesById = new Map(
    sourcePoseVariables.map((variable) => [variable.id, variable]),
  );

  return {
    startPoint: clearMissingPoseVariable(sourceStartPoint, variablesById, true),
    lines: sourceLines.map((line) => ({
      ...line,
      endPoint: clearMissingPoseVariable(line.endPoint, variablesById),
    })),
  };
}

export function normalizePathChains(
  sourceChains: PathChain[] | undefined,
  sourceLines: Line[],
): PathChain[] {
  const validLineIds = new Set(sourceLines.map((ln) => ln.id!).filter(Boolean));
  const cleaned = (sourceChains || [])
    .map((chain) => ({
      ...chain,
      id: chain.id || makeChainId(),
      name: (chain.name || "").trim() || defaultPathChainName,
      color: chain.color || getRandomColor(),
      lineIds: (chain.lineIds || []).filter((id) => validLineIds.has(id)),
    }));

  if (cleaned.length === 0) {
    return [createDefaultPathChain(sourceLines)];
  }

  return cleaned;
}

/** A project file, normalized and ready to measure or draw. */
export interface LoadedProject {
  startPoint: Point;
  lines: Line[];
  sequence: SequenceItem[];
  pathChains: PathChain[];
  variables: Variable[];
  shapes: Shape[];
  settings: Settings;
  /** True when the file carried no obstacles and the field preset was used. */
  usedFieldObstaclePreset: boolean;
}

export interface LoadProjectOptions {
  /** What the file's own settings are layered onto. */
  baseSettings?: Settings;
  /**
   * Measure a file that carries no obstacles of its own against the preset for
   * the field it names, rather than against an empty field. True for a review
   * outside the editor; false in the editor, where an empty obstacle list is
   * the user's own doing and reloading should not undo it.
   */
  useFieldObstaclePreset?: boolean;
}

/** Normalizes a parsed `.pp` file. */
export function loadProjectData(
  data: any,
  options: LoadProjectOptions = {},
): LoadedProject {
  const baseSettings = options.baseSettings ?? DEFAULT_SETTINGS;
  const variables = normalizeVariables(data || {});
  const startPointIn = resolvePointExpressions(
    data?.startPoint || getDefaultStartPoint(),
    variables,
  );
  const lines = applyVariablesToLines(normalizeLines(data?.lines || []), variables);
  const path = applyPoseVariablesToPath(startPointIn, lines, poseVariablesOf(variables));
  const sequence = applyVariablesToSequence(
    (data?.sequence && data.sequence.length
      ? data.sequence
      : path.lines.map((line) => ({ kind: "path", lineId: line.id! }))) as SequenceItem[],
    variables,
  );
  const settings: Settings = { ...baseSettings, ...(data?.settings || {}) };

  // A file with no obstacles of its own is measured against the field it names,
  // so a review is not silently run with nothing in the way.
  const fileShapes: Shape[] = Array.isArray(data?.shapes) ? data.shapes : [];
  const usedFieldObstaclePreset =
    fileShapes.length === 0 && options.useFieldObstaclePreset !== false;

  return {
    startPoint: path.startPoint,
    lines: path.lines,
    sequence,
    pathChains: normalizePathChains(data?.pathChains, path.lines),
    variables,
    shapes: usedFieldObstaclePreset ? getDefaultShapes(settings.fieldMap) : fileShapes,
    settings,
    usedFieldObstaclePreset,
  };
}
