<script lang="ts">
  import type {
    Line,
    BasePoint,
    Settings,
    Point,
    SequenceItem,
    PathChain,
    Shape,
    PoseVariable,
    Variable,
    EventTriggerType,
    TimePrediction,
    ChainProfile,
  } from "./types";
  import * as d3 from "d3";
  import {
    snapToGrid,
    gridSize,
    currentFilePath,
    isUnsaved,
    showGrid,
    dualPathMode,
    secondFilePath,
    activePaths,
  } from "./stores";
  import Two from "two.js";
  import type { Path } from "two.js/src/path";
  import type { Line as PathLine } from "two.js/src/shapes/line";
  import ControlTab from "./lib/ControlTab.svelte";
  import Navbar from "./lib/Navbar.svelte";
  import MathTools from "./lib/MathTools.svelte";
  import SaveDialog from "./lib/components/SaveDialog.svelte";
  import DualPathSaveDialog from "./lib/components/DualPathSaveDialog.svelte";
  import ProgressDialog from "./lib/components/ProgressDialog.svelte";
  import _ from "lodash";
  import hotkeys from "hotkeys-js";
  import { createAnimationController } from "./utils/animation";
  import {
    buildTravelLineTimingMetas,
    calculateCurveLength,
    calculateMotionProfileDistanceAtTime,
    calculateMotionProfileTime,
    calculateMotionProfileVelocityAtDistance,
    calculatePathTime,
    profileVelocityAtDistance,
    getAnimationDuration,
    getPathSpeed,
  } from "./utils";
  import { exportAsGif, downloadBlob } from "./utils/gifExporter";

  import {
    calculateRobotState,
    checkClearance,
    footprintFromSettings,
    generateGhostPathPoints,
    generateOnionLayers,
    midlineRuleFromSettings,
  } from "./utils";
  import type { ManeuverLeg, StationaryPose } from "./utils/clearance";
  import { planPollenSteps, pollenLegsOf } from "./utils/pollenVision";
  import { expandRouteEdits } from "./utils/cameraPick";
  import { buildEventTimingWindows } from "./utils/timeCalculator";
  import MechanismOverlay from "./lib/components/MechanismOverlay.svelte";
  import {
    allianceOf,
    buildMechanismTimeline,
    hiveAimPoint,
    mechanismStateAt,
    TURRET_BEHIND_INCHES,
  } from "./utils/mechanismStates";
  import {
    NECTAR_DIAMETER_INCHES,
    aheadOf,
    nearestPoint,
    scoringPointsFor,
    shotFrame,
    spriteMotion,
    stateWindowsOf,
    vfxAt,
    type ActiveVfx,
    type ScoringPoint,
  } from "./utils/robotStates";
  import RobotVfx from "./lib/components/RobotVfx.svelte";
  import {
    buildExpressionScope,
    easeInOutQuad,
    getCurvePoint,
    getRandomColor,
    migrateLine,
    migrateSequenceGroups,
    migrateSequenceItem,
    migrateVariables,
    poseVariablesOf,
    resolveLineExpressions,
    resolvePointExpressions,
    resolveSequenceItemExpressions,
    resolveVariableValues,
    skippedLineIds,
    isEnabled,
    buildChainRuns,
    buildLineStartPoints,
    buildRoute,
    CHAIN_BREAK_LABELS,
    reconcileSequence,
    quadraticToCubic,
    radiansToDegrees,
    shortestRotation,
    downloadTrajectory,
    loadTrajectoryFromFile,
  } from "./utils";
  import {
    POINT_RADIUS,
    LINE_WIDTH,
    DEFAULT_ROBOT_WIDTH,
    DEFAULT_ROBOT_HEIGHT,
    DEFAULT_SETTINGS,
    FIELD_SIZE,
    getDefaultStartPoint,
    getDefaultLines,
    getDefaultShapes,
    isUntouchedObstaclePreset,
  } from "./config";
  import { loadSettings, saveSettings } from "./utils/settingsPersistence";
  import * as browserFileStore from "./utils/browserFileStore";
  import { onMount, tick } from "svelte";
  import { debounce } from "lodash";
  import { createHistory, type AppState } from "./utils/history";
  import {
    applyPoseVariablesToPath,
    applyVariablesToLines,
    applyVariablesToSequence,
    bindPointToPoseVariable,
    clearMissingPoseVariable,
    createDefaultPathChain,
    defaultPathChainName,
    loadProjectData,
    makeChainId,
    normalizeEventMarkers,
    normalizeLines,
    normalizePathChains,
    normalizeVariable,
    normalizeVariables,
  } from "./utils/projectFile";
  // Browser-only build: file operations use the browser file store and
  // localStorage. Electron-specific APIs have been removed.










  // Canvas state
  let two: Two;
  let twoElement: HTMLDivElement;
  let width = 0;
  let height = 0;
  // Robot state
  $: robotWidth = settings?.rWidth || DEFAULT_ROBOT_WIDTH;
  $: robotHeight = settings?.rHeight || DEFAULT_ROBOT_HEIGHT;
  let robotXY: BasePoint = { x: 0, y: 0 };
  let robotHeading: number = 0;
  let additionalRobotStates: Array<{ xy: BasePoint; heading: number }> = [];
  // Animation state
  let percent: number = 0;
  let playing = false;
  // Playback timing lives entirely in the animation controller.
  // Save dialog state
  let showSaveDialog = false;
  let showDualPathSaveDialog = false;
  let isSaving = false;
  // GIF export state
  let exportingGif = false;
  let gifExportProgress = 0;
  let gifExportStatus = "Preparing...";
  let cancelGifExport = false;
  // Path data
  let settings: Settings = { ...DEFAULT_SETTINGS };
  let startPoint: Point = getDefaultStartPoint();
  let lines: Line[] = normalizeLines(getDefaultLines());
  let variables: Variable[] = [];

  function normalizeLegacyFieldMap(input: Settings): Settings {
    const next = { ...input };

    if (typeof next.fieldMap === "string" && next.fieldMap.startsWith("custom||")) {
      const [, embeddedImage = ""] = next.fieldMap.split("||");
      next.fieldMap = "custom";
      if (embeddedImage && !next.customFieldImage) {
        next.customFieldImage = embeddedImage;
      }
    }

    if (!next.fieldMap) {
      next.fieldMap = DEFAULT_SETTINGS.fieldMap;
    }

    return next;
  }

  $: fieldMapSrc =
    settings.fieldMap === "custom"
      ? settings.customFieldImage || `/fields/${DEFAULT_SETTINGS.fieldMap}`
      : settings.fieldMap
        ? `/fields/${settings.fieldMap}`
        : `/fields/${DEFAULT_SETTINGS.fieldMap}`;
  let sequence: SequenceItem[] = lines.map((ln) => ({
    kind: "path",
    lineId: ln.id!,
  }));
  let pathChains: PathChain[] = [createDefaultPathChain(lines)];
  let shapes: Shape[] = getDefaultShapes(settings.fieldMap);
  let optimizingLineIds: Record<string, boolean> = {};
  let optimizingAll = false;

  // Second path data (for alliance coordination) - DEPRECATED, use additionalPaths
  let secondStartPoint: Point | null = null;
  let secondLines: Line[] = [];
  let secondSequence: SequenceItem[] = [];
  let secondShapes: Shape[] = [];

  // Multiple paths data (new system - supports up to 4 paths total)
  interface AdditionalPathData {
    filePath: string;
    startPoint: Point | null;
    lines: Line[];
    sequence: SequenceItem[];
    shapes: Shape[];
    settings: Settings;
    pathChains?: PathChain[];
    variables?: Variable[];
    color?: string; // Optional custom color for this path
  }
  let additionalPaths: AdditionalPathData[] = [];

  type CanvasPathElement = Path | PathLine;
  type PathOverlayItem = {
    id: string;
    x: number;
    y: number;
    label: string;
    color?: string;
  };
  type EventTimelineOverlay = {
    elements: CanvasPathElement[];
    labels: PathOverlayItem[];
  };
  type SwerveWheelOverlay = {
    id: string;
    x: number;
    y: number;
    angle: number;
    length: number;
    thickness: number;
    opacity: number;
    color: string;
  };
  type OnionLayerState = {
    x: number;
    y: number;
    heading: number;
    corners: BasePoint[];
    lineIndex: number;
    t?: number;
  };
  /** What the timeline says about driving a given path. */
  type TravelState = {
    chain?: ChainProfile;
  };

  type CurveSample = {
    point: BasePoint;
    distance: number;
    t: number;
  };

  const PATH_RENDER_SAMPLES = 100;
  const AUTO_COUNTDOWN_SECONDS = 30;
  const SWERVE_X_LOCK_LOCAL_CSS_ANGLES = [-45, 45, -135, 135];

  function clampNumber(value: number, min: number, max: number): number {
    if (!Number.isFinite(value)) return min;
    return Math.max(min, Math.min(max, value));
  }

  function setImageFallback(e: Event, fallbackSrc: string) {
    (e.currentTarget as HTMLImageElement).src = fallbackSrc;
  }

  function formatSeconds(value: number): string {
    const absolute = Math.abs(value);
    if (absolute >= 60) {
      const minutes = Math.floor(absolute / 60);
      const seconds = absolute % 60;
      return `${minutes}:${seconds.toFixed(1).padStart(4, "0")}`;
    }
    return `${absolute.toFixed(1)}s`;
  }

  function formatCountdown(value: number): string {
    return value < 0 ? `+${formatSeconds(value)}` : formatSeconds(value);
  }

  function getCountdownColor(remainingSeconds: number): string {
    if (remainingSeconds <= 3) return "#dc2626";
    if (remainingSeconds <= 10) return "#f59e0b";
    return "#16a34a";
  }

  function getLineStartPoint(
    sourceStartPoint: Point,
    sourceLines: Line[],
    lineIndex: number,
  ): BasePoint | null {
    if (lineIndex === 0) return sourceStartPoint;
    return sourceLines[lineIndex - 1]?.endPoint || null;
  }

  function sampleLineCurve(
    sourceStartPoint: BasePoint,
    line: Line,
    samples = PATH_RENDER_SAMPLES,
  ): CurveSample[] {
    const sampleCount = Math.max(1, Math.round(samples));
    const curvePoints = [sourceStartPoint, ...line.controlPoints, line.endPoint];
    const sampledPoints: CurveSample[] = [
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

  function getLinePointAtT(
    sourceStartPoint: BasePoint,
    line: Line,
    position: number,
  ): BasePoint {
    const t = clampNumber(position, 0, 1);
    return getCurvePoint(t, [sourceStartPoint, ...line.controlPoints, line.endPoint]);
  }

  function getLinePointAtDistance(
    sourceStartPoint: BasePoint,
    line: Line,
    targetDistance: number,
  ): BasePoint {
    const samples = sampleLineCurve(sourceStartPoint, line);
    const totalDistance = samples[samples.length - 1]?.distance || 0;
    const clampedDistance = clampNumber(targetDistance, 0, totalDistance);

    for (let i = 1; i < samples.length; i++) {
      const previous = samples[i - 1];
      const current = samples[i];
      if (current.distance < clampedDistance) continue;

      const segmentDistance = current.distance - previous.distance;
      const ratio =
        segmentDistance <= 0
          ? 0
          : (clampedDistance - previous.distance) / segmentDistance;

      return {
        x: previous.point.x + (current.point.x - previous.point.x) * ratio,
        y: previous.point.y + (current.point.y - previous.point.y) * ratio,
      };
    }

    return samples[samples.length - 1]?.point || sourceStartPoint;
  }

  function getLineMotionValues(line: Line, routeSettings: Settings) {
    const pathSpeed = getPathSpeed(line);
    return {
      maxVelocity: Math.max(0, Number(routeSettings.maxVelocity) || 0) * pathSpeed,
      maxAcceleration:
        Math.max(0, Number(routeSettings.maxAcceleration) || 0) * pathSpeed,
      maxDeceleration:
        Math.max(
          0,
          Number(routeSettings.maxDeceleration ?? routeSettings.maxAcceleration) || 0,
        ) * pathSpeed,
      referenceVelocity: Math.max(0, Number(routeSettings.maxVelocity) || 0),
    };
  }

  function velocityToColor(ratio: number): string {
    const t = clampNumber(ratio, 0, 1);
    const blue = [37, 99, 235];
    const red = [239, 68, 68];
    const rgb = blue.map((channel, index) =>
      Math.round(channel + (red[index] - channel) * t),
    );
    return `rgb(${rgb[0]}, ${rgb[1]}, ${rgb[2]})`;
  }

  function createStandardPathElement(
    line: Line,
    sourceStartPoint: BasePoint,
  ): CanvasPathElement {
    let lineElem: CanvasPathElement;

    if (line.controlPoints.length > 2) {
      const cps = [sourceStartPoint, ...line.controlPoints, line.endPoint];
      let points = [
        new Two.Anchor(
          x(sourceStartPoint.x),
          y(sourceStartPoint.y),
          0,
          0,
          0,
          0,
          Two.Commands.move,
        ),
      ];
      for (let i = 1; i <= PATH_RENDER_SAMPLES; ++i) {
        const point = getCurvePoint(i / PATH_RENDER_SAMPLES, cps);
        points.push(
          new Two.Anchor(
            x(point.x),
            y(point.y),
            0,
            0,
            0,
            0,
            Two.Commands.line,
          ),
        );
      }
      points.forEach((point) => (point.relative = false));
      lineElem = new Two.Path(points);
      lineElem.automatic = false;
    } else if (line.controlPoints.length > 0) {
      let cp1 = line.controlPoints[1]
        ? line.controlPoints[0]
        : quadraticToCubic(sourceStartPoint, line.controlPoints[0], line.endPoint)
            .Q1;
      let cp2 =
        line.controlPoints[1] ??
        quadraticToCubic(sourceStartPoint, line.controlPoints[0], line.endPoint)
          .Q2;
      let points = [
        new Two.Anchor(
          x(sourceStartPoint.x),
          y(sourceStartPoint.y),
          x(sourceStartPoint.x),
          y(sourceStartPoint.y),
          x(cp1.x),
          y(cp1.y),
          Two.Commands.move,
        ),
        new Two.Anchor(
          x(line.endPoint.x),
          y(line.endPoint.y),
          x(cp2.x),
          y(cp2.y),
          x(line.endPoint.x),
          y(line.endPoint.y),
          Two.Commands.curve,
        ),
      ];
      points.forEach((point) => (point.relative = false));

      lineElem = new Two.Path(points);
      lineElem.automatic = false;
    } else {
      lineElem = new Two.Line(
        x(sourceStartPoint.x),
        y(sourceStartPoint.y),
        x(line.endPoint.x),
        y(line.endPoint.y),
      );
    }

    return lineElem;
  }

  function stylePathElement(
    element: CanvasPathElement,
    id: string,
    stroke: string,
    opacity: number,
    locked?: boolean,
  ) {
    element.id = id;
    element.stroke = stroke;
    element.linewidth = x(LINE_WIDTH);
    element.noFill();
    if (locked) {
      element.dashes = [x(2), x(2)];
      element.opacity = opacity * 0.7;
    } else {
      element.dashes = [];
      element.opacity = opacity;
    }
  }

  /**
   * Colours a path by the speed the robot is really doing there.
   *
   * Two things decide that, and neither is visible from the path on its own:
   * the profile belongs to the whole PathChain, so a path in the middle of one
   * is driven at cruise rather than ramping up and back down; and both cornering
   * and turning cap the speed where they happen. All of it comes from the
   * timeline's own speed curve, so the colours cannot disagree with the clock.
   */
  function createVelocityGradientPathElements(
    line: Line,
    sourceStartPoint: BasePoint,
    idBase: string,
    opacity: number,
    routeSettings: Settings,
    chain?: ChainProfile,
  ): CanvasPathElement[] {
    const samples = sampleLineCurve(sourceStartPoint, line);
    const pathLength = samples[samples.length - 1]?.distance || 0;
    const motion = getLineMotionValues(line, routeSettings);

    // Without a chain the path is its own move, which is what a route that has
    // not been timed yet looks like.
    const profileLength = chain?.length ?? pathLength;
    const profileOffset = chain?.offset ?? 0;
    const maxVelocity = chain?.maxVelocity ?? motion.maxVelocity;
    const maxAcceleration = chain?.maxAcceleration ?? motion.maxAcceleration;
    const maxDeceleration = chain?.maxDeceleration ?? motion.maxDeceleration;

    if (
      pathLength <= 0 ||
      profileLength <= 0 ||
      maxVelocity <= 0 ||
      maxAcceleration <= 0 ||
      maxDeceleration <= 0 ||
      motion.referenceVelocity <= 0
    ) {
      return [];
    }

    const elements: CanvasPathElement[] = [];

    for (let i = 1; i < samples.length; i++) {
      const previous = samples[i - 1];
      const current = samples[i];
      const midpointDistance = (previous.distance + current.distance) / 2;
      const velocity = chain?.profile
        ? profileVelocityAtDistance(chain.profile, profileOffset + midpointDistance)
        : calculateMotionProfileVelocityAtDistance(
            profileOffset + midpointDistance,
            profileLength,
            maxVelocity,
            maxAcceleration,
            maxDeceleration,
          );
      const color = velocityToColor(velocity / motion.referenceVelocity);
      const segment = new Two.Line(
        x(previous.point.x),
        y(previous.point.y),
        x(current.point.x),
        y(current.point.y),
      );
      stylePathElement(segment, `${idBase}-velocity-${i}`, color, opacity, line.locked);
      elements.push(segment);
    }

    return elements;
  }

  function createPathDrawElements(
    line: Line,
    sourceStartPoint: BasePoint,
    idBase: string,
    stroke: string,
    opacity: number,
    routeSettings: Settings,
    travel?: TravelState,
  ): CanvasPathElement[] {
    if (routeSettings.showVelocityGradient) {
      const gradientElements = createVelocityGradientPathElements(
        line,
        sourceStartPoint,
        idBase,
        opacity,
        routeSettings,
        travel?.chain,
      );
      if (gradientElements.length > 0) return gradientElements;
    }

    const lineElem = createStandardPathElement(line, sourceStartPoint);
    stylePathElement(lineElem, idBase, stroke, opacity, line.locked);
    return [lineElem];
  }

  function getLineLength(sourceStartPoint: BasePoint, line: Line): number {
    return calculateCurveLength(sourceStartPoint, line.controlPoints, line.endPoint);
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

  function calculateMotionProfileTimeAtDistance(
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

  function eventTimelineColor(eventName: string, index: number): string {
    const colors = ["#f97316", "#06b6d4", "#22c55e", "#e11d48", "#8b5cf6", "#f59e0b"];
    let hash = index;
    for (const char of eventName) {
      hash = (hash * 31 + char.charCodeAt(0)) % colors.length;
    }
    return colors[Math.abs(hash) % colors.length];
  }

  function buildPathAnnotations(
    idPrefix: string,
    sourceStartPoint: Point,
    sourceLines: Line[],
    timePrediction: TimePrediction | null | undefined,
    routeSettings: Settings,
    sourceSequence: SequenceItem[] = [],
    sourceVariables: Variable[] = [],
  ): PathOverlayItem[] {
    // Labels follow the route, so a path inside a repeat loop or an `if` block
    // is measured from where the robot actually reaches it.
    return buildTravelLineTimingMetas(
      sourceStartPoint,
      sourceLines,
      timePrediction,
      routeSettings,
      sourceSequence,
      sourceVariables,
    )
      .filter((meta) => meta.iteration === 0)
      .map((meta) => {
        const midpoint = getLinePointAtDistance(
          meta.startPoint,
          meta.line,
          meta.length / 2,
        );

        return {
          id: `${idPrefix}-path-label-${meta.lineIndex}`,
          x: x(midpoint.x),
          y: y(midpoint.y),
          label: `${meta.length.toFixed(1)} in / ${formatSeconds(meta.duration)}`,
          color: meta.line.color,
        };
      });
  }

  function buildEventPins(
    idPrefix: string,
    sourceStartPoint: Point,
    sourceLines: Line[],
    timePrediction: TimePrediction | null | undefined,
    routeSettings: Settings,
    sourceSequence: SequenceItem[] = [],
    sourceVariables: Variable[] = [],
  ): PathOverlayItem[] {
    const pins: PathOverlayItem[] = [];

    buildTravelLineTimingMetas(
      sourceStartPoint,
      sourceLines,
      timePrediction,
      routeSettings,
      sourceSequence,
      sourceVariables,
    ).forEach((meta) => {
      const line = meta.line;
      const lineIndex = meta.lineIndex;
      const lineStartPoint = meta.startPoint;
      if (meta.iteration !== 0 || !line?.eventMarkers?.length) return;

      const length = meta.length;
      const duration = meta.duration;
      const motion = getLineMotionValues(line, routeSettings);

      line.eventMarkers.forEach((marker, markerIndex) => {
        // Markers switched off by their own condition are skipped in generated
        // code, so they get no pin either.
        if (!isEnabled(marker, sourceVariables)) return;

        const triggerType =
          marker.triggerType === "temporal" || marker.triggerType === "pose"
            ? marker.triggerType
            : "parametric";
        const position = clampNumber(Number(marker.position ?? 0.5), 0, 1);
        let fieldPoint: BasePoint;

        if (
          triggerType === "pose" &&
          Number.isFinite(Number(marker.poseX)) &&
          Number.isFinite(Number(marker.poseY))
        ) {
          fieldPoint = {
            x: Number(marker.poseX),
            y: Number(marker.poseY),
          };
        } else if (
          triggerType === "temporal" &&
          duration > 0 &&
          motion.maxVelocity > 0 &&
          motion.maxAcceleration > 0 &&
          motion.maxDeceleration > 0
        ) {
          const triggerSeconds = Math.max(0, Number(marker.triggerMs ?? 0) || 0) / 1000;
          const triggerDistance = calculateMotionProfileDistanceAtTime(
            Math.min(triggerSeconds, duration),
            length,
            motion.maxVelocity,
            motion.maxAcceleration,
            motion.maxDeceleration,
          );
          fieldPoint = getLinePointAtDistance(lineStartPoint, line, triggerDistance);
        } else {
          fieldPoint = getLinePointAtT(lineStartPoint, line, position);
        }

        pins.push({
          id: `${idPrefix}-event-pin-${lineIndex}-${marker.id || markerIndex}`,
          x: x(fieldPoint.x),
          y: y(fieldPoint.y),
          label: marker.name || `Event ${markerIndex + 1}`,
          color: "#a855f7",
        });
      });
    });

    return pins;
  }

  function buildEventDurationSegmentElements(
    idBase: string,
    sourceStartPoint: BasePoint,
    line: Line,
    startDistance: number,
    endDistance: number,
    color: string,
  ): CanvasPathElement[] {
    const length = getLineLength(sourceStartPoint, line);
    const start = clampNumber(startDistance, 0, length);
    const end = clampNumber(Math.max(endDistance, start + 0.25), 0, length);
    const segmentDistance = end - start;
    const segmentCount = Math.max(1, Math.min(28, Math.ceil(segmentDistance / 3)));
    const elements: CanvasPathElement[] = [];

    for (let i = 0; i < segmentCount; i++) {
      const d1 = start + (segmentDistance * i) / segmentCount;
      const d2 = start + (segmentDistance * (i + 1)) / segmentCount;
      const p1 = getLinePointAtDistance(sourceStartPoint, line, d1);
      const p2 = getLinePointAtDistance(sourceStartPoint, line, d2);
      const lineElement = new Two.Line(x(p1.x), y(p1.y), x(p2.x), y(p2.y));
      lineElement.id = `${idBase}-duration-${i}`;
      lineElement.stroke = color;
      lineElement.linewidth = x(1.6);
      lineElement.opacity = 0.9;
      lineElement.noFill();
      (lineElement as any).cap = "round";
      elements.push(lineElement);
    }

    return elements;
  }

  function buildEventTimelineOverlays(
    idPrefix: string,
    sourceStartPoint: Point,
    sourceLines: Line[],
    timePrediction: TimePrediction | null | undefined,
    routeSettings: Settings,
    sourceSequence: SequenceItem[] = [],
    sourceVariables: Variable[] = [],
  ): EventTimelineOverlay {
    const elements: CanvasPathElement[] = [];
    const lineMetas = buildTravelLineTimingMetas(
      sourceStartPoint,
      sourceLines,
      timePrediction,
      routeSettings,
      sourceSequence,
      sourceVariables,
    ).map((meta) => ({
      ...meta,
      motion: {
        maxVelocity: meta.maxVelocity,
        maxAcceleration: meta.maxAcceleration,
        maxDeceleration: meta.maxDeceleration,
      },
    }));

    function distanceAtLocalTime(
      meta: NonNullable<(typeof lineMetas)[number]>,
      localTime: number,
    ) {
      const clampedTime = clampNumber(localTime, 0, meta.duration);
      if (meta.hasMotionProfile) {
        return calculateMotionProfileDistanceAtTime(
          clampedTime,
          meta.length,
          meta.motion.maxVelocity,
          meta.motion.maxAcceleration,
          meta.motion.maxDeceleration,
        );
      }
      return meta.length * (meta.duration > 0 ? clampedTime / meta.duration : 1);
    }

    lineMetas.forEach((meta) => {
      if (!meta || !meta.line.eventMarkers?.length) return;

      meta.line.eventMarkers.forEach((marker, markerIndex) => {
        if (!isEnabled(marker, sourceVariables)) return;

        const triggerType =
          marker.triggerType === "temporal" || marker.triggerType === "pose"
            ? marker.triggerType
            : "parametric";
        const position = clampNumber(Number(marker.position ?? 0.5), 0, 1);
        let fieldPoint: BasePoint;
        let triggerDistance = 0;
        let localTriggerTime = 0;

        if (
          triggerType === "pose" &&
          Number.isFinite(Number(marker.poseX)) &&
          Number.isFinite(Number(marker.poseY))
        ) {
          fieldPoint = {
            x: Number(marker.poseX),
            y: Number(marker.poseY),
          };
          triggerDistance = getNearestDistanceOnLine(
            meta.startPoint,
            meta.line,
            fieldPoint,
          );
          localTriggerTime = meta.hasMotionProfile
            ? calculateMotionProfileTimeAtDistance(
                triggerDistance,
                meta.length,
                meta.motion.maxVelocity,
                meta.motion.maxAcceleration,
                meta.motion.maxDeceleration,
              )
            : meta.duration * (meta.length > 0 ? triggerDistance / meta.length : 0);
        } else if (triggerType === "temporal") {
          localTriggerTime = clampNumber(
            Math.max(0, Number(marker.triggerMs ?? 0) || 0) / 1000,
            0,
            meta.duration,
          );
          triggerDistance = distanceAtLocalTime(meta, localTriggerTime);
          fieldPoint = getLinePointAtDistance(
            meta.startPoint,
            meta.line,
            triggerDistance,
          );
        } else {
          fieldPoint = getLinePointAtT(meta.startPoint, meta.line, position);
          triggerDistance = getLineDistanceAtT(meta.startPoint, meta.line, position);
          localTriggerTime = meta.hasMotionProfile
            ? calculateMotionProfileTimeAtDistance(
                triggerDistance,
                meta.length,
                meta.motion.maxVelocity,
                meta.motion.maxAcceleration,
                meta.motion.maxDeceleration,
              )
            : meta.duration * (meta.length > 0 ? triggerDistance / meta.length : position);
        }

        const eventDuration = Math.max(0, Number(marker.durationMs ?? 0) || 0) / 1000;
        const absoluteStartTime = meta.startTime + localTriggerTime;
        // A zero duration fires once — the generated code finishes the event
        // straight away — so it is drawn as a tick at the trigger point rather
        // than as a strip running to the end of the auto.
        const absoluteEndTime = absoluteStartTime + eventDuration;
        const eventName = marker.name || `Event ${markerIndex + 1}`;
        const color = eventTimelineColor(
          eventName,
          markerIndex + meta.lineIndex * 7,
        );

        lineMetas.forEach((spanMeta) => {
          if (!spanMeta || spanMeta.duration <= 0) return;
          const spanStartTime = Math.max(absoluteStartTime, spanMeta.startTime);
          const spanEndTime = Math.min(absoluteEndTime, spanMeta.endTime);
          if (spanEndTime < spanStartTime) return;

          elements.push(
            ...buildEventDurationSegmentElements(
              `${idPrefix}-event-timeline-exec-${meta.executionIndex}-${marker.id || markerIndex}-span-${spanMeta.executionIndex}`,
              spanMeta.startPoint,
              spanMeta.line,
              distanceAtLocalTime(spanMeta, spanStartTime - spanMeta.startTime),
              distanceAtLocalTime(spanMeta, spanEndTime - spanMeta.startTime),
              color,
            ),
          );
        });

      });
    });

    return { elements, labels: [] };
  }

  function tangentCssAngleForLine(
    sourceStartPoint: BasePoint,
    line: Line,
    position: number,
  ): number {
    const t = clampNumber(position, 0, 1);
    const beforeT = clampNumber(t - 0.01, 0, 1);
    const afterT = clampNumber(t + 0.01, 0, 1);
    const curvePoints = [sourceStartPoint, ...line.controlPoints, line.endPoint];
    const before = getCurvePoint(beforeT, curvePoints);
    const after = getCurvePoint(afterT, curvePoints);
    const dx = after.x - before.x;
    const dy = after.y - before.y;

    if (Math.abs(dx) < 1e-9 && Math.abs(dy) < 1e-9) {
      return 0;
    }

    return (Math.atan2(-dy, dx) * 180) / Math.PI;
  }

  function getTravelProgressForSwerve(
    playbackPercent: number,
    sourceStartPoint: Point,
    sourceLines: Line[],
    timePrediction: TimePrediction | null | undefined,
    routeSettings: Settings,
  ): { lineIndex: number; t: number; angle: number } | null {
    if (!timePrediction?.timeline?.length || timePrediction.totalTime <= 0) {
      return null;
    }

    const currentTime = (clampNumber(playbackPercent, 0, 100) / 100) * timePrediction.totalTime;
    const activeEvent =
      timePrediction.timeline.find(
        (event) => currentTime >= event.startTime && currentTime <= event.endTime,
      ) || timePrediction.timeline[timePrediction.timeline.length - 1];

    if (activeEvent.type !== "travel" || activeEvent.lineIndex === undefined) {
      return null;
    }

    const lineIndex = activeEvent.lineIndex;
    const line = sourceLines[lineIndex];
    // Follow the route, not the array: a repeated or branched path does not
    // start where the previous entry in `lines` ends.
    const lineStart =
      activeEvent.startPoint ??
      getLineStartPoint(sourceStartPoint, sourceLines, lineIndex);

    if (!line || !lineStart || activeEvent.duration <= 0) {
      return null;
    }

    const timeIntoEvent = clampNumber(
      currentTime - activeEvent.startTime,
      0,
      activeEvent.duration,
    );
    const lineLength = getLineLength(lineStart, line);
    let lineT = 0;

    const motion = getLineMotionValues(line, routeSettings);
    if (
      lineLength > 0 &&
      motion.maxVelocity > 0 &&
      motion.maxAcceleration > 0 &&
      motion.maxDeceleration > 0
    ) {
      const distance = calculateMotionProfileDistanceAtTime(
        timeIntoEvent,
        lineLength,
        motion.maxVelocity,
        motion.maxAcceleration,
        motion.maxDeceleration,
      );
      lineT = clampNumber(distance / lineLength, 0, 1);
    } else {
      lineT = easeInOutQuad(clampNumber(timeIntoEvent / activeEvent.duration, 0, 1));
    }

    return {
      lineIndex,
      t: lineT,
      angle: tangentCssAngleForLine(lineStart, line, lineT),
    };
  }

  function getSwerveWheelAnglesCss(
    playbackPercent: number,
    sourceStartPoint: Point,
    sourceLines: Line[],
    timePrediction: TimePrediction | null | undefined,
    routeSettings: Settings,
    robotHeadingCss: number,
  ): number[] {
    const travelState = getTravelProgressForSwerve(
      playbackPercent,
      sourceStartPoint,
      sourceLines,
      timePrediction,
      routeSettings,
    );

    if (travelState) {
      return [travelState.angle, travelState.angle, travelState.angle, travelState.angle];
    }

    return SWERVE_X_LOCK_LOCAL_CSS_ANGLES.map(
      (angle) => robotHeadingCss + angle,
    );
  }

  function buildSwerveWheelModules(
    idPrefix: string,
    center: BasePoint,
    robotHeadingCss: number,
    wheelAnglesCss: number[],
    opacity = 1,
    color = "#06b6d4",
    scale = 1,
  ): SwerveWheelOverlay[] {
    const robotPixelWidth = Math.max(1, x(robotWidth));
    const robotPixelHeight = Math.max(1, x(robotHeight));
    const halfForward = robotPixelWidth * 0.34 * scale;
    const halfSide = robotPixelHeight * 0.34 * scale;
    const wheelLength = clampNumber(
      Math.min(robotPixelWidth, robotPixelHeight) * 0.28 * scale,
      6,
      24,
    );
    const wheelThickness = Math.max(3, wheelLength * 0.36);
    const headingRad = (robotHeadingCss * Math.PI) / 180;
    const cos = Math.cos(headingRad);
    const sin = Math.sin(headingRad);
    const localOffsets = [
      { x: halfForward, y: -halfSide },
      { x: halfForward, y: halfSide },
      { x: -halfForward, y: -halfSide },
      { x: -halfForward, y: halfSide },
    ];

    return localOffsets.map((offset, index) => ({
      id: `${idPrefix}-swerve-module-${index}`,
      x: center.x + offset.x * cos - offset.y * sin,
      y: center.y + offset.x * sin + offset.y * cos,
      angle: wheelAnglesCss[index] ?? robotHeadingCss,
      length: wheelLength,
      thickness: wheelThickness,
      opacity,
      color,
    }));
  }

  /**
   * `playbackPercent` is passed in rather than read from the component, because
   * Svelte works out what a reactive block depends on from the names that
   * appear in it. Reading `percent` inside this helper left the blocks that draw
   * the onion bodies with no dependency on it, so with "next point only" on they
   * froze on whichever path was current when something else last changed — while
   * the swerve overlay, whose block does mention the playback position, kept
   * following the robot. The two drifted apart on screen.
   */
  /**
   * Where the onion bodies sit. Depends on the route and the robot, and on
   * nothing about playback.
   *
   * Kept apart from the filter below because it is the expensive half — it walks
   * every path twice and builds a body for each interval — and it was being
   * redone on every animation frame for a result that had not changed. At a 1in
   * spacing that alone blocked the main thread for 37ms a frame, which is 27fps
   * before anything else on the page gets a turn.
   */
  function buildOnionLayers(
    sourceStartPoint: Point,
    sourceLines: Line[],
    routeSettings: Settings,
    sourceStartPoints?: Map<string, BasePoint>,
    sourceHeadingTransitions?: Map<string, { entryHeading: number; catchUp: number }>,
  ): OnionLayerState[] {
    return generateOnionLayers(
      sourceStartPoint,
      sourceLines,
      routeSettings.rWidth,
      routeSettings.rHeight,
      routeSettings.onionLayerSpacing || 6,
      sourceStartPoints,
      sourceHeadingTransitions,
      // The bodies are for judging clearance by eye, so they draw the robot's
      // real shape when CAD has given us one.
      footprintFromSettings(routeSettings),
    ) as OnionLayerState[];
  }

  /**
   * Which path `Next point only` should show, from where playback has got to.
   * This is the half that does depend on the clock, and it is a scan of the
   * timeline rather than any geometry.
   */
  function selectedOnionLineIndex(
    timePrediction: TimePrediction | null | undefined,
    routeSettings: Settings,
    playbackPercent: number,
  ): number | null {
    if (!routeSettings.onionNextPointOnly || !timePrediction?.timeline?.length) {
      return null;
    }

    const currentTime = (timePrediction.totalTime || 0) * (playbackPercent / 100);
    const travelEvents = timePrediction.timeline.filter(
      (event) => event.type === "travel",
    );

    const currentTravel = travelEvents.find(
      (event) => event.startTime <= currentTime && event.endTime >= currentTime,
    );
    if (currentTravel) return currentTravel.lineIndex as number;

    const nextTravel = travelEvents.find((event) => event.startTime > currentTime);
    if (nextTravel) return nextTravel.lineIndex as number;

    return travelEvents.length
      ? (travelEvents[travelEvents.length - 1].lineIndex as number)
      : null;
  }

  function filterOnionLayers(
    layers: OnionLayerState[],
    selectedLineIndex: number | null,
  ): OnionLayerState[] {
    if (selectedLineIndex === null) return layers;
    return layers.filter((layer) => layer.lineIndex === selectedLineIndex);
  }

  function buildOnionSwerveModules(
    idPrefix: string,
    sourceStartPoint: Point,
    sourceLines: Line[],
    layers: OnionLayerState[],
    opacity: number,
    color: string,
    sourceStartPoints?: Map<string, BasePoint>,
  ): SwerveWheelOverlay[] {
    return layers.flatMap((layer, layerIndex) => {
      const line = sourceLines[layer.lineIndex];
      const lineStart =
        (line?.id ? sourceStartPoints?.get(line.id) : undefined) ??
        getLineStartPoint(sourceStartPoint, sourceLines, layer.lineIndex);
      if (!line || !lineStart) return [];

      const wheelAngle = tangentCssAngleForLine(lineStart, line, layer.t ?? 0.5);
      return buildSwerveWheelModules(
        `${idPrefix}-onion-${layerIndex}`,
        { x: x(layer.x), y: y(layer.y) },
        layer.heading,
        [wheelAngle, wheelAngle, wheelAngle, wheelAngle],
        opacity,
        color,
        0.65,
      );
    });
  }

  const history = createHistory();
  const { canUndoStore, canRedoStore } = history;
  const OPTIMIZER_BASE_URL = "https://fpa.pedropathing.com";

  function getAppState(): AppState {
    return {
      startPoint,
      lines,
      shapes,
      sequence,
      settings,
      pathChains,
      variables,
    };
  }

  // Use the stores for reactivity
  $: canUndo = $canUndoStore;
  $: canRedo = $canRedoStore;

  function recordChange() {
    history.record(getAppState());
  }

  function undoAction() {
    const prev = history.undo();
    if (prev) {
      startPoint = prev.startPoint;
      lines = prev.lines;
      shapes = prev.shapes;
      sequence = prev.sequence;
      settings = prev.settings;
      pathChains = prev.pathChains;
      variables = prev.variables || [];
      isUnsaved.set(true);
      two && two.update();
    }

    // undoAction completes; no file-picker behavior here
  }

  function redoAction() {
    const next = history.redo();
    if (next) {
      startPoint = next.startPoint;
      lines = next.lines;
      shapes = next.shapes;
      sequence = next.sequence;
      settings = next.settings;
      pathChains = next.pathChains;
      variables = next.variables || [];
      isUnsaved.set(true);
      two && two.update();
    }
  }


  $: {
    const normalized = normalizePathChains(pathChains, lines);
    const current = JSON.stringify(pathChains);
    const next = JSON.stringify(normalized);
    if (current !== next) {
      pathChains = normalized;
    }
  }

  $: {
    // Ensure arrays are reactive when items are added/removed
    lines = lines;
    shapes = shapes;
  }

  // Two.js groups
  let lineGroup = new Two.Group();
  lineGroup.id = "line-group";
  let pointGroup = new Two.Group();
  pointGroup.id = "point-group";
  let shapeGroup = new Two.Group();
  shapeGroup.id = "shape-group";
  // Coordinate converters
  let x: d3.ScaleLinear<number, number, number>;
  let y: d3.ScaleLinear<number, number, number>;

  // Animation controller
  let loopAnimation = true;
  let animationController: ReturnType<typeof createAnimationController>;
  $: timePrediction = calculatePathTime(startPoint, routeLines, settings, routeSequence, variables);
  $: animationDuration = getAnimationDuration(timePrediction.totalTime / 1000);
  
  // Second path timeline (for dual path mode)
  $: secondTimePrediction = $dualPathMode && secondStartPoint && secondLines.length > 0 
    ? calculatePathTime(secondStartPoint, secondLines, settings, secondSequence, variables)
    : null;
  
  // Calculate max duration across all paths for playbar scaling
  $: effectiveAnimationDuration = (() => {
    // In multi-path mode, only use additional paths for duration
    if ($activePaths.length > 0) {
      let maxTime = 0;
      additionalPaths.forEach((pathData) => {
        if (pathData.startPoint && pathData.lines.length > 0) {
          const pathTime = calculatePathTime(
            pathData.startPoint,
            pathData.lines,
            pathData.settings,
            pathData.sequence,
            pathData.variables,
          );
          if (pathTime) {
            maxTime = Math.max(maxTime, pathTime.totalTime);
          }
        }
      });
      return maxTime > 0 ? getAnimationDuration(maxTime / 1000) : animationDuration;
    }
    
    // In normal/dual mode, check main path and second path
    let maxTime = timePrediction.totalTime;
    
    if ($dualPathMode && secondTimePrediction) {
      maxTime = Math.max(maxTime, secondTimePrediction.totalTime);
    }
    
    return getAnimationDuration(maxTime / 1000);
  })();
  
  // Load additional paths when activePaths changes
  $: {
    loadAdditionalPaths($activePaths);
  }

  async function loadAdditionalPaths(paths: string[]) {
    const newAdditionalPaths: AdditionalPathData[] = [];
    
    // Multi-path mode is isolated - turn off old dual path mode
    if (paths.length > 0) {
      dualPathMode.set(false);
      secondStartPoint = null;
      secondLines = [];
      secondShapes = [];
      secondSequence = [];
      secondFilePath.set(null);
    }
    
    const colors = ['#FF6B6B', '#4ECDC4', '#45B7D1', '#FFA07A']; // Red, Teal, Blue, Salmon

    for (let i = 0; i < Math.min(paths.length, 4); i++) {
      const filePath = paths[i];
      try {
        const content = await browserFileStore.readFile(filePath);
          const data = JSON.parse(content);

        if (data.startPoint && data.lines) {
          const loadedVariables = normalizeVariables(data);
          const normalizedLines = applyVariablesToLines(
            normalizeLines(data.lines || []),
            loadedVariables,
          );
          const normalizedPath = applyPoseVariablesToPath(
            resolvePointExpressions(data.startPoint, loadedVariables),
            normalizedLines,
            poseVariablesOf(loadedVariables),
          );
          const normalizedSequence = applyVariablesToSequence(
            data.sequence || normalizedPath.lines.map((ln: Line) => ({
              kind: "path",
              lineId: ln.id!,
            })),
            loadedVariables,
          );
          newAdditionalPaths.push({
            filePath,
            startPoint: normalizedPath.startPoint,
            lines: normalizedPath.lines,
            shapes: data.shapes || [],
            sequence: normalizedSequence,
            settings: data.settings || { ...DEFAULT_SETTINGS },
            pathChains: normalizePathChains(data.pathChains, normalizedPath.lines),
            variables: loadedVariables,
            color: colors[i],
          });
        }
      } catch (error) {
        console.error(`Failed to load additional path ${filePath}:`, error);
      }
    }

    additionalPaths = newAdditionalPaths;
  }
  
  let secondRobotXY: BasePoint = { x: 0, y: 0 };
  let secondRobotHeading: number = 0;
  /**
   * Converter for X axis from inches to pixels.
   */
  $: x = d3
    .scaleLinear()
    .domain([0, FIELD_SIZE])
    .range([0, width || FIELD_SIZE]);
  /**
   * Converter for Y axis from inches to pixels.
   */
  $: y = d3
    .scaleLinear()
    .domain([0, FIELD_SIZE])
    .range([height || FIELD_SIZE, 0]);
  // The main robot state is computed in a single place further down, which also
  // handles multi-path mode. A second copy here would race with it.

  $: points = (() => {
    let _points = [];
    
    // Only show main path points when NOT in multi-path mode
    if ($activePaths.length === 0) {
      let startPointElem = new Two.Circle(
        x(startPoint.x),
        y(startPoint.y),
        x(POINT_RADIUS),
      );
      startPointElem.id = `point-0-0`;
      // A route can legitimately have no paths (every path deleted, or a file
      // that stores none). Throwing here would break every later reactive
      // statement and freeze the whole editor, so fall back to a plain colour.
      startPointElem.fill = lines[0]?.color || "#ffc516";
      startPointElem.noStroke();

      _points.push(startPointElem);

      lines.forEach((line, idx) => {
        if (!line || !line.endPoint) return; // Skip invalid lines or lines without endPoint
        [line.endPoint, ...line.controlPoints].forEach((point, idx1) => {
          if (idx1 > 0) {
            let pointGroup = new Two.Group();
            pointGroup.id = `point-${idx + 1}-${idx1}`;

            let pointElem = new Two.Circle(
              x(point.x),
              y(point.y),
              x(POINT_RADIUS),
            );
            pointElem.id = `point-${idx + 1}-${idx1}-background`;
            pointElem.fill = line.color;
            pointElem.noStroke();

            let pointText = new Two.Text(
              `${idx1}`,
              x(point.x),
              y(point.y - 0.15),
            );
            pointText.id = `point-${idx + 1}-${idx1}-text`;
            pointText.size = x(1.55);
            pointText.leading = 1;
            pointText.family = "ui-sans-serif, system-ui, sans-serif";
            pointText.alignment = "center";
            pointText.baseline = "middle";
            pointText.fill = "white";
            pointText.noStroke();

            pointGroup.add(pointElem, pointText);
            _points.push(pointGroup);
          } else {
            let pointElem = new Two.Circle(
              x(point.x),
              y(point.y),
              x(POINT_RADIUS),
            );
            pointElem.id = `point-${idx + 1}-${idx1}`;
            pointElem.fill = line.color;
            pointElem.noStroke();
            _points.push(pointElem);
          }
        });
      });
    }
    
    // Add obstacle vertices as draggable points
    shapes.forEach((shape, shapeIdx) => {
      shape.vertices.forEach((vertex, vertexIdx) => {
        let pointGroup = new Two.Group();
        pointGroup.id = `obstacle-${shapeIdx}-${vertexIdx}`;

        let pointElem = new Two.Circle(
          x(vertex.x),
          y(vertex.y),
          x(POINT_RADIUS),
        );
        pointElem.id = `obstacle-${shapeIdx}-${vertexIdx}-background`;
        pointElem.fill = shape.fillColor; // Match obstacle fill color
        pointElem.noStroke();

        let pointText = new Two.Text(
          `${vertexIdx + 1}`,
          x(vertex.x),
          y(vertex.y - 0.15),
        );
        pointText.id = `obstacle-${shapeIdx}-${vertexIdx}-text`;
        pointText.size = x(1.55);
        pointText.leading = 1;
        pointText.family = "ui-sans-serif, system-ui, sans-serif";
        pointText.alignment = "center";
        pointText.baseline = "middle";
        pointText.fill = "white";
        pointText.noStroke();
        pointGroup.add(pointElem, pointText);
        _points.push(pointGroup);
      });
    });

    // Add second path points (for dual path mode) - not in multi-path mode
    if ($activePaths.length === 0 && $dualPathMode && secondStartPoint && secondLines.length > 0) {
      let secondStartPointElem = new Two.Circle(
        x(secondStartPoint.x),
        y(secondStartPoint.y),
        x(POINT_RADIUS),
      );
      secondStartPointElem.id = `second-point-0-0`;
      secondStartPointElem.fill = secondLines[0]?.color || "#888";
      secondStartPointElem.noStroke();
      _points.push(secondStartPointElem);

      secondLines.forEach((line, idx) => {
        if (!line || !line.endPoint) return;
        [line.endPoint, ...line.controlPoints].forEach((point, idx1) => {
          if (idx1 > 0) {
            let pointGroup = new Two.Group();
            pointGroup.id = `second-point-${idx + 1}-${idx1}`;

            let pointElem = new Two.Circle(
              x(point.x),
              y(point.y),
              x(POINT_RADIUS),
            );
            pointElem.id = `second-point-${idx + 1}-${idx1}-background`;
            pointElem.fill = line.color;
            pointElem.noStroke();

            let pointText = new Two.Text(
              `${idx1}`,
              x(point.x),
              y(point.y - 0.15),
            );
            pointText.id = `second-point-${idx + 1}-${idx1}-text`;
            pointText.size = x(1.55);
            pointText.leading = 1;
            pointText.family = "ui-sans-serif, system-ui, sans-serif";
            pointText.alignment = "center";
            pointText.baseline = "middle";
            pointText.fill = "white";
            pointText.noStroke();

            pointGroup.add(pointElem, pointText);
            _points.push(pointGroup);
          } else {
            let pointElem = new Two.Circle(
              x(point.x),
              y(point.y),
              x(POINT_RADIUS),
            );
            pointElem.id = `second-point-${idx + 1}-${idx1}`;
            pointElem.fill = line.color;
            pointElem.noStroke();
            _points.push(pointElem);
          }
        });
      });
    }

    // Add all control points for additional paths (full editing support)
    if ($activePaths.length > 0) {
      additionalPaths.forEach((pathData, pathIdx) => {
        if (!pathData.startPoint || !pathData.lines.length) return;
        
        // Add starting point
        let startPointElem = new Two.Circle(
          x(pathData.startPoint.x),
          y(pathData.startPoint.y),
          x(POINT_RADIUS * 0.9),
        );
        startPointElem.id = `additional-path-${pathIdx}-point-0-0`;
        startPointElem.fill = pathData.color || pathData.lines[0]?.color || "#888";
        startPointElem.noStroke();
        startPointElem.opacity = 0.8;
        _points.push(startPointElem);
        
        // Add all line points and control points
        pathData.lines.forEach((line, lineIdx) => {
          if (!line || !line.endPoint) return;
          
          [line.endPoint, ...line.controlPoints].forEach((point, pointIdx) => {
            if (pointIdx > 0) {
              // Control point with number
              let pointGroup = new Two.Group();
              pointGroup.id = `additional-path-${pathIdx}-point-${lineIdx + 1}-${pointIdx}`;

              let pointElem = new Two.Circle(
                x(point.x),
                y(point.y),
                x(POINT_RADIUS * 0.9),
              );
              pointElem.id = `additional-path-${pathIdx}-point-${lineIdx + 1}-${pointIdx}-background`;
              pointElem.fill = pathData.color || line.color;
              pointElem.noStroke();

              let pointText = new Two.Text(
                `${pointIdx}`,
                x(point.x),
                y(point.y - 0.15),
              );
              pointText.id = `additional-path-${pathIdx}-point-${lineIdx + 1}-${pointIdx}-text`;
              pointText.size = x(1.4);
              pointText.leading = 1;
              pointText.family = "ui-sans-serif, system-ui, sans-serif";
              pointText.alignment = "center";
              pointText.baseline = "middle";
              pointText.fill = "white";
              pointText.noStroke();

              pointGroup.add(pointElem, pointText);
              pointGroup.opacity = 0.8;
              _points.push(pointGroup);
            } else {
              // End point without number
              let pointElem = new Two.Circle(
                x(point.x),
                y(point.y),
                x(POINT_RADIUS * 0.9),
              );
              pointElem.id = `additional-path-${pathIdx}-point-${lineIdx + 1}-${pointIdx}`;
              pointElem.fill = pathData.color || line.color;
              pointElem.noStroke();
              pointElem.opacity = 0.8;
              _points.push(pointElem);
            }
          });
        });
      });
    }

    return _points;
  })();

  /**
   * Every path must appear in the route exactly once. Without this, a save file
   * whose step list has drifted from its paths (an older build, a hand-edited
   * file, a duplicate that renumbered ids) leaves paths that are missing from
   * the route: they sit in the editor but are not drawn, not timed and not
   * exported. Reconciling here keeps the field and the step list describing the
   * same thing no matter what was loaded.
   */
  $: {
    const reconciled = reconcileSequence(lines, sequence);
    if (reconciled !== sequence) sequence = reconciled;
  }

  // Paths individually switched off by their own "Enabled if" condition.
  $: inactiveLineIds = skippedLineIds(lines, variables);

  // Start points follow the sequence, so branches of an if share a start.
  // Camera pick and Turn first, applied: what the robot drives. Editors keep `lines`;
  // everything that times, draws, animates or checks the route reads these.
  $: routeEdits = expandRouteEdits(startPoint, lines, sequence, settings);
  $: routeLines = routeEdits.lines;
  $: routeSequence = routeEdits.sequence;
  $: cameraPicks = routeEdits.previews;

  $: mainRoute = buildRoute(startPoint, routeLines, routeSequence, variables);
  $: lineStartPoints = mainRoute.startPoints;

  /**
   * Consecutive paths are followed as one PathChain, so the robot only comes to
   * a stop where a chain ends. Those stops are what cost time, so the field
   * marks them.
   */
  $: chainRuns = buildChainRuns(mainRoute.steps);

  /**
   * How each path picks up its heading goal. Where two paths meet the goal can
   * jump — a tangential path after a linear one starts wherever its curve points
   * — and the robot answers by rotating on the move. The previews read the same
   * transition the animated robot does so nothing ever snaps.
   */
  /**
   * The chain each path belongs to and how much turning slows it, taken from
   * the timeline so the field cannot disagree with the estimate.
   */
  const travelStatesFrom = (
    prediction: TimePrediction | null | undefined,
    sourceLines: Line[],
  ) => {
    const states = new Map<string, TravelState>();
    prediction?.timeline?.forEach((event) => {
      if (event.type !== "travel" || event.lineIndex === undefined) return;
      const lineId = sourceLines[event.lineIndex]?.id;
      if (!lineId || states.has(lineId)) return;
      states.set(lineId, { chain: event.chain });
    });
    return states;
  };

  const headingTransitionsFrom = (
    prediction: TimePrediction | null | undefined,
    sourceLines: Line[],
  ) => {
    const transitions = new Map<string, { entryHeading: number; catchUp: number }>();
    prediction?.timeline?.forEach((event) => {
      if (event.type !== "travel" || event.lineIndex === undefined) return;
      const lineId = sourceLines[event.lineIndex]?.id;
      if (!lineId || transitions.has(lineId)) return;
      transitions.set(lineId, {
        entryHeading: Number(event.startHeading) || 0,
        catchUp: Number(event.headingCatchUp) || 0,
      });
    });
    return transitions;
  };

  $: travelStates = travelStatesFrom(timePrediction, routeLines);
  $: secondTravelStates = travelStatesFrom(secondTimePrediction, secondLines);
  $: headingTransitions = headingTransitionsFrom(timePrediction, routeLines);

  /**
   * The shape that actually has to fit through the gaps. A path is a curve
   * through the middle of the robot, and the middle of the robot fits places the
   * robot does not.
   */
  $: robotFootprint = footprintFromSettings(settings);

  /**
   * Onion bodies, split so the animation does not pay for them.
   *
   * The geometry depends on the route and the robot; only which slice of it is
   * shown depends on the clock. Recomputing the geometry every frame is what
   * made playback stutter once the spacing got fine.
   */
  $: onionLayersEnabled =
    $activePaths.length === 0 && settings.showOnionLayers && lines.length > 0;
  $: allOnionLayers = onionLayersEnabled
    ? buildOnionLayers(startPoint, routeLines, settings, lineStartPoints, headingTransitions)
    : [];
  $: onionSelectedLineIndex = selectedOnionLineIndex(timePrediction, settings, percent);
  $: visibleOnionLayers = filterOnionLayers(allOnionLayers, onionSelectedLineIndex);

  $: secondOnionLayersEnabled =
    $activePaths.length === 0 &&
    $dualPathMode &&
    settings.showOnionLayers &&
    secondLines.length > 0 &&
    Boolean(secondStartPoint);
  $: allSecondOnionLayers =
    secondOnionLayersEnabled && secondStartPoint
      ? buildOnionLayers(
          secondStartPoint,
          secondLines,
          settings,
          secondLineStartPoints,
          secondHeadingTransitions,
        )
      : [];
  $: secondOnionSelectedLineIndex = selectedOnionLineIndex(
    secondTimePrediction,
    settings,
    percent,
  );
  $: visibleSecondOnionLayers = filterOnionLayers(
    allSecondOnionLayers,
    secondOnionSelectedLineIndex,
  );

  /**
   * Where the robot is closer to an obstacle or a wall than the safety margin
   * allows, checked at the heading the robot is really holding — a path that is
   * clear driving straight can clip a corner once the robot turns on it.
   */
  /**
   * Poses the robot holds without driving. The heading model turns the robot
   * while it drives rather than in place, so a Wait holds whatever pose the
   * previous path left — but that pose belongs to the Wait, and a Wait can sit
   * where no path sample lands. A wait inside a repeat loop runs many times at
   * the same pose, so they are deduplicated by sequence item.
   */
  $: stationaryPoses = (() => {
    const seen = new Set<string>();
    const poses: StationaryPose[] = [];
    for (const event of timePrediction?.timeline || []) {
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
  })();

  /**
   * Every Pollen Pickup the route runs, planned from the pose the robot is in
   * when it begins. The timeline's first maneuver for a step carries that pose -
   * the same numbers the time estimate charged - so the overlay, the row, the
   * collision check and the export all describe one plan.
   */
  /**
   * The effect assigned to whatever the robot is doing right now (States tab).
   * Worked out from the playback position, so scrubbing shows the exact frame.
   */
  $: stateWindows = stateWindowsOf(
    timePrediction?.timeline || [],
    buildEventTimingWindows(startPoint, routeLines, timePrediction, settings, routeSequence, variables),
  );
  $: playbackSeconds = ((timePrediction?.totalTime || 0) * percent) / 100;

  /** Mechanism states at the playhead, drawn on the robot (utils/mechanismStates.ts - TeamCode's rules). */
  $: mechanismTimeline = buildMechanismTimeline(
    startPoint,
    routeLines,
    routeSequence,
    timePrediction,
    settings,
    variables,
  );
  $: mechanismState = mechanismStateAt(mechanismTimeline, playbackSeconds);
  $: mechanismAlliance = allianceOf(startPoint, lines);
  $: mechanismAim = (() => {
    if (!x || !y || !mechanismState.shooting) return { turret: null, aim: null };
    const fieldX = x.invert(robotXY.x);
    const fieldY = y.invert(robotXY.y);
    const headingRad = (-robotHeading * Math.PI) / 180;
    const tx = fieldX - TURRET_BEHIND_INCHES * Math.cos(headingRad);
    const ty = fieldY - TURRET_BEHIND_INCHES * Math.sin(headingRad);
    const aim = hiveAimPoint(mechanismAlliance, { x: fieldX, y: fieldY });
    return { turret: { x: x(tx), y: y(ty) }, aim: { x: x(aim.x), y: y(aim.y) } };
  })();
  $: activeVfx = vfxAt(stateWindows, playbackSeconds, settings.stateVfx);
  $: robotSpriteMotion = spriteMotion(activeVfx, x ? x(robotWidth) : 0);

  /**
   * A Shoot effect aims at the goal nearest the robot right now - on BIOBUZZ,
   * the closest of the four HIVE CELLS - or straight ahead on a field map with
   * no goals listed.
   */
  $: scoringPoints = scoringPointsFor(settings.fieldMap);
  $: robotShot = shotAt(activeVfx, robotXY, robotHeading, scoringPoints, x, y);
  $: shotRecoil = robotShot
    ? {
        x: -Math.cos((robotShot.frame.angle * Math.PI) / 180) * robotShot.frame.recoil * robotShot.pxPerInch * 1.2,
        y: -Math.sin((robotShot.frame.angle * Math.PI) / 180) * robotShot.frame.recoil * robotShot.pxPerInch * 1.2,
      }
    : { x: 0, y: 0 };

  function shotAt(
    vfx: ActiveVfx | null,
    from: { x: number; y: number },
    headingDeg: number,
    goals: ScoringPoint[],
    xScale: typeof x,
    yScale: typeof y,
  ) {
    if (!vfx || vfx.kind !== "shoot" || !xScale || !yScale || !from) return null;
    const pxPerInch = Math.abs(xScale(1) - xScale(0));
    const goal = nearestPoint(goals, xScale.invert(from.x), yScale.invert(from.y));
    const to = goal ? { x: xScale(goal.x), y: yScale(goal.y) } : aheadOf(from, headingDeg, pxPerInch * 48);
    return {
      goal,
      pxPerInch,
      color: goal?.alliance === "red" ? "#ef4444" : goal?.alliance === "blue" ? "#3b82f6" : "#facc15",
      frame: shotFrame(vfx, from, to, pxPerInch),
    };
  }

  $: pollenPlans = planPollenSteps(
    timePrediction?.timeline || [],
    sequence,
    mainRoute.steps,
    startPoint,
    settings,
  );

  /** The legs those plans drive, for the collision check. */
  $: pollenLegs = pollenLegsOf(pollenPlans, sequence) as ManeuverLeg[];

  $: clearanceReport = checkClearance(
    {
      startPoint,
      lines: routeLines,
      footprint: robotFootprint,
      obstacles: shapes,
      lineStartPoints,
      headingTransitions,
      stationaryPoses,
      maneuverLegs: pollenLegs,
    },
    {
      fieldSize: FIELD_SIZE,
      margin: settings.safetyMargin,
      midline: midlineRuleFromSettings(settings),
    },
  );
  $: secondHeadingTransitions = headingTransitionsFrom(
    secondTimePrediction,
    secondLines,
  );
  $: additionalTravelStates = additionalPaths.map((pathData) =>
    travelStatesFrom(
      calculatePathTime(
        pathData.startPoint || startPoint,
        pathData.lines,
        pathData.settings,
        pathData.sequence,
        pathData.variables,
      ),
      pathData.lines,
    ),
  );
  $: additionalHeadingTransitions = additionalPaths.map((pathData) =>
    headingTransitionsFrom(
      calculatePathTime(
        pathData.startPoint || startPoint,
        pathData.lines,
        pathData.settings,
        pathData.sequence,
        pathData.variables,
      ),
      pathData.lines,
    ),
  );
  $: stopPoints = (() => {
    if (!settings.showStopPoints) return [];

    const seen = new Set<string>();
    return chainRuns
      .map((run, runIndex) => {
        const last = run.steps[run.steps.length - 1];
        if (!last?.line?.endPoint) return null;
        // A repeat loop drives the same endpoint on every pass; one marker is
        // enough.
        const key = `${last.lineId}-${run.breakReason}`;
        if (seen.has(key)) return null;
        seen.add(key);

        return {
          id: `stop-${runIndex}-${last.lineId}`,
          x: x(last.line.endPoint.x),
          y: y(last.line.endPoint.y),
          label: CHAIN_BREAK_LABELS[run.breakReason],
        };
      })
      .filter(Boolean) as { id: string; x: number; y: number; label: string }[];
  })();
  $: secondLineStartPoints = secondStartPoint
    ? buildLineStartPoints(secondStartPoint, secondLines, secondSequence, variables)
    : new Map<string, Point>();
  $: additionalLineStartPoints = additionalPaths.map((pathData) =>
    pathData.startPoint
      ? buildLineStartPoints(
          pathData.startPoint,
          pathData.lines,
          pathData.sequence,
          pathData.variables,
        )
      : new Map<string, Point>(),
  );

  $: path = (() => {
    // Hide main path when in multi-path mode (isolated visualization)
    if ($activePaths.length > 0) {
      return [];
    }
    
    let _path: CanvasPathElement[] = [];

    routeLines.forEach((line, idx) => {
      if (!line || !line.endPoint) return; // Skip invalid lines or lines without endPoint
      // No start point means the route does not reach this path, so it is not
      // drawn — the field always matches the step list.
      const _startPoint = lineStartPoints.get(line.id || "");
      if (!_startPoint) return;

      // A path switched off by its own "Enabled if" condition stays visible
      // but faded. Paths inside an if block are never faded — the block header
      // reports whether it runs.
      const baseOpacity = settings.pathOpacity || 1.0;
      const opacity = inactiveLineIds.has(line.id || "")
        ? baseOpacity * 0.25
        : baseOpacity;

      _path.push(
        ...createPathDrawElements(
          line,
          _startPoint,
          `line-${idx + 1}`,
          line.color,
          opacity,
          settings,
          travelStates.get(line.id || ""),
        ),
      );
    });

    return _path;
  })();

  // Second path rendering (for dual path mode)
  $: secondPath = (() => {
    // Don't show second path when in multi-path mode (use activePaths instead)
    if ($activePaths.length > 0 || !$dualPathMode || !secondStartPoint || secondLines.length === 0) {
      return [];
    }

    let _path: CanvasPathElement[] = [];

    secondLines.forEach((line, idx) => {
      if (!line || !line.endPoint) return;
      // Route order, matching the main path: a path the sequence never reaches
      // is not drawn.
      const _startPoint = secondLineStartPoints.get(line.id || "") || null;
      if (!_startPoint) return;

      _path.push(
        ...createPathDrawElements(
          line,
          _startPoint,
          `second-line-${idx + 1}`,
          line.color,
          settings.pathOpacity || 1.0,
          {
            ...settings,
            showVelocityGradient: settings.showVelocityGradient,
          },
          secondTravelStates.get(line.id || ""),
        ),
      );
    });

    return _path;
  })();

  // Render all additional paths
  $: additionalPathElements = additionalPaths.map((pathData, pathIdx) => {
    if (!pathData.startPoint || pathData.lines.length === 0) {
      return [];
    }

    let _path: CanvasPathElement[] = [];
    // All paths should be clearly visible - only slight opacity variation
    const pathOpacityBase = settings.pathOpacity || 1.0;
    const opacity = (1.0 - (pathIdx * 0.1)) * pathOpacityBase;

    const startPoints = additionalLineStartPoints[pathIdx];

    pathData.lines.forEach((line, idx) => {
      if (!line || !line.endPoint) return;
      const _startPoint = startPoints?.get(line.id || "") || null;
      if (!_startPoint) return;

      _path.push(
        ...createPathDrawElements(
          line,
          _startPoint,
          `additional-path-${pathIdx}-line-${idx + 1}`,
          pathData.color || line.color,
          opacity,
          {
            ...pathData.settings,
            showVelocityGradient: settings.showVelocityGradient,
          },
          additionalTravelStates[pathIdx]?.get(line.id || ""),
        ),
      );
    });

    return _path;
  });

  $: activeAutoTotalTime = (() => {
    if ($activePaths.length > 0) {
      return additionalPaths.reduce((maxTime, pathData) => {
        if (!pathData.startPoint || pathData.lines.length === 0) return maxTime;
        const pathTime = calculatePathTime(
          pathData.startPoint,
          pathData.lines,
          pathData.settings,
          pathData.sequence,
          pathData.variables,
        );
        return Math.max(maxTime, pathTime.totalTime || 0);
      }, 0);
    }

    return Math.max(
      timePrediction?.totalTime || 0,
      secondTimePrediction?.totalTime || 0,
    );
  })();

  $: currentAutoTime =
    activeAutoTotalTime > 0
      ? (clampNumber(percent, 0, 100) / 100) * activeAutoTotalTime
      : 0;
  $: autoCountdownRemaining = AUTO_COUNTDOWN_SECONDS - currentAutoTime;
  $: autoCountdownProgress = clampNumber(
    currentAutoTime / AUTO_COUNTDOWN_SECONDS,
    0,
    1,
  );
  $: autoCountdownColor = getCountdownColor(autoCountdownRemaining);

  $: pathAnnotations = (() => {
    if (!settings.showPathAnnotations) return [];

    if ($activePaths.length > 0) {
      return additionalPaths.flatMap((pathData, pathIndex) => {
        if (!pathData.startPoint || pathData.lines.length === 0) return [];
        const pathTime = calculatePathTime(
          pathData.startPoint,
          pathData.lines,
          pathData.settings,
          pathData.sequence,
          pathData.variables,
        );
        return buildPathAnnotations(
          `additional-${pathIndex}`,
          pathData.startPoint,
          pathData.lines,
          pathTime,
          pathData.settings,
          pathData.sequence,
          pathData.variables,
        );
      });
    }

    const annotations = buildPathAnnotations(
      "main",
      startPoint,
      routeLines,
      timePrediction,
      settings,
      routeSequence,
      variables,
    );

    if ($dualPathMode && secondStartPoint && secondLines.length > 0) {
      annotations.push(
        ...buildPathAnnotations(
          "second",
          secondStartPoint,
          secondLines,
          secondTimePrediction,
          settings,
          secondSequence,
          variables,
        ),
      );
    }

    return annotations;
  })();

  $: eventPins = (() => {
    if (!settings.showEventPins) return [];

    if ($activePaths.length > 0) {
      return additionalPaths.flatMap((pathData, pathIndex) => {
        if (!pathData.startPoint || pathData.lines.length === 0) return [];
        const pathTime = calculatePathTime(
          pathData.startPoint,
          pathData.lines,
          pathData.settings,
          pathData.sequence,
          pathData.variables,
        );
        return buildEventPins(
          `additional-${pathIndex}`,
          pathData.startPoint,
          pathData.lines,
          pathTime,
          pathData.settings,
          pathData.sequence,
          pathData.variables,
        );
      });
    }

    const pins = buildEventPins(
      "main",
      startPoint,
      routeLines,
      timePrediction,
      settings,
      routeSequence,
      variables,
    );

    if ($dualPathMode && secondStartPoint && secondLines.length > 0) {
      pins.push(
        ...buildEventPins(
          "second",
          secondStartPoint,
          secondLines,
          secondTimePrediction,
          settings,
          secondSequence,
          variables,
        ),
      );
    }

    return pins;
  })();

  $: eventTimelineOverlays = (() => {
    const empty: EventTimelineOverlay = { elements: [], labels: [] };
    if (!settings.showEventTimeline) return empty;

    if ($activePaths.length > 0) {
      return additionalPaths.reduce<EventTimelineOverlay>((acc, pathData, pathIndex) => {
        if (!pathData.startPoint || pathData.lines.length === 0) return acc;
        const pathTime = calculatePathTime(
          pathData.startPoint,
          pathData.lines,
          pathData.settings,
          pathData.sequence,
          pathData.variables,
        );
        const overlay = buildEventTimelineOverlays(
          `additional-${pathIndex}`,
          pathData.startPoint,
          pathData.lines,
          pathTime,
          pathData.settings,
          pathData.sequence,
          pathData.variables,
        );
        acc.elements.push(...overlay.elements);
        acc.labels.push(...overlay.labels);
        return acc;
      }, empty);
    }

    const overlays = buildEventTimelineOverlays(
      "main",
      startPoint,
      routeLines,
      timePrediction,
      settings,
      routeSequence,
      variables,
    );

    if ($dualPathMode && secondStartPoint && secondLines.length > 0) {
      const secondOverlay = buildEventTimelineOverlays(
        "second",
        secondStartPoint,
        secondLines,
        secondTimePrediction,
        settings,
        secondSequence,
        variables,
      );
      overlays.elements.push(...secondOverlay.elements);
      overlays.labels.push(...secondOverlay.labels);
    }

    return overlays;
  })();
  $: eventTimelineElements = eventTimelineOverlays.elements;
  $: eventTimelineLabels = eventTimelineOverlays.labels;

  $: swerveWheelOverlays = (() => {
    if (!settings.showSwerveModules) return [];

    const overlays: SwerveWheelOverlay[] = [];

    if ($activePaths.length > 0) {
      additionalPaths.forEach((pathData, pathIndex) => {
        if (!pathData.startPoint || pathData.lines.length === 0) return;
        const pathTimePrediction = calculatePathTime(
          pathData.startPoint,
          pathData.lines,
          pathData.settings,
          pathData.sequence,
          pathData.variables,
        );
        const maxDuration = effectiveAnimationDuration;
        const thisDuration = getAnimationDuration(pathTimePrediction.totalTime / 1000);
        const completionPercent = maxDuration > 0 ? (thisDuration / maxDuration) * 100 : 0;
        const actualPercent = Math.min(percent, completionPercent);
        const normalizedPercent =
          completionPercent > 0 ? (actualPercent / completionPercent) * 100 : 0;
        const robotState = additionalRobotStates[pathIndex];

        if (!robotState) return;

        overlays.push(
          ...buildSwerveWheelModules(
            `additional-${pathIndex}`,
            robotState.xy,
            robotState.heading,
            getSwerveWheelAnglesCss(
              normalizedPercent,
              pathData.startPoint,
              pathData.lines,
              pathTimePrediction,
              pathData.settings,
              robotState.heading,
            ),
            0.9 - pathIndex * 0.12,
            pathData.color || "#06b6d4",
          ),
        );
      });

      return overlays;
    }

    overlays.push(
      ...buildSwerveWheelModules(
        "main",
        robotXY,
        robotHeading,
        getSwerveWheelAnglesCss(
          percent,
          startPoint,
          routeLines,
          timePrediction,
          settings,
          robotHeading,
        ),
        1,
        "#06b6d4",
      ),
    );

    if ($dualPathMode && secondStartPoint && secondLines.length > 0 && secondTimePrediction) {
      const maxDuration = effectiveAnimationDuration;
      const thisDuration = getAnimationDuration(secondTimePrediction.totalTime / 1000);
      const completionPercent = maxDuration > 0 ? (thisDuration / maxDuration) * 100 : 0;
      const actualPercent = Math.min(percent, completionPercent);
      const normalizedPercent =
        completionPercent > 0 ? (actualPercent / completionPercent) * 100 : 0;

      overlays.push(
        ...buildSwerveWheelModules(
          "second",
          secondRobotXY,
          secondRobotHeading,
          getSwerveWheelAnglesCss(
            normalizedPercent,
            secondStartPoint,
            secondLines,
            secondTimePrediction,
            settings,
            secondRobotHeading,
          ),
          0.8,
          "#fca5a5",
        ),
      );
    }

    if (settings.showOnionLayers) {
      const mainLayers = visibleOnionLayers;
      overlays.push(
        ...buildOnionSwerveModules(
          "main",
          startPoint,
          routeLines,
          mainLayers,
          0.72,
          settings.onionColor || "#dc2626",
          lineStartPoints,
        ),
      );

      if ($dualPathMode && secondStartPoint && secondLines.length > 0 && secondTimePrediction) {
        const secondLayers = visibleSecondOnionLayers;
        overlays.push(
          ...buildOnionSwerveModules(
            "second",
            secondStartPoint,
            secondLines,
            secondLayers,
            0.65,
            "#fca5a5",
          ),
        );
      }
    }

    return overlays;
  })();

  $: shapeElements = (() => {
    // Obstacles removed: return empty array for shape elements
    let _shapes: Path[] = [];

    shapes.forEach((shape, idx) => {
      if (shape.vertices.length >= 3) {
        // Create polygon from vertices - properly format for Two.js
        let vertices = [];

        // Start with move command for first vertex
        vertices.push(
          new Two.Anchor(
            x(shape.vertices[0].x),
            y(shape.vertices[0].y),
            0,
            0,
            0,
            0,
            Two.Commands.move,
          ),
        );

        // Add line commands for remaining vertices
        for (let i = 1; i < shape.vertices.length; i++) {
          vertices.push(
            new Two.Anchor(
              x(shape.vertices[i].x),
              y(shape.vertices[i].y),
              0,
              0,
              0,
              0,
              Two.Commands.line,
            ),
          );
        }

        // Close the shape
        vertices.push(
          new Two.Anchor(
            x(shape.vertices[0].x),
            y(shape.vertices[0].y),
            0,
            0,
            0,
            0,
            Two.Commands.close,
          ),
        );

        vertices.forEach((point) => (point.relative = false));

        let shapeElement = new Two.Path(vertices);
        shapeElement.id = `shape-${idx}`;
        shapeElement.stroke = shape.color;
        shapeElement.fill = shape.color;
        shapeElement.opacity = 0.4;
        shapeElement.linewidth = x(0.8);
        shapeElement.automatic = false;

        _shapes.push(shapeElement);
      }
    });

    return _shapes;
  })();

  $: ghostPathElement = (() => {
    let ghostPath: Path | null = null;

    // Don't show ghost paths in multi-path mode
    if ($activePaths.length === 0 && settings.showGhostPaths && lines.length > 0) {
      const ghostPoints = generateGhostPathPoints(
        startPoint,
        routeLines,
        settings.rWidth,
        settings.rHeight,
        50,
        lineStartPoints,
        headingTransitions,
      );

      if (ghostPoints.length >= 3) {
        // Create polygon from ghost path points
        let vertices = [];

        // Start with move command for first point
        vertices.push(
          new Two.Anchor(
            x(ghostPoints[0].x),
            y(ghostPoints[0].y),
            0,
            0,
            0,
            0,
            Two.Commands.move,
          ),
        );

        // Add line commands for remaining points
        for (let i = 1; i < ghostPoints.length; i++) {
          vertices.push(
            new Two.Anchor(
              x(ghostPoints[i].x),
              y(ghostPoints[i].y),
              0,
              0,
              0,
              0,
              Two.Commands.line,
            ),
          );
        }

        // Close the shape
        vertices.push(
          new Two.Anchor(
            x(ghostPoints[0].x),
            y(ghostPoints[0].y),
            0,
            0,
            0,
            0,
            Two.Commands.close,
          ),
        );

        vertices.forEach((point) => (point.relative = false));

        ghostPath = new Two.Path(vertices);
        ghostPath.id = "ghost-path";
        ghostPath.stroke = "#a78bfa"; // Light purple/lavender
        ghostPath.fill = "#a78bfa";
        ghostPath.opacity = 0.15;
        ghostPath.linewidth = x(0.5);
        ghostPath.automatic = false;
      }
    }

    return ghostPath;
  })();

  // Second ghost path for dual path mode
  $: secondGhostPathElement = (() => {
    let ghostPath: Path | null = null;

    // Don't show second ghost path in multi-path mode
    if ($activePaths.length === 0 && $dualPathMode && settings.showGhostPaths && secondLines.length > 0 && secondStartPoint) {
      const ghostPoints = generateGhostPathPoints(
        secondStartPoint,
        secondLines,
        settings.rWidth,
        settings.rHeight,
        50,
        secondLineStartPoints,
        secondHeadingTransitions,
      );

      if (ghostPoints.length >= 3) {
        let vertices = [];

        vertices.push(
          new Two.Anchor(
            x(ghostPoints[0].x),
            y(ghostPoints[0].y),
            0,
            0,
            0,
            0,
            Two.Commands.move,
          ),
        );

        for (let i = 1; i < ghostPoints.length; i++) {
          vertices.push(
            new Two.Anchor(
              x(ghostPoints[i].x),
              y(ghostPoints[i].y),
              0,
              0,
              0,
              0,
              Two.Commands.line,
            ),
          );
        }

        vertices.push(
          new Two.Anchor(
            x(ghostPoints[0].x),
            y(ghostPoints[0].y),
            0,
            0,
            0,
            0,
            Two.Commands.close,
          ),
        );

        vertices.forEach((point) => (point.relative = false));

        ghostPath = new Two.Path(vertices);
        ghostPath.id = "ghost-path-2";
        ghostPath.stroke = "#fca5a5"; // Light red/pink for second robot
        ghostPath.fill = "#fca5a5";
        ghostPath.opacity = 0.15;
        ghostPath.linewidth = x(0.5);
        ghostPath.automatic = false;
      }
    }

    return ghostPath;
  })();

  // Ghost paths for additional paths in multi-path mode
  $: additionalGhostPathElements = (() => {
    let ghostPaths: Path[] = [];

    if ($activePaths.length > 0 && settings.showGhostPaths) {
      additionalPaths.forEach((pathData, pathIdx) => {
        if (!pathData.startPoint || !pathData.lines.length) return;

        const ghostPoints = generateGhostPathPoints(
          pathData.startPoint,
          pathData.lines,
          settings.rWidth,
          settings.rHeight,
          50,
          additionalLineStartPoints[pathIdx],
          additionalHeadingTransitions[pathIdx],
        );

        if (ghostPoints.length >= 3) {
          let vertices = [];

          vertices.push(
            new Two.Anchor(
              x(ghostPoints[0].x),
              y(ghostPoints[0].y),
              0,
              0,
              0,
              0,
              Two.Commands.move,
            ),
          );

          for (let i = 1; i < ghostPoints.length; i++) {
            vertices.push(
              new Two.Anchor(
                x(ghostPoints[i].x),
                y(ghostPoints[i].y),
                0,
                0,
                0,
                0,
                Two.Commands.line,
              ),
            );
          }

          vertices.push(
            new Two.Anchor(
              x(ghostPoints[0].x),
              y(ghostPoints[0].y),
              0,
              0,
              0,
              0,
              Two.Commands.close,
            ),
          );

          vertices.forEach((point) => (point.relative = false));

          const ghostPath = new Two.Path(vertices);
          ghostPath.id = `ghost-path-additional-${pathIdx}`;
          ghostPath.stroke = pathData.color || "#a78bfa";
          ghostPath.fill = pathData.color || "#a78bfa";
          ghostPath.opacity = 0.15;
          ghostPath.linewidth = x(0.5);
          ghostPath.automatic = false;
          
          ghostPaths.push(ghostPath);
        }
      });
    }

    return ghostPaths;
  })();

  /**
   * Two.js bodies for the onion layers, rebuilt only when the visible set
   * actually changes.
   *
   * Svelte treats any assignment of an object as a change, so this block still
   * runs on every frame; the guard is what keeps it from allocating a few
   * thousand anchors each time for an identical picture.
   */
  let onionElementCacheSource: OnionLayerState[] | null = null;
  let onionElementCacheSelected: number | null = null;
  let onionElementCacheColor = "";
  let onionElementCacheScale = 0;
  let onionElementCache: Path[] = [];

  $: onionLayerElements = (() => {
    const color = settings.onionColor || "#dc2626";
    // The scale changes when the field is resized, and the bodies are built in
    // screen units, so it belongs in the key.
    const scale = x(1);

    if (
      onionElementCacheSource === allOnionLayers &&
      onionElementCacheSelected === onionSelectedLineIndex &&
      onionElementCacheColor === color &&
      onionElementCacheScale === scale
    ) {
      return onionElementCache;
    }

    let onionLayers: Path[] = [];

    if (onionLayersEnabled) {
      const layers = visibleOnionLayers;

      layers.forEach((layer, idx) => {
        // Create a rectangle from the robot corners
        let vertices: any[] = [];

        // Create path from corners: front-left -> front-right -> back-right -> back-left
        vertices.push(
          new Two.Anchor(
            x(layer.corners[0].x),
            y(layer.corners[0].y),
            0,
            0,
            0,
            0,
            Two.Commands.move,
          ),
        );

        for (let i = 1; i < layer.corners.length; i++) {
          vertices.push(
            new Two.Anchor(
              x(layer.corners[i].x),
              y(layer.corners[i].y),
              0,
              0,
              0,
              0,
              Two.Commands.line,
            ),
          );
        }

        // Close the path by returning to the first corner
        vertices.push(
          new Two.Anchor(
            x(layer.corners[0].x),
            y(layer.corners[0].y),
            0,
            0,
            0,
            0,
            Two.Commands.close,
          ),
        );

        vertices.forEach((point) => (point.relative = false));

        let onionRect = new Two.Path(vertices);
        onionRect.id = `onion-layer-${idx}`;
        onionRect.stroke = color;
        onionRect.noFill();
        // Increase opacity so colliders are more visible
        onionRect.opacity = 0.9;
        onionRect.linewidth = x(0.28);
        onionRect.automatic = false;

        onionLayers.push(onionRect);
      });
    }

    onionElementCacheSource = allOnionLayers;
    onionElementCacheSelected = onionSelectedLineIndex;
    onionElementCacheColor = color;
    onionElementCacheScale = scale;
    onionElementCache = onionLayers;
    return onionLayers;
  })();

  /**
   * The robot drawn where it comes too close to something, so the problem is
   * visible as a shape on the field rather than only as a number in a list.
   * Red for contact, amber for inside the margin.
   *
   * A span that makes contact is drawn where the robot FIRST touches, not at
   * its deepest sample: on anything the robot drives into, the deepest point is
   * well past the first, and marking it there made the collision look like it
   * happened much later along the path than it does. A span that only comes
   * inside the margin is drawn at its worst, which is the tightest point.
   */
  /**
   * Pollen Pickups on the field: what the camera sees from where each step
   * starts (solid out to where a single ball still registers, dashed beyond),
   * the search zone, the expected POLLEN as a draggable marker, and the legs
   * the robot drives - approach, intake, return.
   */
  $: pollenElements = (() => {
    // Layered so every draggable marker is on top of every step's camera view,
    // whichever step it belongs to - otherwise a later step's view swallows the
    // click meant for an earlier step's marker.
    const views: any[] = [];
    const zones: any[] = [];
    const legs: any[] = [];
    const markers: any[] = [];
    if ($activePaths.length > 0) return [];

    const polygon = (points: { x: number; y: number }[], id: string) => {
      const anchors = points.map(
        (point, index) =>
          new Two.Anchor(
            x(point.x),
            y(point.y),
            0,
            0,
            0,
            0,
            index === 0 ? Two.Commands.move : Two.Commands.line,
          ),
      );
      anchors.push(new Two.Anchor(x(points[0].x), y(points[0].y), 0, 0, 0, 0, Two.Commands.close));
      anchors.forEach((anchor) => (anchor.relative = false));
      const shape = new Two.Path(anchors);
      shape.id = id;
      return shape;
    };

    sequence.forEach((item, sequenceIndex) => {
      if (item.kind !== "pollen") return;
      const plan = pollenPlans.get(item.id);
      const px = (inches: number) => x(inches) - x(0);

      if (plan) {
        const full = polygon(plan.footprint, `pollen-view-${sequenceIndex}`);
        full.fill = "rgba(34, 211, 238, 0.05)";
        full.stroke = "rgba(34, 211, 238, 0.55)";
        full.linewidth = px(0.25);
        full.dashes = [px(1.2), px(1)];
        views.push(full);

        if (plan.reliableFootprint.length) {
          const reliable = polygon(plan.reliableFootprint, `pollen-reliable-${sequenceIndex}`);
          reliable.fill = "rgba(34, 211, 238, 0.14)";
          reliable.stroke = "rgba(34, 211, 238, 0.9)";
          reliable.linewidth = px(0.3);
          views.push(reliable);
        }

        const legColor: Record<string, string> = {
          approach: "#facc15",
          intake: "#f97316",
          return: "rgba(255, 255, 255, 0.7)",
        };
        plan.legs.forEach((leg) => {
          if (leg.phase === "search") return;
          if (Math.hypot(leg.to.x - leg.from.x, leg.to.y - leg.from.y) < 0.05) return;
          const line = new Two.Line(x(leg.from.x), y(leg.from.y), x(leg.to.x), y(leg.to.y));
          line.id = `pollen-leg-${sequenceIndex}-${leg.phase}`;
          line.stroke = legColor[leg.phase];
          line.linewidth = px(leg.phase === "intake" ? 0.9 : 0.55);
          if (leg.phase === "return") line.dashes = [px(1.5), px(1)];
          legs.push(line);
        });

        // Where the approach stops, facing the POLLEN.
        const standoff = new Two.Circle(x(plan.standoffPose.x), y(plan.standoffPose.y), px(0.8));
        standoff.id = `pollen-standoff-${sequenceIndex}`;
        standoff.fill = "#facc15";
        standoff.noStroke();
        legs.push(standoff);
      }

      const zone = new Two.Circle(x(item.targetX), y(item.targetY), px(item.zoneRadius));
      zone.id = `pollen-zone-${sequenceIndex}`;
      zone.fill = "rgba(250, 204, 21, 0.10)";
      zone.stroke = "rgba(250, 204, 21, 0.95)";
      zone.linewidth = px(0.3);
      zone.dashes = [px(1.2), px(0.8)];
      zones.push(zone);

      // The grab handle is bigger than the ball so it is easy to pick up.
      const handle = new Two.Circle(x(item.targetX), y(item.targetY), px(3.2));
      handle.id = `pollen-target-${sequenceIndex}`;
      handle.fill = "rgba(250, 204, 21, 0.25)";
      handle.stroke = item.locked ? "rgba(120, 120, 120, 0.9)" : "#111827";
      handle.linewidth = px(0.25);
      markers.push(handle);

      const ball = new Two.Circle(x(item.targetX), y(item.targetY), px(1.4));
      ball.id = `pollen-target-${sequenceIndex}-ball`;
      ball.fill = "#facc15";
      ball.stroke = "#111827";
      ball.linewidth = px(0.3);
      markers.push(ball);
    });

    return [...views, ...zones, ...legs, ...markers];
  })();

  /**
   * Camera pick previews (a path's "Camera pick"): the camera's view from where
   * the frame is read, the stretch of wall the robot may drive to, the planned
   * path past the pick (dashed - what runs when nothing is seen) and the clump,
   * a marker dragged along the wall. The re-routed drive itself is an ordinary
   * path in `routeLines`, so it is drawn, timed and checked like any other.
   */
  $: cameraPickViews = (() => {
    const views: any[] = [];
    if ($activePaths.length > 0 || !x || !y) return views;
    const px = (inches: number) => x(inches) - x(0);
    cameraPicks.forEach((preview) => {
      const g = preview.geometry;
      const lineY = g.pick.wallY > FIELD_SIZE / 2 ? FIELD_SIZE - 1.4 : 1.4;
      if (g.camera.leftX !== null && g.camera.rightX !== null) {
        const anchors = [
          [g.camera.x, g.camera.y],
          [g.camera.leftX, lineY],
          [g.camera.rightX, lineY],
        ].map(([ax, ay], i) => {
          const anchor = new Two.Anchor(x(ax), y(ay), 0, 0, 0, 0, i === 0 ? Two.Commands.move : Two.Commands.line);
          anchor.relative = false;
          return anchor;
        });
        const close = new Two.Anchor(x(g.camera.x), y(g.camera.y), 0, 0, 0, 0, Two.Commands.close);
        close.relative = false;
        anchors.push(close);
        const wedge = new Two.Path(anchors);
        wedge.id = `campick-view-${preview.lineIndex}`;
        wedge.fill = g.clumpInView ? "rgba(34, 211, 238, 0.10)" : "rgba(248, 113, 113, 0.10)";
        wedge.stroke = g.clumpInView ? "rgba(34, 211, 238, 0.7)" : "rgba(248, 113, 113, 0.8)";
        wedge.linewidth = px(0.25);
        wedge.dashes = [px(1.2), px(1)];
        views.push(wedge);
      }
      const stretch = new Two.Line(x(g.pick.xMin), y(lineY), x(g.pick.xMax), y(lineY));
      stretch.id = `campick-stretch-${preview.lineIndex}`;
      stretch.stroke = "rgba(34, 197, 94, 0.75)";
      stretch.linewidth = px(1.1);
      views.push(stretch);
      if (preview.plannedRest.length > 1) {
        const rest = new Two.Path(
          preview.plannedRest.map((point, i) => {
            const anchor = new Two.Anchor(x(point.x), y(point.y), 0, 0, 0, 0, i === 0 ? Two.Commands.move : Two.Commands.line);
            anchor.relative = false;
            return anchor;
          }),
          false,
          false,
        );
        rest.id = `campick-rest-${preview.lineIndex}`;
        rest.noFill();
        rest.stroke = "rgba(255, 255, 255, 0.45)";
        rest.linewidth = px(0.45);
        rest.dashes = [px(1.5), px(1.2)];
        views.push(rest);
      }
      const pickDot = new Two.Circle(x(g.pickPose.x), y(g.pickPose.y), px(1.1));
      pickDot.id = `campick-at-${preview.lineIndex}`;
      pickDot.fill = "#22d3ee";
      pickDot.stroke = "#0f172a";
      pickDot.linewidth = px(0.3);
      views.push(pickDot);
    });
    return views;
  })();

  $: cameraPickMarkers = (() => {
    const markers: any[] = [];
    if ($activePaths.length > 0 || !x || !y) return markers;
    const px = (inches: number) => x(inches) - x(0);
    cameraPicks.forEach((preview) => {
      const g = preview.geometry;
      const lineY = g.pick.wallY > FIELD_SIZE / 2 ? FIELD_SIZE - 1.4 : 1.4;
      const locked = Boolean(lines[preview.lineIndex]?.locked);
      const handle = new Two.Circle(x(g.clumpX), y(lineY), px(3.4));
      handle.id = `campick-${preview.lineIndex}`;
      handle.fill = "rgba(250, 204, 21, 0.25)";
      handle.stroke = locked ? "rgba(120, 120, 120, 0.9)" : g.clumpInView ? "#111827" : "#dc2626";
      handle.linewidth = px(g.clumpInView ? 0.25 : 0.6);
      markers.push(handle);
      [-1.6, 0, 1.6].forEach((dx, i) => {
        const ball = new Two.Circle(x(g.clumpX + dx), y(lineY), px(1.4));
        ball.id = `campick-${preview.lineIndex}-ball${i}`;
        ball.fill = "#facc15";
        ball.stroke = "#111827";
        ball.linewidth = px(0.3);
        markers.push(ball);
      });
    });
    return markers;
  })();

  $: clearanceElements = (() => {
    const elements: Path[] = [];
    if ($activePaths.length > 0 || settings.showClearance === false) return elements;

    clearanceReport.spans.forEach((span, index) => {
      const outline = span.contactFootprint ?? span.worstFootprint;
      if (!outline?.length) return;

      const vertices = outline.map((corner, cornerIndex) =>
        new Two.Anchor(
          x(corner.x),
          y(corner.y),
          0,
          0,
          0,
          0,
          cornerIndex === 0 ? Two.Commands.move : Two.Commands.line,
        ),
      );
      vertices.push(
        new Two.Anchor(
          x(outline[0].x),
          y(outline[0].y),
          0,
          0,
          0,
          0,
          Two.Commands.close,
        ),
      );
      vertices.forEach((point) => (point.relative = false));

      const hit = span.severity === "hit";
      const body = new Two.Path(vertices);
      body.id = `clearance-${index}-${span.lineId}`;
      body.stroke = hit ? "#dc2626" : "#f59e0b";
      body.fill = hit ? "rgba(220, 38, 38, 0.22)" : "rgba(245, 158, 11, 0.16)";
      body.linewidth = x(hit ? 0.4 : 0.3);
      body.opacity = 0.95;
      body.automatic = false;

      elements.push(body);
    });

    return elements;
  })();

  // Second onion layers for dual path mode
  $: secondOnionLayerElements = (() => {
    let onionLayers: Path[] = [];

    // Don't show second onion layers in multi-path mode
    if ($activePaths.length === 0 && $dualPathMode && settings.showOnionLayers && secondLines.length > 0 && secondStartPoint) {
      const layers = visibleSecondOnionLayers;

      layers.forEach((layer, idx) => {
        let vertices: any[] = [];

        vertices.push(
          new Two.Anchor(
            x(layer.corners[0].x),
            y(layer.corners[0].y),
            0,
            0,
            0,
            0,
            Two.Commands.move,
          ),
        );

        for (let i = 1; i < layer.corners.length; i++) {
          vertices.push(
            new Two.Anchor(
              x(layer.corners[i].x),
              y(layer.corners[i].y),
              0,
              0,
              0,
              0,
              Two.Commands.line,
            ),
          );
        }

        vertices.push(
          new Two.Anchor(
            x(layer.corners[0].x),
            y(layer.corners[0].y),
            0,
            0,
            0,
            0,
            Two.Commands.close,
          ),
        );

        vertices.forEach((point) => (point.relative = false));

        let onionRect = new Two.Path(vertices);
        onionRect.id = `second-onion-layer-${idx}`;
        onionRect.stroke = "#fca5a5"; // Light red/pink for second path
        onionRect.noFill();
        onionRect.opacity = 0.9;
        onionRect.linewidth = x(0.28);
        onionRect.automatic = false;

        onionLayers.push(onionRect);
      });
    }

    return onionLayers;
  })();

  let isLoaded = false;
  /**
   * Loading a file replaces every tracked value, which would otherwise trip the
   * watcher below and mark the freshly opened project as unsaved. Loaders set
   * this so the next pass is treated as "already saved".
   */
  let suppressUnsavedOnce = false;
  // Reactively trigger when any saveable data changes
  $: {
    if (isLoaded && (lines || shapes || startPoint || settings || sequence || variables)) {
      if (suppressUnsavedOnce) {
        suppressUnsavedOnce = false;
        isUnsaved.set(false);
      } else {
        isUnsaved.set(true);
      }
    }
  }

  // Allow the app to stabilize before tracking changes
  onMount(() => {
    setTimeout(() => {
      isLoaded = true;
      recordChange();
    }, 500);
  });
  onMount(async () => {
    // Load saved settings
    const savedSettings = await loadSettings();
    settings = normalizeLegacyFieldMap({ ...savedSettings });

    // The obstacles were built from DEFAULT_SETTINGS before the saved field map
    // arrived. If they are still a preset as shipped, move them to this field's.
    if (isUntouchedObstaclePreset(shapes)) {
      shapes = getDefaultShapes(settings.fieldMap);
    }

    // Update robot dimensions from loaded settings
    robotWidth = settings.rWidth;
    robotHeight = settings.rHeight;
  });
  // Debounced save function
  const debouncedSaveSettings = debounce(async (settingsToSave: Settings) => {
    await saveSettings(settingsToSave);
  }, 1000);
  // Save after 1 second of inactivity

  // Watch for settings changes and save
  $: {
    if (settings) {
      debouncedSaveSettings(settings);
    }
  }

  // Initialize animation controller
  onMount(() => {
    animationController = createAnimationController(
      animationDuration,
      (newPercent) => {
        percent = newPercent;
      },
      () => {
        // Animation completed callback
        console.log("Animation completed");
        playing = false;
      },
    );
  });
  $: if (animationController) {
    animationController.setDuration(effectiveAnimationDuration);
  }

  $: if (animationController) {
    animationController.setLoop(loopAnimation);
    // Sync UI state with controller
    playing = animationController.isPlaying();
  }

  // Save Function
  // Save the current project into the browser-backed store (or download)
  async function saveProject() {
    try {
      await saveFile();
    } catch (e) {
      console.error("Failed to save project:", e);
      alert("Failed to save file.");
    }
  }

  // Save an additional path back to its file
  async function saveAdditionalPath(pathIdx: number) {
    const pathData = additionalPaths[pathIdx];
    if (!pathData || !pathData.filePath) return;
    
    try {
      const fileData = JSON.stringify({
        startPoint: pathData.startPoint,
        lines: pathData.lines,
        shapes: pathData.shapes,
        sequence: pathData.sequence,
        pathChains: pathData.pathChains || [],
        variables: pathData.variables || [],
        settings: pathData.settings,
        version: "1.2.1",
        timestamp: new Date().toISOString(),
      });
      
      await browserFileStore.writeFile(pathData.filePath, fileData);
      console.log(`Auto-saved additional path: ${pathData.filePath}`);
    } catch (error) {
      console.error(`Failed to save additional path ${pathData.filePath}:`, error);
      throw error;
    }
  }

  // Keyboard shortcut for save
  hotkeys("cmd+s, ctrl+s", function (event, handler) {
    event.preventDefault();
    if ($activePaths.length > 0) {
      // Multiple paths mode - save all modified paths
      showDualPathSaveDialog = true;
    } else if ($dualPathMode && secondStartPoint && secondLines.length > 0) {
      showDualPathSaveDialog = true;
    } else {
      showSaveDialog = true;
    }
  });

  // Export path animation as GIF
  async function exportPathAsGif() {
    if (!twoElement || !two) {
      alert("Canvas not ready. Please try again.");
      return;
    }
    
    // Two.js can render as canvas or SVG; exporter supports both.
    const rendererElement = two.renderer.domElement;
    if (!rendererElement) {
      alert("Unable to access renderer for export.");
      return;
    }
    
    // Check if we have paths to export
    const hasActivePaths = $activePaths.length > 0;
    const hasDualPath = $dualPathMode && secondStartPoint && secondLines.length > 0;
    const hasSinglePath = lines.length > 0;
    
    if (!hasActivePaths && !hasDualPath && !hasSinglePath) {
      alert("No paths to export. Please create a path first.");
      return;
    }

    try {
      exportingGif = true;
      cancelGifExport = false;
      gifExportProgress = 0;
      gifExportStatus = "Calculating animation duration...";

      const scale = 0.65;
      const viewWidth = twoElement.clientWidth;
      const viewHeight = twoElement.clientHeight;
      const robotPixelWidth = x(robotWidth);
      const robotPixelHeight = x(robotHeight);

      const imageCache = new Map<string, HTMLImageElement>();
      const loadImage = (src: string) =>
        new Promise<HTMLImageElement>((resolve, reject) => {
          if (imageCache.has(src)) {
            resolve(imageCache.get(src)!);
            return;
          }
          const image = new Image();
          image.onload = () => {
            imageCache.set(src, image);
            resolve(image);
          };
          image.onerror = () => reject(new Error(`Failed to load image: ${src}`));
          image.src = src;
        });

      const fieldImage = await loadImage(fieldMapSrc).catch(async () => {
        return loadImage(`/fields/${DEFAULT_SETTINGS.fieldMap}`);
      });
      const robotImage = await loadImage(settings.robotImage || "/robot.png").catch(async () => {
        return loadImage("/robot.png");
      });

      const drawRobot = (
        ctx: CanvasRenderingContext2D,
        xy: BasePoint,
        headingDeg: number,
        opacity = 1,
      ) => {
        ctx.save();
        ctx.globalAlpha = opacity;
        ctx.translate(xy.x * scale, xy.y * scale);
        ctx.rotate((headingDeg * Math.PI) / 180);
        ctx.drawImage(
          robotImage,
          (-robotPixelWidth * scale) / 2,
          (-robotPixelHeight * scale) / 2,
          robotPixelWidth * scale,
          robotPixelHeight * scale,
        );
        ctx.restore();
      };

      // Calculate total animation duration
      let totalDuration = 0;
      
      if (hasActivePaths) {
        // Multiple paths mode - use the longest path duration
        for (const pathData of additionalPaths) {
          if (!pathData.startPoint) continue;
          const pathTime = calculatePathTime(
            pathData.startPoint,
            pathData.lines,
            pathData.settings,
            pathData.sequence,
            pathData.variables,
          );
          totalDuration = Math.max(totalDuration, pathTime?.totalTime || 0);
        }
      } else if (hasDualPath) {
        // Dual path mode - use the longer path
        const path1Time = calculatePathTime(startPoint, lines, settings, sequence, variables);
        const path2Time = secondStartPoint 
          ? calculatePathTime(secondStartPoint, secondLines, settings, secondSequence, variables)
          : { totalTime: 0 };
        totalDuration = Math.max(path1Time?.totalTime || 0, path2Time?.totalTime || 0);
      } else {
        // Single path mode
        const pathTime = calculatePathTime(startPoint, lines, settings, sequence, variables);
        totalDuration = pathTime?.totalTime || 0;
      }

      if (!Number.isFinite(totalDuration) || totalDuration <= 0) {
        alert("Path duration is too short to export.");
        exportingGif = false;
        return;
      }

      gifExportStatus = "Preparing animation...";
      
      // Stop any playing animation and reset to start
      const wasPlaying = playing;
      pause();
      percent = 0;
      animationController.reset();
        two.update(); // Make sure Two.js renders the initial state
      await tick(); // Allow UI to update

      gifExportStatus = "Capturing frames...";
      
      // Export as GIF with manual frame control
      const durationMs = totalDuration * 1000;
      const blob = await exportAsGif({
        source: rendererElement as HTMLCanvasElement | SVGSVGElement,
        duration: durationMs,
        fps: 20, // Higher FPS for smoother animation
        quality: 15, // Slightly lower quality for smaller file size
        scale, // Lower resolution for smaller file size
        shouldCancel: () => cancelGifExport,
        onDrawBackground: (ctx, outputWidth, outputHeight) => {
          ctx.drawImage(fieldImage, 0, 0, outputWidth, outputHeight);
        },
        onDrawForeground: (ctx) => {
          if ($activePaths.length === 0) {
            drawRobot(ctx, robotXY, robotHeading, 1);
            if ($dualPathMode && secondStartPoint && secondLines.length > 0) {
              drawRobot(ctx, secondRobotXY, secondRobotHeading, 0.8);
            }
            return;
          }

          additionalRobotStates.forEach((robotState, idx) => {
            const opacity = Math.max(0.2, 1 - idx * 0.15);
            drawRobot(ctx, robotState.xy, robotState.heading, opacity);
          });
        },
        onProgress: (progress) => {
          gifExportProgress = progress;
          if (progress < 0.5) {
            gifExportStatus = `Capturing frames... ${Math.round(progress * 200)}%`;
          } else {
            gifExportStatus = `Encoding GIF... ${Math.round((progress - 0.5) * 200)}%`;
          }
        },
        onFrameAdvance: async (frameIndex, totalFrames) => {
          // Calculate the percentage for this frame
          const framePercent = (frameIndex / (totalFrames - 1)) * 100;
          
          // Update the animation to this frame
          percent = framePercent;
          animationController.seekToPercent(framePercent);
          two.update(); // Force Two.js to render
          
          // Allow UI to update before capturing
          await tick();
        },
      });

      // Reset animation
      percent = 0;
      animationController.reset();
      
      // Resume playing if it was playing before
      if (wasPlaying) {
        play();
      }

      gifExportStatus = "Saving file...";
      
      // Download the GIF
      const fileName = $currentFilePath
        ? $currentFilePath.split(/[\/\\]/).pop()?.replace(/\.pp$/, "")
        : hasActivePaths
          ? "multiple_paths"
          : hasDualPath
            ? "dual_path"
            : "path_animation";
      
      downloadBlob(blob, `${fileName}.gif`);

      exportingGif = false;
      gifExportProgress = 0;
    } catch (error) {
      const errorMsg = error instanceof Error ? error.message : String(error);
      console.error("GIF export error:", errorMsg);
      
      // Don't show alert if user cancelled
      if (!errorMsg.includes('cancelled')) {
        alert("Failed to export GIF: " + errorMsg);
      }
      
      cancelGifExport = false;
      exportingGif = false;
      gifExportProgress = 0;
      pause();
    }
  }
  $: {
    // This handles both 'travel' (movement) and 'wait' (stationary rotation) events.
    // Don't show main robot in multi-path mode
    if ($activePaths.length === 0 && timePrediction && timePrediction.timeline && lines.length > 0) {
      const state = calculateRobotState(
        percent,
        timePrediction.timeline,
        routeLines,
        startPoint,
        settings,
        x,
        y,
      );
      robotXY = { x: state.x, y: state.y };
      robotHeading = state.heading;
    } else {
      // Fallback for initialization or empty state
      robotXY = { x: x(startPoint.x), y: y(startPoint.y) };
      // Calculate initial heading based on start point settings
      if (startPoint.heading === "linear") robotHeading = -startPoint.startDeg;
      else if (startPoint.heading === "constant")
        robotHeading = -startPoint.degrees;
      else robotHeading = 0;
    }
  }

  // Second robot state calculation (for dual path mode)
  $: {
    // Don't show second robot in multi-path mode
    if (
      $activePaths.length === 0 &&
      $dualPathMode &&
      timePrediction &&
      secondTimePrediction &&
      secondTimePrediction.timeline &&
      secondLines.length > 0 &&
      secondStartPoint
    ) {
      // Calculate actual percent for this path based on max duration
      const maxDuration = effectiveAnimationDuration;
      const thisDuration = getAnimationDuration(secondTimePrediction.totalTime / 1000);
      const completionPercent = (thisDuration / maxDuration) * 100;
      
      // If this path should be complete, cap at 100% (robot waits at end)
      const actualPercent = Math.min(percent, completionPercent);
      const normalizedPercent = completionPercent > 0 ? (actualPercent / completionPercent) * 100 : 0;

      const state = calculateRobotState(
        normalizedPercent,
        secondTimePrediction.timeline,
        secondLines,
        secondStartPoint,
        settings,
        x,
        y,
      );
      secondRobotXY = { x: state.x, y: state.y };
      secondRobotHeading = state.heading;
    } else {
      // Fallback or not in dual mode
      secondRobotXY = { x: 0, y: 0 };
      secondRobotHeading = 0;
    }
  }

  // Calculate robot states for all additional paths
  $: {
    additionalRobotStates = additionalPaths.map((pathData) => {
      if (!pathData.startPoint) {
        return {
          xy: { x: 0, y: 0 },
          heading: 0,
        };
      }

      const pathTimePrediction = calculatePathTime(
        pathData.startPoint,
        pathData.lines,
        pathData.settings,
        pathData.sequence,
        pathData.variables,
      );
      
      if (pathTimePrediction && pathTimePrediction.timeline && pathData.lines.length > 0 && pathData.startPoint) {
        // Calculate actual percent for this path based on max duration
        const maxDuration = effectiveAnimationDuration;
        const thisDuration = getAnimationDuration(pathTimePrediction.totalTime / 1000);
        const completionPercent = (thisDuration / maxDuration) * 100;
        
        // If this path should be complete, cap at 100% (robot waits at end)
        const actualPercent = Math.min(percent, completionPercent);
        const normalizedPercent = completionPercent > 0 ? (actualPercent / completionPercent) * 100 : 0;

        const state = calculateRobotState(
          normalizedPercent,
          pathTimePrediction.timeline,
          pathData.lines,
          pathData.startPoint,
          pathData.settings,
          x,
          y,
        );
        
        return {
          xy: { x: state.x, y: state.y },
          heading: state.heading,
        };
      }
      
      return {
        xy: { x: 0, y: 0 },
        heading: 0,
      };
    });
  }

  // Event markers removed: no runtime visualization created

  $: (() => {
    if (!two) {
      return;
    }

    two.renderer.domElement.style["z-index"] = "30";
    two.renderer.domElement.style["position"] = "absolute";
    two.renderer.domElement.style["top"] = "0px";
    two.renderer.domElement.style["left"] = "0px";
    two.renderer.domElement.style["width"] = "100%";
    two.renderer.domElement.style["height"] = "100%";

    two.clear();

    two.add(...shapeElements);
    if (ghostPathElement) {
      two.add(ghostPathElement);
    }
    if (secondGhostPathElement) {
      two.add(secondGhostPathElement);
    }
    if (additionalGhostPathElements.length > 0) {
      two.add(...additionalGhostPathElements);
    }
    if (onionLayerElements.length > 0) {
      two.add(...onionLayerElements);
    }
    if (secondOnionLayerElements.length > 0) {
      two.add(...secondOnionLayerElements);
    }
    if (pollenElements.length > 0) {
      two.add(...pollenElements);
    }
    if (cameraPickViews.length > 0) {
      two.add(...cameraPickViews);
    }
    if (clearanceElements.length > 0) {
      two.add(...clearanceElements);
    }
    two.add(...path);
    if ($dualPathMode && secondPath.length > 0) {
      two.add(...secondPath);
    }
    // Add all additional paths
    if ($activePaths.length > 0) {
      additionalPathElements.forEach((pathElements) => {
        if (pathElements.length > 0) {
          two.add(...pathElements);
        }
      });
    }
    if (eventTimelineElements.length > 0) {
      two.add(...eventTimelineElements);
    }
    if (cameraPickMarkers.length > 0) {
      two.add(...cameraPickMarkers);
    }
    two.add(...points);

    two.update();
  })();
  async function saveFileAs() {
    const win: any = window as any;
    const content = JSON.stringify(
      {
        startPoint,
        lines,
        shapes,
        sequence,
        pathChains,
        variables,
        settings,
        version: "1.2.1",
        timestamp: new Date().toISOString(),
      },
      null,
      2,
    );

    // Prefer File System Access API if available: opens native Save dialog
    if (win.showSaveFilePicker) {
      try {
        const opts = {
          suggestedName: $currentFilePath
            ? $currentFilePath.split(/[\/]/).pop()
            : "path.pp",
          types: [
            {
              description: "Path files",
              accept: { "application/json": [".pp", ".json"] },
            },
          ],
        };

        const handle = await win.showSaveFilePicker(opts);
        if (!handle) {
          // User cancelled
          return;
        }

        const writable = await handle.createWritable();
        await writable.write(content);
        await writable.close();

        // Update app state to reflect saved file
        try {
          currentFilePath.set(
            handle.name || (typeof handle === "string" ? handle : null),
          );
        } catch (e) {
          // ignore
        }
        isUnsaved.set(false);
        alert(`Saved to: ${handle.name || "selected file"}`);
        return;
      } catch (err) {
        console.error("SaveFilePicker error:", err);
        // fall through to download fallback
      }
    }

    // If showSaveFilePicker is not available or failed, try showOpenFilePicker to let user pick an existing file to overwrite
    if (win.showOpenFilePicker) {
      try {
        const [handle] = await win.showOpenFilePicker({
          types: [
            {
              description: "Path files",
              accept: { "application/json": [".pp", ".json"] },
            },
          ],
          multiple: false,
        });

        if (handle) {
          const writable = await handle.createWritable();
          await writable.write(content);
          await writable.close();
          try {
            currentFilePath.set(handle.name || null);
          } catch (e) {}
          isUnsaved.set(false);
          alert(`Saved to local file: ${handle.name || "selected file"}`);
          return;
        }
      } catch (err) {
        console.error("showOpenFilePicker error:", err);
        // fall through to download fallback
      }
    }

    // Fallback for browsers without File System Access (e.g., Firefox).
    // Automatically save into the app's browser-backed storage to avoid forcing a download.
    try {
      await saveFile();
      alert(
        "Your browser does not support native file dialogs. The project was saved to the app's storage.\n\nOpen the File Manager to download or export the file to your computer.",
      );
    } catch (err) {
      console.error("Failed to save into app storage:", err);
      // As a last resort, download the file
      try {
        downloadTrajectory(
          startPoint,
          lines,
          shapes,
          sequence,
          pathChains,
          variables,
          settings,
        );
      } catch (err2) {
        console.error("Save As fallback failed:", err2);
        alert(
          "Failed to save file. Your browser may not support file picker APIs.",
        );
      }
    }
  }

  function play() {
    animationController.play();
    playing = true;
  }

  function pause() {
    animationController.pause();
    playing = false;
  }

  function resetAnimation() {
    animationController.reset();
    playing = false;
  }

  // Handle slider changes
  function handleSeek(newPercent: number) {
    if (animationController) {
      animationController.seekToPercent(newPercent);
    }
  }

  onMount(() => {
    two = new Two({
      fitted: true,
      type: Two.Types.svg,
    }).appendTo(twoElement);

    let currentElem: string | null = null;
    let isDown = false;
    let dragOffset = { x: 0, y: 0 }; // Store offset to prevent snapping to center

    const isLockedPathElem = (id: string | null): boolean => {
      if (!id || !id.startsWith("point")) return false;
      const parts = id.split("-");
      const lineIdx = Number(parts[1]) - 1;
      const pointIdx = Number(parts[2]);
      if (Number.isNaN(lineIdx)) return false;
      if (lineIdx < 0) return !!startPoint.locked || !!startPoint.poseVariableId;
      if (pointIdx === 0 && lines[lineIdx]?.endPoint?.poseVariableId) return true;
      return !!lines[lineIdx]?.locked;
    };

    two.renderer.domElement.addEventListener("mousemove", (evt: MouseEvent) => {
      const elem = document.elementFromPoint(evt.clientX, evt.clientY);

      if (isDown && currentElem) {
        const parts = currentElem.split("-");
        const isPathPoint = parts[0] === "point";
        const isShapePoint = parts[0] === "shape";

        // Skip dragging locked paths
        if (isPathPoint) {
          const hitLine = Number(parts[1]) - 1;
          if (hitLine >= 0 && lines[hitLine]?.locked) return;
        }

        // Use simple bounding rect math to match D3 scales which are bound to clientWidth/Height
        const rect = two.renderer.domElement.getBoundingClientRect();
        const xPos = evt.clientX - rect.left;
        const yPos = evt.clientY - rect.top;

        // Get current store values for reactivity
        const currentGridSize = $gridSize;
        const currentSnapToGrid = $snapToGrid;
        const currentShowGrid = $showGrid;

        // Apply drag offset (in inches) to the raw mouse position
        let rawInchX = x.invert(xPos) + dragOffset.x;
        let rawInchY = y.invert(yPos) + dragOffset.y;

        let inchX = rawInchX;
        let inchY = rawInchY;

        // Always apply grid snapping when enabled
        if (currentSnapToGrid && currentShowGrid && currentGridSize > 0) {
          // Force snap to nearest grid point
          inchX = Math.round(rawInchX / currentGridSize) * currentGridSize;
          inchY = Math.round(rawInchY / currentGridSize) * currentGridSize;

          // Clamp to field boundaries
          inchX = Math.max(0, Math.min(FIELD_SIZE, inchX));
          inchY = Math.max(0, Math.min(FIELD_SIZE, inchY));
        }

        // Handle path point dragging
        if (currentElem.startsWith("campick-")) {
          // The clump slides along its wall only: the camera pick reads its x.
          const lineIndex = Number(currentElem.split("-")[1]);
          const line = lines[lineIndex];
          if (!line?.cameraPick || line.locked) return;
          const next = [...lines];
          next[lineIndex] = {
            ...line,
            cameraPick: {
              ...line.cameraPick,
              previewX: Math.round(Math.max(0, Math.min(FIELD_SIZE, inchX)) * 100) / 100,
            },
          };
          lines = next;
        } else if (currentElem.startsWith("pollen-target-")) {
          const sequenceIndex = Number(currentElem.split("-")[2]);
          const item = sequence[sequenceIndex];
          if (!item || item.kind !== "pollen" || item.locked) return;
          const next = [...sequence];
          next[sequenceIndex] = {
            ...item,
            // A hundredth of an inch is finer than the camera can place a ball;
            // more digits are just noise in the X and Y fields.
            targetX: Math.round(Math.max(0, Math.min(FIELD_SIZE, inchX)) * 100) / 100,
            targetY: Math.round(Math.max(0, Math.min(FIELD_SIZE, inchY)) * 100) / 100,
          };
          sequence = next;
        } else if (currentElem.startsWith("obstacle-")) {
          // Handle obstacle vertex dragging
          const parts = currentElem.split("-");
          const shapeIdx = Number(parts[1]);
          const vertexIdx = Number(parts[2]);

          shapes[shapeIdx].vertices[vertexIdx].x = inchX;
          shapes[shapeIdx].vertices[vertexIdx].y = inchY;
          shapes = [...shapes];
        } else if (currentElem.startsWith("second-point-")) {
          // Handle second path point dragging
          const parts = currentElem.split("-");
          const line = Number(parts[2]) - 1;
          const point = Number(parts[3]);

          if (line === -1) {
            // This is the second starting point
            if (secondStartPoint?.locked) return;
            if (secondStartPoint) {
              secondStartPoint.x = inchX;
              secondStartPoint.y = inchY;
            }
          } else if (secondLines[line]) {
            if (point === 0 && secondLines[line].endPoint) {
              secondLines[line].endPoint.x = inchX;
              secondLines[line].endPoint.y = inchY;
            } else if (secondLines[line].controlPoints?.[point - 1]) {
              if (secondLines[line].locked) return;
              secondLines[line].controlPoints[point - 1].x = inchX;
              secondLines[line].controlPoints[point - 1].y = inchY;
            }
          }
          secondLines = [...secondLines];
        } else if (currentElem.startsWith("additional-path-")) {
          // Handle additional path point dragging
          const parts = currentElem.split("-");
          const pathIdx = Number(parts[2]);
          const line = Number(parts[4]) - 1;
          const point = Number(parts[5]);

          if (!additionalPaths[pathIdx]) return;

          if (line === -1) {
            // This is the starting point
            if (additionalPaths[pathIdx].startPoint) {
              additionalPaths[pathIdx].startPoint.x = inchX;
              additionalPaths[pathIdx].startPoint.y = inchY;
              additionalPaths = [...additionalPaths];
              // Auto-save changes to additional path files
              saveAdditionalPath(pathIdx).catch(err => console.error('Failed to auto-save additional path:', err));
            }
          } else if (additionalPaths[pathIdx].lines[line]) {
            if (point === 0 && additionalPaths[pathIdx].lines[line].endPoint) {
              additionalPaths[pathIdx].lines[line].endPoint.x = inchX;
              additionalPaths[pathIdx].lines[line].endPoint.y = inchY;
            } else if (additionalPaths[pathIdx].lines[line].controlPoints[point - 1]) {
              additionalPaths[pathIdx].lines[line].controlPoints[point - 1].x = inchX;
              additionalPaths[pathIdx].lines[line].controlPoints[point - 1].y = inchY;
            }
            additionalPaths = [...additionalPaths];
          }
          // Auto-save changes to additional path files
          saveAdditionalPath(pathIdx).catch(err => console.error('Failed to auto-save additional path:', err));
        } else {
          // Handle path point dragging
          const line = Number(currentElem.split("-")[1]) - 1;
          const point = Number(currentElem.split("-")[2]);

          if (line === -1) {
            // This is the starting point
            if (startPoint.locked || startPoint.poseVariableId) return;
            startPoint.x = inchX;
            startPoint.y = inchY;
          } else if (lines[line]) {
            if (point === 0 && lines[line].endPoint) {
              if (lines[line].endPoint.poseVariableId) return;
              lines[line].endPoint.x = inchX;
              lines[line].endPoint.y = inchY;
            } else if (lines[line].controlPoints?.[point - 1]) {
              // The control point can disappear mid-drag (undo, reload, a
              // removal); writing to it blindly would throw and stop the
              // whole editor from updating.
              if (lines[line].locked) return;
              lines[line].controlPoints[point - 1].x = inchX;
              lines[line].controlPoints[point - 1].y = inchY;
            }
          }
        }
      } else {
        if (
          (elem?.id.startsWith("point") && !isLockedPathElem(elem.id)) ||
          elem?.id.startsWith("second-point") ||
          elem?.id.startsWith("additional-path-") ||
          elem?.id.startsWith("obstacle") ||
          elem?.id.startsWith("pollen-target-") ||
          /^campick-\d+(-ball\d)?$/.test(elem?.id || "")
        ) {
          two.renderer.domElement.style.cursor = elem!.id.startsWith("campick-") ? "ew-resize" : "pointer";
          // The ball sits on top of its handle; both drag the same step.
          currentElem = elem!.id.replace(/-ball\d*$/, "");
        } else {
          two.renderer.domElement.style.cursor = "auto";
          currentElem = null;
        }
      }
    });

    two.renderer.domElement.addEventListener("mousedown", (evt: MouseEvent) => {
      if (currentElem && isLockedPathElem(currentElem)) {
        isDown = false;
        return;
      }

      isDown = true;

      if (currentElem) {
        const rect = two.renderer.domElement.getBoundingClientRect();
        const mouseX = x.invert(evt.clientX - rect.left);
        const mouseY = y.invert(evt.clientY - rect.top);

        let objectX = 0;
        let objectY = 0;

        if (currentElem.startsWith("campick-")) {
          const line = lines[Number(currentElem.split("-")[1])];
          if (line?.cameraPick) {
            objectX = line.cameraPick.previewX;
            objectY = mouseY;
          }
        } else if (currentElem.startsWith("pollen-target-")) {
          const item = sequence[Number(currentElem.split("-")[2])];
          if (item && item.kind === "pollen") {
            objectX = item.targetX;
            objectY = item.targetY;
          }
        } else if (currentElem.startsWith("obstacle-")) {
          const parts = currentElem.split("-");
          const shapeIdx = Number(parts[1]);
          const vertexIdx = Number(parts[2]);
          if (shapes[shapeIdx]?.vertices[vertexIdx]) {
            objectX = shapes[shapeIdx].vertices[vertexIdx].x;
            objectY = shapes[shapeIdx].vertices[vertexIdx].y;
          }
        } else if (currentElem.startsWith("second-point-")) {
          const parts = currentElem.split("-");
          const line = Number(parts[2]) - 1;
          const point = Number(parts[3]);

          if (line === -1) {
            if (secondStartPoint) {
              objectX = secondStartPoint.x;
              objectY = secondStartPoint.y;
            }
          } else if (secondLines[line]) {
            if (point === 0 && secondLines[line].endPoint) {
              objectX = secondLines[line].endPoint.x;
              objectY = secondLines[line].endPoint.y;
            } else if (secondLines[line].controlPoints[point - 1]) {
              objectX = secondLines[line].controlPoints[point - 1].x;
              objectY = secondLines[line].controlPoints[point - 1].y;
            }
          }
        } else if (currentElem.startsWith("additional-path-")) {
          const parts = currentElem.split("-");
          const pathIdx = Number(parts[2]);
          const line = Number(parts[4]) - 1;
          const point = Number(parts[5]);

          if (additionalPaths[pathIdx]) {
            if (line === -1) {
              // Starting point
              if (additionalPaths[pathIdx].startPoint) {
                objectX = additionalPaths[pathIdx].startPoint.x;
                objectY = additionalPaths[pathIdx].startPoint.y;
              }
            } else if (additionalPaths[pathIdx].lines[line]) {
              if (point === 0 && additionalPaths[pathIdx].lines[line].endPoint) {
                objectX = additionalPaths[pathIdx].lines[line].endPoint.x;
                objectY = additionalPaths[pathIdx].lines[line].endPoint.y;
              } else if (additionalPaths[pathIdx].lines[line].controlPoints[point - 1]) {
                objectX = additionalPaths[pathIdx].lines[line].controlPoints[point - 1].x;
                objectY = additionalPaths[pathIdx].lines[line].controlPoints[point - 1].y;
              }
            }
          }
        } else {
          const line = Number(currentElem.split("-")[1]) - 1;
          const point = Number(currentElem.split("-")[2]);

          if (line === -1) {
            objectX = startPoint.x;
            objectY = startPoint.y;
          } else if (lines[line]) {
            if (point === 0 && lines[line].endPoint) {
              objectX = lines[line].endPoint.x;
              objectY = lines[line].endPoint.y;
            } else if (lines[line].controlPoints[point - 1]) {
              objectX = lines[line].controlPoints[point - 1].x;
              objectY = lines[line].controlPoints[point - 1].y;
            }
          }
        }

        dragOffset = {
          x: objectX - mouseX,
          y: objectY - mouseY,
        };
      }
    });

    two.renderer.domElement.addEventListener("mouseup", () => {
      isDown = false;
      dragOffset = { x: 0, y: 0 };
      recordChange();
    });

    // Double-click on the field to create a new path at that position
    two.renderer.domElement.addEventListener("dblclick", (evt: MouseEvent) => {
      // Ignore dblclicks on existing points/lines
      const elem = document.elementFromPoint(evt.clientX, evt.clientY);
      if (
        elem?.id &&
        (elem.id.startsWith("point") ||
          elem.id.startsWith("obstacle") ||
          elem.id.startsWith("pollen-") ||
          elem.id.startsWith("campick-") ||
          elem.id.startsWith("line"))
      ) {
        return;
      }

      const rect = two.renderer.domElement.getBoundingClientRect();
      const rawInchX = x.invert(evt.clientX - rect.left);
      const rawInchY = y.invert(evt.clientY - rect.top);

      // Apply grid snapping if enabled
      const currentGridSize = $gridSize;
      const currentSnapToGrid = $snapToGrid;
      const currentShowGrid = $showGrid;

      let inchX = rawInchX;
      let inchY = rawInchY;

      if (currentSnapToGrid && currentShowGrid && currentGridSize > 0) {
        inchX = Math.round(rawInchX / currentGridSize) * currentGridSize;
        inchY = Math.round(rawInchY / currentGridSize) * currentGridSize;
      }

      // Clamp to field boundaries
      inchX = Math.max(0, Math.min(FIELD_SIZE, inchX));
      inchY = Math.max(0, Math.min(FIELD_SIZE, inchY));

      // Create a new line with endPoint at the clicked position
      const newLine: Line = {
        id: `line-${Math.random().toString(36).slice(2)}`,
        endPoint: {
          x: inchX,
          y: inchY,
          heading: "tangential",
          reverse: false,
        },
        controlPoints: [],
        color: getRandomColor(),
        speed: 1,
        locked: false,
        waitBeforeMs: 0,
        waitAfterMs: 0,
        waitBeforeName: "",
        waitAfterName: "",
      };

      lines = [...lines, newLine];
      sequence = [...sequence, { kind: "path", lineId: newLine.id! }];
      recordChange();
      two.update();
    });
  });
  document.addEventListener("keydown", function (evt) {
    if (evt.code === "Space" && document.activeElement === document.body) {
      if (playing) {
        pause();
      } else {
        play();
      }
    }
  });
  async function saveFile() {
    try {
      const content = JSON.stringify(
        {
          startPoint,
          lines,
          shapes,
          sequence,
          pathChains,
          variables,
          settings,
          version: "1.2.1",
          timestamp: new Date().toISOString(),
        },
        null,
        2,
      );

      if ($currentFilePath) {
        await browserFileStore.writeFile($currentFilePath, content);
        isUnsaved.set(false);
        // Provide simple feedback
        alert(`Saved to project storage: ${$currentFilePath}`);
      } else {
        // No current project file selected — save into browser cache as a new file
        const defaultName = `path_${Date.now()}.pp`;
        await browserFileStore.writeFile(defaultName, content);
        currentFilePath.set(defaultName);
        isUnsaved.set(false);
        alert(`Saved to project storage as: ${defaultName}`);
      }
    } catch (err) {
      console.error("Failed to save project to storage:", err);
      alert("Failed to save project to browser storage.");
    }
  }

  async function loadFile(evt: Event) {
    const elem = evt.target as HTMLInputElement;
    const file = elem.files?.[0];

    if (!file) return;

    // Check if file is a .pp file
    if (!file.name.endsWith(".pp")) {
      alert("Please select a .pp file");
      // Reset the file input
      elem.value = "";
      return;
    }

	    // Parse and load the uploaded file, then cache it into the browser store.
	    loadTrajectoryFromFile(evt, async (data) => {
	      const loadedVariables = normalizeVariables(data);

      // Ensure startPoint has all required fields
      const loadedStartPoint = resolvePointExpressions(data.startPoint || {
        x: 72,
        y: 72,
        heading: "tangential",
        reverse: false,
      }, loadedVariables);

      // Normalize lines with all required fields
	      const normalizedLines = applyVariablesToLines(
	        normalizeLines(data.lines || []),
	        loadedVariables,
	      );
      const normalizedPath = applyPoseVariablesToPath(
        loadedStartPoint,
        normalizedLines,
        poseVariablesOf(loadedVariables),
      );
      startPoint = normalizedPath.startPoint;
	      lines = normalizedPath.lines;
	      variables = loadedVariables;

      // Derive sequence from data or create default
	      sequence = applyVariablesToSequence(
	        (
	          data.sequence && data.sequence.length
	            ? data.sequence
	            : normalizedLines.map((ln) => ({
	                kind: "path",
	                lineId: ln.id!,
	              }))
	        ) as SequenceItem[],
	        loadedVariables,
	      );
      pathChains = normalizePathChains(data.pathChains, normalizedPath.lines);

      // Load shapes with defaults
      shapes = data.shapes || [];

      // Load settings (including robot size) if present
      if (data.settings) {
        settings = { ...settings, ...data.settings };
        robotWidth = settings.rWidth;
        robotHeight = settings.rHeight;
      }

      suppressUnsavedOnce = true;
      isUnsaved.set(false);
      recordChange();

      // Cache the uploaded file into the browser-backed store for later access
      try {
        const content = JSON.stringify(data);
        await browserFileStore.writeFile(file.name, content);
        currentFilePath.set(file.name);
      } catch (err) {
        console.warn("Failed to cache uploaded file to store:", err);
      }
    });

    // Reset the file input
    elem.value = "";
  }

  // Electron file-copying logic removed — browser store and upload are used instead.

  // Helper function to load data into app state
  /**
   * Loads a parsed project into the editor. The normalization lives in
   * `utils/projectFile` so the MCP reviewer reads a file exactly the way the
   * editor does; the editor keeps an empty obstacle list as the user left it,
   * which is the one difference and why the preset fallback is turned off here.
   */
  function loadData(data: any) {
    const project = loadProjectData(data, {
      baseSettings: settings,
      useFieldObstaclePreset: false,
    });

    startPoint = project.startPoint;
    lines = project.lines;
    variables = project.variables;
    sequence = project.sequence;
    pathChains = project.pathChains;
    shapes = project.shapes;

    if (data.settings) {
      settings = project.settings;
      robotWidth = settings.rWidth;
      robotHeight = settings.rHeight;
    }

    suppressUnsavedOnce = true;
    isUnsaved.set(false);
    recordChange();
  }

  function toHeadingDegrees(point: Point, position: "start" | "end"): number {
    if (!point) return 0;
    if (point.heading === "linear") {
      return position === "start" ? (point.startDeg ?? 0) : (point.endDeg ?? 0);
    }
    if (point.heading === "constant") {
      return point.degrees ?? 0;
    }
    return 0;
  }

  function buildOptimizationPayload(lineIndex: number) {
    const line = lines[lineIndex];
    if (!line) throw new Error("Line not found");

    // Optimize the path where the robot actually drives it: inside a repeat loop
    // or an `if` block that is not the previous entry in `lines`.
    const startPt =
      lineStartPoints.get(line.id || "") ??
      (lineIndex === 0 ? startPoint : lines[lineIndex - 1]?.endPoint);
    if (!startPt) throw new Error("Missing start point for optimization");

    const waypoints = [startPt, ...line.controlPoints, line.endPoint].map(
      (p) => [p.x, p.y],
    );

    return {
      waypoints,
      start_heading_degrees: toHeadingDegrees(startPt, "start"),
      end_heading_degrees: toHeadingDegrees(line.endPoint, "end"),
      x_velocity: settings.xVelocity,
      y_velocity: settings.yVelocity,
      angular_velocity: settings.aVelocity,
      friction_coefficient: settings.kFriction,
      robot_width: settings.rWidth,
      robot_height: settings.rHeight,
      min_coord_field: 0,
      max_coord_field: FIELD_SIZE,
      interpolation:
        line.endPoint.heading === "tangential"
          ? "tangent"
          : line.endPoint.heading === "constant"
            ? "constant"
            : "linear",
      obstacles: shapes.map((shape) => shape.vertices.map((v) => [v.x, v.y])),
    };
  }

  function sleep(ms: number) {
    return new Promise((res) => setTimeout(res, ms));
  }

  function formatOptimizerError(
    status: number,
    errorText: string,
    fallbackStatusText: string,
  ): string {
    let message = errorText || fallbackStatusText;
    let trace = "";

    try {
      const parsed = JSON.parse(errorText);
      message = parsed?.message || parsed?.error || message;
      trace = parsed?.trace || "";
    } catch {
      // The hosted optimizer sometimes returns plain text instead of JSON.
    }

    if (`${message}\n${trace}`.includes("min_collisions")) {
      return `Optimizer request failed (${status}): Pedro's hosted optimizer hit its collision-count bug. The path was not changed; try again after removing collision obstacles or simplifying the path.`;
    }

    return `Optimizer request failed (${status}): ${message || fallbackStatusText}`;
  }

  async function runOptimization(payload: any) {
    const response = await fetch(`${OPTIMIZER_BASE_URL}`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload),
    });

    if (!response.ok) {
      const errorText = await response.text().catch(() => "");
      throw new Error(
        formatOptimizerError(response.status, errorText, response.statusText),
      );
    }

    const data = await response.json();
    if (data?.status === "completed" && data.result) {
      return data.result;
    }
    if (data?.status === "error") {
      throw new Error(
        `Optimization failed: ${data.message || "Unknown error"}`,
      );
    }
    throw new Error("Unexpected API response format");
  }

  async function optimizeLine(
    lineId: string,
    targetControlPointIndex?: number,
  ) {
    const lineIndex = lines.findIndex((l) => l.id === lineId);
    if (lineIndex === -1) {
      alert("Could not find line to optimize.");
      return;
    }

    if (optimizingLineIds[lineId]) return;
    optimizingLineIds = { ...optimizingLineIds, [lineId]: true };

    try {
      const payload = buildOptimizationPayload(lineIndex);
      const result = await runOptimization(payload);

      const optimizedWaypoints = Array.isArray(result?.optimized_waypoints)
        ? result.optimized_waypoints
        : Array.isArray(result)
          ? result
          : null;

      if (!optimizedWaypoints || optimizedWaypoints.length < 2) {
        throw new Error("Unexpected optimizer response format.");
      }

      const interior = optimizedWaypoints
        .slice(1, optimizedWaypoints.length - 1)
        .map((p: number[]) => ({ x: p[0], y: p[1] }));

      const newLines = [...lines];
      const current = newLines[lineIndex];

      if (typeof targetControlPointIndex === "number") {
        // Only replace the targeted control point; keep others and endpoint untouched
        const replacement =
          interior[targetControlPointIndex] ?? interior[interior.length - 1];
        if (replacement) {
          const cps = [...current.controlPoints];
          if (cps[targetControlPointIndex]) {
            cps[targetControlPointIndex] = replacement;
            newLines[lineIndex] = {
              ...current,
              controlPoints: cps,
            };
            lines = normalizeLines(newLines);
            recordChange();
          }
        }
      } else {
        // Replace entire line (control points and endpoint)
        newLines[lineIndex] = {
          ...current,
          endPoint: {
            ...current.endPoint,
            x: optimizedWaypoints[optimizedWaypoints.length - 1][0],
            y: optimizedWaypoints[optimizedWaypoints.length - 1][1],
          },
          controlPoints: interior,
        };
        lines = normalizeLines(newLines);
        recordChange();
      }
    } catch (err) {
      console.error(err);
      alert((err as Error).message || "Optimization failed.");
    } finally {
      optimizingLineIds = { ...optimizingLineIds, [lineId]: false };
    }
  }

  async function optimizeAllLines() {
    if (optimizingAll) return;
    optimizingAll = true;
    try {
      for (const ln of lines) {
        if (!ln?.id) continue;
        await optimizeLine(ln.id);
      }
    } finally {
      optimizingAll = false;
    }
  }

  function addNewLine() {
    lines = [
      ...lines,
      {
        id: `line-${Math.random().toString(36).slice(2)}`,
        endPoint: {
          x: _.random(36, 108),
          y: _.random(36, 108),
          heading: "tangential",
          reverse: true,
        } as Point,
        controlPoints: [],
        color: getRandomColor(),
        speed: 1,
        locked: false,
        waitBeforeMs: 0,
        waitAfterMs: 0,
        waitBeforeName: "",
        waitAfterName: "",
      },
    ];
    sequence = [
      ...sequence,
      { kind: "path", lineId: lines[lines.length - 1].id! },
    ];
    recordChange();
  }

  function addControlPoint() {
    if (lines.length > 0) {
      const lastLine = lines[lines.length - 1];
      lastLine.controlPoints.push({
        x: _.random(36, 108),
        y: _.random(36, 108),
      });
      recordChange();
    }
  }

  function removeControlPoint() {
    if (lines.length > 0) {
      const lastLine = lines[lines.length - 1];
      if (lastLine.controlPoints.length > 0) {
        lastLine.controlPoints.pop();
        recordChange();
      }
    }
  }

  // Keyboard shortcuts for quick path editing
  hotkeys("w", function (event, handler) {
    event.preventDefault();
    addNewLine();
  });
  hotkeys("a", function (event, handler) {
    event.preventDefault();
    addControlPoint();
    two.update();
  });
  hotkeys("s", function (event, handler) {
    event.preventDefault();
    removeControlPoint();
    two.update();
  });
  hotkeys("cmd+z, ctrl+z", function (event) {
    event.preventDefault();
    undoAction();
  });
  hotkeys("cmd+shift+z, ctrl+shift+z, ctrl+y", function (event) {
    event.preventDefault();
    redoAction();
  });
  function applyTheme(theme: "light" | "dark" | "auto") {
    let actualTheme = theme;
    if (theme === "auto") {
      // Check system preference
      if (
        window.matchMedia &&
        window.matchMedia("(prefers-color-scheme: dark)").matches
      ) {
        actualTheme = "dark";
      } else {
        actualTheme = "light";
      }
    }

    if (actualTheme === "dark") {
      document.documentElement.classList.add("dark");
    } else {
      document.documentElement.classList.remove("dark");
    }
  }

  // Watch for theme changes in settings
  $: if (settings) {
    applyTheme(settings.theme);
  }

  // Watch for system theme changes if auto mode is enabled
  let mediaQuery: MediaQueryList;
  onMount(() => {
    if (settings?.theme === "auto") {
      mediaQuery = window.matchMedia("(prefers-color-scheme: dark)");
      const handleSystemThemeChange = () => {
        if (settings.theme === "auto") {
          applyTheme("auto");
        }
      };
      mediaQuery.addEventListener("change", handleSystemThemeChange);

      return () => {
        mediaQuery.removeEventListener("change", handleSystemThemeChange);
      };
    }
  });

  // Auto-export for CI/testing: if the app is loaded with URL hash #export-gif-test, automatically run GIF export once mounted
  onMount(() => {
    if (
      typeof window !== "undefined" &&
      window.location &&
      window.location.hash === "#export-gif-test"
    ) {
      // Delay slightly to allow initial rendering and Two.js to initialize
      setTimeout(async () => {
        try {
          // auto GIF export removed (exportGif deleted)
          console.log("Auto GIF export skipped (exportGif removed)");
        } catch (err) {
          console.error("Auto GIF export failed:", err);
        }
      }, 1500);
    }

    // Handle save dialog event
    const handleSaveDialog = async (event: any) => {
      const { fileName } = event.detail;
      isSaving = true;
      try {
        // Create a full file path with .pp extension if not present
        const fullFileName = fileName.endsWith(".pp") ? fileName : fileName + ".pp";
        
        // Call the file manager's save function through the browser file store
        const fileData = JSON.stringify({
          startPoint,
          lines,
          shapes,
          sequence,
          pathChains,
          variables,
          settings,
        });
        
        await browserFileStore.writeFile(fullFileName, fileData);
        currentFilePath.set(fullFileName);
        isUnsaved.set(false);
        
        // Show success feedback
        showSaveDialog = false;
      } catch (error) {
        console.error("Save failed:", error);
        alert("Failed to save file: " + (error instanceof Error ? error.message : String(error)));
      } finally {
        isSaving = false;
      }
    };

    window.addEventListener("save", handleSaveDialog);

    // Handle dual path save dialog event
    const handleDualPathSave = async (event: any) => {
      const { target } = event.detail;
      isSaving = true;
      try {
        if (target === "first" && $currentFilePath) {
          const fileData = JSON.stringify({
            startPoint,
            lines,
            shapes,
            sequence,
            pathChains,
            variables,
            settings,
            version: "1.2.1",
            timestamp: new Date().toISOString(),
          });
          await browserFileStore.writeFile($currentFilePath, fileData);
          isUnsaved.set(false);
        } else if (target === "second" && $secondFilePath) {
          const fileData = JSON.stringify({
            startPoint: secondStartPoint,
            lines: secondLines,
            shapes: secondShapes,
            sequence: secondSequence,
            settings,
            version: "1.2.1",
            timestamp: new Date().toISOString(),
          });
          await browserFileStore.writeFile($secondFilePath, fileData);
        } else if (target === "both") {
          // Save first path
          if ($currentFilePath) {
            const fileData1 = JSON.stringify({
              startPoint,
              lines,
              shapes,
              sequence,
              pathChains,
              variables,
              settings,
              version: "1.2.1",
              timestamp: new Date().toISOString(),
            });
            await browserFileStore.writeFile($currentFilePath, fileData1);
          }
          // Save second path
          if ($secondFilePath) {
            const fileData2 = JSON.stringify({
              startPoint: secondStartPoint,
              lines: secondLines,
              shapes: secondShapes,
              sequence: secondSequence,
              settings,
              version: "1.2.1",
              timestamp: new Date().toISOString(),
            });
            await browserFileStore.writeFile($secondFilePath, fileData2);
          }
          isUnsaved.set(false);
        }
        showDualPathSaveDialog = false;
      } catch (error) {
        console.error("Dual path save failed:", error);
        alert("Failed to save: " + (error instanceof Error ? error.message : String(error)));
      } finally {
        isSaving = false;
      }
    };

    window.addEventListener("saveDualPath", handleDualPathSave);

    return () => {
      window.removeEventListener("save", handleSaveDialog);
      window.removeEventListener("saveDualPath", handleDualPathSave);
    };
  });
</script>

<svelte:window
  on:mousemove={(e) => {
    // (debug window dragging removed)
  }}
  on:mouseup={() => {}}
/>

<Navbar
  {clearanceReport}
  bind:lines
  bind:startPoint
  bind:shapes
  bind:sequence
  bind:pathChains
  bind:variables
  bind:secondStartPoint
  bind:secondLines
  bind:secondShapes
  bind:secondSequence
  bind:settings
  bind:robotWidth
  bind:robotHeight
  {percent}
  {saveProject}
  {saveFileAs}
  {loadFile}
  {undoAction}
  {redoAction}
  {recordChange}
  {canUndo}
  {canRedo}
  {optimizeAllLines}
  {optimizingAll}
  {twoElement}
  {exportPathAsGif}
/>

<SaveDialog
  bind:isOpen={showSaveDialog}
  bind:isSaving
  fileName={$currentFilePath?.split(/[\\/]/).pop()?.replace(/\.pp$/, "") || "my_path"}
/>

<DualPathSaveDialog bind:isOpen={showDualPathSaveDialog} />

<ProgressDialog
  bind:isOpen={exportingGif}
  progress={gifExportProgress}
  statusMessage={gifExportStatus}
  onCancel={() => {
    cancelGifExport = true;
    gifExportStatus = "Cancelling...";
  }}
/>

<!--   {saveFile} -->
<div
  class="w-screen h-screen pt-20 p-2 flex flex-row justify-center items-center gap-2"
>
  <div class="flex h-full shrink-0 justify-center items-center">
    <div
      bind:this={twoElement}
      bind:clientWidth={width}
      bind:clientHeight={height}
      class="h-full aspect-square rounded-lg shadow-md bg-neutral-50 dark:bg-neutral-900 relative overflow-clip"
      role="application"
      style="
    user-select: none;
    -webkit-user-select: none;
    -moz-user-select: none;
    -ms-user-select: none;
    -webkit-touch-callout: none;
    -webkit-tap-highlight-color: transparent;
    user-drag: none;
    -webkit-user-drag: none;
    -khtml-user-drag: none;
    -moz-user-drag: none;
    -ms-user-drag: none;
    -o-user-drag: none;
  "
      on:contextmenu={(e) => e.preventDefault()}
      on:dragstart={(e) => e.preventDefault()}
      on:selectstart={(e) => e.preventDefault()}
      tabindex="-1"
    >
      <img
        src={fieldMapSrc}
        alt="Field"
        class="absolute top-0 left-0 w-full h-full rounded-lg z-10"
        style="
    background: transparent; 
    pointer-events: none; 
    user-select: none; 
    -webkit-user-select: none;
    -moz-user-select: none;
    -ms-user-select: none;
    -webkit-touch-callout: none;
    -webkit-tap-highlight-color: transparent;
    user-drag: none;
    -webkit-user-drag: none;
    -moz-user-drag: none;
    -ms-user-drag: none;
    -o-user-drag: none;
  "
        draggable="false"
        on:error={(e) => {
          console.error("Failed to load field map:", settings.fieldMap);
          setImageFallback(e, `/fields/${DEFAULT_SETTINGS.fieldMap}`);
        }}
        on:dragstart={(e) => e.preventDefault()}
        on:selectstart={(e) => e.preventDefault()}
      />
      <MathTools {x} {y} {twoElement} {robotXY} />
      {#if settings.showAutoCountdown && activeAutoTotalTime > 0}
        <div
          class="absolute top-3 left-1/2 -translate-x-1/2 z-[45] pointer-events-none w-[min(78%,28rem)]"
        >
          <div
            class="rounded-md border border-black/15 dark:border-white/15 bg-white/90 dark:bg-neutral-950/85 shadow-sm px-3 py-2"
            style={`--countdown-color: ${autoCountdownColor};`}
          >
            <div class="flex items-center justify-between gap-3 text-xs font-semibold text-neutral-800 dark:text-neutral-100">
              <span>Auto</span>
              <span style="color: var(--countdown-color);">
                {formatCountdown(autoCountdownRemaining)}
              </span>
            </div>
            <div class="mt-1 h-1.5 overflow-hidden rounded-full bg-neutral-200 dark:bg-neutral-800">
              <div
                class="h-full rounded-full"
                style={`width: ${Math.max(3, autoCountdownProgress * 100)}%; background: var(--countdown-color);`}
              />
            </div>
          </div>
        </div>
      {/if}

      {#each pathAnnotations as annotation (annotation.id)}
        <div
          class="absolute z-[42] pointer-events-none -translate-x-1/2 -translate-y-1/2 whitespace-nowrap rounded border border-black/10 dark:border-white/15 bg-white/90 dark:bg-neutral-950/85 px-2 py-0.5 text-[11px] font-semibold text-neutral-800 dark:text-neutral-100 shadow-sm"
          style={`left: ${annotation.x}px; top: ${annotation.y}px; border-left: 3px solid ${annotation.color || "#2563eb"};`}
        >
          {annotation.label}
        </div>
      {/each}

      <!--
        Where the robot actually comes to a full stop. Everything else is driven
        straight through as one PathChain.
      -->
      {#each stopPoints as stop (stop.id)}
        <div
          class="absolute z-[42] pointer-events-none -translate-x-1/2 -translate-y-1/2"
          style={`left: ${stop.x}px; top: ${stop.y}px;`}
          title={`Full stop: ${stop.label}`}
        >
          <div
            class="h-3.5 w-3.5 rounded-full border-2 border-amber-500 bg-amber-100/70 dark:bg-amber-950/70 shadow-sm"
          />
        </div>
      {/each}

      {#each eventPins as marker (marker.id)}
        <div
          class="absolute z-[43] pointer-events-none flex -translate-x-1/2 -translate-y-full flex-col items-center"
          style={`left: ${marker.x}px; top: ${marker.y}px; color: ${marker.color || "#a855f7"};`}
        >
          <div
            class="rounded-md border border-current bg-white/95 dark:bg-neutral-950/90 px-1.5 py-0.5 text-[11px] font-semibold leading-tight text-neutral-900 dark:text-neutral-50 shadow-sm"
          >
            {marker.label}
          </div>
          <div
            class="h-3 w-3 rotate-45 border-b border-r border-current bg-white/95 dark:bg-neutral-950/90 -mt-1"
          />
          <div class="h-2 w-2 rounded-full bg-current shadow-sm" />
        </div>
      {/each}

      {#each eventTimelineLabels as marker (marker.id)}
        <div
          class="absolute z-[43] pointer-events-none -translate-x-1/2 translate-y-2 whitespace-nowrap rounded-full border bg-white/95 dark:bg-neutral-950/90 px-2 py-0.5 text-[10px] font-bold leading-tight text-neutral-900 dark:text-neutral-50 shadow-sm"
          style={`left: ${marker.x}px; top: ${marker.y}px; border-color: ${marker.color || "#f97316"}; box-shadow: 0 0 0 2px ${marker.color || "#f97316"}22;`}
        >
          {marker.label}
        </div>
      {/each}

      {#each swerveWheelOverlays as module (module.id)}
        <div
          class="absolute z-[44] pointer-events-none rounded-full border border-black/40 dark:border-white/50 shadow-sm"
          style={`left: ${module.x}px; top: ${module.y}px; width: ${module.length}px; height: ${module.thickness}px; margin-left: ${-module.length / 2}px; margin-top: ${-module.thickness / 2}px; transform: rotate(${module.angle}deg); opacity: ${module.opacity}; background: ${module.color};`}
        />
      {/each}

      <!-- Main robot: only show in normal mode -->
      {#if $activePaths.length === 0}
        <img
          src={settings.robotImage || "/robot.png"}
          alt="Robot"
          style={`position: absolute; top: ${robotXY.y}px;
left: ${robotXY.x}px; transform: translate(-50%, -50%) translate(${shotRecoil.x.toFixed(2)}px, ${shotRecoil.y.toFixed(2)}px) rotate(${robotHeading}deg) ${robotSpriteMotion.transform}; z-index: 20; width: ${x(robotWidth)}px; height: ${x(robotHeight)}px;user-select: none; -webkit-user-select: none; -moz-user-select: none;-ms-user-select: none;
pointer-events: none;`}
          draggable="false"
          on:error={(e) => {
            console.error("Failed to load robot image:", settings.robotImage);
            setImageFallback(e, "/robot.png");
          }}
          on:dragstart={(e) => e.preventDefault()}
          on:selectstart={(e) => e.preventDefault()}
        />
        <RobotVfx
          x={robotXY.x}
          y={robotXY.y}
          heading={robotHeading}
          lengthPx={x(robotWidth)}
          widthPx={x(robotHeight)}
          vfx={activeVfx}
          lift={robotSpriteMotion.lift}
          shot={robotShot?.frame ?? null}
          ballPx={(robotShot?.pxPerInch ?? 0) * NECTAR_DIAMETER_INCHES}
          color={robotShot?.color ?? "#facc15"}
        />
        <MechanismOverlay
          x={robotXY.x}
          y={robotXY.y}
          heading={robotHeading}
          lengthPx={x(robotWidth)}
          widthPx={x(robotHeight)}
          pxPerInch={Math.abs(x(1) - x(0))}
          state={mechanismState}
          turret={mechanismAim.turret}
          aim={mechanismAim.aim}
        />
        <!-- Heading arrow for main robot -->
        {#if settings.showHeadingArrow}
          <svg
            style={`position: absolute; top: ${robotXY.y}px; left: ${robotXY.x}px; z-index: 21; pointer-events: none; overflow: visible;`}
            width="1"
            height="1"
          >
            <defs>
              <marker
                id="arrowhead-main"
                markerWidth="10"
                markerHeight="10"
                refX="6.5"
                refY="3"
                orient="auto"
              >
                <polygon
                  points="0 0, 7 3, 0 6"
                  fill={settings.headingArrowColor || "#ffffff"}
                />
              </marker>
            </defs>
            <line
              x1="0"
              y1="0"
              x2="{(settings.headingArrowLength || 50) * Math.cos(-robotHeading * Math.PI / 180)}"
              y2="{(settings.headingArrowLength || 50) * -Math.sin(-robotHeading * Math.PI / 180)}"
              stroke={settings.headingArrowColor || "#ffffff"}
              stroke-width={settings.headingArrowThickness || 3}
              marker-end="url(#arrowhead-main)"
            />
          </svg>
        {/if}
      {/if}
      <!-- Second robot: only show in dual path mode (not multi-path mode) -->
      {#if $activePaths.length === 0 && $dualPathMode}
        <img
          src={settings.robotImage || "/robot.png"}
          alt="Robot 2"
          style={`position: absolute; top: ${secondRobotXY.y}px;
left: ${secondRobotXY.x}px; transform: translate(-50%, -50%) rotate(${secondRobotHeading}deg); z-index: 19; width: ${x(robotWidth)}px; height: ${x(robotHeight)}px;user-select: none; -webkit-user-select: none; -moz-user-select: none;-ms-user-select: none;
pointer-events: none; opacity: 0.8;`}
          draggable="false"
          on:error={(e) => {
            console.error("Failed to load robot image:", settings.robotImage);
            setImageFallback(e, "/robot.png");
          }}
          on:dragstart={(e) => e.preventDefault()}
          on:selectstart={(e) => e.preventDefault()}
        />
        <!-- Heading arrow for second robot -->
        {#if settings.showHeadingArrow}
          <svg
            style={`position: absolute; top: ${secondRobotXY.y}px; left: ${secondRobotXY.x}px; z-index: 19; pointer-events: none; overflow: visible; opacity: 0.8;`}
            width="1"
            height="1"
          >
            <defs>
              <marker
                id="arrowhead-second"
                markerWidth="10"
                markerHeight="10"
                refX="6.5"
                refY="3"
                orient="auto"
              >
                <polygon
                  points="0 0, 7 3, 0 6"
                  fill={settings.headingArrowColor || "#ffffff"}
                />
              </marker>
            </defs>
            <line
              x1="0"
              y1="0"
              x2="{(settings.headingArrowLength || 50) * Math.cos(-secondRobotHeading * Math.PI / 180)}"
              y2="{(settings.headingArrowLength || 50) * -Math.sin(-secondRobotHeading * Math.PI / 180)}"
              stroke={settings.headingArrowColor || "#ffffff"}
              stroke-width={settings.headingArrowThickness || 3}
              marker-end="url(#arrowhead-second)"
            />
          </svg>
        {/if}
      {/if}
      <!-- Additional robots: only show in multi-path mode -->
      {#if $activePaths.length > 0}
        {#each additionalRobotStates as robotState, idx}
          <img
            src={settings.robotImage || "/robot.png"}
            alt="Robot {idx + 1}"
            style={`position: absolute; top: ${robotState.xy.y}px;
left: ${robotState.xy.x}px; transform: translate(-50%, -50%) rotate(${robotState.heading}deg); z-index: ${20 - idx}; width: ${x(robotWidth)}px; height: ${x(robotHeight)}px;user-select: none; -webkit-user-select: none; -moz-user-select: none;-ms-user-select: none;
pointer-events: none; opacity: ${1.0 - idx * 0.15};`}
            draggable="false"
            on:error={(e) => {
              console.error("Failed to load robot image:", settings.robotImage);
              setImageFallback(e, "/robot.png");
            }}
            on:dragstart={(e) => e.preventDefault()}
            on:selectstart={(e) => e.preventDefault()}
          />
          <!-- Heading arrow for additional robots -->
          {#if settings.showHeadingArrow}
            <svg
              style={`position: absolute; top: ${robotState.xy.y}px; left: ${robotState.xy.x}px; z-index: ${20 - idx}; pointer-events: none; overflow: visible; opacity: ${1.0 - idx * 0.15};`}
              width="1"
              height="1"
            >
              <defs>
                <marker
                  id="arrowhead-{idx}"
                  markerWidth="10"
                  markerHeight="10"
                  refX="6.5"
                  refY="3"
                  orient="auto"
                >
                  <polygon
                    points="0 0, 7 3, 0 6"
                    fill={settings.headingArrowColor || "#ffffff"}
                  />
                </marker>
              </defs>
              <line
                x1="0"
                y1="0"
                x2="{(settings.headingArrowLength || 50) * Math.cos(-robotState.heading * Math.PI / 180)}"
                y2="{(settings.headingArrowLength || 50) * -Math.sin(-robotState.heading * Math.PI / 180)}"
                stroke={settings.headingArrowColor || "#ffffff"}
                stroke-width={settings.headingArrowThickness || 3}
                marker-end="url(#arrowhead-{idx})"
              />
            </svg>
          {/if}
        {/each}
      {/if}
    </div>
  </div>
  <ControlTab
    {pollenPlans}
    bind:playing
    {play}
    {pause}
    bind:startPoint
    bind:lines
    bind:sequence
    bind:pathChains
    bind:variables
    bind:robotWidth
    bind:robotHeight
    bind:settings
    bind:percent
    bind:robotXY
    bind:robotHeading
    bind:shapes
    {x}
    {y}
    {handleSeek}
    bind:loopAnimation
    {recordChange}
    {optimizeLine}
    {optimizingLineIds}
    {clearanceReport}
  />
</div>

<style>
  /*
   * Pollen Pickup overlays are pictures, not handles: the camera view, the zone
   * and the legs cover big areas and must never take a click meant for a path
   * point, an obstacle vertex, or a target marker underneath them.
   */
  :global([id^="pollen-view-"]),
  :global([id^="pollen-reliable-"]),
  :global([id^="pollen-zone-"]),
  :global([id^="pollen-leg-"]),
  :global([id^="pollen-standoff-"]) {
    pointer-events: none;
  }
</style>
