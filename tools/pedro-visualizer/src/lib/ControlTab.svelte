<script lang="ts">
  import { expandRouteEdits } from "../utils/cameraPick";
  import type {
    Point,
    Line,
    BasePoint,
    ControlPoint,
    Settings,
    Shape,
    SequenceItem,
    PathChain,
    PathVariable,
    SequenceConditionalItem,
    SequenceGroupItem,
    SequenceGroupMember,
    SequenceWaitItem,
    SequenceEventItem,
    PoseVariable,
    Variable,
    VariableType,
  } from "../types";
  import _ from "lodash";
  import { tick } from "svelte";
  import { FIELD_SIZE } from "../config/defaults";
  import {
    buildChainRuns,
    buildExpressionScope,
    buildRoute,
    calculatePathTime,
    canMoveEndpoint,
    CHAIN_BREAK_LABELS,
    checkClearance,
    EMPTY_CLEARANCE_REPORT,
    footprintFromSettings,
    getAngularDifference,
    getLineStartHeading,
    buildEventTimingWindows,
    createVariable,
    expressionDisplayValue,
    getRandomColor,
    groupHoldingLine,
    groupLineIds,
    groupMembers,
    isConditionalActive,
    isGroupItem,
    mirrorPathData,
    groupHoldingMember,
    memberKey,
    moveItemIntoGroup,
    moveItemOutOfGroups,
    poseVariablesOf,
    skippedLineIds,
    resolveLineExpressions,
    resolvePointExpressions,
    resolveSequenceExpressions,
    resolveSequenceItemExpressions,
    resolveVariableValues,
    clearanceTargetName,
    describeClearanceSpan,
    TOUCH_TOLERANCE,
    FIX_HANDLE_LABELS,
    midlineRuleFromSettings,
    suggestClearanceFix,
    suggestPoseVariableFix,
    suggestStartPointFix,
    uniqueVariableName,
    type ClearanceFix,
    type ClearanceReport,
    type MirrorAxis,
  } from "../utils";
  import ObstaclesSection from "./components/ObstaclesSection.svelte";
  import TelemetryPanel from "./components/TelemetryPanel.svelte";
  import StatesSection from "./components/StatesSection.svelte";
  import StartingPointSection from "./components/StartingPointSection.svelte";
  import PathLineSection from "./components/PathLineSection.svelte";
  import VariablesSection from "./components/VariablesSection.svelte";
  import ExpressionInput from "./components/ExpressionInput.svelte";
  import PlaybackControls from "./components/PlaybackControls.svelte";
  import WaitRow from "./components/WaitRow.svelte";
  import PollenPickupRow from "./components/PollenPickupRow.svelte";
  import {
    makePollenItem,
    reliableRangeInches,
    robotToField,
    visionOf,
    type PollenStepPlan,
  } from "../utils/pollenVision";
  import type { SequencePollenItem } from "../types";
  import { getLineEndHeading } from "../utils/math";

  export let percent: number;
  export let playing: boolean;
  export let play: () => any;
  export let pause: () => any;
  export let startPoint: Point;
  export let lines: Line[];
  export let variables: Variable[] = [];
  export let sequence: SequenceItem[];
  export let pathChains: PathChain[] = [];
  export let robotWidth: number = 16;
  export let robotHeight: number = 16;
  export let robotXY: BasePoint;
  export let robotHeading: number;
  export let x: d3.ScaleLinear<number, number, number>;
  export let y: d3.ScaleLinear<number, number, number>;
  export let settings: Settings;
  export let handleSeek: (percent: number) => void;
  export let loopAnimation: boolean;
  export let optimizeLine: (lineId: string, targetControlPointIndex?: number) => void;
  export let optimizingLineIds: Record<string, boolean> = {};

  export let shapes: Shape[];
  export let recordChange: () => void;
  /**
   * Where the robot's body comes too close to an obstacle or a wall. Computed
   * once alongside the field drawing and passed in, so the chips on the rows and
   * the shapes on the field can never disagree.
   */
  export let clearanceReport: ClearanceReport = EMPTY_CLEARANCE_REPORT;
  /** Every Pollen Pickup the route runs, planned by App from the timeline. */
  export let pollenPlans: Map<string, PollenStepPlan> = new Map();

  // Pose variables still drive the point-binding dropdowns, so keep a
  // derived view of them rather than a second source of truth.
  $: poseVariables = poseVariablesOf(variables);

  const makeChainId = () =>
    `${Date.now().toString(36)}-${Math.random().toString(36).slice(2, 8)}`;
  const defaultChainName = "Main Chain";

  let selectedChainId = "";
  let selectedChain: PathChain | null = null;
  /**
   * The member being dragged, by key: a path's line id, or a wait's or event's
   * own id. Groups hold all three, so the drag cannot be about paths alone.
   */
  let draggedLineId = "";
  let dragOverRepeatId = "";
  let dragOverTopLevel = false;
  let routeScrollEl: HTMLElement | null = null;
  let dragPointerX = 0;
  let dragPointerY = 0;
  let autoScrollFrame = 0;

  // Which block the dragged path came from; empty when it is already top level.
  $: draggedFromGroupId = draggedLineId
    ? groupHoldingMember(sequence, draggedLineId)?.id || ""
    : "";
  $: showDragOutZone = Boolean(draggedLineId && draggedFromGroupId);

  const getChainById = (chainId: string): PathChain | null =>
    pathChains.find((chain) => chain.id === chainId) || null;

  function getLinePrimaryChainId(lineId: string): string {
    for (const chain of pathChains) {
      if ((chain.lineIds || []).includes(lineId)) return chain.id;
    }
    return pathChains[0]?.id || "";
  }

  /**
   * Chains are background bookkeeping now, so they no longer own path colour.
   * Kept as a no-op hook because grouping still runs behind the scenes.
   */
  function syncLineColorsToChains() {
    let changed = false;
    const nextLines = lines.map((line) => {
      if (!line.color) {
        changed = true;
        return { ...line, color: getRandomColor() };
      }
      return line;
    });
    if (changed) {
      lines = nextLines;
    }
  }

  function ensureDefaultChain() {
    if (pathChains.length === 0) {
      pathChains = [
        {
          id: makeChainId(),
          name: defaultChainName,
          color: getRandomColor(),
          lineIds: lines.map((ln) => ln.id!).filter(Boolean),
        },
      ];
      selectedChainId = pathChains[0]?.id || "";
      return;
    }

    if (!selectedChainId || !pathChains.some((c) => c.id === selectedChainId)) {
      selectedChainId = pathChains[0]?.id || "";
    }
  }

  $: {
    const normalized = pathChains.map((chain) => ({
      ...chain,
      color: chain.color || getRandomColor(),
      lineIds: chain.lineIds || [],
    }));
    if (JSON.stringify(normalized) !== JSON.stringify(pathChains)) {
      pathChains = normalized;
    }
  }

  $: ensureDefaultChain();

  $: selectedChain =
    pathChains.find((chain) => chain.id === selectedChainId) || pathChains[0] || null;


  function ensureLineInDefaultChain(lineId: string) {
    if (!lineId || !pathChains.length) return;
    assignLineToChain(lineId, pathChains[0].id);
  }

  function removeLineFromChains(lineId: string) {
    if (!lineId) return;
    const updated = pathChains.map((chain) => ({
        ...chain,
        lineIds: chain.lineIds.filter((id) => id !== lineId),
      }));
    pathChains = updated;
    ensureDefaultChain();
    syncLineColorsToChains();
  }

  function assignLineToChain(lineId: string, chainId: string) {
    if (!lineId || !chainId) return;
    pathChains = pathChains.map((chain) => ({
      ...chain,
      lineIds: chain.lineIds.filter((id) => id !== lineId),
    }));

    const target = getChainById(chainId);
    if (!target) return;

    pathChains = pathChains.map((chain) => {
      if (chain.id !== chainId) return chain;
      return {
        ...chain,
        lineIds: Array.from(new Set([...(chain.lineIds || []), lineId])),
      };
    });

    syncLineColorsToChains();
    recordChange?.();
  }

  // Paths individually switched off by their own "Enabled if" condition.
  $: inactiveLineIds = skippedLineIds(lines, variables);

  /**
   * Consecutive paths are followed as one PathChain, which the robot drives
   * without stopping. Each path row reports whether the robot carries on
   * through its endpoint or comes to a stop there, and why — stops are what
   * cost time now, so they are the thing worth seeing at a glance.
   */
  /** One walk of the step list, shared by everything here that needs the route. */
  $: mainRoute = buildRoute(startPoint, lines, sequence, variables);
  $: chainRuns = buildChainRuns(mainRoute.steps);
  /**
   * Where a path's heading goal does not pick up where the previous one left
   * off. The robot catches up on the move rather than snapping, but a large
   * jump means it is fighting its heading while it drives, so it is worth
   * seeing — and on a linear path, worth fixing in one click.
   */
  $: headingJumpByLineId = (() => {
    const jumps = new Map<string, { entry: number; jump: number }>();
    timePrediction?.timeline?.forEach((event) => {
      if (event.type !== "travel" || event.lineIndex === undefined) return;
      const lineId = lines[event.lineIndex]?.id;
      if (!lineId || jumps.has(lineId)) return;
      const entry = Number(event.startHeading) || 0;
      const goal = Number(event.targetHeading) || 0;
      jumps.set(lineId, { entry, jump: getAngularDifference(entry, goal) });
    });
    return jumps;
  })();

  /** Points a linear path's start heading at the heading the robot arrives with. */
  function matchEntryHeading(lineId: string) {
    const entry = headingJumpByLineId.get(lineId)?.entry;
    if (entry === undefined) return;

    lines = lines.map((line) => {
      if (line.id !== lineId || line.endPoint.heading !== "linear") return line;
      return {
        ...line,
        endPoint: { ...line.endPoint, startDeg: entry, startDegExpression: undefined },
      };
    });
    recordChange?.();
  }

  /** Only the paths with something to say get a chip. */
  $: clearanceByLineId = clearanceReport.byLine;

  /**
   * Clearance reported against a Wait rather than a path: the pose the robot
   * holds while it stands there.
   */
  /**
   * Chip props for a Wait row. Empty when the robot standing there is fine.
   */
  function waitClearanceProps(id: string) {
    const span = waitClearanceById.get(id);
    if (!span) return { clearanceLabel: "", clearanceHit: false, clearanceTitle: "" };
    const held =
      span.stationary?.seconds !== undefined
        ? ` for ${span.stationary.seconds.toFixed(1)}s`
        : "";
    return {
      clearanceLabel: describeClearanceSpan(span),
      clearanceHit: span.severity === "hit",
      clearanceTitle:
        span.severity === "hit"
          ? `The robot stands${held} ${
              span.kind === "midline"
                ? `${Math.abs(span.worstClearance).toFixed(2)}in over the midline`
                : `${Math.abs(span.worstClearance).toFixed(2)}in inside ${clearanceTargetName(span)}`
            }.`
          : `The robot stands${held} within ${span.worstClearance.toFixed(2)}in of ${clearanceTargetName(span)}.`,
    };
  }

  $: waitClearanceById = new Map(
    clearanceReport.spans
      .filter((span) => span.stationary)
      .map((span) => [span.stationary!.id, span]),
  );

  /**
   * Where the robot is staged, when that is itself too close to something. No
   * path can fix this one, so it is surfaced on the starting point instead.
   */
  $: startPoseClearance = (() => {
    const pose = clearanceReport.startPose;
    if (!pose || pose.clearance >= clearanceReport.margin) return null;
    // Starting against the wall is what G304 asks for.
    if (pose.againstWall) return null;
    return {
      hit: pose.clearance < -TOUCH_TOLERANCE,
      clearance: pose.clearance,
      target:
        pose.kind === "wall" ? "the field wall" : pose.obstacleName?.trim() || "an obstacle",
    };
  })();

  /** Per-path result of the last "fix collision issues" click. */
  let clearanceFixNotes: Record<string, string> = {};
  let fixingClearanceLineId = "";

  /**
   * Moves a path's endpoint on X and Y until the robot's footprint clears
   * whatever it was hitting.
   *
   * The endpoint sets the shape of the whole curve and the start of whatever
   * follows, so this searches for the smallest move that clears both rather
   * than pushing straight away from the obstacle. It reports what it managed:
   * a path whose tightest point is its start or a bulge in the middle cannot be
   * fixed from the far end, and saying so beats moving the endpoint for nothing.
   */
  async function fixClearance(lineId: string) {
    const target = lines.find((line) => line.id === lineId);
    if (!target || !canMoveEndpoint(target)) return;

    fixingClearanceLineId = lineId;
    clearanceFixNotes = { ...clearanceFixNotes, [lineId]: "" };
    // Let the button paint its pending state before the search blocks the frame.
    await new Promise((resolve) => setTimeout(resolve, 0));

    try {
      const fix = suggestClearanceFix(
        clearanceInput(),
        clearanceOptions,
        lineId,
      );

      if (!fix) {
        clearanceFixNotes = { ...clearanceFixNotes, [lineId]: describeNoFix(lineId) };
        return;
      }

      applyClearanceFix(fix);
      clearanceFixNotes = { ...clearanceFixNotes, [lineId]: describeApplied(fix) };
      recordChange?.();
    } finally {
      fixingClearanceLineId = "";
    }
  }

  /** Moves where the robot is staged until its start pose clears. */
  async function fixStartPointClearance() {
    if (startPoint.locked || startPoint.poseVariableId) return;

    fixingClearanceLineId = START_POINT_FIX_ID;
    clearanceFixNotes = { ...clearanceFixNotes, [START_POINT_FIX_ID]: "" };
    await new Promise((resolve) => setTimeout(resolve, 0));

    try {
      const fix = suggestStartPointFix(clearanceInput(), clearanceOptions);

      if (!fix) {
        clearanceFixNotes = {
          ...clearanceFixNotes,
          [START_POINT_FIX_ID]: "No nearby staging position clears it",
        };
        return;
      }

      applyClearanceFix(fix);
      clearanceFixNotes = {
        ...clearanceFixNotes,
        [START_POINT_FIX_ID]: describeApplied(fix),
      };
      recordChange?.();
    } finally {
      fixingClearanceLineId = "";
    }
  }

  const START_POINT_FIX_ID = "__start-point__";
  const ALL_FIX_ID = "__fix-all__";

  /**
   * Moving a pose variable moves every point bound to it, which is the only
   * handle that can shift an endpoint the pose owns.
   */
  async function fixPoseVariableClearance(poseVariableId: string) {
    fixingClearanceLineId = poseVariableId;
    clearanceFixNotes = { ...clearanceFixNotes, [poseVariableId]: "" };
    await new Promise((resolve) => setTimeout(resolve, 0));

    try {
      const fix = suggestPoseVariableFix(
        clearanceInput(),
        clearanceOptions,
        poseVariableId,
      );

      if (!fix) {
        clearanceFixNotes = {
          ...clearanceFixNotes,
          [poseVariableId]: "No move of this pose clears everything using it",
        };
        return;
      }

      applyClearanceFix(fix);
      clearanceFixNotes = { ...clearanceFixNotes, [poseVariableId]: describeApplied(fix) };
      recordChange?.();
    } finally {
      fixingClearanceLineId = "";
    }
  }

  /** Pose variables that something currently flagged depends on. */
  $: flaggedPoseVariableIds = (() => {
    const flagged = new Set<string>();

    lines.forEach((line) => {
      const poseId = line.endPoint?.poseVariableId;
      if (!poseId) return;
      const entry = clearanceReport.byLine.get(line.id || "");
      if (entry?.spans.length) flagged.add(poseId);
    });

    // A path handed a bad pose by the path before it is the pose's problem too.
    mainRoute.steps.forEach((step, index) => {
      if (step.kind !== "path") return;
      const entry = clearanceReport.byLine.get(step.lineId);
      const worst = entry?.spans[0];
      if (!worst || worst.startDistance > settings.rWidth / 2) return;

      const order = mainRoute.steps.filter((item) => item.kind === "path");
      const position = order.findIndex((item) => item.lineId === step.lineId);
      const previousId = position > 0 ? order[position - 1].lineId : null;
      const previous = previousId ? lines.find((item) => item.id === previousId) : null;
      if (previous?.endPoint?.poseVariableId) flagged.add(previous.endPoint.poseVariableId);
      void index;
    });

    if (startPoint.poseVariableId && startPoseClearance) {
      flagged.add(startPoint.poseVariableId);
    }

    return flagged;
  })();

  /**
   * Works through everything currently flagged, worst first, applying the best
   * fix for each until nothing more can be improved.
   *
   * Targets are re-derived after every applied fix rather than planned up front:
   * moving one waypoint moves the start of the path after it, so the list of
   * what is still in trouble changes as it goes. A target that cannot be fixed
   * is not retried — the geometry has not changed for it, and retrying would
   * spin.
   */
  async function fixAllCollisions() {
    if (fixingClearanceLineId) return;

    fixingClearanceLineId = ALL_FIX_ID;
    clearanceFixNotes = { ...clearanceFixNotes, [ALL_FIX_ID]: "Working…" };
    await tick();

    const options = clearanceOptions;
    const before = checkClearance(clearanceInput(), options);
    const applied: ClearanceFix[] = [];
    const exhausted = new Set<string>();

    try {
      for (let round = 0; round < MAX_FIX_ALL_ROUNDS; round++) {
        // Let the page breathe: each round runs a few hundred candidate
        // measurements and would otherwise freeze the frame.
        await new Promise((resolve) => setTimeout(resolve, 0));

        const input = clearanceInput();
        const report = checkClearance(input, options);
        const target = nextFixTarget(report, exhausted);
        if (!target) break;

        clearanceFixNotes = {
          ...clearanceFixNotes,
          [ALL_FIX_ID]: `Working… ${applied.length} fixed`,
        };

        const fix =
          target.kind === "startPoint"
            ? suggestStartPointFix(input, options)
            : target.kind === "poseVariable"
              ? suggestPoseVariableFix(input, options, target.id)
              : suggestClearanceFix(input, options, target.id);

        if (!fix) {
          exhausted.add(target.key);
          continue;
        }

        applyClearanceFix(fix);
        applied.push(fix);
        // The route and the timeline both move with it.
        await tick();
      }

      const after = checkClearance(clearanceInput(), options);
      clearanceFixNotes = {
        ...clearanceFixNotes,
        [ALL_FIX_ID]: summariseFixAll(applied, before, after),
      };
      if (applied.length) recordChange?.();
    } finally {
      fixingClearanceLineId = "";
    }
  }

  const MAX_FIX_ALL_ROUNDS = 24;

  type FixTarget = { kind: "path" | "startPoint" | "poseVariable"; id: string; key: string };

  /**
   * The worst thing still flagged. Poses come before the paths bound to them:
   * a path whose endpoint the pose owns cannot be fixed on its own, so trying it
   * first would only burn a round marking it unfixable.
   */
  function nextFixTarget(report: ClearanceReport, exhausted: Set<string>): FixTarget | null {
    const candidates: Array<FixTarget & { severity: number }> = [];

    flaggedPoseVariableIds.forEach((poseId) => {
      candidates.push({
        kind: "poseVariable",
        id: poseId,
        key: `pose:${poseId}`,
        // Ahead of a path at the same clearance, never ahead of a worse one.
        severity: report.minClearance - 1e-6,
      });
    });

    report.byLine.forEach((entry) => {
      if (!entry.spans.length) return;
      candidates.push({
        kind: "path",
        id: entry.lineId,
        key: `path:${entry.lineId}`,
        severity: entry.minClearance,
      });
    });

    const pose = report.startPose;
    if (pose && pose.clearance < report.margin && !startPoint.locked && !startPoint.poseVariableId) {
      candidates.push({
        kind: "startPoint",
        id: START_POINT_FIX_ID,
        key: "start",
        severity: pose.clearance,
      });
    }

    return (
      candidates
        .filter((candidate) => !exhausted.has(candidate.key))
        .sort((a, b) => a.severity - b.severity)[0] ?? null
    );
  }

  function summariseFixAll(
    applied: ClearanceFix[],
    before: ClearanceReport,
    after: ClearanceReport,
  ): string {
    if (!applied.length) {
      return before.spans.length
        ? "Nothing here could be moved to clear it — see each path for why"
        : "Nothing to fix";
    }

    const moves = `${applied.length} ${applied.length === 1 ? "move" : "moves"}`;
    if (after.hitCount === 0 && after.spans.length === 0) {
      return `${moves} — everything clears by ${settings.safetyMargin}in`;
    }
    if (after.hitCount === 0) {
      return `${moves} — no collisions left, ${after.spans.length} still inside the margin`;
    }
    return `${moves} — ${before.hitCount} collisions down to ${after.hitCount}`;
  }

  /**
   * The same question the report asks, so a fix the search calls clear really is
   * clear - including the AUTO midline rule when it is on.
   */
  $: clearanceOptions = {
    fieldSize: FIELD_SIZE,
    margin: settings.safetyMargin,
    midline: midlineRuleFromSettings(settings),
  };

  function clearanceInput() {
    return {
      startPoint,
      lines,
      footprint: footprintFromSettings(settings),
      obstacles: shapes,
      lineStartPoints: mainRoute.startPoints,
      headingTransitions: clearanceHeadingTransitions,
    };
  }

  /**
   * Writes the move onto whichever handle the search picked. Coordinates become
   * literals, because an expression that used to drive them would fight the move
   * the next time it is evaluated.
   */
  function applyClearanceFix(fix: ClearanceFix) {
    const move = (point: BasePoint): BasePoint => ({
      ...point,
      x: point.x + fix.dx,
      y: point.y + fix.dy,
      xExpression: undefined,
      yExpression: undefined,
    });

    if (fix.handle === "startPoint") {
      startPoint = move(startPoint) as Point;
      return;
    }

    // A pose is moved at the variable, not at the points: the points take their
    // position from it, so writing them directly would be overwritten on the
    // next resolve. Every point bound to it follows through the usual sync.
    if (fix.handle === "poseVariable" && fix.poseVariableId) {
      const variable = variables.find((item) => item.id === fix.poseVariableId);
      if (!variable || variable.type !== "pose") return;
      handleVariableChange({
        ...variable,
        x: Number(variable.x) + fix.dx,
        y: Number(variable.y) + fix.dy,
        xExpression: undefined,
        yExpression: undefined,
      });
      return;
    }

    lines = lines.map((line) => {
      if (line.id !== fix.lineId) return line;
      // `previousEndPoint` already carries the id of the path it belongs to, so
      // it is written exactly like that path's own endpoint.
      if (fix.handle === "endPoint" || fix.handle === "previousEndPoint") {
        return { ...line, endPoint: move(line.endPoint) as Point };
      }
      return {
        ...line,
        controlPoints: line.controlPoints.map((point, index) =>
          index === fix.controlPointIndex ? (move(point) as ControlPoint) : point,
        ),
      };
    });
  }

  /**
   * Why no move helped, in terms of the thing the user would have to change.
   *
   * "Nothing works" is useless on its own. The common case is a collision in the
   * first inch of a path, which is not really this path's at all — the robot is
   * put there by whatever came before, and only that endpoint can move it.
   */
  function describeNoFix(lineId: string): string {
    const entry = clearanceReport.byLine.get(lineId);
    const worst = entry?.spans.reduce((low, span) =>
      span.worstClearance < low.worstClearance ? span : low,
    );
    const line = lines.find((item) => item.id === lineId);

    // Within a robot's half-width of the start, the pose is the one handed over
    // by the previous path rather than anything this path chose.
    const atHandover = worst !== undefined && worst.startDistance <= settings.rWidth / 2;

    if (atHandover) {
      const order = mainRoute.steps.filter((step) => step.kind === "path");
      const position = order.findIndex((step) => step.lineId === lineId);
      const previousId = position > 0 ? order[position - 1].lineId : null;
      const previous = previousId ? lines.find((item) => item.id === previousId) : null;

      if (!previous) {
        return "The robot is already this close where it is staged — use the fix on the Starting Point";
      }
      const name = previous.name?.trim() || "the previous path";
      if (previous.endPoint?.poseVariableId) {
        return `This starts where ${name} ends, and that endpoint comes from a pose variable — change the pose`;
      }
      if (previous.endPoint?.locked) {
        return `This starts where ${name} ends, and that endpoint is locked — unlock it to move it`;
      }
      return `This starts where ${name} ends — no move of that endpoint clears both paths`;
    }

    if (line?.endPoint?.poseVariableId) {
      return "This path's endpoint comes from a pose variable, and its control points cannot clear it — change the pose";
    }
    if (line?.endPoint?.locked) {
      return "This path's endpoint is locked, and its control points cannot clear it";
    }
    return "No move of this path's points clears it — the gap may be tighter than the robot";
  }

  function describeApplied(fix: ClearanceFix): string {
    const poseName =
      fix.handle === "poseVariable"
        ? variables.find((item) => item.id === fix.poseVariableId)?.name?.trim()
        : "";

    const what =
      fix.handle === "controlPoint"
        ? `control point ${(fix.controlPointIndex ?? 0) + 1}`
        : fix.handle === "poseVariable"
          ? `pose ${poseName || "variable"}`
          : FIX_HANDLE_LABELS[fix.handle];

    return fix.meetsMargin
      ? `Moved ${what} ${fix.distance.toFixed(2)}in — now ${fix.after.toFixed(2)}in clear`
      : `Moved ${what} ${fix.distance.toFixed(2)}in — ${fix.after.toFixed(2)}in clear, short of the ${settings.safetyMargin}in margin`;
  }

  /**
   * The heading each path picks up, for the fix search. Read from the same
   * timeline the clearance report uses so a candidate is scored the same way
   * the report will score the result.
   */
  $: clearanceHeadingTransitions = (() => {
    const transitions = new Map<string, { entryHeading: number; catchUp: number }>();
    timePrediction?.timeline?.forEach((event) => {
      if (event.type !== "travel" || event.lineIndex === undefined) return;
      const lineId = lines[event.lineIndex]?.id;
      if (!lineId || transitions.has(lineId)) return;
      transitions.set(lineId, {
        entryHeading: Number(event.startHeading) || 0,
        catchUp: Number(event.headingCatchUp) || 0,
      });
    });
    return transitions;
  })();

  $: stopReasonByLineId = (() => {
    const reasons = new Map<string, string>();
    chainRuns.forEach((run) => {
      const last = run.steps[run.steps.length - 1];
      if (!last || reasons.has(last.lineId)) return;
      reasons.set(last.lineId, CHAIN_BREAK_LABELS[run.breakReason]);
    });
    return reasons;
  })();

  /**
   * A tangential start point has no heading of its own — it faces along the
   * first path — so the start heading field shows that direction rather than a
   * meaningless zero.
   */
  $: startTangentialHeading = (() => {
    if (startPoint.heading !== "tangential") return null;
    const firstStep = mainRoute.steps.find((step) => step.kind === "path");
    if (!firstStep || firstStep.kind !== "path") return null;
    return getLineStartHeading(firstStep.line, startPoint);
  })();


  $: syncLineColorsToChains();

  // Reference exported but unused props to silence Svelte unused-export warnings

  $: robotWidth;
  $: robotHeight;

  function timelineEventColor(eventName: string, index: number) {
    const colors = ["#f97316", "#06b6d4", "#22c55e", "#e11d48", "#8b5cf6", "#f59e0b"];
    let hash = index;
    for (const char of eventName) {
      hash = (hash * 31 + char.charCodeAt(0)) % colors.length;
    }
    return colors[Math.abs(hash) % colors.length];
  }

  // Compute timeline markers for the UI (start of each travel segment)
  // The route as driven (Camera pick and Turn first applied), as the field and the header time it.
  $: routeEdits = expandRouteEdits(startPoint, lines, sequence, settings);
  $: timePrediction = calculatePathTime(startPoint, routeEdits.lines, settings, routeEdits.sequence, variables);
  $: markers = (() => {
    const _markers: {
      percent?: number;
      startPercent?: number;
      endPercent?: number;
      color: string;
      name: string;
      label?: string;
      type?: "marker" | "duration";
    }[] = [];
    if (
      !timePrediction ||
      !timePrediction.timeline ||
      timePrediction.totalTime <= 0
    )
      return _markers;

    const eventWindows = buildEventTimingWindows(
      startPoint,
      lines,
      timePrediction,
      settings,
      sequence,
      variables,
    );

    timePrediction.timeline.forEach((ev) => {
      if ((ev as any).type === "travel") {
        const start = (ev as any).startTime as number;
        const end = (ev as any).endTime as number;
        const pct = (end / timePrediction.totalTime) * 100;
        const lineIndex = (ev as any).lineIndex as number;
        const line = lines[lineIndex];
        const color = line?.color || "#ffffff";
        const name = line?.name || `Path ${lineIndex + 1}`;
        _markers.push({ percent: pct, color, name });
      }
    });

    eventWindows.forEach((eventWindow, index) => {
      const eventColor = timelineEventColor(eventWindow.name, index + eventWindow.lineIndex * 7);
      _markers.push({
        percent: eventWindow.startPercent,
        color: eventColor,
        name: `${eventWindow.name} on ${eventWindow.lineName}`,
        type: "marker",
      });
      _markers.push({
        startPercent: eventWindow.startPercent,
        endPercent: eventWindow.endPercent,
        color: eventColor,
        name: `${eventWindow.name} on ${eventWindow.lineName}`,
        label:
          eventWindow.durationMs > 0
            ? `${eventWindow.name} ${(eventWindow.durationMs / 1000).toFixed(1)}s`
            : `${eventWindow.name} hold`,
        type: "duration",
      });
    });

    return _markers;
  })();


  // State for collapsed sections
  let collapsedSections = {
    obstacles: shapes.map(() => true),
    lines: lines.map(() => false),
    controlPoints: lines.map(() => true), // Start with control points collapsed
  };

  // Collapsed state for obstacles (default collapsed)
  let collapsedObstacles = shapes.map(() => true);

  type EditorTab = "route" | "variables" | "field" | "states" | "telemetry";

  const TABS: { id: EditorTab; label: string; hint: string }[] = [
    { id: "route", label: "Route", hint: "Start point, paths, waits, events and if blocks" },
    { id: "variables", label: "Variables", hint: "Reusable numbers, booleans, poses and paths" },
    { id: "field", label: "Field", hint: "Obstacles and mirroring" },
    { id: "states", label: "States", hint: "Fire, roll and flip effects for each robot state" },
    { id: "telemetry", label: "Telemetry", hint: "Timing and robot state" },
  ];

  let activeTab: EditorTab = "route";

  $: tabCounts = {
    route: sequence.length,
    variables: variables.length,
    field: shapes.length,
    states: Object.values(settings.stateVfx || {}).filter((kind) => kind && kind !== "none").length,
    telemetry: 0,
  } as Record<EditorTab, number>;

  function setAllLinesCollapsed(collapsed: boolean) {
    collapsedSections = {
      ...collapsedSections,
      lines: lines.map(() => collapsed),
    };
  }

  // Reactive statements to update UI state when lines or shapes change from file load
  $: if (lines.length !== collapsedSections.lines.length) {
    collapsedSections = {
      obstacles: collapsedSections.obstacles ?? shapes.map(() => true),
      lines: lines.map(() => false),
      controlPoints: lines.map(() => true),
    };
  }

  // Keep obstacle collapse state aligned with shapes list
  $: if (shapes.length !== collapsedObstacles.length) {
    collapsedObstacles = shapes.map(() => true);
  }

  $: if (!collapsedSections.obstacles || shapes.length !== collapsedSections.obstacles.length) {
    collapsedSections = {
      ...collapsedSections,
      obstacles: shapes.map(() => true),
    };
  }

  const makeId = () =>
    `${Date.now().toString(36)}-${Math.random().toString(36).slice(2, 8)}`;

  function cloneJson<T>(value: T): T {
    return JSON.parse(JSON.stringify(value));
  }

  function inputValue(event: Event): string {
    return (event.currentTarget as HTMLInputElement).value;
  }

  function inputNumber(event: Event): number {
    return Number(inputValue(event));
  }

  function addVariable(type: VariableType) {
    // Seed a new pose from the start point so it lands somewhere meaningful.
    const seed =
      type === "pose"
        ? {
            x: Number(startPoint.x) || 0,
            y: Number(startPoint.y) || 0,
            heading: startPointHeadingDegrees(),
          }
        : {};

    variables = [...variables, createVariable(type, variables, seed)];
    recordChange?.();
  }

  function duplicateVariable(variableId: string) {
    const variable = variables.find((item) => item.id === variableId);
    if (!variable) return;

    const clone = cloneJson(variable) as Variable;
    clone.id = `var-${makeId()}`;
    clone.name = uniqueVariableName(variables, `${variable.name || "value"}Copy`);

    if (clone.type === "path") {
      clone.lines = clone.lines.map((line, index) => ({
        ...line,
        id: `path-variable-line-${makeId()}-${index}`,
      }));
    }

    variables = [...variables, clone];
    recordChange?.();
  }

  function removeVariable(variableId: string) {
    const variable = variables.find((item) => item.id === variableId);
    variables = variables.filter((item) => item.id !== variableId);

    // Points bound to a removed pose keep their coordinates but lose the link.
    if (variable?.type === "pose") {
      if (startPoint.poseVariableId === variableId) {
        startPoint = clearPointPoseVariable(startPoint);
      }
      lines = lines.map((line) =>
        line.endPoint?.poseVariableId === variableId
          ? { ...line, endPoint: clearPointPoseVariable(line.endPoint) }
          : line,
      );
    }

    syncVariableUsage();
    recordChange?.();
  }

  function handleVariableChange(variable: Variable) {
    // Recompute variables that reference the edited one, then everything using them.
    variables = resolveVariableValues(
      variables.map((item) => (item.id === variable.id ? variable : item)),
    );
    syncVariableUsage();
  }

  /** Re-resolves every expression in the path after a variable changes. */
  function syncVariableUsage() {
    const scope = buildExpressionScope(variables);

    poseVariablesOf(variables).forEach((poseVariable) => {
      syncPoseVariableUsage(poseVariable.id, poseVariable);
    });

    startPoint = resolvePointExpressions(startPoint, variables, scope);
    lines = lines.map((line) => resolveLineExpressions(line, variables, scope));
    sequence = resolveSequenceExpressions(sequence, variables, scope);
  }

  /** Writes an expression plus its resolved literal onto a line. */
  function setLineExpression(
    lineId: string,
    field: "speedExpression" | "waitBeforeExpression" | "waitAfterExpression" | "enabledExpression",
    raw: string,
  ) {
    lines = lines.map((line) => {
      if (line.id !== lineId) return line;
      const next = { ...line, [field]: raw.trim() ? raw : undefined };
      return resolveLineExpressions(next, variables);
    });
  }

  function setSequenceExpression(
    seqIndex: number,
    field: "durationExpression" | "countExpression" | "enabledExpression",
    raw: string,
  ) {
    const item = sequence[seqIndex];
    if (!item) return;

    const scope = buildExpressionScope(variables);
    const next = { ...item, [field]: raw.trim() ? raw : undefined } as SequenceItem;
    const newSeq = [...sequence];
    newSeq[seqIndex] = resolveSequenceItemExpressions(next, variables, scope);
    sequence = newSeq;
  }

  const conditionalActive = (item: SequenceConditionalItem) =>
    isConditionalActive(item, variables);

  /** Repeat loops and `if` blocks both wrap a group of paths. */
  function sequencePathLineIds(sourceSequence: SequenceItem[] = sequence): string[] {
    const ids: string[] = [];
    sourceSequence.forEach((item) => {
      if (item.kind === "path") {
        ids.push(item.lineId);
      } else if (isGroupItem(item)) {
        ids.push(...(item.lineIds || []));
      }
    });
    return ids;
  }

  function orderedLineIdsForSelection(sourceLineIds: string[]): string[] {
    const sourceSet = new Set(sourceLineIds);
    const ordered = sequencePathLineIds().filter((lineId) => sourceSet.has(lineId));
    sourceLineIds.forEach((lineId) => {
      if (!ordered.includes(lineId)) ordered.push(lineId);
    });
    return ordered;
  }

  function lineIsLocked(lineId: string): boolean {
    return lines.find((line) => line.id === lineId)?.locked ?? false;
  }

  /* ---------------------------------------------------------------
   * Editing a wait or an event that lives inside a group
   * ------------------------------------------------------------ */

  /** The wait/event members of a group, for the rows rendered inside it. */
  function groupHold(
    groupId: string,
    key: string,
  ): SequenceWaitItem | SequenceEventItem | null {
    const group = sequence.find(
      (item): item is SequenceGroupItem => isGroupItem(item) && item.id === groupId,
    );
    if (!group) return null;
    const member = groupMembers(group).find((entry) => memberKey(entry) === key);
    return member && member.kind !== "path" ? member : null;
  }

  function withGroupMembers(
    groupId: string,
    transform: (members: SequenceGroupMember[]) => SequenceGroupMember[],
  ) {
    sequence = sequence.map((item) => {
      if (!isGroupItem(item) || item.id !== groupId) return item;
      return { ...item, members: transform(groupMembers(item)), lineIds: undefined };
    });
  }

  function updateGroupHold(
    groupId: string,
    key: string,
    // Only the fields a wait and an event share; intersecting the two types
    // would make `kind` impossible and the patch unusable.
    patch: Partial<Omit<SequenceWaitItem, "kind" | "id">>,
  ) {
    withGroupMembers(groupId, (members) =>
      members.map((member) =>
        memberKey(member) === key && member.kind !== "path"
          ? ({ ...(member as SequenceWaitItem), ...patch } as SequenceGroupMember)
          : member,
      ),
    );
  }

  function removeGroupMember(groupId: string, key: string) {
    withGroupMembers(groupId, (members) =>
      members.filter((member) => memberKey(member) !== key),
    );
    recordChange?.();
  }

  function insertGroupHoldAfter(groupId: string, key: string, kind: "wait" | "event") {
    withGroupMembers(groupId, (members) => {
      const next = [...members];
      const at = next.findIndex((member) => memberKey(member) === key);
      const item = kind === "event" ? makeShootEventItem() : makeWaitItem();
      next.splice(at >= 0 ? at + 1 : next.length, 0, item as SequenceGroupMember);
      return next;
    });
    recordChange?.();
  }

  function moveGroupMember(groupId: string, key: string, delta: number) {
    withGroupMembers(groupId, (members) => {
      const at = members.findIndex((member) => memberKey(member) === key);
      const to = at + delta;
      if (at < 0 || to < 0 || to >= members.length) return members;
      const next = [...members];
      const [moved] = next.splice(at, 1);
      next.splice(to, 0, moved);
      return next;
    });
    recordChange?.();
  }

  /** Whether the thing being dragged is pinned, whichever kind it is. */
  function memberIsLocked(key: string): boolean {
    const line = lines.find((item) => item.id === key);
    if (line) return line.locked ?? false;

    const top = sequence.find(
      (item) => (item.kind === "wait" || item.kind === "event") && item.id === key,
    );
    if (top && (top.kind === "wait" || top.kind === "event")) {
      return top.locked ?? false;
    }

    for (const item of sequence) {
      if (!isGroupItem(item)) continue;
      const member = groupMembers(item).find((entry) => memberKey(entry) === key);
      if (member && member.kind !== "path") return member.locked ?? false;
    }

    return false;
  }

  /**
   * The step list does not scroll on its own during a drag, so a block that is
   * off screen would be unreachable. While dragging, hovering near the top or
   * bottom edge scrolls the list, faster the closer to the edge.
   */
  function startAutoScroll() {
    if (autoScrollFrame) return;

    const step = () => {
      if (!draggedLineId || !routeScrollEl) {
        autoScrollFrame = 0;
        return;
      }

      const rect = routeScrollEl.getBoundingClientRect();
      const edge = 72;
      const maxSpeed = 20;

      if (dragPointerY > 0 && dragPointerY < rect.top + edge) {
        const intensity = Math.min(1, (rect.top + edge - dragPointerY) / edge);
        routeScrollEl.scrollTop -= maxSpeed * intensity;
      } else if (dragPointerY > rect.bottom - edge) {
        const intensity = Math.min(1, (dragPointerY - (rect.bottom - edge)) / edge);
        routeScrollEl.scrollTop += maxSpeed * intensity;
      }

      autoScrollFrame = requestAnimationFrame(step);
    };

    autoScrollFrame = requestAnimationFrame(step);
  }

  function stopAutoScroll() {
    if (autoScrollFrame) cancelAnimationFrame(autoScrollFrame);
    autoScrollFrame = 0;
    dragPointerX = 0;
    dragPointerY = 0;
  }

  /**
   * Path dragging uses pointer events, not HTML5 drag-and-drop.
   *
   * The native API put the browser into a modal drag loop that could outlive
   * the gesture — the cursor stayed stuck in "grabbing" and even Cmd+R was
   * swallowed until the page was reloaded by hand. Pointer events with
   * pointer capture never enter that loop, so the gesture always ends: on
   * pointerup, pointercancel, Escape, or the window losing focus.
   */
  function beginPathDrag(event: PointerEvent, lineId: string) {
    if (!lineId || memberIsLocked(lineId)) return;
    if (event.button !== 0) return; // Left button only.

    // Stops text selection and any native drag starting alongside this one.
    event.preventDefault();

    draggedLineId = lineId;
    dragPointerX = event.clientX;
    dragPointerY = event.clientY;
    dragOverRepeatId = "";
    dragOverTopLevel = false;

    // Capture keeps move/up events coming even once the pointer leaves the
    // handle, so the gesture cannot be lost part way through.
    const target = event.currentTarget as HTMLElement | null;
    try {
      target?.setPointerCapture?.(event.pointerId);
    } catch {
      // Capture is a nicety; the window listeners already cover the gesture.
    }

    if (typeof document !== "undefined") {
      document.body.style.cursor = "grabbing";
    }
    startAutoScroll();
  }

  /** Resolves whatever sits under the pointer into a drop target. */
  function updateDropTarget() {
    if (typeof document === "undefined") return;

    const element = document.elementFromPoint(dragPointerX, dragPointerY);
    const groupElement = element?.closest?.("[data-drop-group-id]") || null;
    const dropOutElement = element?.closest?.("[data-drop-out]") || null;

    const groupId = groupElement?.getAttribute("data-drop-group-id") || "";
    const group = groupId
      ? sequence.find(
          (item): item is SequenceGroupItem =>
            isGroupItem(item) && item.id === groupId,
        )
      : undefined;

    dragOverRepeatId = group && !group.locked ? groupId : "";
    dragOverTopLevel = Boolean(dropOutElement) && !dragOverRepeatId;
  }

  function handlePathDragMove(event: PointerEvent) {
    if (!draggedLineId) return;
    dragPointerX = event.clientX;
    dragPointerY = event.clientY;
    updateDropTarget();
  }

  function resetDragState() {
    stopAutoScroll();
    if (typeof document !== "undefined") {
      document.body.style.cursor = "";
    }
    if (!draggedLineId && !dragOverRepeatId && !dragOverTopLevel) return;
    draggedLineId = "";
    dragOverRepeatId = "";
    dragOverTopLevel = false;
  }

  /** Applies the pending move when the pointer ended over a valid target. */
  function finishPathDrag() {
    const lineId = draggedLineId;
    const targetGroupId = dragOverRepeatId;
    const toTopLevel = dragOverTopLevel;

    resetDragState();
    if (!lineId || memberIsLocked(lineId)) return;

    const next = targetGroupId
      ? moveItemIntoGroup(sequence, targetGroupId, lineId)
      : toTopLevel
        ? moveItemOutOfGroups(sequence, lineId)
        : sequence;

    if (next === sequence) return;

    sequence = next;
    syncLinesToSequence(next);
    recordChange?.();
  }

  function handlePathDragCancel(event?: KeyboardEvent | Event) {
    if (event instanceof KeyboardEvent && event.key !== "Escape") return;
    if (!draggedLineId) return;
    resetDragState();
  }

  function offsetPoint<T extends BasePoint>(point: T, dx: number, dy: number): T {
    if (point.poseVariableId) return { ...point };
    return {
      ...point,
      x: Number(point.x || 0) + dx,
      y: Number(point.y || 0) + dy,
    };
  }

  function cloneLineForRoute(line: Line, index: number, offset = 0): Line {
    const clone = cloneJson(line);
    clone.id = makeId();
    clone.name = `${line.name || `Path ${lines.length + index + 1}`} Copy`;
    clone.endPoint = offsetPoint(clone.endPoint, offset, offset);
    clone.controlPoints = (clone.controlPoints || []).map((point) =>
      offsetPoint(point, offset, offset),
    );
    return clone;
  }

  function getLineStartPoint(lineId: string): Point {
    const lineIndex = lines.findIndex((line) => line.id === lineId);
    if (lineIndex <= 0) return cloneJson(startPoint);
    return cloneJson(lines[lineIndex - 1].endPoint);
  }

  /**
   * Stores the current route as a reusable path variable. Passing an existing
   * id replaces that variable's contents instead of creating a new one.
   */
  function storeChainAsVariable(existingId?: string) {
    const orderedIds = sequencePathLineIds();
    const selectedLines = orderedIds
      .map((lineId) => lines.find((line) => line.id === lineId))
      .filter(Boolean) as Line[];

    if (!selectedLines.length) return;

    const storedLines = selectedLines.map((line, index) => {
      const clone = cloneJson(line);
      clone.id = `path-variable-line-${makeId()}-${index}`;
      return clone;
    });
    const storedStartPoint = getLineStartPoint(selectedLines[0].id || "");

    if (existingId) {
      variables = variables.map((variable) =>
        variable.id === existingId && variable.type === "path"
          ? { ...variable, startPoint: storedStartPoint, lines: storedLines }
          : variable,
      );
      recordChange?.();
      return;
    }

    const variable: PathVariable = {
      id: `var-${makeId()}`,
      type: "path",
      name: uniqueVariableName(variables, "path"),
      startPoint: storedStartPoint,
      lines: storedLines,
    };

    variables = [...variables, variable];
    recordChange?.();
  }

  function insertPathVariable(variableId: string) {
    const variable = variables.find(
      (item) => item.id === variableId && item.type === "path",
    ) as PathVariable | undefined;
    if (!variable || !variable.lines.length) return;

    const clonedLines = variable.lines.map((line, index) =>
      cloneLineForRoute(
        {
          ...line,
          name: `${variable.name || "Path Variable"} ${index + 1}`,
        },
        index,
        0,
      ),
    );

    lines = [...lines, ...clonedLines];
    sequence = [
      ...sequence,
      ...clonedLines.map((line) => ({ kind: "path", lineId: line.id! }) as SequenceItem),
    ];
    collapsedSections.lines.push(...clonedLines.map(() => false));
    collapsedSections.controlPoints.push(...clonedLines.map(() => true));
    collapsedSections = { ...collapsedSections };

    const insertedChain: PathChain = {
      id: makeChainId(),
      name: variable.name || `Inserted Path ${pathChains.length + 1}`,
      color: getRandomColor(),
      lineIds: clonedLines.map((line) => line.id!).filter(Boolean),
    };
    pathChains = [...pathChains, insertedChain];
    selectedChainId = insertedChain.id;
    syncLineColorsToChains();
    recordChange?.();
  }

  /**
   * The heading the robot is placed at. A linear start point keeps it in
   * `startDeg` — `endDeg` is never read for the start point — so seeding a pose
   * from it has to read the same field the start heading editor does.
   */
  function startPointHeadingDegrees(): number {
    if (startPoint.heading === "constant") return Number(startPoint.degrees) || 0;
    if (startPoint.heading === "linear") return Number(startPoint.startDeg) || 0;
    return startTangentialHeading ?? 0;
  }

  function firstFinite(...values: unknown[]): number {
    for (const value of values) {
      const numeric = Number(value);
      if (Number.isFinite(numeric)) return numeric;
    }
    return 0;
  }

  function bindPointToPoseVariable(
    point: Point,
    variable: PoseVariable,
    forcePoseHeading = false,
    preferLinear = false,
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

    // The start point passes `forcePoseHeading`: its heading lives in `startDeg`
    // (or `degrees`), never in `endDeg`, so the linear branch below would write
    // the pose heading into a field nothing reads and leave the real heading
    // untouched. Pin it down as a definite heading instead.
    if (forcePoseHeading) {
      return {
        ...basePoint,
        heading: "constant",
        degrees: targetHeading,
      };
    }

    if (preferLinear || point.heading === "linear") {
      return {
        ...basePoint,
        heading: "linear",
        startDeg: firstFinite(point.startDeg, point.degrees, point.endDeg, targetHeading),
        endDeg: targetHeading,
        headingCurve: point.heading === "linear" ? point.headingCurve ?? 1 : 1,
      };
    }

    if (!forcePoseHeading && point.heading === "tangential") {
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

  function clearPointPoseVariable(point: Point): Point {
    const { poseVariableId, ...nextPoint } = point;
    return nextPoint as Point;
  }

  function syncPoseVariableUsage(
    poseVariableId: string,
    sourceVariable?: PoseVariable,
  ) {
    const variable =
      sourceVariable || poseVariables.find((item) => item.id === poseVariableId);
    if (!variable) return;

    if (startPoint.poseVariableId === poseVariableId) {
      // Force the pose heading, exactly as the initial binding does, so editing
      // the pose keeps moving the start heading with it.
      startPoint = bindPointToPoseVariable(startPoint, variable, true);
    }

    const nextLines = lines.map((line) =>
      line.endPoint?.poseVariableId === poseVariableId
        ? { ...line, endPoint: bindPointToPoseVariable(line.endPoint, variable) }
        : line,
    );

    if (JSON.stringify(nextLines) !== JSON.stringify(lines)) {
      lines = nextLines;
    }
  }

  function handleStartPoseVariableChange(poseVariableId: string) {
    if (!poseVariableId) {
      startPoint = clearPointPoseVariable(startPoint);
      recordChange?.();
      return;
    }

    const variable = poseVariables.find((item) => item.id === poseVariableId);
    if (!variable) return;

    startPoint = bindPointToPoseVariable(startPoint, variable, true);
    recordChange?.();
  }

  function handleLinePoseVariableChange(lineId: string, poseVariableId: string) {
    if (!lineId) return;

    const variable = poseVariables.find((item) => item.id === poseVariableId);
    lines = lines.map((line) => {
      if (line.id !== lineId) return line;
      return {
        ...line,
        endPoint: variable
          ? bindPointToPoseVariable(line.endPoint, variable, false, true)
          : clearPointPoseVariable(line.endPoint),
      };
    });
    recordChange?.();
  }

  function buildPointWithHeadingMode(point: Point, mode: string): Point {
    const variable = point.poseVariableId
      ? poseVariables.find((item) => item.id === point.poseVariableId)
      : null;
    const targetHeading = variable ? Number(variable.heading) || 0 : null;
    const previousHeading = firstFinite(
      point.startDeg,
      point.degrees,
      point.endDeg,
      targetHeading,
    );
    const basePoint = {
      x: Number(point.x) || 0,
      y: Number(point.y) || 0,
      locked: point.locked,
      poseVariableId: point.poseVariableId,
    };

    if (mode === "linear") {
      return {
        ...basePoint,
        heading: "linear",
        startDeg: firstFinite(point.startDeg, previousHeading),
        endDeg: targetHeading ?? firstFinite(point.endDeg, previousHeading),
        headingCurve: point.heading === "linear" ? point.headingCurve ?? 1 : 1,
      };
    }

    if (mode === "tangential") {
      return {
        ...basePoint,
        heading: "tangential",
        reverse: point.reverse ?? false,
      };
    }

    return {
      ...basePoint,
      heading: "constant",
      degrees: targetHeading ?? previousHeading,
    };
  }

  function handleLineHeadingModeChange(lineId: string, mode: string) {
    if (!lineId) return;

    // Start a linear path at the heading the robot arrives with. Falling back
    // to whatever the point happened to hold is what creates heading jumps in
    // the first place, and inside a PathChain the robot cannot stop to make
    // one up.
    const entryHeading = headingJumpByLineId.get(lineId)?.entry;

    lines = lines.map((line) => {
      if (line.id !== lineId) return line;
      const endPoint = buildPointWithHeadingMode(line.endPoint, mode);
      if (endPoint.heading !== "linear" || entryHeading === undefined) {
        return { ...line, endPoint };
      }
      return {
        ...line,
        endPoint: { ...endPoint, startDeg: entryHeading, startDegExpression: undefined },
      };
    });
    recordChange?.();
  }

  function mirrorCurrentPath(axis: MirrorAxis) {
    const mirrored = mirrorPathData(
      {
        startPoint,
        lines,
        variables,
      },
      axis,
    );

    startPoint = mirrored.startPoint || startPoint;
    // Never fall back to an empty route: a failed mirror must not delete paths.
    lines = mirrored.lines || lines;
    variables = mirrored.variables || variables;
    percent = 0;
    recordChange?.();
  }

  function getWait(i: any) {
    return i as any;
  }
  function makeWaitItem(name = "Wait", durationMs = 0): SequenceItem {
    return {
      kind: "wait",
      id: makeId(),
      name,
      durationMs,
      locked: false,
    } as SequenceItem;
  }
  function makeShootEventItem(): SequenceItem {
    return {
      kind: "event",
      id: makeId(),
      name: "Shoot",
      durationMs: 500,
      locked: false,
    } as SequenceItem;
  }

  function insertLineAfter(seqIndex: number) {
    const seqItem = sequence[seqIndex];
    if (!seqItem || seqItem.kind !== "path") return;
    const lineIndex = lines.findIndex((l) => l.id === seqItem.lineId);
    const currentLine = lines[lineIndex];

    // Find the next path item in the sequence after seqIndex
    let nextPathSeqIndex = -1;
    for (let i = seqIndex + 1; i < sequence.length; i++) {
      if (sequence[i].kind === "path") {
        nextPathSeqIndex = i;
        break;
      }
    }

    // If there is no next path in sequence, fall back to addLine behavior (append new randomized point)
    let newPoint: Point | null = null;
    if (nextPathSeqIndex !== -1) {
      const nextLineId = (sequence[nextPathSeqIndex] as any).lineId;
      const nextLine = lines.find((l) => l.id === nextLineId);
      if (
        nextLine &&
        nextLine.endPoint &&
        currentLine &&
        currentLine.endPoint
      ) {
        const a = currentLine.endPoint;
        const b = nextLine.endPoint;
        const midX = (Number(a.x) + Number(b.x)) / 2;
        const midY = (Number(a.y) + Number(b.y)) / 2;
        newPoint = {
          x: midX,
          y: midY,
          heading: "tangential",
          reverse: false,
        };
      }
    }

    if (!newPoint) {
      // fallback: random nearby point from current end
      if (currentLine && currentLine.endPoint) {
        newPoint = {
          x: (currentLine.endPoint.x ?? 72) + _.random(-12, 12),
          y: (currentLine.endPoint.y ?? 72) + _.random(-12, 12),
          heading: "tangential",
          reverse: false,
        };
      } else {
        newPoint = {
          x: _.random(0, 141.5),
          y: _.random(0, 141.5),
          heading: "tangential",
          reverse: false,
        };
      }
    }

    const newLine = {
      id: makeId(),
      endPoint: newPoint,
      controlPoints: [],
      color: getRandomColor(),
      speed: currentLine?.speed ?? 1,
      name: `Path ${lines.length + 1}`,
      waitBeforeMs: 0,
      waitAfterMs: 0,
      waitBeforeName: "",
      waitAfterName: "",
    };

    // Insert the new line after the current one and a sequence item after current seq index
    const newLines = [...lines];
    newLines.splice(lineIndex + 1, 0, newLine);
    lines = newLines;

    const newSeq = [...sequence];
    newSeq.splice(seqIndex + 1, 0, { kind: "path", lineId: newLine.id! });
    sequence = newSeq;
    ensureLineInDefaultChain(newLine.id!);

    collapsedSections.lines.splice(lineIndex + 1, 0, false);
    collapsedSections.controlPoints.splice(lineIndex + 1, 0, true);

    // Force reactivity
    collapsedSections = { ...collapsedSections };
  }

  function duplicatePathAfter(seqIndex: number) {
    const seqItem = sequence[seqIndex];
    if (!seqItem || seqItem.kind !== "path") return;

    const lineIndex = lines.findIndex((line) => line.id === seqItem.lineId);
    const sourceLine = lines[lineIndex];
    if (!sourceLine) return;

    const clone = cloneLineForRoute(sourceLine, 0, 4);
    const newLines = [...lines];
    newLines.splice(lineIndex + 1, 0, clone);
    lines = newLines;

    const newSeq = [...sequence];
    newSeq.splice(seqIndex + 1, 0, { kind: "path", lineId: clone.id! });
    sequence = newSeq;

    const ownerChainId = getLinePrimaryChainId(sourceLine.id || "");
    if (ownerChainId) {
      pathChains = pathChains.map((chain) => {
        if (chain.id !== ownerChainId) return chain;
        const sourceIndex = chain.lineIds.indexOf(sourceLine.id || "");
        const nextLineIds = [...chain.lineIds];
        nextLineIds.splice(sourceIndex >= 0 ? sourceIndex + 1 : nextLineIds.length, 0, clone.id!);
        return { ...chain, lineIds: nextLineIds };
      });
    } else {
      ensureLineInDefaultChain(clone.id!);
    }

    collapsedSections.lines.splice(lineIndex + 1, 0, false);
    collapsedSections.controlPoints.splice(lineIndex + 1, 0, true);
    collapsedSections = { ...collapsedSections };
    syncLineColorsToChains();
    recordChange?.();
  }

  function wrapPathInRepeat(seqIndex: number) {
    const seqItem = sequence[seqIndex];
    if (!seqItem || seqItem.kind !== "path") return;

    const line = lines.find((item) => item.id === seqItem.lineId);
    const repeatItem: SequenceItem = {
      kind: "repeat",
      id: makeId(),
      name: "Repeat Loop",
      count: 2,
      members: [{ kind: "path", lineId: seqItem.lineId }],
      locked: false,
    };

    const newSeq = [...sequence];
    newSeq.splice(seqIndex, 1, repeatItem);
    sequence = newSeq;
    syncLinesToSequence(newSeq);
    if (line) selectedChainId = getLinePrimaryChainId(line.id || "") || selectedChainId;
    recordChange?.();
  }

  function unwrapRepeat(seqIndex: number) {
    const seqItem = sequence[seqIndex];
    if (!seqItem || !isGroupItem(seqItem)) return;

    const replacement = (seqItem.lineIds || []).map(
      (lineId) => ({ kind: "path", lineId }) as SequenceItem,
    );
    const newSeq = [...sequence];
    newSeq.splice(seqIndex, 1, ...replacement);
    sequence = newSeq;
    syncLinesToSequence(newSeq);
    recordChange?.();
  }

  /**
   * Duplicates a repeat loop or `if` block. The paths inside are copied too:
   * reusing the same path ids would put one path in two blocks, so editing the
   * copy would silently edit the original and the field would only ever draw
   * one of them.
   */
  function duplicateRepeatAfter(seqIndex: number) {
    const seqItem = sequence[seqIndex];
    if (!seqItem || !isGroupItem(seqItem)) return;

    const clonedLines: Line[] = [];
    const clonedIds: string[] = [];

    // Paths get fresh copies so editing one block cannot edit the other; waits
    // and events carry no geometry, so a fresh id is all they need.
    const clonedMembers = groupMembers(seqItem)
      .map((member) => {
        if (member.kind !== "path") {
          return { ...cloneJson(member), id: makeId() } as typeof member;
        }
        const source = lines.find((line) => line.id === member.lineId);
        if (!source) return null;
        const clone = cloneLineForRoute(source, clonedLines.length, 0);
        clonedLines.push(clone);
        clonedIds.push(clone.id!);
        return { kind: "path", lineId: clone.id! } as const;
      })
      .filter(Boolean) as ReturnType<typeof groupMembers>;

    if (clonedLines.length) {
      lines = [...lines, ...clonedLines];
      collapsedSections = {
        ...collapsedSections,
        lines: [...collapsedSections.lines, ...clonedLines.map(() => false)],
        controlPoints: [
          ...collapsedSections.controlPoints,
          ...clonedLines.map(() => true),
        ],
      };
      clonedIds.forEach((lineId) => ensureLineInDefaultChain(lineId));
    }

    const newSeq = [...sequence];
    newSeq.splice(seqIndex + 1, 0, {
      ...cloneJson(seqItem),
      id: makeId(),
      name: `${seqItem.name || "Group"} Copy`,
      members: clonedMembers,
      lineIds: undefined,
    });
    sequence = newSeq;
    syncLinesToSequence(newSeq);
    recordChange?.();
  }

  function duplicateLineInsideRepeat(seqIndex: number, lineId: string) {
    const seqItem = sequence[seqIndex];
    if (!seqItem || !isGroupItem(seqItem)) return;

    const lineIndex = lines.findIndex((line) => line.id === lineId);
    const sourceLine = lines[lineIndex];
    if (!sourceLine) return;

    const clone = cloneLineForRoute(sourceLine, 0, 4);
    const newLines = [...lines];
    newLines.splice(lineIndex + 1, 0, clone);
    lines = newLines;

    const currentMembers = groupMembers(seqItem);
    const insertIndex = currentMembers.findIndex(
      (member) => member.kind === "path" && member.lineId === lineId,
    );
    const nextMembers = [...currentMembers];
    nextMembers.splice(
      insertIndex >= 0 ? insertIndex + 1 : nextMembers.length,
      0,
      { kind: "path", lineId: clone.id! },
    );

    const newSeq = [...sequence];
    newSeq[seqIndex] = { ...seqItem, members: nextMembers, lineIds: undefined };
    sequence = newSeq;

    const ownerChainId = getLinePrimaryChainId(sourceLine.id || "");
    if (ownerChainId) {
      pathChains = pathChains.map((chain) => {
        if (chain.id !== ownerChainId) return chain;
        const sourceIndex = chain.lineIds.indexOf(sourceLine.id || "");
        const nextIds = [...chain.lineIds];
        nextIds.splice(sourceIndex >= 0 ? sourceIndex + 1 : nextIds.length, 0, clone.id!);
        return { ...chain, lineIds: nextIds };
      });
    }

    collapsedSections.lines.splice(lineIndex + 1, 0, false);
    collapsedSections.controlPoints.splice(lineIndex + 1, 0, true);
    collapsedSections = { ...collapsedSections };
    syncLineColorsToChains();
    recordChange?.();
  }

  /** Wraps the path at `seqIndex` in a new `if` block. */
  function wrapPathInConditional(seqIndex: number) {
    const seqItem = sequence[seqIndex];
    if (!seqItem || seqItem.kind !== "path") return;

    const conditionalItem: SequenceItem = {
      kind: "conditional",
      id: makeId(),
      name: "If",
      condition: "",
      members: [{ kind: "path", lineId: seqItem.lineId }],
      locked: false,
    };

    const newSeq = [...sequence];
    newSeq.splice(seqIndex, 1, conditionalItem);
    sequence = newSeq;
    syncLinesToSequence(newSeq);
    recordChange?.();
  }

  /** Appends an empty `if` block for paths to be dragged into. */
  function addConditionalBlock() {
    sequence = [
      ...sequence,
      {
        kind: "conditional",
        id: makeId(),
        name: "If",
        condition: "",
        members: [],
        locked: false,
      },
    ];
    recordChange?.();
  }

  function updateConditionalItem(
    seqIndex: number,
    patch: Partial<{ name: string; condition: string; locked: boolean }>,
    commit = false,
  ) {
    const seqItem = sequence[seqIndex];
    if (!seqItem || seqItem.kind !== "conditional") return;

    const newSeq = [...sequence];
    newSeq[seqIndex] = { ...seqItem, ...patch };
    sequence = newSeq;
    if (commit) recordChange?.();
  }

  function updateRepeatItem(
    seqIndex: number,
    patch: Partial<{ name: string; count: number; locked: boolean }>,
    commit = false,
  ) {
    const seqItem = sequence[seqIndex];
    if (!seqItem || seqItem.kind !== "repeat") return;

    const newSeq = [...sequence];
    newSeq[seqIndex] = {
      ...seqItem,
      ...patch,
      count: Math.max(1, Math.min(20, Math.round(Number(patch.count ?? seqItem.count) || 1))),
    };
    sequence = newSeq;
    if (commit) recordChange?.();
  }

  // Insert a midpoint between this path and the next path in sequence
  function insertMidpointAfter(seqIndex: number) {
    const seqItem = sequence[seqIndex];
    if (!seqItem || seqItem.kind !== "path") return;
    const lineIndex = lines.findIndex((l) => l.id === seqItem.lineId);
    const currentLine = lines[lineIndex];

    // Find the next path in sequence
    let nextPathSeqIndex = -1;
    for (let i = seqIndex + 1; i < sequence.length; i++) {
      if (sequence[i].kind === "path") {
        nextPathSeqIndex = i;
        break;
      }
    }

    if (nextPathSeqIndex === -1) {
      // no next path -> do nothing or fallback
      return;
    }

    const nextLineId = (sequence[nextPathSeqIndex] as any).lineId;
    const nextLine = lines.find((l) => l.id === nextLineId);
    if (!currentLine || !nextLine) return;

    const a = currentLine.endPoint;
    const b = nextLine.endPoint;
    const midX = (Number(a.x) + Number(b.x)) / 2;
    const midY = (Number(a.y) + Number(b.y)) / 2;

    const newLine: Line = {
      id: makeId(),
      endPoint: {
        x: midX,
        y: midY,
        heading: "tangential",
        reverse: false,
      },
      controlPoints: [],
      color: getRandomColor(),
      speed: currentLine?.speed ?? 1,
      name: `Path ${lines.length + 1}`,
      waitBeforeMs: 0,
      waitAfterMs: 0,
      waitBeforeName: "",
      waitAfterName: "",
    };

    // Insert into lines right after current line index
    const newLines = [...lines];
    newLines.splice(lineIndex + 1, 0, newLine);
    lines = newLines;

    // Insert into sequence right after seqIndex
    const newSeq = [...sequence];
    newSeq.splice(seqIndex + 1, 0, { kind: "path", lineId: newLine.id! });
    sequence = newSeq;
    ensureLineInDefaultChain(newLine.id!);

    collapsedSections.lines.splice(lineIndex + 1, 0, false);
    collapsedSections.controlPoints.splice(lineIndex + 1, 0, true);

    collapsedSections = { ...collapsedSections };
    recordChange();
  }

  function removeLine(idx: number) {
    const removedId = lines[idx]?.id;
    let _lns = lines;
    lines.splice(idx, 1);
    lines = _lns;
    if (removedId) {
      sequence = sequence
        .map((item) =>
          isGroupItem(item)
            ? {
                ...item,
                members: groupMembers(item).filter(
                  (member) => !(member.kind === "path" && member.lineId === removedId),
                ),
                lineIds: undefined,
              }
            : item,
        )
        // Drop a repeat loop once it is empty; keep an empty `if` block so its
        // condition survives while paths are being moved around.
        .filter((s) =>
          s.kind === "path"
            ? s.lineId !== removedId
            : s.kind === "repeat"
              ? groupMembers(s).length > 0
              : true,
        );
      removeLineFromChains(removedId);
    }
    collapsedSections.lines.splice(idx, 1);
    collapsedSections.controlPoints.splice(idx, 1);
    recordChange();
  }

  /**
   * Deletes a repeat loop or `if` block together with the paths it holds.
   *
   * Dropping only the sequence item would leave those paths orphaned in
   * `lines` — gone from the route list but still drawn on the field. Use
   * "Unwrap" to keep the paths and discard just the wrapper.
   */
  function removeGroupItem(seqIndex: number) {
    const item = sequence[seqIndex];
    if (!item || !isGroupItem(item)) return;

    const doomed = new Set((item.lineIds || []).filter(Boolean));

    const newSeq = [...sequence];
    newSeq.splice(seqIndex, 1);
    sequence = newSeq.filter((entry) =>
      entry.kind === "path" ? !doomed.has(entry.lineId) : true,
    );

    if (doomed.size) {
      const keptIndexes: number[] = [];
      lines.forEach((line, index) => {
        if (!doomed.has(line.id || "")) keptIndexes.push(index);
      });

      lines = keptIndexes.map((index) => lines[index]);
      collapsedSections = {
        ...collapsedSections,
        lines: keptIndexes.map((index) => collapsedSections.lines[index]),
        controlPoints: keptIndexes.map(
          (index) => collapsedSections.controlPoints[index],
        ),
      };

      doomed.forEach((lineId) => removeLineFromChains(lineId));
    }

    recordChange?.();
  }

  function addLine() {
    const newLine: Line = {
      id: makeId(),
      name: `Path ${lines.length + 1}`,
      endPoint: {
        x: _.random(0, 141.5),
        y: _.random(0, 141.5),
        heading: "tangential",
        reverse: false,
      },
      controlPoints: [],
      color: getRandomColor(),
      speed: 1,
      waitBeforeMs: 0,
      waitAfterMs: 0,
      waitBeforeName: "",
      waitAfterName: "",
    };
    lines = [...lines, newLine];
    sequence = [...sequence, { kind: "path", lineId: newLine.id! }];
    ensureLineInDefaultChain(newLine.id!);
    collapsedSections.lines.push(false);
    collapsedSections.controlPoints.push(true);
    recordChange();
  }

  function addControlPointToLineById(lineId: string) {
    const lineIndex = lines.findIndex((line) => line.id === lineId);
    if (lineIndex === -1) return;
    const line = lines[lineIndex];
    line.controlPoints = line.controlPoints || [];
    const prevPt = lineIndex === 0 ? startPoint : lines[lineIndex - 1].endPoint;
    const endPt = line.endPoint || { x: 72, y: 72 };
    const mx = ((prevPt?.x ?? 72) + (endPt?.x ?? 72)) / 2;
    const my = ((prevPt?.y ?? 72) + (endPt?.y ?? 72)) / 2;
    line.controlPoints.push({
      x: mx + _.random(-4, 4),
      y: my + _.random(-4, 4),
    });
    collapsedSections.controlPoints[lineIndex] = false;
    lines = [...lines];
    collapsedSections = { ...collapsedSections };
    recordChange?.();
  }

  // Add a control point to the line represented by `seqIndex` in the sequence
  function addControlPointToLine(seqIndex: number) {
    const seqItem = sequence[seqIndex];
    if (!seqItem || seqItem.kind !== "path") return;
    addControlPointToLineById(seqItem.lineId);
  }

  // Add a control point to the last path in `lines` (fallback: create a new line)
  function addControlPointToLastLine() {
    if (!lines || lines.length === 0) {
      // No lines exist: create a new line instead
      addLine();
      return;
    }

    // Prefer adding to the first line whose control points are expanded (user is focusing it)
    let targetIdx = collapsedSections.controlPoints.findIndex(
      (v) => v === false,
    );
    if (targetIdx === -1) targetIdx = lines.length - 1;

    const line = lines[targetIdx];
    line.controlPoints = line.controlPoints || [];
    // Insert a control point near the line midpoint for convenience
    const prevPt = targetIdx === 0 ? startPoint : lines[targetIdx - 1].endPoint;
    const endPt = line.endPoint || { x: 72, y: 72 };
    const mx = ((prevPt?.x ?? 72) + (endPt?.x ?? 72)) / 2;
    const my = ((prevPt?.y ?? 72) + (endPt?.y ?? 72)) / 2;
    line.controlPoints.push({
      x: mx + _.random(-4, 4),
      y: my + _.random(-4, 4),
    });
    // Ensure control points UI is expanded for this line
    collapsedSections.controlPoints[targetIdx] = false;
    lines = [...lines];
    collapsedSections = { ...collapsedSections };
    recordChange?.();
  }

  function addWait() {
    sequence = [...sequence, makeWaitItem()];
  }

  /** Pollen Pickup rows the user has open. A new one opens itself. */
  let pollenExpanded: Record<string, boolean> = {};
  $: vision = visionOf(settings);

  /**
   * Somewhere sensible to look first: straight out from wherever the route
   * currently ends, well inside the range a single ball registers at. The user
   * drags it where the POLLEN really is.
   */
  function pollenTargetAhead(pose: { x: number; y: number; heading: number }) {
    const reach = Math.max(12, Math.min(36, reliableRangeInches(vision) * 0.6));
    const yaw = (vision.mountYawDeg * Math.PI) / 180;
    const point = robotToField(pose, {
      x: vision.mountForwardInches + reach * Math.cos(yaw),
      y: vision.mountLeftInches + reach * Math.sin(yaw),
    });
    return {
      x: Math.max(0, Math.min(141.5, point.x)),
      y: Math.max(0, Math.min(141.5, point.y)),
    };
  }

  function routeEndPose() {
    const pathSteps = mainRoute.steps.filter((step) => step.kind === "path");
    const last = pathSteps[pathSteps.length - 1];
    if (!last || last.kind !== "path") {
      const degrees =
        startPoint.heading === "constant"
          ? Number(startPoint.degrees) || 0
          : startPoint.heading === "linear"
            ? Number(startPoint.startDeg) || 0
            : 0;
      return { x: startPoint.x, y: startPoint.y, heading: degrees };
    }
    return {
      x: last.line.endPoint.x,
      y: last.line.endPoint.y,
      heading: getLineEndHeading(last.line, last.startPoint),
    };
  }

  function addPollenPickup() {
    const item = makePollenItem(makeId(), pollenTargetAhead(routeEndPose()));
    pollenExpanded = { ...pollenExpanded, [item.id]: true };
    sequence = [...sequence, item];
    recordChange?.();
  }

  function insertPollenAfter(seqIndex: number) {
    const item = makePollenItem(makeId(), pollenTargetAhead(routeEndPose()));
    pollenExpanded = { ...pollenExpanded, [item.id]: true };
    const next = [...sequence];
    next.splice(seqIndex + 1, 0, item);
    sequence = next;
    recordChange?.();
  }

  function updatePollen(seqIndex: number, patch: Partial<SequencePollenItem>) {
    const current = sequence[seqIndex];
    if (!current || current.kind !== "pollen") return;
    const next = [...sequence];
    next[seqIndex] = { ...current, ...patch };
    sequence = next;
  }

  function aimPollenAhead(seqIndex: number) {
    const current = sequence[seqIndex];
    if (!current || current.kind !== "pollen") return;
    const plan = pollenPlans.get(current.id);
    if (!plan) return;
    const target = pollenTargetAhead(plan.searchPose);
    updatePollen(seqIndex, { targetX: target.x, targetY: target.y });
    recordChange?.();
  }

  const pollenSpansFor = (report: ClearanceReport, id: string) =>
    report.spans.filter((span) => span.maneuver?.id === id);

  function addEvent() {
    sequence = [...sequence, makeShootEventItem()];
  }

  function addRepeatLoop() {
    sequence = [
      ...sequence,
      {
        kind: "repeat",
        id: makeId(),
        name: "Repeat Loop",
        count: 2,
        members: [],
        locked: false,
      } as SequenceItem,
    ];
    recordChange?.();
  }

  function addWaitAtStart() {
    sequence = [makeWaitItem(), ...sequence];
  }

  function addEventAtStart() {
    sequence = [makeShootEventItem(), ...sequence];
  }

  function addPathAtStart() {
    const newLine: Line = {
      id: makeId(),
      name: `Path ${lines.length + 1}`,
      endPoint: {
        x: _.random(0, 141.5),
        y: _.random(0, 141.5),
        heading: "tangential",
        reverse: false,
      },
      controlPoints: [],
      color: getRandomColor(),
      speed: 1,
      waitBeforeMs: 0,
      waitAfterMs: 0,
      waitBeforeName: "",
      waitAfterName: "",
    };
    lines = [newLine, ...lines];
    sequence = [{ kind: "path", lineId: newLine.id! }, ...sequence];
    ensureLineInDefaultChain(newLine.id!);
    collapsedSections.lines = [false, ...collapsedSections.lines];
    collapsedSections.controlPoints = [
      true,
      ...collapsedSections.controlPoints,
    ];
    recordChange();
  }

  function insertWaitAfter(seqIndex: number) {
    const newSeq = [...sequence];
    newSeq.splice(seqIndex + 1, 0, makeWaitItem());
    sequence = newSeq;
  }

  function insertEventAfter(seqIndex: number) {
    const newSeq = [...sequence];
    newSeq.splice(seqIndex + 1, 0, makeShootEventItem());
    sequence = newSeq;
  }

  function insertPathAfter(seqIndex: number) {
    // Create a new line with default settings
    const newLine: Line = {
      id: makeId(),
      name: `Path ${lines.length + 1}`,
      endPoint: {
        x: _.random(36, 108),
        y: _.random(36, 108),
        heading: "tangential",
        reverse: false,
      },
      controlPoints: [],
      color: getRandomColor(),
      speed: 1,
      waitBeforeMs: 0,
      waitAfterMs: 0,
      waitBeforeName: "",
      waitAfterName: "",
    };

    // Add the new line to the lines array
    lines = [...lines, newLine];

    // Insert the new path in the sequence after the wait
    const newSeq = [...sequence];
    newSeq.splice(seqIndex + 1, 0, { kind: "path", lineId: newLine.id! });
    sequence = newSeq;
    ensureLineInDefaultChain(newLine.id!);

    // Add UI state for the new line
    collapsedSections.lines.push(false);
    collapsedSections.controlPoints.push(true);

    // Force reactivity
    collapsedSections = { ...collapsedSections };
    recordChange();
  }

  function syncLinesToSequence(newSeq: SequenceItem[]) {
    const pathOrder = sequencePathLineIds(newSeq);

    const indexedLines = lines.map((line, idx) => ({
      line,
      collapsed: collapsedSections.lines[idx],
      control: collapsedSections.controlPoints[idx],
    }));

    const byId = new Map(indexedLines.map((entry) => [entry.line.id, entry]));
    const reordered: typeof indexedLines = [];

    pathOrder.forEach((id) => {
      const entry = byId.get(id);
      if (entry) {
        reordered.push(entry);
        byId.delete(id);
      }
    });

    // Append any lines that are not currently in the sequence to preserve data
    reordered.push(...byId.values());

    lines = reordered.map((entry) => entry.line);
    collapsedSections = {
      ...collapsedSections,
      lines: reordered.map((entry) => entry.collapsed ?? false),
      controlPoints: reordered.map((entry) => entry.control ?? true),
    };
    // No collapsedEventMarkers to update
  }

  function moveSequenceItem(seqIndex: number, delta: number) {
    const targetIndex = seqIndex + delta;
    if (targetIndex < 0 || targetIndex >= sequence.length) return;

    // Prevent moving if either the source or target is a locked path or a locked wait
    const isLockedSequenceItem = (index: number) => {
      const it = sequence[index];
      if (!it) return false;
      if (it.kind === "path") {
        const ln = lines.find((l) => l.id === it.lineId);
        return ln?.locked ?? false;
      }
      if (it.kind === "wait" || it.kind === "event") {
        return (it as any).locked ?? false;
      }
      if (isGroupItem(it)) {
        const groupLocked = (it as any).locked ?? false;
        return groupLocked || groupLineIds(it).some((lineId) =>
          lines.find((line) => line.id === lineId)?.locked,
        );
      }
      return false;
    };

    if (isLockedSequenceItem(seqIndex) || isLockedSequenceItem(targetIndex))
      return;

    const newSeq = [...sequence];
    const [item] = newSeq.splice(seqIndex, 1);
    newSeq.splice(targetIndex, 0, item);
    sequence = newSeq;

    syncLinesToSequence(newSeq);
    recordChange?.();
  }
</script>
<svelte:window
  on:pointermove={handlePathDragMove}
  on:pointerup={finishPathDrag}
  on:pointercancel={handlePathDragCancel}
  on:blur={handlePathDragCancel}
  on:keydown={handlePathDragCancel}
/>

<div class="flex-1 min-w-0 flex flex-col justify-start items-center gap-2 h-full min-h-0">
  <!-- Section tabs: keeps each area one click away instead of one long scroll -->
  <nav
    class="w-full flex flex-wrap gap-1 rounded-lg bg-neutral-50 dark:bg-neutral-900 shadow-md p-1.5"
    aria-label="Editor sections"
  >
    {#each TABS as tab (tab.id)}
      <button
        on:click={() => (activeTab = tab.id)}
        aria-current={activeTab === tab.id ? "page" : undefined}
        title={tab.hint}
        class="flex items-center gap-1.5 px-2.5 py-1.5 text-xs font-semibold rounded-md transition {activeTab ===
        tab.id
          ? 'bg-[#fe55a2] text-white shadow-sm'
          : 'text-neutral-600 dark:text-neutral-300 hover:bg-neutral-200/70 dark:hover:bg-neutral-800'}"
      >
        {tab.label}
        {#if tabCounts[tab.id] > 0}
          <span
            class="rounded-full px-1.5 py-0.5 text-[10px] leading-none {activeTab ===
            tab.id
              ? 'bg-white/25'
              : 'bg-neutral-200 dark:bg-neutral-800'}"
          >
            {tabCounts[tab.id]}
          </span>
        {/if}
      </button>
    {/each}
  </nav>

  <div
    bind:this={routeScrollEl}
    class="flex flex-col justify-start items-start w-full rounded-lg bg-neutral-50 dark:bg-neutral-900 shadow-md p-4 overflow-y-scroll overflow-x-hidden h-full min-h-0 gap-6"
  >
    {#if activeTab === "route"}
      <StartingPointSection
        bind:startPoint
        {poseVariables}
        {variables}
        {recordChange}
        tangentialHeading={startTangentialHeading}
        onPoseVariableChange={handleStartPoseVariableChange}
        {addPathAtStart}
        {addWaitAtStart}
        {addEventAtStart}
        clearance={startPoseClearance}
        onFixClearance={fixStartPointClearance}
        fixingClearance={fixingClearanceLineId === START_POINT_FIX_ID}
        clearanceFixNote={clearanceFixNotes[START_POINT_FIX_ID] || ""}
      />

      <!-- Sticky step toolbar: add and collapse without scrolling -->
      <div
        class="sticky top-0 z-10 w-full flex flex-wrap items-center gap-2 rounded-md border border-neutral-200 dark:border-neutral-700 bg-white dark:bg-neutral-800 p-2 shadow-sm"
      >
        <span
          class="text-xs font-semibold uppercase tracking-wide text-neutral-500 dark:text-neutral-300"
        >
          Add
        </span>
        <button
          on:click={addLine}
          class="px-2 py-1 text-xs font-semibold rounded bg-green-100 text-green-700 dark:bg-green-900 dark:text-green-200"
        >
          Path
        </button>
        <button
          on:click={addWait}
          class="px-2 py-1 text-xs font-semibold rounded bg-amber-100 text-amber-700 dark:bg-amber-900 dark:text-amber-200"
        >
          Wait
        </button>
        <button
          on:click={addEvent}
          class="px-2 py-1 text-xs font-semibold rounded bg-purple-100 text-purple-700 dark:bg-purple-900 dark:text-purple-200"
        >
          Event
        </button>
        <button
          on:click={addPollenPickup}
          title="Find POLLEN with the camera, drive to it, intake, and come back"
          class="px-2 py-1 text-xs font-semibold rounded bg-yellow-200 text-yellow-900 dark:bg-yellow-800 dark:text-yellow-50"
        >
          Pollen
        </button>
        <button
          on:click={addRepeatLoop}
          class="px-2 py-1 text-xs font-semibold rounded bg-cyan-100 text-cyan-700 dark:bg-cyan-900 dark:text-cyan-200"
        >
          Repeat
        </button>
        <button
          on:click={addConditionalBlock}
          title="Wrap paths in an if block driven by a boolean"
          class="px-2 py-1 text-xs font-semibold rounded bg-violet-100 text-violet-700 dark:bg-violet-900 dark:text-violet-200"
        >
          If
        </button>

        <!--
          One pass over everything flagged. The per-path buttons each fix one
          thing; this walks the whole route so a chain of hand-offs does not
          have to be clicked through one at a time.
        -->
        {#if clearanceReport.spans.length || startPoseClearance}
          <button
            on:click={fixAllCollisions}
            disabled={Boolean(fixingClearanceLineId)}
            title="Move control points, endpoints, poses and the starting point until nothing on the route is closer than the safety margin"
            class="px-2 py-1 text-xs font-semibold rounded disabled:opacity-60 {clearanceReport.hitCount >
            0
              ? 'bg-red-100 text-red-700 dark:bg-red-900 dark:text-red-200'
              : 'bg-amber-100 text-amber-700 dark:bg-amber-900 dark:text-amber-200'}"
          >
            {fixingClearanceLineId === ALL_FIX_ID
              ? "Fixing…"
              : `Fix all collisions (${clearanceReport.hitCount || clearanceReport.spans.length})`}
          </button>
        {/if}

        {#if clearanceFixNotes[ALL_FIX_ID]}
          <span
            class="px-2 py-1 text-[11px] font-semibold rounded bg-neutral-100 text-neutral-600 dark:bg-neutral-800 dark:text-neutral-300"
            title={clearanceFixNotes[ALL_FIX_ID]}
          >
            {clearanceFixNotes[ALL_FIX_ID]}
          </span>
        {/if}

        <div class="ml-auto flex items-center gap-2">
          <button
            on:click={() => setAllLinesCollapsed(true)}
            class="px-2 py-1 text-xs rounded bg-neutral-100 text-neutral-700 dark:bg-neutral-800 dark:text-neutral-200"
            title="Collapse every path"
          >
            Collapse all
          </button>
          <button
            on:click={() => setAllLinesCollapsed(false)}
            class="px-2 py-1 text-xs rounded bg-neutral-100 text-neutral-700 dark:bg-neutral-800 dark:text-neutral-200"
            title="Expand every path"
          >
            Expand all
          </button>
        </div>
      </div>

      {#if sequence.length === 0}
        <p class="text-xs text-neutral-500 dark:text-neutral-400">
          No steps yet — use <strong>Add</strong> above to place the first path.
        </p>
      {/if}

      <div
        role="region"
        aria-label="Move path out of its block"
        data-drop-out="true"
        class="w-full overflow-hidden rounded-lg border-dashed text-center text-xs font-semibold transition-all {showDragOutZone
          ? 'my-1 border-2 p-4 opacity-100'
          : 'h-0 border-0 p-0 opacity-0 pointer-events-none'} {dragOverTopLevel
          ? 'border-sky-500 bg-sky-50 text-sky-700 dark:bg-sky-950/40 dark:text-sky-200'
          : 'border-neutral-300 text-neutral-500 dark:border-neutral-700 dark:text-neutral-400'}"
      >
        Drop here to move this path out of its block
      </div>

      <!-- Unified sequence render: paths and waits -->
      {#each sequence as item, sIdx}
        <div class="w-full">
          {#if item.kind === "path"}
            {#each lines.filter((l) => l.id === item.lineId) as ln (ln.id)}
              <div
                class="rounded-lg border border-transparent transition {draggedLineId === (ln.id || "") ? 'opacity-60' : ''}"
                role="listitem"
              >
                <PathLineSection
                  bind:line={ln}
                  idx={lines.findIndex((l) => l.id === ln.id)}
                  bind:lines
                  bind:collapsed={
                    collapsedSections.lines[lines.findIndex((l) => l.id === ln.id)]
                  }
                  bind:collapsedControlPoints={
                    collapsedSections.controlPoints[
                      lines.findIndex((l) => l.id === ln.id)
                    ]
                  }
                  onRemove={() =>
                    removeLine(lines.findIndex((l) => l.id === ln.id))}
                  onInsertAfter={() => addControlPointToLineById(ln.id || "")}
                  onInsertMidpoint={() => insertMidpointAfter(sIdx)}
                  onDuplicate={() => duplicatePathAfter(sIdx)}
                  onWrapRepeat={() => wrapPathInRepeat(sIdx)}
                  onWrapConditional={() => wrapPathInConditional(sIdx)}
                  onPointerDown={(event) => beginPathDrag(event, ln.id || "")}
                  dragging={draggedLineId === (ln.id || "")}
                  onAddWaitAfter={() => insertWaitAfter(sIdx)}
                  onAddEventAfter={() => insertEventAfter(sIdx)}
                  onMoveUp={() => moveSequenceItem(sIdx, -1)}
                  onMoveDown={() => moveSequenceItem(sIdx, 1)}
                  canMoveUp={sIdx !== 0}
                  canMoveDown={sIdx !== sequence.length - 1}
                  optimizeLine={optimizeLine}
                  optimizing={optimizingLineIds?.[ln.id ?? ""] ?? false}
                  {poseVariables}
                  {variables}
                  inactive={inactiveLineIds.has(ln.id || "")}
                  stopReason={stopReasonByLineId.get(ln.id || "") ?? null}
                  headingJump={headingJumpByLineId.get(ln.id || "")?.jump ?? null}
                  clearance={clearanceByLineId.get(ln.id || "") ?? null}
                  onFixClearance={() => fixClearance(ln.id || "")}
                  fixingClearance={fixingClearanceLineId === (ln.id || "")}
                  clearanceFixNote={clearanceFixNotes[ln.id || ""] || ""}
                  onMatchEntryHeading={() => matchEntryHeading(ln.id || "")}
                  onPoseVariableChange={handleLinePoseVariableChange}
                  onHeadingModeChange={handleLineHeadingModeChange}
                  {recordChange}
                />
              </div>
            {/each}
          {:else if item.kind === "repeat"}
            <div
              class="w-full rounded-lg border bg-cyan-50 dark:bg-cyan-950/30 p-3 shadow-sm transition {dragOverRepeatId === item.id ? 'border-cyan-500 ring-2 ring-cyan-400/70 dark:ring-cyan-500/60' : 'border-cyan-300 dark:border-cyan-800'}"
              role="group"
              data-drop-group-id={item.id}
            >
              <div class="flex flex-wrap items-center gap-2 mb-2">
                <span class="text-xs font-semibold uppercase tracking-wide text-cyan-700 dark:text-cyan-200">
                  Repeat Loop
                </span>
                {#if dragOverRepeatId === item.id}
                  <span class="rounded-full bg-cyan-600 px-2 py-0.5 text-[11px] font-semibold text-white">
                    Drop path here
                  </span>
                {/if}
                <input
                  type="text"
                  value={item.name}
                  on:input={(event) =>
                    updateRepeatItem(sIdx, {
                      name: inputValue(event),
                    })}
                  on:blur={() => recordChange?.()}
                  class="min-w-0 flex-1 px-2 py-1 text-xs rounded border border-cyan-200 dark:border-cyan-800 bg-white dark:bg-neutral-950"
                />
                <span class="text-xs text-neutral-600 dark:text-neutral-300">x</span>
                <div class="w-28">
                  <ExpressionInput
                    compact
                    {variables}
                    disabled={item.locked}
                    placeholder="1"
                    title="Repeat count (literal or expression)"
                    value={expressionDisplayValue(item.countExpression, item.count)}
                    onInput={(raw) =>
                      setSequenceExpression(sIdx, "countExpression", raw)}
                    onCommit={() => recordChange?.()}
                  />
                </div>
                <button
                  on:click={() => updateRepeatItem(sIdx, { locked: !item.locked }, true)}
                  class="px-2 py-1 text-xs rounded bg-neutral-100 text-neutral-700 dark:bg-neutral-800 dark:text-neutral-200"
                >
                  {item.locked ? "Unlock" : "Lock"}
                </button>
                <button
                  on:click={() => duplicateRepeatAfter(sIdx)}
                  class="px-2 py-1 text-xs rounded bg-indigo-100 text-indigo-700 dark:bg-indigo-900 dark:text-indigo-200"
                >
                  Duplicate
                </button>
                <button
                  on:click={() => unwrapRepeat(sIdx)}
                  class="px-2 py-1 text-xs rounded bg-amber-100 text-amber-700 dark:bg-amber-900 dark:text-amber-200"
                >
                  Unwrap
                </button>
                <button
                  on:click={() => removeGroupItem(sIdx)}
                  title="Delete this block and the paths inside it"
                  class="px-2 py-1 text-xs rounded bg-rose-100 text-rose-700 dark:bg-rose-900 dark:text-rose-200"
                >
                  Remove
                </button>
                <button
                  on:click={() => moveSequenceItem(sIdx, -1)}
                  disabled={sIdx === 0 || item.locked}
                  class="px-2 py-1 text-xs rounded bg-neutral-100 text-neutral-700 dark:bg-neutral-800 dark:text-neutral-200 disabled:opacity-40"
                >
                  Up
                </button>
                <button
                  on:click={() => moveSequenceItem(sIdx, 1)}
                  disabled={sIdx === sequence.length - 1 || item.locked}
                  class="px-2 py-1 text-xs rounded bg-neutral-100 text-neutral-700 dark:bg-neutral-800 dark:text-neutral-200 disabled:opacity-40"
                >
                  Down
                </button>
              </div>
              <div class="flex flex-col gap-2">
                {#if groupMembers(item).length === 0}
                  <div class="rounded border border-dashed border-cyan-300 dark:border-cyan-700 bg-white/70 dark:bg-neutral-950/50 p-4 text-center text-xs font-semibold text-cyan-700 dark:text-cyan-200">
                    Drag paths, waits or events into this loop
                  </div>
                {/if}
                {#each groupMembers(item) as member, loopLineIndex (memberKey(member))}
                  {#if member.kind !== "path"}
                    <!--
                      A wait or an event that lives in this block. It runs
                      in its place in the order, on every pass of a loop.
                    -->
                    <div
                      class="rounded border border-cyan-200 dark:border-cyan-900 bg-white dark:bg-neutral-900 p-1 transition {draggedLineId === memberKey(member) ? 'opacity-60' : ''}"
                      role="listitem"
                    >
                      <WaitRow
                        {...waitClearanceProps(memberKey(member))}
                        label={member.kind === "event" ? "Event" : "Wait"}
                        name={member.name}
                        durationMs={member.durationMs}
                        durationExpression={member.durationExpression}
                        enabledExpression={member.enabledExpression}
                        {variables}
                        locked={member.locked ?? false}
                        onToggleLock={() =>
                          updateGroupHold(item.id, memberKey(member), {
                            locked: !(groupHold(item.id, memberKey(member))?.locked ?? false),
                          })}
                        onChange={(newName, newDuration) =>
                          updateGroupHold(item.id, memberKey(member), {
                            name: newName,
                            durationMs: Math.max(0, Number(newDuration) || 0),
                          })}
                        onDurationExpressionChange={(raw) =>
                          updateGroupHold(item.id, memberKey(member), {
                            durationExpression: raw.trim() ? raw : undefined,
                          })}
                        onEnabledExpressionChange={(raw) =>
                          updateGroupHold(item.id, memberKey(member), {
                            enabledExpression: raw.trim() ? raw : undefined,
                          })}
                        onCommit={() => recordChange?.()}
                        onRemove={() => removeGroupMember(item.id, memberKey(member))}
                        onInsertAfter={() =>
                          insertGroupHoldAfter(item.id, memberKey(member), "wait")}
                        onAddPathAfter={() => addLine()}
                        onAddEventAfter={() =>
                          insertGroupHoldAfter(item.id, memberKey(member), "event")}
                        onMoveUp={() => moveGroupMember(item.id, memberKey(member), -1)}
                        onMoveDown={() => moveGroupMember(item.id, memberKey(member), 1)}
                        canMoveUp={loopLineIndex !== 0}
                        canMoveDown={loopLineIndex !== groupMembers(item).length - 1}
                        onPointerDown={(event) => beginPathDrag(event, memberKey(member))}
                        dragging={draggedLineId === memberKey(member)}
                      />
                    </div>
                  {:else}
                  {@const lineId = member.lineId}
                  {#each lines.filter((l) => l.id === lineId) as ln (ln.id)}
                    <div
                      class="rounded border border-cyan-200 dark:border-cyan-900 bg-white dark:bg-neutral-900 p-2 transition {draggedLineId === (ln.id || '') ? 'opacity-60' : ''}"
                      role="listitem"
                    >
                      <PathLineSection
                        bind:line={ln}
                        idx={lines.findIndex((l) => l.id === ln.id)}
                        bind:lines
                        bind:collapsed={
                          collapsedSections.lines[lines.findIndex((l) => l.id === ln.id)]
                        }
                        bind:collapsedControlPoints={
                          collapsedSections.controlPoints[
                            lines.findIndex((l) => l.id === ln.id)
                          ]
                        }
                        onRemove={() =>
                          removeLine(lines.findIndex((l) => l.id === ln.id))}
                        onInsertAfter={() => addControlPointToLineById(ln.id || "")}
                        onInsertMidpoint={() => insertMidpointAfter(sIdx)}
                        onDuplicate={() => duplicateLineInsideRepeat(sIdx, ln.id || "")}
                        onWrapRepeat={() => {}}
                        onPointerDown={(event) => beginPathDrag(event, ln.id || "")}
                        dragging={draggedLineId === (ln.id || "")}
                        onAddWaitAfter={() => insertWaitAfter(sIdx)}
                        onAddEventAfter={() => insertEventAfter(sIdx)}
                        onMoveUp={() => moveSequenceItem(sIdx, -1)}
                        onMoveDown={() => moveSequenceItem(sIdx, 1)}
                        canMoveUp={sIdx !== 0}
                        canMoveDown={sIdx !== sequence.length - 1}
                        optimizeLine={optimizeLine}
                        optimizing={optimizingLineIds?.[ln.id ?? ""] ?? false}
                        {poseVariables}
                        {variables}
                        inactive={inactiveLineIds.has(ln.id || "")}
                        stopReason={stopReasonByLineId.get(ln.id || "") ?? null}
                        headingJump={headingJumpByLineId.get(ln.id || "")?.jump ?? null}
                        clearance={clearanceByLineId.get(ln.id || "") ?? null}
                        onFixClearance={() => fixClearance(ln.id || "")}
                        fixingClearance={fixingClearanceLineId === (ln.id || "")}
                        clearanceFixNote={clearanceFixNotes[ln.id || ""] || ""}
                        onMatchEntryHeading={() => matchEntryHeading(ln.id || "")}
                        onPoseVariableChange={handleLinePoseVariableChange}
                        onHeadingModeChange={handleLineHeadingModeChange}
                        {recordChange}
                      />
                    </div>
                  {/each}
                  {/if}
                {/each}
              </div>
            </div>
          {:else if item.kind === "conditional"}
            <div
              class="w-full rounded-lg border bg-violet-50 dark:bg-violet-950/30 p-3 shadow-sm transition {dragOverRepeatId ===
              item.id
                ? 'border-violet-500 ring-2 ring-violet-400/70 dark:ring-violet-500/60'
                : 'border-violet-300 dark:border-violet-800'}"
              role="group"
              data-drop-group-id={item.id}
            >
              <div class="flex flex-wrap items-center gap-2 mb-2">
                <span
                  class="font-mono text-xs font-bold text-violet-700 dark:text-violet-200"
                >
                  if
                </span>
                {#if dragOverRepeatId === item.id}
                  <span
                    class="rounded-full bg-violet-600 px-2 py-0.5 text-[11px] font-semibold text-white"
                  >
                    Drop path here
                  </span>
                {/if}
                <div class="min-w-0 flex-1">
                  <ExpressionInput
                    compact
                    kind="boolean"
                    {variables}
                    disabled={item.locked}
                    placeholder="always — e.g. isRedAlliance"
                    title="Boolean expression. These paths are skipped when it is false, and wrapped in a Java if on export."
                    value={item.condition || ""}
                    onInput={(raw) =>
                      updateConditionalItem(sIdx, { condition: raw })}
                    onCommit={() => recordChange?.()}
                  />
                </div>
                <span
                  class="rounded-full px-2 py-0.5 text-[10px] font-semibold {conditionalActive(
                    item,
                  )
                    ? 'bg-emerald-100 text-emerald-700 dark:bg-emerald-900 dark:text-emerald-200'
                    : 'bg-neutral-200 text-neutral-600 dark:bg-neutral-800 dark:text-neutral-300'}"
                  title="Whether this branch runs with the current variable values"
                >
                  {conditionalActive(item) ? "Running" : "Skipped"}
                </span>
                <button
                  on:click={() =>
                    updateConditionalItem(sIdx, { locked: !item.locked }, true)}
                  class="px-2 py-1 text-xs rounded bg-neutral-100 text-neutral-700 dark:bg-neutral-800 dark:text-neutral-200"
                >
                  {item.locked ? "Unlock" : "Lock"}
                </button>
                <button
                  on:click={() => duplicateRepeatAfter(sIdx)}
                  class="px-2 py-1 text-xs rounded bg-indigo-100 text-indigo-700 dark:bg-indigo-900 dark:text-indigo-200"
                  title="Duplicate — pair with a negated condition to make an else branch"
                >
                  Duplicate
                </button>
                <button
                  on:click={() => unwrapRepeat(sIdx)}
                  class="px-2 py-1 text-xs rounded bg-amber-100 text-amber-700 dark:bg-amber-900 dark:text-amber-200"
                >
                  Unwrap
                </button>
                <button
                  on:click={() => removeGroupItem(sIdx)}
                  title="Delete this block and the paths inside it"
                  class="px-2 py-1 text-xs rounded bg-rose-100 text-rose-700 dark:bg-rose-900 dark:text-rose-200"
                >
                  Remove
                </button>
                <button
                  on:click={() => moveSequenceItem(sIdx, -1)}
                  disabled={sIdx === 0 || item.locked}
                  class="px-2 py-1 text-xs rounded bg-neutral-100 text-neutral-700 dark:bg-neutral-800 dark:text-neutral-200 disabled:opacity-40"
                >
                  Up
                </button>
                <button
                  on:click={() => moveSequenceItem(sIdx, 1)}
                  disabled={sIdx === sequence.length - 1 || item.locked}
                  class="px-2 py-1 text-xs rounded bg-neutral-100 text-neutral-700 dark:bg-neutral-800 dark:text-neutral-200 disabled:opacity-40"
                >
                  Down
                </button>
              </div>

              <div class="flex flex-col gap-2">
                {#if groupMembers(item).length === 0}
                  <div
                    class="rounded border border-dashed border-violet-300 dark:border-violet-700 bg-white/70 dark:bg-neutral-950/50 p-4 text-center text-xs font-semibold text-violet-700 dark:text-violet-200"
                  >
                    Drag paths into this if block
                  </div>
                {/if}
                {#each groupMembers(item) as member, blockLineIndex (memberKey(member))}
                  {#if member.kind !== "path"}
                    <!--
                      A wait or an event that lives in this block. It runs
                      in its place in the order, on every pass of a loop.
                    -->
                    <div
                      class="rounded border border-violet-200 dark:border-violet-900 bg-white dark:bg-neutral-900 p-1 transition {draggedLineId === memberKey(member) ? 'opacity-60' : ''}"
                      role="listitem"
                    >
                      <WaitRow
                        {...waitClearanceProps(memberKey(member))}
                        label={member.kind === "event" ? "Event" : "Wait"}
                        name={member.name}
                        durationMs={member.durationMs}
                        durationExpression={member.durationExpression}
                        enabledExpression={member.enabledExpression}
                        {variables}
                        locked={member.locked ?? false}
                        onToggleLock={() =>
                          updateGroupHold(item.id, memberKey(member), {
                            locked: !(groupHold(item.id, memberKey(member))?.locked ?? false),
                          })}
                        onChange={(newName, newDuration) =>
                          updateGroupHold(item.id, memberKey(member), {
                            name: newName,
                            durationMs: Math.max(0, Number(newDuration) || 0),
                          })}
                        onDurationExpressionChange={(raw) =>
                          updateGroupHold(item.id, memberKey(member), {
                            durationExpression: raw.trim() ? raw : undefined,
                          })}
                        onEnabledExpressionChange={(raw) =>
                          updateGroupHold(item.id, memberKey(member), {
                            enabledExpression: raw.trim() ? raw : undefined,
                          })}
                        onCommit={() => recordChange?.()}
                        onRemove={() => removeGroupMember(item.id, memberKey(member))}
                        onInsertAfter={() =>
                          insertGroupHoldAfter(item.id, memberKey(member), "wait")}
                        onAddPathAfter={() => addLine()}
                        onAddEventAfter={() =>
                          insertGroupHoldAfter(item.id, memberKey(member), "event")}
                        onMoveUp={() => moveGroupMember(item.id, memberKey(member), -1)}
                        onMoveDown={() => moveGroupMember(item.id, memberKey(member), 1)}
                        canMoveUp={blockLineIndex !== 0}
                        canMoveDown={blockLineIndex !== groupMembers(item).length - 1}
                        onPointerDown={(event) => beginPathDrag(event, memberKey(member))}
                        dragging={draggedLineId === memberKey(member)}
                      />
                    </div>
                  {:else}
                  {@const lineId = member.lineId}
                  {#each lines.filter((l) => l.id === lineId) as ln (ln.id)}
                    <div
                      class="rounded border border-violet-200 dark:border-violet-900 bg-white dark:bg-neutral-900 p-2 transition {draggedLineId ===
                      (ln.id || '')
                        ? 'opacity-60'
                        : ''}"
                      role="listitem"
                    >
                      <PathLineSection
                        bind:line={ln}
                        idx={lines.findIndex((l) => l.id === ln.id)}
                        bind:lines
                        bind:collapsed={
                          collapsedSections.lines[lines.findIndex((l) => l.id === ln.id)]
                        }
                        bind:collapsedControlPoints={
                          collapsedSections.controlPoints[
                            lines.findIndex((l) => l.id === ln.id)
                          ]
                        }
                        onRemove={() =>
                          removeLine(lines.findIndex((l) => l.id === ln.id))}
                        onInsertAfter={() => addControlPointToLineById(ln.id || "")}
                        onInsertMidpoint={() => insertMidpointAfter(sIdx)}
                        onDuplicate={() => duplicateLineInsideRepeat(sIdx, ln.id || "")}
                        onWrapRepeat={() => {}}
                        onPointerDown={(event) => beginPathDrag(event, ln.id || "")}
                        dragging={draggedLineId === (ln.id || "")}
                        onAddWaitAfter={() => insertWaitAfter(sIdx)}
                        onAddEventAfter={() => insertEventAfter(sIdx)}
                        onMoveUp={() => moveSequenceItem(sIdx, -1)}
                        onMoveDown={() => moveSequenceItem(sIdx, 1)}
                        canMoveUp={sIdx !== 0}
                        canMoveDown={sIdx !== sequence.length - 1}
                        optimizeLine={optimizeLine}
                        optimizing={optimizingLineIds?.[ln.id ?? ""] ?? false}
                        {poseVariables}
                        {variables}
                        inactive={inactiveLineIds.has(ln.id || "")}
                        stopReason={stopReasonByLineId.get(ln.id || "") ?? null}
                        headingJump={headingJumpByLineId.get(ln.id || "")?.jump ?? null}
                        clearance={clearanceByLineId.get(ln.id || "") ?? null}
                        onFixClearance={() => fixClearance(ln.id || "")}
                        fixingClearance={fixingClearanceLineId === (ln.id || "")}
                        clearanceFixNote={clearanceFixNotes[ln.id || ""] || ""}
                        onMatchEntryHeading={() => matchEntryHeading(ln.id || "")}
                        onPoseVariableChange={handleLinePoseVariableChange}
                        onHeadingModeChange={handleLineHeadingModeChange}
                        {recordChange}
                      />
                    </div>
                  {/each}
                  {/if}
                {/each}
              </div>
            </div>
          {:else if item.kind === "pollen"}
            <PollenPickupRow
              {item}
              plan={pollenPlans.get(item.id) ?? null}
              {vision}
              {variables}
              clearanceSpans={pollenSpansFor(clearanceReport, item.id)}
              expanded={pollenExpanded[item.id] ?? false}
              onExpandedChange={(value) =>
                (pollenExpanded = { ...pollenExpanded, [item.id]: value })}
              onChange={(patch) => updatePollen(sIdx, patch)}
              onCommit={() => recordChange?.()}
              onEnabledExpressionChange={(raw) =>
                setSequenceExpression(sIdx, "enabledExpression", raw)}
              onToggleLock={() => {
                updatePollen(sIdx, { locked: !item.locked });
                recordChange?.();
              }}
              onRemove={() => {
                const next = [...sequence];
                next.splice(sIdx, 1);
                sequence = next;
                recordChange?.();
              }}
              onMoveUp={() => moveSequenceItem(sIdx, -1)}
              onMoveDown={() => moveSequenceItem(sIdx, 1)}
              onAddPathAfter={() => insertPathAfter(sIdx)}
              onAimAhead={() => aimPollenAhead(sIdx)}
              canMoveUp={sIdx !== 0}
              canMoveDown={sIdx !== sequence.length - 1}
            />
          {:else}
            <WaitRow
              {...waitClearanceProps(item.id)}
              label={item.kind === "event" ? "Event" : "Wait"}
              name={getWait(item).name}
              durationMs={getWait(item).durationMs}
              durationExpression={getWait(item).durationExpression}
              enabledExpression={getWait(item).enabledExpression}
              {variables}
              locked={getWait(item).locked ?? false}
              onToggleLock={() => {
                const newSeq = [...sequence];
                const current = getWait(newSeq[sIdx]);
                newSeq[sIdx] = {
                  ...current,
                  locked: !(current.locked ?? false),
                };
                sequence = newSeq;
                recordChange?.();
              }}
              onChange={(newName, newDuration) => {
                const newSeq = [...sequence];
                // Read from `sequence`, not the each-block `item`: an expression
                // change may have already replaced this entry this tick.
                newSeq[sIdx] = {
                  ...getWait(newSeq[sIdx]),
                  name: newName,
                  durationMs: Math.max(0, Number(newDuration) || 0),
                };
                sequence = newSeq;
              }}
              onDurationExpressionChange={(raw) =>
                setSequenceExpression(sIdx, "durationExpression", raw)}
              onEnabledExpressionChange={(raw) =>
                setSequenceExpression(sIdx, "enabledExpression", raw)}
              onCommit={() => recordChange?.()}
              onRemove={() => {
                const newSeq = [...sequence];
                newSeq.splice(sIdx, 1);
                sequence = newSeq;
              }}
              onInsertAfter={() => {
                const newSeq = [...sequence];
                newSeq.splice(sIdx + 1, 0, makeWaitItem());
                sequence = newSeq;
              }}
              onAddPathAfter={() => insertPathAfter(sIdx)}
              onAddEventAfter={() => insertEventAfter(sIdx)}
              onMoveUp={() => moveSequenceItem(sIdx, -1)}
              onMoveDown={() => moveSequenceItem(sIdx, 1)}
              canMoveUp={sIdx !== 0}
              canMoveDown={sIdx !== sequence.length - 1}
              onPointerDown={(event) => beginPathDrag(event, getWait(item).id)}
              dragging={draggedLineId === getWait(item).id}
            />
          {/if}
        </div>
      {/each}

    {:else if activeTab === "variables"}
      <VariablesSection
        bind:variables
        canStoreChain={lines.length > 0}
        onAdd={addVariable}
        onRemove={removeVariable}
        onDuplicate={duplicateVariable}
        onChange={handleVariableChange}
        onCommit={recordChange}
        onStoreChain={storeChainAsVariable}
        onInsertPath={insertPathVariable}
        flaggedPoseIds={flaggedPoseVariableIds}
        onFixPoseClearance={fixPoseVariableClearance}
        fixingPoseId={fixingClearanceLineId}
        {clearanceFixNotes}
      />
    {:else if activeTab === "field"}
      <ObstaclesSection
        bind:shapes
        bind:collapsedObstacles
        {variables}
        {recordChange}
      />

      <div
        class="w-full rounded-md border border-neutral-200 dark:border-neutral-700 p-3 bg-white dark:bg-neutral-800"
      >
        <div class="flex items-center gap-2">
          <p
            class="text-xs font-semibold uppercase tracking-wide text-neutral-500 dark:text-neutral-300"
          >
            Mirror
          </p>
          <button
            on:click={() => mirrorCurrentPath("x")}
            class="px-2.5 py-1.5 text-xs font-semibold rounded bg-sky-100 text-sky-700 dark:bg-sky-900 dark:text-sky-200"
            title="Mirror X coordinates across the field centerline"
          >
            X
          </button>
          <button
            on:click={() => mirrorCurrentPath("y")}
            class="px-2.5 py-1.5 text-xs font-semibold rounded bg-teal-100 text-teal-700 dark:bg-teal-900 dark:text-teal-200"
            title="Mirror Y coordinates across the field centerline"
          >
            Y
          </button>
        </div>
      </div>
    {:else if activeTab === "states"}
      <StatesSection
        bind:settings
        {timePrediction}
        {startPoint}
        {lines}
        {sequence}
        {variables}
        {percent}
        {handleSeek}
        {recordChange}
      />
    {:else}
      <TelemetryPanel
        {percent}
        {timePrediction}
        {startPoint}
        {sequence}
        {variables}
        {settings}
        {lines}
        stopCount={chainRuns.length}
        {clearanceReport}
        {robotXY}
        {robotHeading}
        {x}
        {y}
      />
    {/if}
  </div>

  <PlaybackControls
    bind:playing
    {play}
    {pause}
    bind:percent
    {handleSeek}
    bind:loopAnimation
    {markers}
    totalTime={timePrediction?.totalTime ?? 0}
  />
</div>
