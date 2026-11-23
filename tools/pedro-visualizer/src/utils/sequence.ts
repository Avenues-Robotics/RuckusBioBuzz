import type {
  Line,
  Point,
  SequenceConditionalItem,
  SequenceEventItem,
  SequenceGroupItem,
  SequenceGroupMember,
  SequenceItem,
  SequencePollenItem,
  SequenceWaitItem,
  Variable,
} from "../types";
import { buildExpressionScope } from "./numberExpressions";
import { isConditionalActive, isEnabled } from "./variables";

/**
 * Pure sequence transformations behind the drag handles.
 *
 * A path lives either at the top level (a `path` item) or inside exactly one
 * group — a repeat loop or an `if` block. These helpers move a path between
 * those homes without ever duplicating or losing it.
 */

export function isGroupItem(item: SequenceItem): item is SequenceGroupItem {
  return item.kind === "repeat" || item.kind === "conditional";
}

/**
 * What identifies a member inside a group. A path is referenced by the id of
 * the line it draws; a wait or an event carries its own.
 */
export function memberKey(member: SequenceGroupMember): string {
  return member.kind === "path" ? member.lineId : member.id;
}

/**
 * A group's contents, in order.
 *
 * Groups used to hold nothing but a list of path ids, so a file written by an
 * older build is read through here and turned into members on the way past. The
 * old field stays on the type as deprecated rather than being deleted, because
 * a project saved months ago still has to open.
 */
export function groupMembers(item: SequenceGroupItem): SequenceGroupMember[] {
  if (Array.isArray(item.members)) return item.members;
  return (item.lineIds || []).map((lineId) => ({ kind: "path", lineId }) as const);
}

/** Just the paths in a group, for everything that only cares about geometry. */
export function groupLineIds(item: SequenceGroupItem): string[] {
  return groupMembers(item)
    .filter((member): member is { kind: "path"; lineId: string } => member.kind === "path")
    .map((member) => member.lineId);
}

/** Normalises a loaded sequence so nothing downstream has to know about `lineIds`. */
export function migrateSequenceGroups(sequence: SequenceItem[] = []): SequenceItem[] {
  let changed = false;

  const next = sequence.map((item) => {
    if (!isGroupItem(item) || Array.isArray(item.members)) return item;
    changed = true;
    const { lineIds, ...rest } = item;
    return { ...rest, members: groupMembers(item) } as SequenceItem;
  });

  return changed ? next : sequence;
}

/** The group holding a path, or undefined when it sits at the top level. */
export function groupHoldingLine(
  sequence: SequenceItem[],
  lineId: string,
): SequenceGroupItem | undefined {
  if (!lineId) return undefined;
  return sequence.find(
    (item): item is SequenceGroupItem =>
      isGroupItem(item) && groupLineIds(item).includes(lineId),
  );
}

/** The group holding any member — path, wait or event. */
export function groupHoldingMember(
  sequence: SequenceItem[],
  key: string,
): SequenceGroupItem | undefined {
  if (!key) return undefined;
  return sequence.find(
    (item): item is SequenceGroupItem =>
      isGroupItem(item) && groupMembers(item).some((member) => memberKey(member) === key),
  );
}

/**
 * An emptied repeat loop is meaningless, so it is dropped. An emptied `if`
 * block is kept: its condition is the point, and paths get dragged back in.
 */
function keepEmptiedGroup(item: SequenceGroupItem): boolean {
  return item.kind !== "repeat";
}

/**
 * Moves a member into the given group, removing it from wherever it was.
 *
 * Works for paths, waits and events alike: a loop that drives somewhere, pauses,
 * then drives on is an ordinary thing to want, and it is the order between them
 * that makes it mean anything. Returns the original array when the move is not
 * possible.
 */
export function moveItemIntoGroup(
  sequence: SequenceItem[],
  groupId: string,
  key: string,
): SequenceItem[] {
  if (!groupId || !key) return sequence;

  const target = sequence.find(
    (item): item is SequenceGroupItem => isGroupItem(item) && item.id === groupId,
  );
  if (!target || target.locked) return sequence;

  // The member's own data has to come with it. A path is just an id, but a wait
  // or an event carries its name and duration, so it is lifted out of wherever
  // it currently lives rather than recreated.
  let moving: SequenceGroupMember | null = null;

  sequence.forEach((item) => {
    if (item.kind === "path" && item.lineId === key) {
      moving = { kind: "path", lineId: key };
      return;
    }
    if ((item.kind === "wait" || item.kind === "event") && item.id === key) {
      moving = item;
      return;
    }
    if (isGroupItem(item)) {
      const found = groupMembers(item).find((member) => memberKey(member) === key);
      if (found) moving = found;
    }
  });

  if (!moving) return sequence;
  if (target.locked) return sequence;

  const next: SequenceItem[] = [];
  let targetIndex = -1;

  sequence.forEach((item) => {
    if (item.kind === "path") {
      if (item.lineId !== key) next.push(item);
      return;
    }

    if (item.kind === "wait" || item.kind === "event") {
      if (item.id !== key) next.push(item);
      return;
    }

    if (isGroupItem(item)) {
      const members = groupMembers(item).filter(
        (member) => memberKey(member) !== key,
      );

      if (item.id === groupId) {
        targetIndex = next.length;
        next.push({ ...item, members, lineIds: undefined });
        return;
      }

      // Drop a repeat loop that this move emptied.
      if (members.length === 0 && !keepEmptiedGroup(item)) return;

      next.push({ ...item, members, lineIds: undefined });
      return;
    }

    next.push(item);
  });

  if (targetIndex < 0) return sequence;

  const updated = next[targetIndex];
  if (!isGroupItem(updated)) return sequence;
  next[targetIndex] = {
    ...updated,
    members: [...groupMembers(updated), moving],
    lineIds: undefined,
  };

  return next;
}

/**
 * Pulls a member out of whatever group holds it and appends it to the main
 * route. Returns the original array when it was already top level.
 */
export function moveItemOutOfGroups(
  sequence: SequenceItem[],
  key: string,
): SequenceItem[] {
  if (!key) return sequence;

  const holder = groupHoldingMember(sequence, key);
  if (!holder) return sequence;

  const moving = groupMembers(holder).find((member) => memberKey(member) === key);
  if (!moving) return sequence;

  const next: SequenceItem[] = [];

  sequence.forEach((item) => {
    if (isGroupItem(item)) {
      const members = groupMembers(item).filter(
        (member) => memberKey(member) !== key,
      );
      if (members.length === 0 && !keepEmptiedGroup(item)) return;
      next.push({ ...item, members, lineIds: undefined });
      return;
    }

    // A stale top-level entry would duplicate the member once re-added.
    if (item.kind === "path" && item.lineId === key) return;
    if ((item.kind === "wait" || item.kind === "event") && item.id === key) return;

    next.push(item);
  });

  next.push(moving.kind === "path" ? { kind: "path", lineId: key } : moving);
  return next;
}

/** Back-compat names for the path-only callers. */
export const moveLineIntoGroup = moveItemIntoGroup;
export const moveLineOutOfGroups = moveItemOutOfGroups;

/**
 * Makes the step list describe exactly the paths that exist.
 *
 * The route list and the `lines` array are two views of the same thing, and
 * nothing stops a save file — or an older build — from disagreeing: a step can
 * point at a path that was deleted, and a path can be missing from the list
 * entirely. Both are invisible failures, because everything downstream (the
 * field, the time estimate, the exporters) follows the list, so a path the list
 * forgot simply stops existing while still sitting in the editor.
 *
 * This drops steps whose path is gone and appends a step for every path the
 * list never mentions, so each path appears exactly once. Returns the original
 * array when it is already consistent, so it is safe to call reactively.
 */
export function reconcileSequence(
  lines: Line[] = [],
  sequence: SequenceItem[] = [],
): SequenceItem[] {
  const knownIds = new Set(
    lines.map((line) => line.id).filter((id): id is string => Boolean(id)),
  );

  const seen = new Set<string>();
  const next: SequenceItem[] = [];
  let changed = false;

  sequence.forEach((item) => {
    if (item.kind === "path") {
      if (!knownIds.has(item.lineId) || seen.has(item.lineId)) {
        changed = true;
        return;
      }
      seen.add(item.lineId);
      next.push(item);
      return;
    }

    if (isGroupItem(item)) {
      const before = groupMembers(item);

      // Only paths are reconciled against `lines`. A wait or an event carries
      // its own data, so there is nothing for it to have gone stale against —
      // dropping one here would quietly delete a step the user put in a loop.
      const members = before.filter((member) => {
        if (member.kind !== "path") return true;
        if (!knownIds.has(member.lineId) || seen.has(member.lineId)) return false;
        seen.add(member.lineId);
        return true;
      });

      if (members.length !== before.length) changed = true;
      if (item.lineIds !== undefined) changed = true;

      // An emptied repeat loop is meaningless; an emptied `if` keeps its
      // condition so steps can be dragged back in.
      if (members.length === 0 && !keepEmptiedGroup(item)) return;

      next.push({ ...item, members, lineIds: undefined });
      return;
    }

    next.push(item);
  });

  lines.forEach((line) => {
    if (!line.id || seen.has(line.id)) return;
    seen.add(line.id);
    next.push({ kind: "path", lineId: line.id });
    changed = true;
  });

  return changed ? next : sequence;
}

/** One path the robot actually drives, in execution order. */
export type RoutePathStep = {
  kind: "path";
  lineId: string;
  line: Line;
  /** Index into the `lines` array, for name/colour/speed lookups. */
  lineIndex: number;
  startPoint: Point;
  /** 0-based repeat iteration; always 0 outside a repeat loop. */
  iteration: number;
  /** Id of the repeat loop or `if` block holding this path, if any. */
  groupId?: string;
};

/** One stationary step the robot actually runs. */
export type RouteHoldStep = {
  kind: "wait" | "event";
  item: SequenceWaitItem | SequenceEventItem;
  /** Set when the hold sits inside a repeat loop or an `if` block. */
  groupId?: string;
  /** Which pass of a repeat loop this hold belongs to. */
  iteration?: number;
};

/**
 * A Pollen Pickup the robot actually runs. Top-level only: it drives legs that
 * depend on what the camera sees, which a repeat loop's fixed slots cannot hold.
 */
export type RoutePollenStep = {
  kind: "pollen";
  item: SequencePollenItem;
  /** Where the route has the robot when the step begins - the search pose. */
  atPoint: Point;
};

export type RouteStep = RoutePathStep | RouteHoldStep | RoutePollenStep;

export type Route = {
  /** Exactly what runs, in order — repeats expanded, skipped steps removed. */
  steps: RouteStep[];
  /**
   * Where each path the sequence mentions begins, including paths in branches
   * that are not currently taken, so the field can still draw them.
   */
  startPoints: Map<string, Point>;
};

const clampRepeatCount = (value: unknown) =>
  Math.max(1, Math.min(20, Math.round(Number(value) || 1)));

/**
 * Walks the sequence once and reports both what runs and where each path
 * begins. Every consumer — the time estimate, the animation, the field
 * overlays and the exporters — reads the route from here so they can never
 * disagree about the order, the start points, or which steps are skipped.
 *
 * The rules it encodes:
 *
 * - A run of adjacent `if` blocks is one if / else-if chain. Every branch
 *   starts from the position the chain is reached at, so sibling branches
 *   share a start instead of the second continuing from the end of the first.
 *   Only the first branch whose condition holds runs, and only that branch
 *   moves the robot on.
 * - A repeat loop runs its group `count` times, each iteration continuing from
 *   where the previous one ended.
 * - A step whose `Enabled if` condition is false is skipped exactly as the
 *   generated Java skips it: it costs no time and does not move the robot.
 */
export function buildRoute(
  startPoint: Point,
  lines: Line[] = [],
  sequence: SequenceItem[] = [],
  variables: Variable[] = [],
): Route {
  const scope = buildExpressionScope(variables);
  const lineIndexById = new Map<string, number>();
  lines.forEach((line, index) => {
    if (line.id && !lineIndexById.has(line.id)) lineIndexById.set(line.id, index);
  });

  const steps: RouteStep[] = [];
  const startPoints = new Map<string, Point>();

  // With no sequence, fall back to plain array order.
  const items: SequenceItem[] = sequence.length
    ? sequence
    : lines.filter((line) => line.id).map((line) => ({
        kind: "path",
        lineId: line.id!,
      }));

  let current: Point = startPoint;

  /**
   * `record` writes the drawing start point (only the first pass of a repeat
   * needs it); `execute` says whether this pass actually runs.
   */
  const runLine = (
    lineId: string,
    from: Point,
    record: boolean,
    execute: boolean,
    iteration: number,
    groupId?: string,
  ): Point => {
    const lineIndex = lineIndexById.get(lineId);
    const line = lineIndex === undefined ? undefined : lines[lineIndex];
    if (!line?.endPoint) return from;

    if (record) startPoints.set(lineId, from);

    // A disabled path never moves the robot, so the route carries on from the
    // same place whether or not this pass runs.
    if (!isEnabled(line, variables, scope)) return from;

    if (execute) {
      steps.push({
        kind: "path",
        lineId,
        line,
        lineIndex: lineIndex as number,
        startPoint: from,
        iteration,
        groupId,
      });
    }

    return line.endPoint as Point;
  };

  const walkGroup = (
    members: SequenceGroupMember[] | undefined,
    from: Point,
    record: boolean,
    execute: boolean,
    iteration: number,
    groupId: string,
  ): Point => {
    let position = from;
    (members || []).forEach((member) => {
      if (member.kind === "path") {
        position = runLine(member.lineId, position, record, execute, iteration, groupId);
        return;
      }

      // A wait or an event inside a loop happens on every pass, in its place in
      // the order — that is the whole reason for putting one there. `execute` is
      // false on the pass that only records where paths sit, so a disabled loop
      // still draws its geometry without spending time it never spends.
      if (execute && isEnabled(member, variables, scope)) {
        steps.push({ kind: member.kind, item: member, groupId, iteration });
      }
    });
    return position;
  };

  for (let index = 0; index < items.length; index++) {
    const item = items[index];

    if (item.kind === "path") {
      current = runLine(item.lineId, current, true, true, 0);
      continue;
    }

    if (item.kind === "wait" || item.kind === "event") {
      if (isEnabled(item, variables, scope)) {
        steps.push({ kind: item.kind, item });
      }
      continue;
    }

    // A Pollen Pickup returns the robot to where it started (or it is flagged
    // when it does not), so the route carries on from the same place.
    if (item.kind === "pollen") {
      if (isEnabled(item, variables, scope)) {
        steps.push({ kind: "pollen", item, atPoint: current });
      }
      continue;
    }

    if (item.kind === "repeat") {
      const runs = isEnabled(item, variables, scope)
        ? clampRepeatCount(item.count)
        : 0;

      if (runs === 0) {
        // Still record where the paths sit so they can be drawn.
        walkGroup(groupMembers(item), current, true, false, 0, item.id);
        continue;
      }

      for (let iteration = 0; iteration < runs; iteration++) {
        current = walkGroup(
          groupMembers(item),
          current,
          iteration === 0,
          true,
          iteration,
          item.id,
        );
      }
      continue;
    }

    if (item.kind === "conditional") {
      const branchStart = current;
      let continuation: Point | null = null;

      while (index < items.length && items[index].kind === "conditional") {
        const branch = items[index] as SequenceConditionalItem;
        const runs =
          continuation === null && isConditionalActive(branch, variables, scope);
        const branchEnd = walkGroup(
          groupMembers(branch),
          branchStart,
          true,
          runs,
          0,
          branch.id,
        );
        if (runs) continuation = branchEnd;
        index++;
      }
      index--; // The outer loop advances past the last branch.

      current = continuation ?? branchStart;
      continue;
    }
  }

  return { steps, startPoints };
}

/** Why the robot has to come to a stop at the end of a chain. */
export type ChainBreakReason =
  | "end"
  | "wait"
  | "loop"
  | "branch"
  | "stopAtEnd"
  | "speed"
  | "condition"
  | "turn";

export const CHAIN_BREAK_LABELS: Record<ChainBreakReason, string> = {
  end: "end of the route",
  wait: "a wait, event or pollen pickup follows",
  loop: "the repeat loop starts its next pass",
  branch: "the block ends",
  stopAtEnd: "Stop at end is on",
  speed: "the next path uses a different speed",
  condition: "a path here can be switched off at runtime",
  turn: "the next path turns in place first",
};

/**
 * A run of consecutive paths driven as one Pedro path chain.
 *
 * A chain is exported as a single `Paths.path(...)`. Pedro 3's Foresight
 * follower brakes only for the end of the whole thing: when it would start
 * braking on a path that is not the last, it skips on to the next one instead
 * (`ForesightConfig.pathSkip`, on by default). So merging consecutive paths is
 * what stops the robot from braking to zero and settling on every waypoint.
 * Everything reads the grouping from here: the time estimate profiles each chain
 * as one move, the exporter emits one `Paths.path(...)` per chain, and the
 * editor marks where the robot really stops.
 */
export type ChainRun = {
  index: number;
  steps: RoutePathStep[];
  /** Uniform across the run — a speed change starts a new chain. */
  speed: number;
  /** Why the robot stops at the end of this run. */
  breakReason: ChainBreakReason;
};

const resolvedSpeed = (line: Line): number => {
  const speed = Number(line.speed ?? 1);
  if (!Number.isFinite(speed)) return 1;
  return Math.max(0.05, Math.min(1, speed));
};

/** A path that can be switched off at runtime has to stay its own chain. */
const isSwitchable = (step: RoutePathStep): boolean =>
  Boolean(step.line.enabledExpression?.trim());

/**
 * Groups the route's paths into the chains they are driven as.
 *
 * A chain is cut where the robot genuinely cannot carry on: a stationary step
 * follows, control flow changes, the power would have to change mid-chain, a
 * path might be skipped at runtime, or the author asked for a stop.
 */
export function buildChainRuns(steps: RouteStep[] = []): ChainRun[] {
  const runs: ChainRun[] = [];
  let current: RoutePathStep[] = [];
  let pendingBreak: ChainBreakReason = "end";

  const flush = (reason: ChainBreakReason) => {
    if (!current.length) return;
    runs.push({
      index: runs.length,
      steps: current,
      speed: resolvedSpeed(current[0].line),
      breakReason: reason,
    });
    current = [];
  };

  steps.forEach((step, stepIndex) => {
    if (step.kind !== "path") {
      // A wait or event holds the robot still, ending the chain before it.
      flush("wait");
      return;
    }

    current.push(step);

    // Look ahead to decide whether the chain carries on into the next path.
    const next = steps[stepIndex + 1];

    if (!next) {
      pendingBreak = "end";
      flush("end");
      return;
    }

    if (next.kind !== "path") return; // The hold step above will flush it.

    if (step.line.stopAtEnd) {
      flush("stopAtEnd");
      return;
    }

    // Turn first: the robot stops and turns in place before the next path.
    if (next.line.turnFirst) {
      flush("turn");
      return;
    }

    if (isSwitchable(step) || isSwitchable(next)) {
      flush("condition");
      return;
    }

    if (step.groupId !== next.groupId) {
      flush("branch");
      return;
    }

    if (step.iteration !== next.iteration) {
      flush("loop");
      return;
    }

    if (resolvedSpeed(step.line) !== resolvedSpeed(next.line)) {
      flush("speed");
      return;
    }
  });

  flush(pendingBreak);
  return runs;
}

/**
 * Where each path begins, following the sequence rather than array order.
 *
 * Paths the sequence never reaches deliberately get no entry: they are not
 * part of the route, so callers can tell them apart and skip drawing them
 * rather than showing a path that the list does not contain.
 */
export function buildLineStartPoints(
  startPoint: Point,
  lines: Line[] = [],
  sequence: SequenceItem[] = [],
  variables: Variable[] = [],
): Map<string, Point> {
  return buildRoute(startPoint, lines, sequence, variables).startPoints;
}

/**
 * Groups adjacent `if` blocks into if / else-if chains.
 *
 * Returns, for each conditional item, the conditions of the branches that
 * precede it in its chain. A branch runs only when its own condition holds
 * *and* no earlier branch in the chain claimed the turn.
 */
export function conditionalChainPredecessors(
  sequence: SequenceItem[] = [],
): Map<string, string[]> {
  const predecessors = new Map<string, string[]>();

  let run: SequenceConditionalItem[] = [];
  const flush = () => {
    run.forEach((branch, index) => {
      predecessors.set(
        branch.id,
        run
          .slice(0, index)
          .map((earlier) => earlier.condition?.trim() || "")
          .filter(Boolean),
      );
    });
    run = [];
  };

  sequence.forEach((item) => {
    if (item.kind === "conditional") {
      run.push(item);
      return;
    }
    flush();
  });
  flush();

  return predecessors;
}

/** Ids of the `if` blocks that actually run, one per if / else-if chain. */
export function activeConditionalIds(
  sequence: SequenceItem[] = [],
  variables: Variable[] = [],
): Set<string> {
  const active = new Set<string>();

  let claimed = false;
  sequence.forEach((item, index) => {
    if (item.kind !== "conditional") return;

    const startsChain =
      index === 0 || sequence[index - 1].kind !== "conditional";
    if (startsChain) claimed = false;

    if (claimed) return;
    if (!isConditionalActive(item, variables)) return;

    active.add(item.id);
    claimed = true;
  });

  return active;
}
