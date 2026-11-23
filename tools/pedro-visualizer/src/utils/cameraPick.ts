/**
 * Camera pick and Turn first - the two route edits the RED / BLUE solo autos
 * hand to TeamCode's auto/modules/CameraPick.java.
 *
 * Camera pick (on a path): partway down the path the robot reads ONE frame of
 * PollenDetectionPipeline, picks the biggest clump against the end wall and is
 * re-routed into it, ending pressed on the wall. The editor cannot see a
 * camera, so the clump is a draggable marker (`previewX`) and the route is
 * drawn, timed, animated and collision-checked as if the robot had picked it.
 * The geometry here is CameraPick.java's, line for line:
 *
 *   - the switch happens where the path passes the point nearest
 *     (path end x, pickY) - Java: addPoseCallback(0, new Pose(POINT.x,
 *     cameraPick.pickY()), ...), which fires once the curve parameter passes the
 *     closest parameter to that pose;
 *   - end = (clamp(clump x, xMin, xMax), wallY), heading = the path's end heading;
 *   - CameraPick.slamPath(from, end): cubic from -> (from.x, from.y + 0.45 dy)
 *     -> (end.x, end.y - 0.35 dy) -> end, dy = end.y - from.y, heading held;
 *   - the next path starts from `end`, its own control points unchanged.
 *
 * Turn first (on a path): as soon as the previous path ends - before any waits
 * or events between the two, so a Shoot there happens after the turn - the robot backs straight off any
 * field wall (holding its heading) until its corners' sweep clears it, turns
 * in place there to the path's end heading, then drives the path from that
 * point holding it - Java runTurnStep + TurnInPlace.clearOfWalls. (Until
 * 2026-11-23 it turned where it stood: pressed on the wall after a camera
 * pick, an 18 in robot's corners swept into the wall.)
 *
 * `expandRouteEdits` turns both into plain lines / sequence items, so every
 * consumer that times, draws, animates or checks the route sees them without
 * knowing about either feature. The exporter reads the original lines instead.
 */
import type { BasePoint, CameraPickConfig, Line, Point, SequenceItem, SequenceWaitItem } from "../types";
import { FIELD_SIZE } from "../config/defaults";
import { getCurvePoint, getLineEndHeading, getLineStartHeading, goalHeadingAt } from "./math";

/** AutoConstants.Autonomous: TURN_TOLERANCE_DEG / TURN_TIMEOUT_MS. The time model charges the rotation itself. */
export const TURN_TIMEOUT_SECONDS = 1.2;
/** A short settle on top of the pure rotation time, for the hold to converge inside 4 deg. */
export const TURN_SETTLE_SECONDS = 0.15;

/** SystemConstants.Camera.HFOV_DEG / CAMERA_OFFSET forward. Both ESTIMATED on the robot. */
export const DEFAULT_HFOV_DEG = 43;
export const DEFAULT_CAMERA_FORWARD = 9;
/** AutoConstants.Autonomous.BALL_LINE_Y: clump centres this far out from the wall. */
export const BALL_LINE_OFFSET = 1.4;

/** Suffix of the synthetic line that carries the re-routed leg. */
export const PICK_LEG_SUFFIX = "::camera-pick";
export const TURN_BACKOFF_SUFFIX = "::turn-backoff";
/** AutoConstants.Autonomous.TURN_WALL_MARGIN_IN: clearance past the corners' sweep. */
export const TURN_WALL_MARGIN_IN = 0.75;
/** AutoConstants.Autonomous.TURN_BACKOFF_TOLERANCE_IN: a back-off shorter than this is skipped. */
export const TURN_BACKOFF_TOLERANCE_IN = 0.5;

export interface RobotFootprint {
  /** Along the heading, in (Settings rWidth). */
  length: number;
  /** Across the heading, in (Settings rHeight). */
  width: number;
}

/**
 * TurnInPlace.clearOfWalls: the point a turn happens at - moved straight off any field wall until the
 * circle the corners sweep (half-diagonal + TURN_WALL_MARGIN_IN) clears it. Obstacles are not considered.
 */
export function turnClearPoint(at: BasePoint, robot: RobotFootprint): BasePoint {
  const r = Math.hypot(robot.length / 2, robot.width / 2) + TURN_WALL_MARGIN_IN;
  const clamp = (v: number) => Math.max(r, Math.min(FIELD_SIZE - r, v));
  return { x: clamp(at.x), y: clamp(at.y) };
}

/** The robot the route edits assume: the project's rWidth x rHeight, or TeamCode's 18 x 15 in. */
export function footprintOf(settings?: { rWidth?: number; rHeight?: number } | null): RobotFootprint {
  const length = Number(settings?.rWidth);
  const width = Number(settings?.rHeight);
  return {
    length: Number.isFinite(length) && length > 0 ? length : 18,
    width: Number.isFinite(width) && width > 0 ? width : 15,
  };
}
export const TURN_ITEM_PREFIX = "turn-first::";

const finite = (value: unknown, fallback: number) =>
  Number.isFinite(Number(value)) ? Number(value) : fallback;

/**
 * Fills in a camera pick block. The RED defaults are AutoConstants.Autonomous
 * (PICK_Y 59, WALL_Y 9, PICK_X 8.5..81); a block on the far half of the field
 * gets the BLUE mirror (CameraPick.java turns the red constants 180 deg).
 */
export function normalizeCameraPick(input: unknown, endPoint?: BasePoint): CameraPickConfig | undefined {
  if (!input || typeof input !== "object") return undefined;
  const source = input as Partial<CameraPickConfig>;
  const blue =
    source.alliance === "BLUE" ||
    (source.alliance !== "RED" && finite(source.wallY, 9) > FIELD_SIZE / 2);
  const mirror = (value: number) => (blue ? FIELD_SIZE - value : value);
  const xMinRed = 8.5;
  const xMaxRed = 81;
  let xMin = finite(source.xMin, blue ? FIELD_SIZE - xMaxRed : xMinRed);
  let xMax = finite(source.xMax, blue ? FIELD_SIZE - xMinRed : xMaxRed);
  if (xMin > xMax) [xMin, xMax] = [xMax, xMin];
  return {
    enabled: source.enabled !== false,
    alliance: blue ? "BLUE" : "RED",
    pickY: finite(source.pickY, mirror(59)),
    wallY: finite(source.wallY, mirror(9)),
    xMin,
    xMax,
    previewX: finite(source.previewX, finite(endPoint?.x, (xMin + xMax) / 2)),
    hfovDeg: Math.max(1, Math.min(170, finite(source.hfovDeg, DEFAULT_HFOV_DEG))),
    cameraForward: finite(source.cameraForward, DEFAULT_CAMERA_FORWARD),
  };
}

export function activeCameraPick(line: Line | undefined): CameraPickConfig | undefined {
  const pick = line?.cameraPick ? normalizeCameraPick(line.cameraPick, line.endPoint) : undefined;
  return pick?.enabled ? pick : undefined;
}

/** The wall's ball line: clump centres sit BALL_LINE_OFFSET in from the wall the robot drives at. */
export function ballLineY(pick: CameraPickConfig): number {
  return pick.wallY > FIELD_SIZE / 2 ? FIELD_SIZE - BALL_LINE_OFFSET : BALL_LINE_OFFSET;
}

/** Splits a Bezier of any degree at `t` (de Casteljau); returns the control polygon of [0, t]. */
function splitBezierLeft(points: BasePoint[], t: number): BasePoint[] {
  const left: BasePoint[] = [];
  let level = points.map((p) => ({ x: p.x, y: p.y }));
  while (level.length) {
    left.push(level[0]);
    const next: BasePoint[] = [];
    for (let i = 0; i + 1 < level.length; i++) {
      next.push({
        x: level[i].x + (level[i + 1].x - level[i].x) * t,
        y: level[i].y + (level[i + 1].y - level[i].y) * t,
      });
    }
    level = next;
  }
  return left;
}

export interface CameraPickGeometry {
  pick: CameraPickConfig;
  /** Curve parameter where the frame is read and the path is replaced. */
  pickT: number;
  /** Robot pose at the pick, heading in degrees. */
  pickPose: { x: number; y: number; heading: number };
  /** The path up to the pick: control points and end. */
  before: { controlPoints: BasePoint[]; endPoint: BasePoint };
  /** CameraPick.slamPath: two control points and the end against the wall. */
  slam: { controlPoints: BasePoint[]; endPoint: BasePoint; heading: number };
  /** The clump as dragged, and the x driven to after the clamp. */
  clumpX: number;
  clamped: boolean;
  /** Camera on the robot at the pick, and where the edges of its view meet the ball line (null: never). */
  camera: { x: number; y: number; leftX: number | null; rightX: number | null };
  /** False when the clump is outside the camera's view at the pick. */
  clumpInView: boolean;
}

/** The whole camera-pick picture for one path started at `start`. */
export function cameraPickGeometry(line: Line, start: BasePoint): CameraPickGeometry | null {
  const pick = activeCameraPick(line);
  if (!pick || !line.endPoint) return null;
  const curve: BasePoint[] = [start, ...(line.controlPoints || []), line.endPoint];

  // Closest parameter to (end x, pickY): a coarse scan, then a fine one around the best.
  const target = { x: line.endPoint.x, y: pick.pickY };
  const distanceAt = (t: number) => {
    const p = getCurvePoint(t, curve);
    return Math.hypot(p.x - target.x, p.y - target.y);
  };
  let bestT = 0;
  let best = Infinity;
  for (let i = 0; i <= 200; i++) {
    const d = distanceAt(i / 200);
    if (d < best) {
      best = d;
      bestT = i / 200;
    }
  }
  for (let i = -20; i <= 20; i++) {
    const t = Math.max(0, Math.min(1, bestT + i / 4000));
    const d = distanceAt(t);
    if (d < best) {
      best = d;
      bestT = t;
    }
  }
  // A pick at either end leaves nothing to re-route; keep a sliver of path either side.
  const pickT = Math.max(0.001, Math.min(0.999, bestT));
  const from = getCurvePoint(pickT, curve);
  const pickHeading = goalHeadingAt(line, curve, pickT);
  const endHeading = getLineEndHeading(line, start as Point);

  const left = splitBezierLeft(curve, pickT);
  const before = {
    controlPoints: left.slice(1, -1),
    endPoint: left[left.length - 1],
  };

  const clumpX = pick.previewX;
  const endX = Math.max(pick.xMin, Math.min(pick.xMax, clumpX));
  const end = { x: endX, y: pick.wallY };
  const dy = end.y - from.y;
  const slam = {
    controlPoints: [
      { x: from.x, y: from.y + 0.45 * dy },
      { x: end.x, y: end.y - 0.35 * dy },
    ],
    endPoint: end,
    heading: endHeading,
  };

  // The camera wedge: lens CAMERA_FORWARD ahead of centre along the pick heading,
  // edges at +-HFOV/2, intersected with the ball line - WallClumpPicker's model.
  const h = (pickHeading * Math.PI) / 180;
  const camX = from.x + pick.cameraForward * Math.cos(h);
  const camY = from.y + pick.cameraForward * Math.sin(h);
  const lineY = ballLineY(pick);
  const edgeX = (offsetDeg: number): number | null => {
    const b = h + (offsetDeg * Math.PI) / 180;
    const sy = Math.sin(b);
    if (Math.abs(sy) < 1e-3) return null;
    const dist = (lineY - camY) / sy;
    if (dist <= 0) return null;
    return camX + dist * Math.cos(b);
  };
  const leftX = edgeX(pick.hfovDeg / 2);
  const rightX = edgeX(-pick.hfovDeg / 2);
  const clumpInView =
    leftX !== null && rightX !== null
      ? clumpX >= Math.min(leftX, rightX) - 1e-6 && clumpX <= Math.max(leftX, rightX) + 1e-6
      : false;

  return {
    pick,
    pickT,
    pickPose: { x: from.x, y: from.y, heading: pickHeading },
    before,
    slam,
    clumpX,
    clamped: Math.abs(endX - clumpX) > 1e-6,
    camera: { x: camX, y: camY, leftX, rightX },
    clumpInView,
  };
}

/** A line's heading held at `deg` the whole way. */
function heldHeading(line: Line, deg: number): Point {
  const { heading, degrees, startDeg, endDeg, headingCurve, reverse, ...rest } = line.endPoint as any;
  return { ...rest, heading: "linear", startDeg: deg, endDeg: deg, headingCurve: 1 } as Point;
}

/** The path with its heading held at its own end heading (Turn first, after the in-place turn). */
export function holdEndHeading(line: Line, start?: BasePoint): Line {
  const deg = getLineEndHeading(line, (start ?? line.endPoint) as Point);
  return { ...line, endPoint: heldHeading(line, deg) };
}

/** End heading of a path, as Turn first turns to it. */
export function turnFirstHeading(line: Line, start: BasePoint): number {
  return getLineEndHeading(line, start as Point);
}

/**
 * The route the robot drives once both edits are applied, as plain lines and
 * sequence items:
 *
 * - a camera-pick path is cut at the pick, and a synthetic line (id + PICK_LEG_SUFFIX,
 *   appended at the END of the array so every original index still means the same path)
 *   carries the drive into the wall, right after it in the sequence;
 * - a Turn first path gets its heading held at its end heading; before it, a synthetic
 *   back-off line (id + TURN_BACKOFF_SUFFIX, appended like the pick leg) when the robot
 *   is too close to a wall to turn, then a wait item that turns it in place (`turnToDeg`).
 *
 * Top-level sequence items only, like Pollen Pickup: a repeat loop's fixed slots
 * cannot hold legs that depend on what the camera sees.
 */
export interface CameraPickPreview {
  /** Index and id of the path in the EDITOR's lines. */
  lineIndex: number;
  lineId: string;
  start: BasePoint;
  geometry: CameraPickGeometry;
  /** The planned path past the pick - what runs when nothing is picked. Sampled. */
  plannedRest: BasePoint[];
}

export function expandRouteEdits(
  startPoint: Point,
  lines: Line[],
  sequence: SequenceItem[],
  settings?: { rWidth?: number; rHeight?: number } | null,
): { lines: Line[]; sequence: SequenceItem[]; previews: CameraPickPreview[] } {
  const robot = footprintOf(settings);
  const previews: CameraPickPreview[] = [];
  const hasEdit = lines.some((line) => activeCameraPick(line) || line.turnFirst);
  if (!hasEdit) return { lines, sequence, previews };

  const byId = new Map<string, number>();
  lines.forEach((line, index) => {
    if (line.id) byId.set(line.id, index);
  });
  const outLines = lines.slice();
  const outSequence: SequenceItem[] = [];
  const items: SequenceItem[] = sequence.length
    ? sequence
    : lines.filter((line) => line.id).map((line) => ({ kind: "path", lineId: line.id! }));

  // Where in outSequence the robot last moved: a Turn first goes here, ahead of the waits / events after it.
  let afterLastMove = 0;
  // Where the robot is along the top-level route, to place picks and turns.
  let current: BasePoint = startPoint;
  // Heading the robot holds at `current`, degrees (needed to back off a wall without turning).
  const sp = startPoint as any;
  let currentDeg: number = Number.isFinite(Number(sp.degrees)) ? Number(sp.degrees)
    : Number.isFinite(Number(sp.startDeg)) ? Number(sp.startDeg) : 0;
  for (const item of items) {
    if (item.kind !== "path") {
      outSequence.push(item);
      // Groups move the robot by their own paths; follow them by their last path's end.
      if (item.kind === "repeat" || item.kind === "conditional" || item.kind === "pollen") {
        afterLastMove = outSequence.length;
      }
      if (item.kind === "repeat" || item.kind === "conditional") {
        const members = (item.members || []).filter((member) => member.kind === "path");
        const last = members[members.length - 1];
        const index = last && last.kind === "path" ? byId.get(last.lineId) : undefined;
        if (index !== undefined && lines[index]?.endPoint) current = lines[index].endPoint;
      }
      continue;
    }
    const index = byId.get(item.lineId);
    const line = index === undefined ? undefined : lines[index];
    if (!line?.endPoint || index === undefined) {
      outSequence.push(item);
      afterLastMove = outSequence.length;
      continue;
    }

    let effective = line;
    const turnSteps: SequenceItem[] = [];
    if (line.turnFirst) {
      const deg = turnFirstHeading(line, current);
      effective = { ...effective, endPoint: heldHeading(effective, deg) };
      const clear = turnClearPoint(current, robot);
      if (Math.hypot(clear.x - current.x, clear.y - current.y) > TURN_BACKOFF_TOLERANCE_IN) {
        const backId = `${line.id}${TURN_BACKOFF_SUFFIX}`;
        const back: Line = {
          id: backId,
          name: `${line.name || "Path"} - back off the wall`,
          color: line.color,
          speed: line.speed,
          controlPoints: [],
          endPoint: {
            x: clear.x,
            y: clear.y,
            heading: "linear",
            startDeg: currentDeg,
            endDeg: currentDeg,
            headingCurve: 1,
          } as Point,
          stopAtEnd: true,
          eventMarkers: [],
        };
        outLines.push(back);
        turnSteps.push({ kind: "path", lineId: backId });
        current = clear;
      }
      const turn: SequenceWaitItem = {
        kind: "wait",
        id: `${TURN_ITEM_PREFIX}${line.id}`,
        name: "Turn",
        durationMs: 0,
        turnToDeg: deg,
      };
      turnSteps.push(turn);
      // The path before the turn ends at a full stop (Java: followPathStep / followPickStep wait for
      // !isBusy() before the turn step) - with no wait or event left between them, the chain rules would
      // otherwise drive it straight on into the back-off.
      const prev = outSequence[afterLastMove - 1];
      if (prev && prev.kind === "path") {
        const at = outLines.findIndex((candidate) => candidate.id === prev.lineId);
        if (at >= 0) outLines[at] = { ...outLines[at], stopAtEnd: true };
      }
      outSequence.splice(afterLastMove, 0, ...turnSteps);
    }

    const geometry = cameraPickGeometry(effective, current);
    if (geometry) {
      const startHeading = getLineStartHeading(effective, current as Point);
      const pickDeg = geometry.pickPose.heading;
      const before: Line = {
        ...effective,
        controlPoints: geometry.before.controlPoints,
        endPoint: {
          ...(effective.endPoint as any),
          x: geometry.before.endPoint.x,
          y: geometry.before.endPoint.y,
          heading: "linear",
          startDeg: startHeading,
          endDeg: pickDeg,
          headingCurve: 1,
          degrees: undefined,
          reverse: undefined,
        } as Point,
        // The pick switch is mid-path: the robot carries on into the new path without stopping.
        stopAtEnd: false,
        // Markers stay on the drive up to the pick (Java: they are on the original path).
        eventMarkers: line.eventMarkers,
      };
      const legId = `${line.id}${PICK_LEG_SUFFIX}`;
      const leg: Line = {
        id: legId,
        name: `${line.name || "Path"} - camera pick`,
        color: line.color,
        speed: line.speed,
        controlPoints: geometry.slam.controlPoints,
        endPoint: {
          x: geometry.slam.endPoint.x,
          y: geometry.slam.endPoint.y,
          heading: "linear",
          startDeg: geometry.slam.heading,
          endDeg: geometry.slam.heading,
          headingCurve: 1,
        } as Point,
        stopAtEnd: line.stopAtEnd,
        eventMarkers: [],
      };
      const fullCurve = [current, ...(effective.controlPoints || []), effective.endPoint];
      const plannedRest: BasePoint[] = [];
      for (let i = 0; i <= 40; i++) {
        plannedRest.push(getCurvePoint(geometry.pickT + ((1 - geometry.pickT) * i) / 40, fullCurve));
      }
      previews.push({ lineIndex: index, lineId: line.id!, start: current, geometry, plannedRest });
      outLines[index] = before;
      outLines.push(leg);
      outSequence.push(item, { kind: "path", lineId: legId });
      afterLastMove = outSequence.length;
      current = leg.endPoint;
      currentDeg = geometry.slam.heading;
      continue;
    }

    if (effective !== line) outLines[index] = effective;
    outSequence.push(item);
    afterLastMove = outSequence.length;
    currentDeg = getLineEndHeading(effective, current as Point);
    current = line.endPoint;
  }

  return { lines: outLines, sequence: outSequence, previews };
}
