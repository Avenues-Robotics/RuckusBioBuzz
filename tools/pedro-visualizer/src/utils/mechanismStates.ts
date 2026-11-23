/**
 * What the auto's mechanisms are doing at every moment of playback - TeamCode's
 * own rules, so the editor shows what the robot will do, not what the event
 * names suggest.
 *
 * Rules (TeamCode auto/modules/AutoMechanisms.java + the generated autos):
 *   - "Intake On" starts the intake and it STAYS on (finishing the event does nothing);
 *     "Intake Off" stops it; a timed "Intake" runs it for its duration.
 *   - A camera-pick path (followPickStep) sends "Intake Off" when the robot stops at the wall.
 *   - "Shoot": while it runs the intake turns only while the gate is open, feeding the shooter;
 *     its FINISH stops the intake until the next collect. The editor cannot know when the gate
 *     opens (the robot waits for the wheel at speed and the turret aimed), so the whole Shoot
 *     span is drawn as "feeding" - read it as "the gate may be open".
 *   - "Scoopula Down" / "Scoopula Up" flip the scoopula at the marker; the marker's duration
 *     (300 ms) is the servo travelling, drawn as "lowering" / "raising".
 *   - Camera pick: the instant the frame is read (start of the pick leg). Turn first: the
 *     back-off leg and the in-place turn, as their own phases.
 * Any other event is listed by name while it runs.
 */
import type {
  BasePoint,
  Line,
  Point,
  SequenceItem,
  Settings,
  TimePrediction,
  Variable,
} from "../types";
import { buildEventTimingWindows } from "./timeCalculator";
import { PICK_LEG_SUFFIX, TURN_BACKOFF_SUFFIX, TURN_ITEM_PREFIX } from "./cameraPick";

export type IntakeMode = "off" | "on" | "feed";
export type ScoopulaMode = "up" | "down" | "lowering" | "raising";
export type MotionPhase = "backoff" | "turn" | null;

export interface MechanismSpan {
  start: number;
  end: number;
  label: string;
}

export interface MechanismChange {
  time: number;
  lane: "intake" | "scoopula" | "shoot" | "phase" | "pick" | "event";
  label: string;
}

export interface MechanismTimeline {
  total: number;
  intakeOn: MechanismSpan[];
  intakeFeed: MechanismSpan[];
  scoopulaDown: MechanismSpan[];
  scoopulaMoving: MechanismSpan[];
  shoot: MechanismSpan[];
  backoff: MechanismSpan[];
  turn: MechanismSpan[];
  picks: number[];
  otherEvents: MechanismSpan[];
  changes: MechanismChange[];
}

export interface MechanismState {
  intake: IntakeMode;
  scoopula: ScoopulaMode;
  shooting: boolean;
  phase: MotionPhase;
  /** Within half a second after the camera pick fired. */
  pickFlash: boolean;
  /** Other named events running now. */
  events: string[];
}

type Action = { time: number; order: number; kind: "start" | "finish" | "pickLegEnd"; name: string };

const KNOWN = new Set(["Intake On", "Intake Off", "Intake", "Shoot", "Scoopula Down", "Scoopula Up"]);

export function buildMechanismTimeline(
  startPoint: Point,
  routeLines: Line[],
  routeSequence: SequenceItem[],
  timePrediction: TimePrediction | null | undefined,
  settings: Settings,
  variables: Variable[] = [],
): MechanismTimeline {
  const timeline = timePrediction?.timeline || [];
  const total = Math.max(0, Number(timePrediction?.totalTime) || 0);
  const result: MechanismTimeline = {
    total,
    intakeOn: [],
    intakeFeed: [],
    scoopulaDown: [],
    scoopulaMoving: [],
    shoot: [],
    backoff: [],
    turn: [],
    picks: [],
    otherEvents: [],
    changes: [],
  };
  if (!timeline.length || total <= 0) return result;

  const eventItemIds = new Set(
    routeSequence.filter((item) => item.kind === "event").map((item) => (item as any).id as string),
  );
  const actions: Action[] = [];
  // Finishes before starts at the same instant (a Shoot ending as the next marker fires).
  const push = (time: number, kind: Action["kind"], name: string) =>
    actions.push({ time, order: kind === "start" ? 1 : 0, kind, name });

  for (const event of timeline) {
    if (event.type === "travel") {
      const id = routeLines[event.lineIndex ?? -1]?.id || "";
      if (id.endsWith(PICK_LEG_SUFFIX)) {
        result.picks.push(event.startTime);
        result.changes.push({ time: event.startTime, lane: "pick", label: "Camera pick" });
        push(event.endTime, "pickLegEnd", "Intake Off");
      } else if (id.endsWith(TURN_BACKOFF_SUFFIX)) {
        result.backoff.push({ start: event.startTime, end: event.endTime, label: "Back off" });
        result.changes.push({ time: event.startTime, lane: "phase", label: "Back off the wall" });
      }
      continue;
    }
    if (event.type !== "wait") continue;
    if (event.itemId?.startsWith(TURN_ITEM_PREFIX)) {
      result.turn.push({ start: event.startTime, end: event.endTime, label: "Turn" });
      result.changes.push({ time: event.startTime, lane: "phase", label: "Turn in place" });
      continue;
    }
    if (event.itemId && eventItemIds.has(event.itemId)) {
      const name = event.name?.trim() || "Event";
      push(event.startTime, "start", name);
      push(event.endTime, "finish", name);
    }
  }

  // Path markers, timed on the route as driven.
  for (const window of buildEventTimingWindows(
    startPoint,
    routeLines,
    timePrediction,
    settings,
    routeSequence,
    variables,
  )) {
    push(window.startTime, "start", window.name);
    // A zero duration fires once (Java finishes it straight away; for Intake On that is a no-op).
    push(window.endTime, "finish", window.name);
  }

  actions.sort((a, b) => a.time - b.time || a.order - b.order);

  let intaking = false;
  let shooting = false;
  let scoopDown = false;
  const scoopMoves: { start: number; end: number; down: boolean }[] = [];
  const otherStarts = new Map<string, number[]>();
  let lastIntake: IntakeMode = "off";
  let intakeSince = 0;
  let lastShoot = false;
  let shootSince = 0;
  let lastScoop = false;
  let scoopSince = 0;

  const intakeMode = (): IntakeMode => (shooting ? "feed" : intaking ? "on" : "off");
  const flush = (time: number) => {
    const mode = intakeMode();
    if (mode !== lastIntake) {
      if (lastIntake === "on") result.intakeOn.push({ start: intakeSince, end: time, label: "Intake on" });
      if (lastIntake === "feed") result.intakeFeed.push({ start: intakeSince, end: time, label: "Feeding" });
      result.changes.push({
        time,
        lane: "intake",
        label: mode === "on" ? "Intake on" : mode === "feed" ? "Intake feeds shooter" : "Intake off",
      });
      lastIntake = mode;
      intakeSince = time;
    }
    if (shooting !== lastShoot) {
      if (lastShoot) result.shoot.push({ start: shootSince, end: time, label: "Shoot" });
      if (shooting) result.changes.push({ time, lane: "shoot", label: "Shoot" });
      lastShoot = shooting;
      shootSince = time;
    }
    if (scoopDown !== lastScoop) {
      if (lastScoop) result.scoopulaDown.push({ start: scoopSince, end: time, label: "Scoopula down" });
      result.changes.push({ time, lane: "scoopula", label: scoopDown ? "Scoopula down" : "Scoopula up" });
      lastScoop = scoopDown;
      scoopSince = time;
    }
  };

  let i = 0;
  while (i < actions.length) {
    const time = actions[i].time;
    while (i < actions.length && actions[i].time === time) {
      const a = actions[i++];
      if (a.kind === "pickLegEnd") {
        intaking = false;
        continue;
      }
      if (a.kind === "start") {
        switch (a.name) {
          case "Intake On":
          case "Intake":
            intaking = true;
            break;
          case "Intake Off":
            intaking = false;
            break;
          case "Shoot":
            shooting = true;
            break;
          case "Scoopula Down":
            scoopDown = true;
            scoopMoves.push({ start: time, end: time, down: true });
            break;
          case "Scoopula Up":
            scoopDown = false;
            scoopMoves.push({ start: time, end: time, down: false });
            break;
          default: {
            const starts = otherStarts.get(a.name) || [];
            starts.push(time);
            otherStarts.set(a.name, starts);
            result.changes.push({ time, lane: "event", label: a.name });
          }
        }
      } else {
        switch (a.name) {
          case "Intake":
            intaking = false;
            break;
          case "Shoot":
            shooting = false;
            intaking = false; // AutoMechanisms.finish("Shoot")
            break;
          case "Scoopula Down":
          case "Scoopula Up": {
            const move = [...scoopMoves].reverse().find((m) => m.end === m.start && m.down === (a.name === "Scoopula Down"));
            if (move) move.end = time;
            break;
          }
          default:
            if (!KNOWN.has(a.name)) {
              const starts = otherStarts.get(a.name);
              const start = starts?.shift();
              if (start !== undefined && time > start) result.otherEvents.push({ start, end: time, label: a.name });
            }
        }
      }
    }
    flush(time);
  }
  flush(total);
  // Close what is still running at the end of auto.
  // (casts: TypeScript does not see flush() assign these)
  const endIntake = lastIntake as IntakeMode;
  if (endIntake === "on") result.intakeOn.push({ start: intakeSince, end: total, label: "Intake on" });
  if (endIntake === "feed") result.intakeFeed.push({ start: intakeSince, end: total, label: "Feeding" });
  if (lastShoot) result.shoot.push({ start: shootSince, end: total, label: "Shoot" });
  if (lastScoop) result.scoopulaDown.push({ start: scoopSince, end: total, label: "Scoopula down" });
  for (const move of scoopMoves) {
    if (move.end > move.start) {
      result.scoopulaMoving.push({ start: move.start, end: move.end, label: move.down ? "lowering" : "raising" });
    }
  }
  result.changes.sort((a, b) => a.time - b.time);
  return result;
}

const within = (spans: MechanismSpan[], t: number) => spans.some((s) => t >= s.start && t < s.end);

export function mechanismStateAt(mt: MechanismTimeline, t: number): MechanismState {
  const moving = mt.scoopulaMoving.find((s) => t >= s.start && t < s.end);
  return {
    intake: within(mt.intakeFeed, t) ? "feed" : within(mt.intakeOn, t) ? "on" : "off",
    scoopula: moving ? (moving.label === "lowering" ? "lowering" : "raising") : within(mt.scoopulaDown, t) ? "down" : "up",
    shooting: within(mt.shoot, t),
    phase: within(mt.backoff, t) ? "backoff" : within(mt.turn, t) ? "turn" : null,
    pickFlash: mt.picks.some((p) => t >= p && t < p + 0.5),
    events: mt.otherEvents.filter((s) => t >= s.start && t < s.end).map((s) => s.label),
  };
}

/** SystemConstants.HivePosition: the hive x for the alliance, the upper / lower end by the robot's half. */
export function hiveAimPoint(alliance: "RED" | "BLUE", robot: BasePoint): BasePoint {
  return { x: alliance === "BLUE" ? 92 : 64, y: robot.y >= 71 ? 90 : 52 };
}

/** SystemConstants.Turret.pollenTurretOffset: 5.91 in behind the robot centre, on the centre line. */
export const TURRET_BEHIND_INCHES = 5.91;

/** Alliance for the aim point: a camera pick says it; otherwise the half the route starts on. */
export function allianceOf(startPoint: BasePoint, lines: Line[]): "RED" | "BLUE" {
  const pick = lines.find((line) => line.cameraPick?.enabled)?.cameraPick;
  if (pick?.alliance === "BLUE" || pick?.alliance === "RED") return pick.alliance;
  return startPoint.x > 70.75 ? "BLUE" : "RED";
}
