"""One front door for driving the robot from a host session. See ROBOT_CONTROL.md at repo root.

Every ad-hoc ``urllib`` snippet a session writes is a fresh chance to forget the watchdog, retry a
drive command, skip the box check or miss an UNKNOWN COMMAND. This wraps the Swerve Bring-Up HTTP
API with those rules built in, so the same checks run every time.

    python robot.py check                 gates: reachable / live / started / pose / box; exit code says which failed
    python robot.py state [--json]        compact snapshot (or the raw /state JSON)
    python robot.py cmd ACTION [k=v ...] [--confirmed-floor | --on-blocks]
                                          send one command, then report what the OpMode said about
                                          it (motion actions need one of the two surface flags)
    python robot.py drive --f F [--s S] [--t T] --sec SEC (--confirmed-floor | --on-blocks) [--cap 0.30] [--rec LABEL]
    python robot.py pull LABEL            fetch the recorder CSV into runs/ and print true loop rate
    python robot.py stop                  zero the drive and put the OpMode in IDLE (OpMode keeps running)
    python robot.py estop                 STOP the OpMode through FTC Dashboard (kills it; operator restarts)
    python robot.py constants [--write] [--from FILE]
                                          export the tool's tuning as Constants.java declarations and
                                          splice them in by name (dry run without --write)
    python robot.py pinpoint FILE [--write]
                                          import Pedro's Pinpoint Tuner output (offsets and
                                          directions) into the TUNED VALUES block
    python robot.py foresight FILE [--write]
                                          import Pedro's Foresight Tuner output (results list or
                                          code block, pasted into FILE) into the TUNED VALUES block

Diagnostic tooling (tools/swervetune). It reads and commands the existing API only and changes
nothing the OpMode measures.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import sys
import time

from swervebench import BASE, BenchError, _archive, _get, _mean, parse_csv

# Actions the OpMode itself treats as motion (SwerveBringUp.isMotionCommand - keep the two in
# step). These are sent exactly once: a retried drive or rawServo lands late and restarts a
# timer, and a duplicate is worse than a dropped packet. Every one needs the floor/blocks
# confirmation (surface_refusal) and, off blocks, the box gate.
MOTION = {
    "wireScan", "sweep", "pulseMotor", "spinServo", "nudge", "pidStep", "pidStepAll", "rawServo",
    "autoTune", "headingStep", "headingGoto", "drive", "calGoto", "calHome", "calPositional",
    "pedroStart", "pedroLine", "pedroHold", "pedroCurve", "pedroChain",
    "foresight",
}

# Actions that move the pose frame and so destroy the safe-area box (CLAUDE.md rule 6).
CLEARS_BOX = {"resetImu", "odoConfig", "boxClear"}

# Actions that WRITE a pose into the Pinpoint and arm a fence in it. The pose must be one the
# operator measured on the real field and reported - a session never invents where the robot is.
REFRAMES = {"fieldFence", "borderFence", "setPose"}

HERE = os.path.dirname(os.path.abspath(__file__))
CONSTANTS_JAVA = os.path.normpath(os.path.join(
    HERE, "..", "..", "TeamCode", "src", "main", "java", "org", "firstinspires", "ftc",
    "teamcode", "pedroPathing", "Constants.java"))

# The OpMode's drive watchdog is 400 ms (SwerveBringUp.DRIVE_WATCHDOG_MS). 75 ms matches
# boxdrive.py and leaves room for five missed packets before it trips.
DRIVE_PERIOD_S = 0.075

# SWERVE_TASK.md speed cap until steering criteria hold. Above HARD_CAP needs --allow-fast.
DEFAULT_CAP = 0.30
HARD_CAP = 0.50

EXIT_OK, EXIT_UNREACHABLE, EXIT_NOT_LIVE, EXIT_NOT_STARTED, EXIT_NO_POSE, EXIT_NO_BOX = 0, 2, 3, 4, 5, 6
EXIT_REFUSED = 7


def state(retries: int = 2, timeout: float = 3.0) -> dict:
    return json.loads(_get("/state", timeout=timeout, retries=retries))


def send(action: str, **params) -> None:
    retries = 1 if action in MOTION else 4
    _get("/cmd", {"action": action, **params}, timeout=3.0, retries=retries)


def outcome(action: str, before_msg: str, window_s: float = 2.0) -> tuple[bool, str]:
    """Reads back what the OpMode said about a command.

    /swerve/cmd always answers {"ok":true} - it only queues. The verdict is in a later published
    snapshot's ``message`` (or an errors entry for an unknown action). /state is the LAST
    published snapshot, so a read straight after the command still shows the previous one - that
    is the bug that ended every drivecapture chunk after one sample. Poll for the message to
    change (identity), not for a fixed delay (timing).
    """
    t_end = time.time() + window_s
    st = state()
    while st.get("message", "") == before_msg and time.time() < t_end:
        time.sleep(0.1)
        st = state()
    msg = st.get("message", "")
    errs = " | ".join(str(e) for e in st.get("errors", []))
    bad_markers = ("UNKNOWN COMMAND", "REFUSED", "Press START", "first.", "needs ", "Cannot ",
                   "failed", "No pinpoint", "No pose", "No box", "not available", "ERROR in ")
    bad = any(m in msg for m in bad_markers) or f'UNKNOWN COMMAND "{action}"' in errs
    if msg == before_msg:
        msg += ("   (UNCONFIRMED: message did not change in 2 s - either the command repeats the "
                "last message or it has not been drained. Verify the effect in /state.)")
    return (not bad), msg


def summarize(st: dict) -> str:
    h, p, b, r = st.get("heading", {}), st.get("pose", {}), st.get("box", {}), st.get("rec", {})
    lines = [
        f"live={st.get('live')} started={st.get('started')} mode={st.get('mode')} "
        f"busy={st.get('busy')} volts={st.get('voltage')} publishHz={st.get('timing', {}).get('publishHz')}",
        f"pose ok={p.get('ok')} x={p.get('x')} y={p.get('y')} vx={p.get('vx')} vy={p.get('vy')}   "
        f"heading ok={h.get('ok')} deg={h.get('deg')} hold={h.get('hold')} xLock={st.get('xLock')}",
    ]
    if b.get("valid"):
        lines.append(
            f"box ARMED ({b.get('kind', 'marked')}) x {b.get('minX')}..{b.get('maxX')} "
            f"y {b.get('minY')}..{b.get('maxY')} ({_span(b, 'X')} x {_span(b, 'Y')} in) "
            f"clamped={b.get('clamped')}"
        )
        if b.get("kind") in ("field", "border"):
            names = ", ".join(k.get("name", "?") for k in b.get("keepOuts", []))
            lines.append(
                f"{b.get('kind')} fence: robot {b.get('robotL')} x {b.get('robotW')} in, "
                f"keep-out clearance {b.get('keepOutClear')} in, keep-outs: {names or 'none'}"
            )
    else:
        lines.append(f"box NOT ARMED (marked0={b.get('marked0')})")
    if p.get("ok") and b.get("valid"):
        lines.append(f"inside box: {_inside(p, b)}")
    o = st.get("odo", {})
    if o:
        lines.append(
            f"odometry x pod {o.get('xPodOffset')} in {'REV' if o.get('xPodReversed') else 'FWD'}"
            f", y pod {o.get('yPodOffset')} in {'REV' if o.get('yPodReversed') else 'FWD'}  "
            f"{'= Constants.java' if o.get('shipped') else 'DIFFERS from Constants.java'}"
        )
    pe = st.get("pedro", {})
    if pe:
        lines.append(
            f"pedro active={pe.get('active')} job={pe.get('job')} pods={pe.get('pods')} "
            f"busy={pe.get('busy')} gainsShipped={pe.get('gainsShipped')} "
            f"staticsShipped={pe.get('staticsShipped')}"
        )
    lines.append(
        f"rec recording={r.get('recording')} runId={r.get('runId')} samples={r.get('samples')} "
        f"overflowed={r.get('overflowed')} label={r.get('label')!r}"
    )
    lines.append(f"message: {st.get('message', '')}")
    for e in st.get("errors", []):
        lines.append(f"ERROR: {e}")
    for pod in st.get("pods", []):
        lines.append(
            f"  pod {pod.get('i')} {pod.get('label', ''):<2} enc={pod.get('hasEnc')} "
            f"v={pod.get('volts')} wheel={pod.get('wheelDeg')} tgt={pod.get('tgtDeg')} "
            f"pwr={pod.get('cmdPower')}"
        )
    return "\n".join(lines)


def _span(b: dict, axis: str) -> str:
    try:
        return f"{float(b['max' + axis]) - float(b['min' + axis]):.1f}"
    except (KeyError, TypeError, ValueError):
        return "?"


def _inside(p: dict, b: dict) -> bool:
    """Centre inside the rect, and - on a field fence - the robot clear of every keep-out."""
    try:
        inside = (float(b["minX"]) <= float(p["x"]) <= float(b["maxX"])
                  and float(b["minY"]) <= float(p["y"]) <= float(b["maxY"]))
    except (KeyError, TypeError, ValueError):
        return False
    clear = b.get("keepOutClear")
    return inside and (clear is None or float(clear) >= 0)


def gates(need_box: bool) -> tuple[int, dict | None, str]:
    """The checks every motion command needs. Returns (exit code, state, reason)."""
    try:
        st = state()
    except BenchError as e:
        return EXIT_UNREACHABLE, None, (
            f"Robot unreachable at {BASE}: {e}. Is this laptop on the Control Hub WiFi? "
            f"Is the Robot Controller app running?")
    if not st.get("live"):
        return EXIT_NOT_LIVE, st, "Web server up but Swerve Bring-Up is not running (live=false)."
    if not st.get("started"):
        return EXIT_NOT_STARTED, st, "OpMode is in INIT. Motion commands are refused until START."
    if need_box and not st.get("pose", {}).get("ok"):
        return EXIT_NO_POSE, st, "No pose from the Pinpoint - translation is refused with a box armed."
    if need_box and not st.get("box", {}).get("valid"):
        return EXIT_NO_BOX, st, "Safe-area box is NOT armed. Operator must re-mark corners A and B."
    if need_box and not _inside(st["pose"], st["box"]):
        return EXIT_NO_BOX, st, ("Pose is OUTSIDE the armed box or overlapping a keep-out - the "
                                 "frame or the fence is wrong.")
    return EXIT_OK, st, "all gates pass"


# ---- subcommands --------------------------------------------------------------------


def cmd_check(_a) -> int:
    code, st, reason = gates(need_box=True)
    if st is not None:
        print(summarize(st))
    print(("READY: " if code == EXIT_OK else "NOT READY: ") + reason)
    return code


def cmd_state(a) -> int:
    try:
        st = state()
    except BenchError as e:
        print(f"unreachable: {e}")
        return EXIT_UNREACHABLE
    print(json.dumps(st, indent=1) if a.json else summarize(st))
    return EXIT_OK


def cmd_cmd(a) -> int:
    params = {}
    for kv in a.params:
        if "=" not in kv:
            print(f"bad parameter {kv!r}: use key=value")
            return EXIT_REFUSED
        k, v = kv.split("=", 1)
        params[k] = v
    if a.action == "drive":
        print("Use `robot.py drive` - a one-shot drive command trips the 400 ms watchdog and "
              "leaves nobody holding the stop.")
        return EXIT_REFUSED
    if a.action in REFRAMES and not a.pose_from_operator:
        print(f"{a.action} writes x/y/headingDeg into the Pinpoint. Those must be the robot's "
              "real pose as the OPERATOR measured and reported it (fieldFence: x = 0 red wall, "
              "y = 0 audience wall; borderFence: inches from the border's x = 0 and y = 0 edges). "
              "Re-run with --pose-from-operator once they have, and ask them to confirm the "
              "dashboard's drawn fence matches the floor before motion.")
        return EXIT_REFUSED
    if a.action in CLEARS_BOX and not a.clears_box_ok:
        print(f"{a.action} clears the safe-area box (new pose frame). Re-run with --clears-box-ok "
              "and put 're-mark corners A and B' in the next OPS REQUEST.")
        return EXIT_REFUSED
    if a.action in MOTION:
        # The same gates `drive` uses (CLAUDE.md rule 8). headingStep is an open-loop 0.35
        # spin and pulseMotor drives a wheel; the follower commands translate the robot - none
        # of them is safer than a drive, and they used to pass with no floor confirmation (and
        # headingStep / pulseMotor with no box either).
        refusal = surface_refusal(a)
        if refusal:
            print(f"REFUSED {a.action}: {refusal}")
            return EXIT_REFUSED
        # Off blocks the box is mandatory. The follower needs it even on blocks (the OpMode
        # refuses pedroStart without one), so it is checked here rather than discovered there.
        code, _, reason = gates(need_box=not a.on_blocks or a.action.startswith("pedro"))
        if code != EXIT_OK:
            print(f"REFUSED {a.action}: {reason}")
            return code
        print(f"surface={'blocks' if a.on_blocks else 'floor'}")
    try:
        before = state().get("message", "")
        send(a.action, **params)
        ok, msg = outcome(a.action, before)
    except BenchError as e:
        print(f"transport failure: {e}")
        return EXIT_UNREACHABLE
    print(("OK   " if ok else "FAIL ") + msg)
    return EXIT_OK if ok else EXIT_REFUSED


def surface_refusal(a) -> str | None:
    """None when exactly one surface confirmation was passed, else why not. Every motion command
    (`drive` and the MOTION set through `cmd`) goes through this."""
    if a.confirmed_floor == a.on_blocks:
        return ("pass exactly one of --confirmed-floor (operator confirmed THIS session: on the "
                "floor, inside the taped box, area clear) or --on-blocks (operator confirmed: on "
                "blocks, chassis strapped, hands and leads clear of the wheels).")
    return None


def script_gates(argv: list[str], box_always: bool = False) -> tuple[list[str], bool]:
    """The `drive` gates for standalone bench scripts that move the robot (drivetune.py,
    phase2_plant.py, pedrotune.py): exactly one of --confirmed-floor / --on-blocks on the command
    line, then live/started and - unless on blocks, or always with ``box_always`` - pose and box.
    Exits on refusal. Returns (argv without the two flags, on_blocks)."""
    floor = "--confirmed-floor" in argv
    blocks = "--on-blocks" in argv
    rest = [x for x in argv if x not in ("--confirmed-floor", "--on-blocks")]
    refusal = surface_refusal(argparse.Namespace(confirmed_floor=floor, on_blocks=blocks))
    if refusal:
        print(f"REFUSED: {refusal}")
        sys.exit(EXIT_REFUSED)
    code, _, reason = gates(need_box=box_always or not blocks)
    if code != EXIT_OK:
        print(f"REFUSED: {reason}")
        sys.exit(code)
    print(f"surface={'blocks' if blocks else 'floor'}")
    return rest, blocks


def safe_stop(actions=(("drive", {"f": 0, "s": 0, "t": 0}), ("stop", {}))) -> None:
    """For a script's finally: sends each action straight to /cmd, retried (all idempotent), and
    keeps going if one fails - a stop that depends on a state() read succeeding is no stop."""
    for action, params in actions:
        try:
            _get("/cmd", {"action": action, **params}, timeout=2.0, retries=4)
        except BenchError as e:
            print(f"WARNING: could not send {action}: {e} - operator: STOP on the Driver Station.")


def cmd_drive(a) -> int:
    refusal = surface_refusal(a)
    if refusal:
        print(f"REFUSED: {refusal}")
        return EXIT_REFUSED
    cap = a.cap
    if cap > HARD_CAP and not a.allow_fast:
        print(f"REFUSED: cap {cap} > {HARD_CAP} needs --allow-fast (and a reason in the report).")
        return EXIT_REFUSED
    translating = abs(a.f) > 0 or abs(a.s) > 0
    # On blocks the pose cannot move, so the fence is irrelevant; on the floor it is mandatory.
    code, st, reason = gates(need_box=not a.on_blocks)
    if code != EXIT_OK:
        print(f"REFUSED drive: {reason}")
        return code
    f = max(-cap, min(cap, a.f))
    s = max(-cap, min(cap, a.s))
    t = max(-cap, min(cap, a.t))
    if (f, s, t) != (a.f, a.s, a.t):
        print(f"capped to f={f} s={s} t={t} (cap {cap})")
    if a.sec > 10:
        print("REFUSED: one drive call is at most 10 s. Chain calls, re-checking /state between.")
        return EXIT_REFUSED

    p0, h0, v0 = st["pose"], st["heading"].get("deg"), st.get("voltage")
    surface = "blocks" if a.on_blocks else "floor"
    print(f"drive f={f} s={s} t={t} for {a.sec}s  surface={surface}  volts={v0}  "
          f"start x={p0.get('x')} y={p0.get('y')} h={h0}")
    if a.rec:
        send("recStart", label=a.rec)
        time.sleep(0.15)

    clamped = 0
    fails = 0
    ticks = 0
    last = st
    t_end = time.time() + a.sec
    try:
        while time.time() < t_end:
            try:
                send("drive", f=round(f, 3), s=round(s, 3), t=round(t, 3))
                last = state(retries=1, timeout=0.5)
                fails = 0
                clamped += 1 if last.get("box", {}).get("clamped") else 0
                if translating and not a.on_blocks and not last.get("box", {}).get("valid"):
                    print("ABORT: box disarmed mid-drive.")
                    break
            except BenchError as e:
                fails += 1
                if fails >= 2:
                    print(f"ABORT: two consecutive transport failures mid-drive ({e}). The "
                          f"watchdog stops the robot within 400 ms of the last command. "
                          f"If it is still moving: operator presses STOP.")
                    break
            ticks += 1
            time.sleep(DRIVE_PERIOD_S)
    finally:
        # Always leave the robot commanded to zero, then idle. Send both even if one fails.
        for action, params in (("drive", {"f": 0, "s": 0, "t": 0}), ("stop", {})):
            try:
                _get("/cmd", {"action": action, **params}, timeout=2.0, retries=4)
            except BenchError as e:
                print(f"WARNING: could not send {action} after the drive: {e}")
        if a.rec:
            time.sleep(0.2)
            try:
                _get("/cmd", {"action": "recStop"}, timeout=2.0, retries=4)
            except BenchError as e:
                print(f"WARNING: recStop failed: {e}")

    time.sleep(0.4)
    try:
        end = state()
        p1 = end["pose"]
        dx = float(p1["x"]) - float(p0["x"])
        dy = float(p1["y"]) - float(p0["y"])
        print(f"end x={p1['x']} y={p1['y']} h={end['heading'].get('deg')}  moved "
              f"{math.hypot(dx, dy):.1f} in (dx {dx:+.1f}, dy {dy:+.1f})  mode={end.get('mode')}  "
              f"volts={end.get('voltage')}")
    except (BenchError, KeyError, TypeError, ValueError) as e:
        print(f"could not read end state: {e}")
    print(f"ticks={ticks}  clamped_ticks={clamped}  "
          f"(a clamped tick means the fence modified the command - azimuth near an edge is suspect)")
    if a.rec:
        return _pull(a.rec)
    return EXIT_OK


def _pull(label: str) -> int:
    try:
        csv_text = _get("/rec.csv", timeout=30.0, retries=3)
        tr = parse_csv(csv_text)
    except BenchError as e:
        print(f"pull failed: {e}")
        return EXIT_UNREACHABLE
    path = _archive(csv_text, {"label": label})
    dts = [x for x in tr.get("dt", []) if x == x and x > 0]
    st = sorted(dts)
    if dts:
        print(f"saved tools/swervetune/{path}  n={len(dts)}  loop_hz_true={1 / _mean(dts):.1f}  "
              f"loop_dt_mean_ms={1000 * _mean(dts):.2f}  "
              f"loop_dt_p90_ms={1000 * st[int(0.9 * (len(st) - 1))]:.2f}")
    else:
        print(f"saved tools/swervetune/{path}  (no dt samples)")
    if "overflowed=true" in csv_text.split("\n", 1)[0]:
        print("NOTE: recorder hit 3000 samples and stopped - the tail of the run is missing.")
    return EXIT_OK


def cmd_pull(a) -> int:
    return _pull(a.label)


def cmd_stop(_a) -> int:
    rc = EXIT_OK
    for action, params in (("drive", {"f": 0, "s": 0, "t": 0}), ("stop", {})):
        try:
            _get("/cmd", {"action": action, **params}, timeout=2.0, retries=4)
        except BenchError as e:
            print(f"could not send {action}: {e}")
            rc = EXIT_UNREACHABLE
    if rc == EXIT_OK:
        time.sleep(0.3)
        try:
            print(f"mode={state().get('mode')}  message: {state().get('message')}")
        except BenchError:
            pass
    else:
        print("If the robot is moving: the operator presses STOP on the Driver Station, or run "
              "`python robot.py estop`.")
    return rc


def cmd_estop(_a) -> int:
    try:
        from ftcdash import stop_op_mode
        print(json.dumps(stop_op_mode(), indent=1)[:800])
        print("OpMode STOPPED. The HTTP server stays up but /state now reads live=false; the "
              "operator must re-start Swerve Bring-Up (and re-check the box) before any motion.")
        return EXIT_OK
    except Exception as e:  # noqa: BLE001 - report whatever went wrong, then escalate to a human
        print(f"estop via FTC Dashboard FAILED: {e}. Operator: press STOP on the Driver Station.")
        return EXIT_UNREACHABLE


# ---- constants export ----------------------------------------------------------------

# One declaration per name, in the TUNED VALUES block. The value runs to the first ';' - none of
# the tuned values (numbers, booleans, quoted encoder names) can contain one.
_DECL = re.compile(r"^public static final ([\w\[\]]+) (\w+) = ([^;]*);\s*$")


def parse_export(text: str) -> tuple[list[tuple[str, str, str]], list[str]]:
    """(type, name, value) per declaration line, and the WARNINGS comment lines."""
    decls, warnings, in_warn = [], [], False
    for line in text.splitlines():
        m = _DECL.match(line.strip())
        if m:
            decls.append((m.group(1), m.group(2), m.group(3).strip()))
            continue
        if line.startswith("// WARNINGS"):
            in_warn = True
        elif in_warn and line.startswith("//"):
            warnings.append(line[2:].strip())
    return decls, warnings


def splice(source: str, decls: list[tuple[str, str, str]]) -> tuple[str, list[str], list[str]]:
    """Replaces each declaration's initialiser by name. Returns (new source, changes, missing).

    A name must occur exactly once as `public static final <type> <name> = ...;` - anything else
    is reported as missing rather than guessed at, so a refactor of Constants.java can never make
    the splice land on the wrong line.
    """
    changes, missing = [], []
    for typ, name, value in decls:
        pat = re.compile(r"(public static final " + re.escape(typ) + r" " + re.escape(name)
                         + r" = )([^;]*)(;)")
        hits = pat.findall(source)
        if len(hits) != 1:
            missing.append(f"{typ} {name} ({len(hits)} matches)")
            continue
        old = hits[0][1].strip()
        if old != value:
            changes.append(f"{name}: {old}  ->  {value}")
            source = pat.sub(lambda m: m.group(1) + value + m.group(3), source, count=1)
    return source, changes, missing


def cmd_constants(a) -> int:
    if a.src:
        with open(a.src, encoding="utf-8") as f:
            text = f.read()
        label = os.path.basename(a.src)
    else:
        try:
            st = state()
            if not st.get("live"):
                print("Swerve Bring-Up is not running (live=false) - nothing to export.")
                return EXIT_NOT_LIVE
            before = st.get("message", "")
            send("export")
            ok, msg = outcome("export", before)
            text = state().get("export", "")
        except BenchError as e:
            print(f"transport failure: {e}")
            return EXIT_UNREACHABLE
        if not ok or not text.strip():
            print(f"export failed: {msg}")
            return EXIT_REFUSED
        os.makedirs(os.path.join(HERE, "runs"), exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        saved = os.path.join(HERE, "runs", f"constants_{stamp}.java")
        with open(saved, "w", encoding="utf-8") as f:
            f.write(text)
        label = os.path.relpath(saved, os.path.join(HERE, "..", ".."))
        print(f"export saved to {label}")

    decls, warnings = parse_export(text)
    if not decls:
        print("The export holds no declarations - is the robot running a build from before "
              "2026-10-01 (old pod-factory export)?")
        return EXIT_REFUSED
    with open(a.target, encoding="utf-8") as f:
        source = f.read()
    new_source, changes, missing = splice(source, decls)

    print(f"{len(decls)} declarations from {label}; {len(changes)} differ from Constants.java:")
    for c in changes:
        print(f"  {c}")
    for m in missing:
        print(f"  NOT FOUND in Constants.java, skipped: {m}")
    for w in warnings:
        print(f"  WARNING: {w}")
    if missing:
        print("Refusing to write: a declaration the export names is not in Constants.java's "
              "TUNED VALUES block exactly once. Fix the file (or the exporter), then re-run.")
        return EXIT_REFUSED
    if not changes:
        print("Constants.java already matches the tool. Nothing to write.")
        return EXIT_OK
    if not a.write:
        print("Dry run. Re-run with --write to apply, then build, commit with the evidence "
              "(CLAUDE.md rule 11), and say these are SHIPPED changes.")
        return EXIT_OK
    with open(a.target, "w", encoding="utf-8") as f:
        f.write(new_source)
    print(f"Wrote {os.path.relpath(a.target)}. SHIPPED code changed: build "
          "(./gradlew :TeamCode:assembleDebug), then commit with the evidence for each value.")
    return EXIT_OK


# ---- Foresight Tuner import ------------------------------------------------------------

# Pedro's ForesightTuner result() names -> Constants.java TUNED VALUES names.
FORESIGHT_RESULTS = {
    "maxAchievableForwardVelocity": "maxAchievableForwardVelocity",
    "maxAchievableStrafeVelocity": "maxAchievableStrafeVelocity",
    "naturalForwardDeceleration": "naturalForwardDeceleration",
    "naturalStrafeDeceleration": "naturalStrafeDeceleration",
    "headingBrakingLinearCoefficient": "headingBrakeLinear",
    "headingBrakingQuadraticCoefficient": "headingBrakeQuadratic",
    "heading kP": "headingKP",
    "forwardBrakingLinearCoefficient": "forwardBrakeLinear",
    "forwardBrakingQuadraticCoefficient": "forwardBrakeQuadratic",
    "strafeBrakingLinearCoefficient": "strafeBrakeLinear",
    "strafeBrakingQuadraticCoefficient": "strafeBrakeQuadratic",
    "forwardTranslational Primary kP": "forwardTranslationalPrimaryKP",
    "forwardTranslational Secondary kP": "forwardTranslationalSecondaryKP",
    "strafeTranslational Primary kP": "strafeTranslationalPrimaryKP",
    "strafeTranslational Secondary kP": "strafeTranslationalSecondaryKP",
    "coast kV": "coastKV",
    "brake kV": "brakeKV",
}

_NUM = r"(-?\d+(?:\.\d*)?(?:[eE][-+]?\d+)?)"


def parse_foresight(text: str) -> dict[str, float]:
    """Reads the tuner's results list ("name: value") and/or its generated code block."""
    out: dict[str, float] = {}
    for key, name in FORESIGHT_RESULTS.items():
        m = re.search(re.escape(key) + r"\s*[:=\t ]\s*" + _NUM, text)
        if m:
            out[name] = float(m.group(1))
    # The code block the tuner prints (ForesightTuner.java's code(...)).
    code = {
        "forwardTranslationalPrimaryKP": r"primaryTranslationalForward\s*=\s*Controller\.proportional\(" + _NUM,
        "forwardTranslationalSecondaryKP": r"secondaryTranslationalForward\s*=\s*Controller\.proportional\(" + _NUM,
        "strafeTranslationalPrimaryKP": r"primaryTranslationalLateral\s*=\s*Controller\.proportional\(" + _NUM,
        "strafeTranslationalSecondaryKP": r"secondaryTranslationalLateral\s*=\s*Controller\.proportional\(" + _NUM,
        "coastKV": r"c\.coast\.set\(Controller\.proportionalFeedforward\(" + _NUM,
        "brakeKV": r"c\.brake\.set\(Controller\.proportionalFeedforward\(" + _NUM,
        "headingKP": r"c\.headingFeedback\.set\(Controller\.proportional\(" + _NUM,
        "maxAchievableForwardVelocity": r"c\.maxAchievableForwardVelocity\.set\(" + _NUM,
        "maxAchievableStrafeVelocity": r"c\.maxAchievableStrafeVelocity\.set\(" + _NUM,
        "naturalForwardDeceleration": r"c\.naturalForwardDeceleration\.set\(" + _NUM,
        "naturalStrafeDeceleration": r"c\.naturalStrafeDeceleration\.set\(" + _NUM,
    }
    for name, pat in code.items():
        m = re.search(pat, text)
        if m and name not in out:
            out[name] = float(m.group(1))
    pairs = {
        ("headingBrakeLinear", "headingBrakeQuadratic"):
            r"headingBrakeCoefficients\.set\(Vector2D\.cartesian\(" + _NUM + r"\s*,\s*" + _NUM,
        ("forwardBrakeLinear", "strafeBrakeLinear"):
            r"linearBrakeCoefficients\.set\(Matrix\.diag\(" + _NUM + r"\s*,\s*" + _NUM,
        ("forwardBrakeQuadratic", "strafeBrakeQuadratic"):
            r"quadraticBrakeCoefficients\.set\(Matrix\.diag\(" + _NUM + r"\s*,\s*" + _NUM,
    }
    for (a, b), pat in pairs.items():
        m = re.search(pat, text)
        if m:
            out.setdefault(a, float(m.group(1)))
            out.setdefault(b, float(m.group(2)))
    return out


def _java_double(v: float) -> str:
    s = repr(float(v))
    return s if ("." in s or "e" in s or "E" in s or "inf" in s or "nan" in s) else s + ".0"


def parse_pinpoint(text: str) -> dict[str, str]:
    """Pinpoint Tuner results list ("xPodOffset: -5.37") or its code block -> TUNED decl values."""
    out: dict[str, str] = {}
    for key, name in (("xPodOffset", "pinpointXPodOffset"), ("yPodOffset", "pinpointYPodOffset")):
        m = (re.search(r"c\." + key + r"\.set\(\s*" + _NUM, text)
             or re.search(r"\b" + key + r"\s*[:=\t ]\s*" + _NUM, text))
        if m:
            out[name] = _java_double(float(m.group(1)))
    for key, name in (("xPodDirection", "pinpointXPodReversed"),
                      ("yPodDirection", "pinpointYPodReversed")):
        m = re.search(r"\b" + key + r"\b[^\n]*?\b(REVERSED|FORWARD)\b", text)
        if m:
            out[name] = "true" if m.group(1) == "REVERSED" else "false"
    return out


def cmd_pinpoint(a) -> int:
    with open(a.file, encoding="utf-8") as f:
        text = f.read()
    vals = parse_pinpoint(text)
    if len(vals) != 4:
        print(f"Found {len(vals)}/4 Pinpoint Tuner values ({', '.join(vals) or 'none'}). Paste "
              "the tuner's results list or its code block.")
        return EXIT_REFUSED
    decls = [("double" if n.endswith("Offset") else "boolean", n, v) for n, v in vals.items()]
    with open(a.target, encoding="utf-8") as f:
        source = f.read()
    new_source, changes, missing = splice(source, decls)
    print(f"Pinpoint Tuner -> Constants.java: {len(changes)} change(s)")
    for c in changes:
        print(f"  {c}")
    if missing:
        print("NOT FOUND in Constants.java: " + ", ".join(missing))
        return EXIT_REFUSED
    print("Validate before trusting it: paired +/-45 deg rotations (orbit <= 0.6 in per 45 deg) "
          "and a taped line, through odoConfig on the bench (TUNING_TASK.md stage 2).")
    if not a.write or not changes:
        print("Dry run." if changes else "Nothing to write.")
        return EXIT_OK
    with open(a.target, "w", encoding="utf-8") as f:
        f.write(new_source)
    print(f"Wrote {os.path.relpath(a.target)}. SHIPPED code changed.")
    return EXIT_OK


def cmd_foresight(a) -> int:
    with open(a.file, encoding="utf-8") as f:
        text = f.read()
    vals = parse_foresight(text)
    missing = [n for n in FORESIGHT_RESULTS.values() if n not in vals]
    if missing and not a.partial:
        print(f"Found {len(vals)}/17 Foresight Tuner values; missing: {', '.join(missing)}. "
              "Paste the tuner's whole results list or code block, or pass --partial to import "
              "only what is there.")
        return EXIT_REFUSED
    bad = [n for n, v in vals.items() if not math.isfinite(v)
           or (n.startswith(("maxAchievable", "natural")) and v <= 0)]
    if bad:
        print(f"REFUSED: non-physical values: {', '.join(f'{n}={vals[n]}' for n in bad)}")
        return EXIT_REFUSED
    decls = [("double", n, _java_double(v)) for n, v in vals.items()]
    # The tuner identifies P-only translational controllers. Its kP were fitted with no I or D,
    # so carrying the 2.x kD under them would ship a combination nobody measured.
    if any(n.startswith(("forwardTranslational", "strafeTranslational")) for n in vals):
        decls += [("double", "translationalKI", "0.0"), ("double", "translationalKD", "0.0")]
    with open(a.target, encoding="utf-8") as f:
        source = f.read()
    new_source, changes, missing_decl = splice(source, decls)
    print(f"{len(decls)} values from {a.file}; {len(changes)} differ from Constants.java:")
    for c in changes:
        print(f"  {c}")
    for m in missing_decl:
        print(f"  NOT FOUND in Constants.java: {m}")
    if missing_decl:
        return EXIT_REFUSED
    print("NOTE: the Foresight Tuner is identification, not validation. Next: build, deploy, "
          "then pedrocheck.py suite --pods shipped (TUNING_TASK.md stage 6). FORESIGHT_MEASURED "
          "stays false until that passes.")
    if not a.write or not changes:
        print("Dry run." if changes else "Nothing to write.")
        return EXIT_OK
    with open(a.target, "w", encoding="utf-8") as f:
        f.write(new_source)
    print(f"Wrote {os.path.relpath(a.target)}. SHIPPED code changed.")
    return EXIT_OK


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    sub = ap.add_subparsers(dest="sub", required=True)
    sub.add_parser("check").set_defaults(fn=cmd_check)
    p = sub.add_parser("state")
    p.add_argument("--json", action="store_true")
    p.set_defaults(fn=cmd_state)
    p = sub.add_parser("cmd")
    p.add_argument("action")
    p.add_argument("params", nargs="*")
    p.add_argument("--clears-box-ok", action="store_true")
    p.add_argument("--pose-from-operator", action="store_true")
    p.add_argument("--confirmed-floor", action="store_true",
                   help="motion actions: operator confirmed on the floor, in the box, area clear")
    p.add_argument("--on-blocks", action="store_true",
                   help="motion actions: operator confirmed on blocks, strapped, hands clear")
    p.set_defaults(fn=cmd_cmd)
    p = sub.add_parser("drive")
    p.add_argument("--f", type=float, default=0.0, help="forward, robot frame, [-1, 1]")
    p.add_argument("--s", type=float, default=0.0, help="strafe, robot frame, [-1, 1]")
    p.add_argument("--t", type=float, default=0.0, help="turn, [-1, 1]")
    p.add_argument("--sec", type=float, required=True)
    p.add_argument("--cap", type=float, default=DEFAULT_CAP)
    p.add_argument("--allow-fast", action="store_true")
    p.add_argument("--confirmed-floor", action="store_true")
    p.add_argument("--on-blocks", action="store_true")
    p.add_argument("--rec", default=None, help="record the drive under this label and pull it")
    p.set_defaults(fn=cmd_drive)
    p = sub.add_parser("pull")
    p.add_argument("label")
    p.set_defaults(fn=cmd_pull)
    sub.add_parser("stop").set_defaults(fn=cmd_stop)
    sub.add_parser("estop").set_defaults(fn=cmd_estop)
    p = sub.add_parser("constants")
    p.add_argument("--write", action="store_true", help="apply the splice (default: dry run)")
    p.add_argument("--from", dest="src", default=None,
                   help="splice a saved export instead of asking the robot")
    p.add_argument("--target", default=CONSTANTS_JAVA, help=argparse.SUPPRESS)
    p.set_defaults(fn=cmd_constants)
    p = sub.add_parser("pinpoint")
    p.add_argument("file", help="text file holding the Pinpoint Tuner's results or code block")
    p.add_argument("--write", action="store_true")
    p.add_argument("--target", default=CONSTANTS_JAVA, help=argparse.SUPPRESS)
    p.set_defaults(fn=cmd_pinpoint)
    p = sub.add_parser("foresight")
    p.add_argument("file", help="text file holding the Foresight Tuner's results or code block")
    p.add_argument("--write", action="store_true")
    p.add_argument("--partial", action="store_true")
    p.add_argument("--target", default=CONSTANTS_JAVA, help=argparse.SUPPRESS)
    p.set_defaults(fn=cmd_foresight)
    a = ap.parse_args()
    return a.fn(a)


if __name__ == "__main__":
    sys.exit(main())
