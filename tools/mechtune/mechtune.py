"""Front door for a host session on the Mechanism Tuner (and the operator notification board).

Diagnostic tooling. Talks to the robot's /tune routes (TeamCode/.../diagnostics/tuning). Standard
library only. See MECHANISM_TUNING_TASK.md at repo root for the procedure this serves.

    python mechtune.py check                       reachable / live / started; exit code says which failed
    python mechtune.py state [--json]              compact snapshot
    python mechtune.py cmd ACTION [k=v ...]        one command, then what the OpMode said about it

  Talking to the operator (works with ANY OpMode running, including Swerve Bring-Up):
    python mechtune.py ask "TEXT" [--timeout S]    banner + beep on /tune and /swerve; waits; prints the reply
    python mechtune.py say "TEXT"                  info-only note, no answer needed
    python mechtune.py clear                       dismiss every open note

  Flywheels (wheel = pollen | nectar). Anything that spins needs --confirmed-clear: the operator
  has confirmed hands and pieces are clear of the wheel (ask them first).
    python mechtune.py spin WHEEL VEL --confirmed-clear
    python mechtune.py off [WHEEL]
    python mechtune.py pidf WHEEL P I D F
    python mechtune.py step WHEEL --to 2000 [--from 0] [--sec 3] [--n 1] --confirmed-clear
    python mechtune.py kick WHEEL --target 2000 [--ms 120] [--n 10] --confirmed-clear
    python mechtune.py shots WHEEL --target 2000 [--n 5] --confirmed-clear
    python mechtune.py ff WHEEL [--powers 0.3,0.5,0.7,0.9,1.0] [--hold 2] --confirmed-clear
    python mechtune.py ab WHEEL --a "P I D F" --b "P I D F" --test kick|step|shots --target 2000 [--n 10] --confirmed-clear
                                                   (an arm that never holds speed counts as a failure, not a crash)
    python mechtune.py dircheck WHEEL [--power 0.5] [--ask] --confirmed-clear
                                                   encoder-vs-motor sign check; --ask: one "shooting direction?" banner
    python mechtune.py autotune WHEEL --target T --confirmed-clear
                                                   unattended SEARCH: ff x2, closed-loop F, 4-candidate grid,
                                                   recommendation + the shots A/B command to validate it
    python mechtune.py hold WHEEL --a "P I D F" --b "P I D F" --target T [--n 6] [--sec 4] --confirmed-clear
                                                   interleaved holds: in-band fraction (when an arm cannot arm for shots)
    python mechtune.py rapidfire WHEEL --target T --confirmed-clear
                                                   hold speed, log every shot, operator presses Done to stop
    python mechtune.py events                      the shot/kick event table

  Device bindings and the stall guard (OpMode side):
    python mechtune.py cmd flyDevice wheel=pollen device=sm0      saved on the hub (device=default undoes)
    python mechtune.py cmd stallGuard on=0|1                      on by default; trips stop everything
    python mechtune.py pull LABEL                  recorder CSV -> runs/LABEL.csv, prints loop_hz_true

  Color sensor:
    python mechtune.py color show
    python mechtune.py color sample LABEL [--n 40] [--ask "hold the RED nectar at the sensor"]
    python mechtune.py color apply                 apply the suggested thresholds live
    python mechtune.py color csv                   save every sample to runs/color-<time>.csv

  Shipping:
    python mechtune.py export [--write] [--only flywheel,intake,...]
                                                   splice the tool's values into SystemConstants.java by name
                                                   (dry run without --write). A SHIPPED change.
    python mechtune.py export --only regression --write
                                                   the Regression tab's saved fits -> modules/ShootingRegression.java

Override the robot address with MECHTUNE_HOST=ip:port.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import random
import re
import statistics
import sys
import time
import urllib.error
import urllib.parse
import urllib.request

HOST = os.environ.get("MECHTUNE_HOST", "192.168.43.1:8080")
BASE = f"http://{HOST}/tune"
HERE = os.path.dirname(os.path.abspath(__file__))
RUNS = os.path.join(HERE, "runs")
TRIALS = os.path.join(HERE, "trials.jsonl")
TEAMCODE = os.path.normpath(os.path.join(
    HERE, "..", "..", "TeamCode", "src", "main", "java", "org", "firstinspires", "ftc", "teamcode"))

EXIT_OK, EXIT_UNREACHABLE, EXIT_NOT_LIVE, EXIT_NOT_STARTED, EXIT_REFUSED, EXIT_SKIPPED, EXIT_TIMEOUT = (
    0, 2, 3, 4, 7, 8, 9)

# Commands that move something. Sent once: a retried spin or kick that lands late is worse than
# a dropped packet.
MOTION = {"fly", "flyPower", "step", "kick", "ffSweep", "turretPos", "turretDeg", "servoPos",
          "gatePulse", "intake", "intakeTest"}


# Host commands that spin a flywheel: they need --confirmed-clear, arm the stall-guard baseline,
# and send flyOff if they fail.
SPINNING = {"spin", "step", "kick", "shots", "ff", "ab", "dircheck", "hold", "rapidfire", "autotune"}


class RobotError(Exception):
    pass


# ---------------------------------------------------------------------------------------- HTTP

def _get(path: str, params: dict | None = None, timeout: float = 3.0, retries: int = 4) -> str:
    url = BASE + path
    if params:
        url += "?" + urllib.parse.urlencode({k: v for k, v in params.items() if v is not None})
    last = None
    for attempt in range(max(1, retries)):
        try:
            with urllib.request.urlopen(url, timeout=timeout) as r:
                return r.read().decode("utf-8")
        except (urllib.error.URLError, TimeoutError, ConnectionError, OSError) as e:
            last = e
            # The hub WiFi drops for 1-2 s several times an hour.
            time.sleep(0.75)
    raise RobotError(f"{url}: {last}")


def state() -> dict:
    return json.loads(_get("/state"))


def send(action: str, **params) -> dict:
    """Sends one command and waits until the OpMode has processed it, then returns state."""
    retries = 1 if action in MOTION else 4
    reply = json.loads(_get("/cmd", {"action": action, **params}, retries=retries))
    seq = reply.get("seq", 0)
    if not reply.get("live"):
        raise RobotError("Mechanism Tuner is not running (live=false); the command was queued but "
                         "nothing is draining it.")
    deadline = time.time() + 3.0
    while time.time() < deadline:
        s = state()
        if s.get("cmdSeq", 0) >= seq:
            msg = s.get("message", "")
            if msg.startswith(("UNKNOWN COMMAND", "REFUSED", "ERROR")):
                raise RobotError(msg)
            return s
        time.sleep(0.05)
    raise RobotError(f"{action}: the OpMode did not acknowledge seq {seq} within 3 s")


def wheel_of(s: dict, name: str) -> dict:
    for w in s["wheels"]:
        if w["name"] == name:
            return w
    raise RobotError(f"no wheel {name}")


# ---------------------------------------------------------------------------------- notes

def post_note(text: str, level: str = "action", await_: str = "ack") -> int:
    reply = json.loads(_get("/notify", {"text": text, "level": level, "await": await_,
                                        "from": "claude"}))
    if not reply.get("ok"):
        raise RobotError(reply.get("message", "notify failed"))
    return reply["id"]


def wait_note(note_id: int, timeout: float, keepalive: bool = True) -> dict | None:
    """Polls until the note is resolved. Also reads /tune/state so the OpMode's no-client
    watchdog does not stop the flywheels while the operator is busy."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        notes = json.loads(_get("/notes", {"since": note_id - 1}))["notes"]
        for n in notes:
            if n["id"] == note_id and n["resolved"]:
                return n
        if keepalive:
            try:
                _get("/state", retries=1)
            except RobotError:
                pass
        time.sleep(0.5)
    return None


def ask(text: str, timeout: float = 900, await_: str = "ack") -> dict:
    nid = post_note(text, "action", await_)
    print(f"[asked operator #{nid}] {text}", flush=True)
    n = wait_note(nid, timeout)
    if n is None:
        raise TimeoutError(f"operator did not answer note #{nid} within {timeout:.0f} s")
    print(f"[operator #{nid}] {n['resolution']}" + (f": \"{n['reply']}\"" if n["reply"] else ""),
          flush=True)
    return n


# ---------------------------------------------------------------------------------- stats

def _betacf(a: float, b: float, x: float) -> float:
    qab, qap, qam = a + b, a + 1, a - 1
    c, d = 1.0, 1 - qab * x / qap
    d = 1 / (d if abs(d) > 1e-30 else 1e-30)
    h = d
    for m in range(1, 200):
        m2 = 2 * m
        aa = m * (b - m) * x / ((qam + m2) * (a + m2))
        d = 1 + aa * d
        d = 1 / (d if abs(d) > 1e-30 else 1e-30)
        c = 1 + aa / c if abs(1 + aa / c) > 1e-30 else 1e-30
        h *= d * c
        aa = -(a + m) * (qab + m) * x / ((a + m2) * (qap + m2))
        d = 1 + aa * d
        d = 1 / (d if abs(d) > 1e-30 else 1e-30)
        c = 1 + aa / c if abs(1 + aa / c) > 1e-30 else 1e-30
        de = d * c
        h *= de
        if abs(de - 1) < 3e-12:
            break
    return h


def _ibeta(a: float, b: float, x: float) -> float:
    if x <= 0:
        return 0.0
    if x >= 1:
        return 1.0
    lbt = math.lgamma(a + b) - math.lgamma(a) - math.lgamma(b) + a * math.log(x) + b * math.log(1 - x)
    if x < (a + 1) / (a + b + 2):
        return math.exp(lbt) * _betacf(a, b, x) / a
    return 1 - math.exp(lbt) * _betacf(b, a, 1 - x) / b


def t_two_sided_p(t: float, df: float) -> float:
    return _ibeta(df / 2, 0.5, df / (df + t * t))


def t_crit95(df: float) -> float:
    lo, hi = 0.0, 50.0
    for _ in range(80):
        mid = (lo + hi) / 2
        if t_two_sided_p(mid, df) > 0.05:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2


def welch(a: list[float], b: list[float]) -> dict:
    """B minus A, Welch t, 95% CI. Needs >= 2 per arm."""
    ma, mb = statistics.fmean(a), statistics.fmean(b)
    va, vb = statistics.variance(a), statistics.variance(b)
    se = math.sqrt(va / len(a) + vb / len(b))
    diff = mb - ma
    if se == 0:
        return {"diff": diff, "ci": [diff, diff], "t": math.inf, "p": 0.0, "df": None}
    df = (va / len(a) + vb / len(b)) ** 2 / (
        (va / len(a)) ** 2 / (len(a) - 1) + (vb / len(b)) ** 2 / (len(b) - 1))
    t = diff / se
    tc = t_crit95(df)
    return {"diff": diff, "ci": [diff - tc * se, diff + tc * se], "t": t,
            "p": t_two_sided_p(abs(t), df), "df": df}


def describe(xs: list[float]) -> str:
    if not xs:
        return "n=0"
    if len(xs) == 1:
        return f"n=1, {xs[0]:.1f}"
    m = statistics.fmean(xs)
    sd = statistics.stdev(xs)
    half = t_crit95(len(xs) - 1) * sd / math.sqrt(len(xs))
    return (f"n={len(xs)}, mean {m:.1f}, sd {sd:.1f}, 95% CI [{m - half:.1f}, {m + half:.1f}], "
            f"median {statistics.median(xs):.1f}, range {min(xs):.1f}-{max(xs):.1f}")


def log_trial(kind: str, record: dict) -> None:
    record = {"ts": time.strftime("%Y-%m-%dT%H:%M:%S"), "kind": kind, **record}
    with open(TRIALS, "a", encoding="utf-8") as fh:
        fh.write(json.dumps(record) + "\n")


# ---------------------------------------------------------------------------------- flywheel ops

def need_clear(args) -> None:
    if not getattr(args, "confirmed_clear", False):
        raise RobotError("refused: pass --confirmed-clear only after the operator has confirmed "
                         "hands and pieces are clear of the flywheel (use `ask`).")


class NotArmed(RobotError):
    """The wheel never held the at-speed band long enough to arm the dip detector."""


# Stall-guard trip counts per wheel when the current command started; a higher count in /state
# means the OpMode's guard stopped everything (jam, no encoder, encoder backwards).
_GUARD_BASE: dict[str, int] = {}


def arm_guard() -> None:
    s = state()
    for w in s.get("wheels", []):
        _GUARD_BASE[w["name"]] = w.get("stallTrips", 0)


def check_guard(s: dict) -> None:
    for w in s.get("wheels", []):
        if w.get("stallTrips", 0) > _GUARD_BASE.get(w["name"], w.get("stallTrips", 0)):
            _GUARD_BASE[w["name"]] = w["stallTrips"]
            raise RobotError(f"STALL GUARD tripped on {w['name']}: {w.get('lastStall', '')}. "
                             "Everything is stopped. Ask the operator to check the wheel before spinning again.")


def gstate() -> dict:
    """state() that raises if the OpMode's stall guard tripped since the command began."""
    s = state()
    check_guard(s)
    return s


def wait_armed(wheel: str, timeout: float = 8.0) -> dict:
    """Waits until the wheel has sat in its at-speed band long enough for the dip detector."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        s = gstate()
        if wheel_of(s, wheel)["dipState"] == 1:
            return s
        time.sleep(0.05)
    w = wheel_of(gstate(), wheel)
    raise NotArmed(f"{wheel} never armed (vel {w['vel']} vs target {w['target']}). The wheel "
                   "is not holding inside the at-speed band - check PIDF / band / target.")


def wait_event(wheel: str, after_count: int, timeout: float) -> dict | None:
    deadline = time.time() + timeout
    while time.time() < deadline:
        s = gstate()
        if s["eventCount"] > after_count:
            for e in reversed(s["events"]):
                if e["id"] > after_count and e["wheel"] == wheel:
                    return e
        time.sleep(0.05)
    return None


def run_step(wheel: str, to: float, frm: float | None, pre: float, sec: float) -> dict:
    before = wheel_of(state(), wheel)["lastStep"]
    params = {"wheel": wheel, "to": to, "sec": sec, "pre": pre}
    if frm is not None:
        params["from"] = frm
    send("step", **params)
    deadline = time.time() + (pre if frm is not None else 0) + sec + 5
    while time.time() < deadline:
        w = wheel_of(gstate(), wheel)
        if w["stepPhase"] == 0 and w["lastStep"] and w["lastStep"] != before:
            return w["lastStep"]
        time.sleep(0.1)
    raise RobotError("step did not finish")


def run_kick(wheel: str, target: float, ms: float) -> dict:
    s = state()
    if wheel_of(s, wheel)["target"] != target or wheel_of(s, wheel)["mode"] != "VEL":
        send("fly", wheel=wheel, vel=target)
    s = wait_armed(wheel)
    count = s["eventCount"]
    send("kick", wheel=wheel, ms=ms)
    e = wait_event(wheel, count, 5.0)
    if e is None:
        raise RobotError("kick produced no dip event - try a longer --ms")
    return e


def run_shot(wheel: str, target: float, i: int, n: int, timeout: float) -> dict | None:
    s = state()
    if wheel_of(s, wheel)["target"] != target or wheel_of(s, wheel)["mode"] != "VEL":
        send("fly", wheel=wheel, vel=target)
    s = wait_armed(wheel, 15.0)
    count = s["eventCount"]
    piece = "POLLEN" if wheel == "pollen" else "NECTAR"
    nid = post_note(f"Feed ONE {piece} into the {wheel.upper()} turret now (shot {i}/{n}).",
                    "action", f"shot:{wheel}")
    print(f"[asked operator #{nid}] shot {i}/{n}", flush=True)
    deadline = time.time() + timeout
    while time.time() < deadline:
        e = wait_event(wheel, count, 0.5)
        if e is not None and e["source"] == "shot":
            return e
        notes = json.loads(_get("/notes", {"since": nid - 1}))["notes"]
        note = next((x for x in notes if x["id"] == nid), None)
        if note and note["resolved"] and note["resolution"] in ("skipped", "done", "cleared"):
            # Done-anyway without a detected dip means the shot did not register.
            e = wait_event(wheel, count, 1.0)
            if e is not None:
                return e
            print(f"[operator #{nid}] {note['resolution']} - no dip detected", flush=True)
            return None
    _get("/ack", {"id": nid, "resolution": "timed out"})
    return None


def set_pidf(wheel: str, coeffs: list[float]) -> None:
    p, i, d, f = coeffs
    send("pidf", wheel=wheel, p=p, i=i, d=d, f=f)


# ---------------------------------------------------------------------------------- export

# Exported values live in SystemConstants.java (one nested @Config class per mechanism, field names
# unique across the file), except group "regression", which goes to modules/ShootingRegression.java
# (each entry carries its file); --only filters on the export entry's group.
FILE_KEYS = ("flywheel", "turret", "intake", "gate", "scoopula", "shooter", "ColorCheck", "zone",
             "regression")


def splice(export: list[dict], write: bool, only: set[str] | None) -> int:
    files: dict[str, list[dict]] = {}
    for e in export:
        group = e.get("group") or os.path.splitext(os.path.basename(e["file"]))[0]
        if only and group not in only:
            continue
        files.setdefault(e["file"], []).append(e)
    changed = 0
    for rel, items in files.items():
        path = os.path.join(TEAMCODE, rel.replace("/", os.sep))
        src = open(path, encoding="utf-8").read()
        new = src
        print(f"--- {rel}")
        for e in items:
            pat = re.compile(
                r"^(\s*(?:(?:public|private|protected|static|final)\s+)*[\w\[\].<>]+\s+"
                + re.escape(e["name"]) + r"\s*=\s*)([^;]+)(;)", re.M)
            hits = pat.findall(new)
            if len(hits) != 1:
                print(f"  !! {e['name']}: {len(hits)} declarations found, not touched")
                continue
            old = hits[0][1].strip()
            if _same(old, e["value"]):
                print(f"     {e['name']} = {old}  (unchanged)")
                continue
            new = pat.sub(lambda m: m.group(1) + e["value"] + m.group(3), new, count=1)
            print(f"  -> {e['name']}: {old}  =>  {e['value']}")
            changed += 1
        if write and new != src:
            with open(path, "w", encoding="utf-8", newline="") as fh:
                fh.write(new)
    if changed and not write:
        print("\nDry run. Re-run with --write to splice. This is a SHIPPED change: build, then "
              "commit each value with its evidence.")
    elif changed:
        print(f"\nWrote {changed} value(s). Build (gradlew :TeamCode:assembleDebug) and commit with evidence.")
    return changed


def _same(a: str, b: str) -> bool:
    norm = lambda s: re.sub(r"\s+", "", s).replace(".0,", ",").replace(".0)", ")").replace(".0}", "}")
    if norm(a) == norm(b):
        return True
    try:
        return float(a) == float(b)
    except ValueError:
        return False


# ---------------------------------------------------------------------------------- commands

def cmd_check(_args) -> int:
    try:
        s = state()
    except RobotError as e:
        print(f"UNREACHABLE: {e}\nJoin the robot WiFi (and keep a second route to the internet).")
        return EXIT_UNREACHABLE
    if not s.get("live"):
        print("App reachable, Mechanism Tuner NOT running." +
              (" Swerve Bring-Up is running." if s.get("swerveLive") else "") +
              " Notes (ask/say) still work.")
        return EXIT_NOT_LIVE
    if not s.get("started"):
        print(f"Mechanism Tuner in INIT (press START). Battery {s['volts']:.2f} V.")
        return EXIT_NOT_STARTED
    missing = [w["device"] for w in s["wheels"] if not w["ok"]] + \
        [t["device"] for t in s["turrets"] + s["servos"] if not t["ok"]]
    print(f"OK - running, battery {s['volts']:.2f} V, loop {s['loopHzTrue']} Hz true "
          f"(dt mean {s['dtMeanMs']} ms, p90 {s['dtP90Ms']} ms)." +
          (f" Missing devices: {', '.join(missing)}" if missing else ""))
    dev = s.get("deviceFile", "{}")
    if dev not in ("", "{}"):
        print(f"  flywheel device bindings from the hub file: {dev}")
    if not s.get("stallGuard", True):
        print("  WARNING: stall guard is OFF (cmd stallGuard on=1)")
    for w in s["wheels"]:
        if w.get("stallTrips"):
            print(f"  {w['name']}: stall guard tripped {w['stallTrips']}x, last: {w.get('lastStall')}")
    return EXIT_OK


def cmd_state(args) -> int:
    s = state()
    if args.json:
        print(json.dumps(s, indent=1))
        return EXIT_OK
    if not s.get("live"):
        print("not live" + (" (Swerve Bring-Up running)" if s.get("swerveLive") else ""))
        return EXIT_NOT_LIVE
    print(f"started={s['started']} volts={s['volts']} loopHzTrue={s['loopHzTrue']} "
          f"band={s['atSpeedFrac']}  msg: {s['message']}")
    for w in s["wheels"]:
        print(f"  {w['name']:7s} {w['mode']:5s} tgt {w['target']:7.1f} vel {w['vel']:7.1f} "
              f"atSpeed={w['atSpeed']} armed={w['dipState'] == 1} pidf={w['pidf']} hub=[{w['hubPidf']}]")
    for t in s["turrets"]:
        print(f"  turret {t['name']:7s} pos {t['pos']} deg {t['deg']} center {t['center']} "
              f"rev {t['reversed']} gear {t['grA']}/{t['grB']}")
    for sv in s["servos"]:
        print(f"  servo {sv['name']:9s} pos {sv['pos']} marks {sv['marks']}")
    c = s["color"]
    print(f"  color r{c['r']} g{c['g']} b{c['b']} a{c['a']} redDom={c['redDominant']} "
          f"blueDom={c['blueDominant']} samples={c['samples']}  intake {s['intake']['mode']} "
          f"{s['intake']['alliance']}")
    print(f"  rec {s['rec']}")
    return EXIT_OK


def cmd_cmd(args) -> int:
    params = dict(kv.split("=", 1) for kv in args.params)
    s = send(args.action, **params)
    print(s["message"])
    return EXIT_OK


def cmd_ask(args) -> int:
    n = ask(args.text, args.timeout)
    return EXIT_OK if n["resolution"] == "done" else EXIT_SKIPPED


def cmd_say(args) -> int:
    print(f"posted #{post_note(args.text, 'info', 'none')}")
    return EXIT_OK


def cmd_clear(_args) -> int:
    print(_get("/notify", {"clear": 1}))
    return EXIT_OK


def cmd_spin(args) -> int:
    need_clear(args)
    s = send("fly", wheel=args.wheel, vel=args.vel)
    print(s["message"])
    return EXIT_OK


def cmd_off(args) -> int:
    print(send("flyOff", wheel=args.wheel or "both")["message"])
    return EXIT_OK


def cmd_pidf(args) -> int:
    set_pidf(args.wheel, [args.p, args.i, args.d, args.f])
    w = wheel_of(state(), args.wheel)
    print(f"{args.wheel} pidf={w['pidf']} hub reads back [{w['hubPidf']}]")
    return EXIT_OK


def cmd_step(args) -> int:
    need_clear(args)
    results = []
    for k in range(args.n):
        r = run_step(args.wheel, args.to, args.frm, args.pre, args.sec)
        results.append(r)
        print(json.dumps(r))
        log_trial("step", {"wheel": args.wheel, **r})
    if args.n > 1:
        for key in ("rise10_90S", "firstInBandS", "settleS", "overshootPct", "ssErrStd"):
            xs = [r[key] for r in results if r.get(key) is not None]
            print(f"{key:14s} {describe(xs)}")
        print(f"never settled: {sum(1 for r in results if r.get('settleS') is None)}/{len(results)}")
    send("flyOff", wheel=args.wheel)
    return EXIT_OK


def cmd_kick(args) -> int:
    need_clear(args)
    evs = []
    for k in range(args.n):
        e = run_kick(args.wheel, args.target, args.ms)
        evs.append(e)
        print(f"kick {k + 1}/{args.n}: dip {e['dipPct']}%  back at speed {e['recoverMs']} ms  "
              f"over {e['overPct']}%  {e['volts']} V")
        log_trial("kick", {"wheel": args.wheel, "ms": args.ms, **e})
    send("flyOff", wheel=args.wheel)
    rec = [e["recoverMs"] for e in evs if e["recoverMs"] >= 0]
    print(f"recoverMs  {describe(rec)}   timeouts {len(evs) - len(rec)}/{len(evs)}")
    print(f"dipPct     {describe([e['dipPct'] for e in evs])}")
    return EXIT_OK


def cmd_shots(args) -> int:
    need_clear(args)
    evs = []
    for k in range(args.n):
        e = run_shot(args.wheel, args.target, k + 1, args.n, args.timeout)
        if e is None:
            print(f"shot {k + 1}/{args.n}: not detected")
            continue
        evs.append(e)
        print(f"shot {k + 1}/{args.n}: dip {e['dipPct']}%  back at speed {e['recoverMs']} ms  "
              f"over {e['overPct']}%  {e['volts']} V")
        log_trial("shot", {"wheel": args.wheel, **e})
    if not args.keep_spinning:
        send("flyOff", wheel=args.wheel)
    rec = [e["recoverMs"] for e in evs if e["recoverMs"] >= 0]
    print(f"recoverMs  {describe(rec)}   timeouts {len(evs) - len(rec)}/{len(evs)}")
    print(f"dipPct     {describe([e['dipPct'] for e in evs])}")
    return EXIT_OK


def cmd_ff(args) -> int:
    need_clear(args)
    powers = args.powers
    send("ffSweep", wheel=args.wheel, powers=powers, hold=args.hold)
    n = len(powers.split(","))
    deadline = time.time() + n * args.hold + 10
    while time.time() < deadline:
        w = wheel_of(gstate(), args.wheel)
        if not w["ffRunning"] and w["ff"]:
            print(json.dumps(w["ff"], indent=1))
            log_trial("ff", {"wheel": args.wheel, **w["ff"]})
            return EXIT_OK
        time.sleep(0.25)
    raise RobotError("ff sweep did not finish")


def cmd_ab(args) -> int:
    """Randomised, interleaved A/B on one wheel: blocks of (A,B) in random order, so battery
    sag lands on both arms (CLAUDE.md section 8 rule 2)."""
    need_clear(args)
    arms = {"A": [float(x) for x in args.a.split()], "B": [float(x) for x in args.b.split()]}
    for k, v in arms.items():
        if len(v) != 4:
            raise RobotError(f"--{k.lower()} needs four numbers: P I D F")
    data: dict[str, list[float]] = {"A": [], "B": []}
    # Trials that produced no number because the gains failed (never held the band, or never got
    # back inside it within 3 s). Counted per arm, never silently dropped: a broken arm must not
    # look better by losing its worst trials (2026-11-12: shipped pollen gains never armed).
    failed = {"A": 0, "B": 0}
    before = wheel_of(state(), args.wheel)["pidf"]
    rng = random.Random(args.seed)
    print(f"A/B {args.test} on {args.wheel}, metric {args.metric}, n={args.n}/arm, seed {args.seed}")
    for block in range(args.n):
        order = ["A", "B"]
        rng.shuffle(order)
        for arm in order:
            set_pidf(args.wheel, arms[arm])
            time.sleep(0.3)
            try:
                if args.test == "kick":
                    r = run_kick(args.wheel, args.target, args.ms)
                elif args.test == "shots":
                    r = run_shot(args.wheel, args.target, block * 2 + order.index(arm) + 1,
                                 args.n * 2, args.timeout)
                    if r is None:
                        print(f"  block {block + 1} {arm}: shot not detected (operator/detector), dropped")
                        continue
                else:
                    r = run_step(args.wheel, args.target, args.frm, args.pre, args.sec)
            except NotArmed as e:
                failed[arm] += 1
                print(f"  block {block + 1} {arm}: FAILED - {e}")
                log_trial(f"ab-{args.test}", {"wheel": args.wheel, "arm": arm, "pidf": arms[arm],
                                              "block": block, "seed": args.seed, "failed": "never armed"})
                continue
            v = r.get(args.metric)
            print(f"  block {block + 1} {arm}: {args.metric}={v}  volts={r.get('volts')}")
            log_trial(f"ab-{args.test}", {"wheel": args.wheel, "arm": arm, "pidf": arms[arm],
                                          "block": block, "seed": args.seed, **r})
            if v is None or (args.metric == "recoverMs" and v < 0):
                failed[arm] += 1
            else:
                data[arm].append(float(v))
    send("flyOff", wheel=args.wheel)
    set_pidf(args.wheel, before)
    print(f"(gains restored to {before})")
    print(f"A {arms['A']}: {describe(data['A'])}   failed {failed['A']}/{args.n}")
    print(f"B {arms['B']}: {describe(data['B'])}   failed {failed['B']}/{args.n}")
    if failed["A"] or failed["B"]:
        print("  Failures are excluded from the means above; compare the failure counts first. "
              "If one arm cannot hold speed at all, use `hold` for that comparison.")
    if len(data["A"]) >= 2 and len(data["B"]) >= 2:
        w = welch(data["A"], data["B"])
        print(f"B - A = {w['diff']:+.2f}, 95% CI [{w['ci'][0]:+.2f}, {w['ci'][1]:+.2f}], "
              f"t={w['t']:.2f}, p={w['p']:.3g}, Welch df={w['df'] and round(w['df'], 1)}")
    return EXIT_OK


def cmd_dircheck(args) -> int:
    """Open-loop spin: checks the encoder agrees with the motor (else velocity mode runs away) and,
    with --ask, asks the operator ONCE whether it is the shooting direction."""
    need_clear(args)
    send("flyPower", wheel=args.wheel, p=args.power)
    vels, v, reply = [], 0.0, None
    try:
        t0 = time.time()
        while time.time() - t0 < args.sec:
            vels.append(wheel_of(gstate(), args.wheel)["vel"])
            time.sleep(0.1)
        v = statistics.fmean(vels[len(vels) // 2:])
        if args.ask and abs(v) >= 100:
            n = ask(f"The {args.wheel.upper()} wheel is spinning now at {int(args.power * 100)}% power. "
                    "Is it turning in the SHOOTING direction? Reply yes or no.", 300)
            reply = (n.get("reply") or n["resolution"]).strip().lower()
    finally:
        send("flyOff", wheel=args.wheel)
    if abs(v) < 100:
        verdict = (f"NOT TURNING at power {args.power}: {v:.0f} ticks/s. Below breakaway, jammed, or no "
                   "encoder. Try a higher --power, then check the wheel and encoder cable.")
    elif v < 0:
        verdict = (f"ENCODER BACKWARDS: +{args.power} power reads {v:.0f} ticks/s. Velocity mode would run "
                   "away. flyDir flips motor AND encoder together, so this needs a wiring/encoder fix.")
    else:
        verdict = f"encoder agrees with motor: +{args.power} power -> {v:.0f} ticks/s."
    if reply is not None:
        verdict += f" Operator says shooting direction: {reply!r}" + (
            "" if reply.startswith(("y", "done")) else " -> run `cmd flyDir wheel=.. reversed=1` and re-check.")
    print(verdict)
    log_trial("dircheck", {"wheel": args.wheel, "power": args.power, "vel": round(v, 1),
                           "operator": reply})
    return EXIT_OK


def _hold_metrics(label: str, target: float, band: float, window: float) -> tuple[list, dict]:
    with open(os.path.join(RUNS, f"{label}.csv"), encoding="utf-8") as fh:
        rows = [r for r in fh.read().splitlines() if r and not r.startswith("#")]
    head = rows[0].split(",")
    ix = {k: head.index(k) for k in ("t", "dtMs", "pVel", "nVel", "volts")}
    data = [r.split(",") for r in rows[1:]]
    t = [float(r[ix["t"]]) for r in data]
    win = [r for r, tt in zip(data, t) if tt >= t[-1] - window]
    return win, ix


def cmd_hold(args) -> int:
    """Randomised interleaved velocity holds: fraction of time inside the at-speed band, longest
    unbroken in-band stretch, velocity sd, whether the dip detector armed. The comparison to use
    when one arm cannot hold speed at all (then shots/kicks cannot run on it)."""
    need_clear(args)
    arms = {"A": [float(x) for x in args.a.split()], "B": [float(x) for x in args.b.split()]}
    col = "pVel" if args.wheel == "pollen" else "nVel"
    before = wheel_of(state(), args.wheel)["pidf"]
    rng = random.Random(args.seed)
    res: dict[str, list[float]] = {"A": [], "B": []}
    print(f"hold A/B on {args.wheel} at {args.target:.0f} ticks/s, {args.sec:.1f} s holds, "
          f"n={args.n}/arm, seed {args.seed}")
    for block in range(args.n):
        order = ["A", "B"]
        rng.shuffle(order)
        for arm in order:
            set_pidf(args.wheel, arms[arm])
            send("flyOff", wheel=args.wheel)
            time.sleep(1.5)
            label = f"hold-{args.wheel}-{args.seed}-{arm}-{block}"
            send("recStart", label=label)
            send("fly", wheel=args.wheel, vel=args.target)
            armed = False
            t0 = time.time()
            while time.time() - t0 < args.sec:
                s = gstate()
                armed |= wheel_of(s, args.wheel)["dipState"] in (1, 2, 3)
                time.sleep(0.1)
            band = s["atSpeedFrac"] * args.target
            send("recStop")
            send("flyOff", wheel=args.wheel)
            cmd_pull(argparse.Namespace(label=label), quiet=True)
            win, ix = _hold_metrics(label, args.target, band, args.sec - 1.5)
            run = best = in_t = tot = 0.0
            vs = []
            for r in win:
                dt = float(r[ix["dtMs"]]) / 1000
                v = float(r[ix[col]])
                vs.append(v)
                tot += dt
                if abs(v - args.target) <= band:
                    in_t += dt
                    run += dt
                    best = max(best, run)
                else:
                    run = 0.0
            frac = in_t / tot if tot else 0.0
            rec = {"wheel": args.wheel, "arm": arm, "pidf": arms[arm], "block": block, "seed": args.seed,
                   "target": args.target, "band": band, "inBandFrac": round(frac, 4),
                   "longestInBandMs": round(best * 1000), "velSd": round(statistics.pstdev(vs), 1),
                   "armed": armed, "volts": float(win[-1][ix["volts"]]), "label": label}
            log_trial("hold", rec)
            res[arm].append(frac)
            print(f"  block {block + 1} {arm} {arms[arm]}: inBand {frac:.3f}  longest {rec['longestInBandMs']} ms  "
                  f"velSd {rec['velSd']}  armed={armed}  {rec['volts']:.2f} V")
    set_pidf(args.wheel, before)
    print(f"(gains restored to {before})")
    for k in ("A", "B"):
        xs = res[k]
        sd = statistics.stdev(xs) if len(xs) > 1 else 0.0
        print(f"{k} {arms[k]}: inBandFrac n={len(xs)}, mean {statistics.fmean(xs):.3f}, sd {sd:.3f}, "
              f"range {min(xs):.3f}-{max(xs):.3f}")
    if len(res["A"]) >= 2 and len(res["B"]) >= 2:
        w = welch(res["A"], res["B"])
        print(f"B - A = {w['diff']:+.3f}, 95% CI [{w['ci'][0]:+.3f}, {w['ci'][1]:+.3f}], "
              f"t={w['t']:.2f}, p={w['p']:.3g}")
    return EXIT_OK


def cmd_rapidfire(args) -> int:
    """Holds the wheel at --target while the operator fires at their own pace; logs every detected
    shot; ends when the operator presses Done on the banner (or the stall guard trips)."""
    need_clear(args)
    count = state()["eventCount"]
    send("fly", wheel=args.wheel, vel=args.target)
    piece = "POLLEN" if args.wheel == "pollen" else "NECTAR"
    nid = post_note(f"RAPID FIRE: {args.wheel.upper()} wheel holding {int(args.target)} ticks/s. Fire as many "
                    f"{piece} as you like. Press Done when finished and the wheel stops.", "action", "ack")
    print(f"[asked operator #{nid}] rapid fire at {args.target:.0f}", flush=True)
    evs, t0, seen = [], time.time(), count
    try:
        while True:
            s = gstate()
            for e in s["events"]:
                if e["id"] > seen and e["wheel"] == args.wheel:
                    seen = e["id"]
                    if e["source"] != "shot":
                        continue
                    evs.append(e)
                    log_trial("rapidfire-shot", {"wheel": args.wheel, "tSession": round(time.time() - t0, 2), **e})
                    print(f"t={time.time() - t0:6.1f}s shot: dip {e['dipPct']}%  back at speed {e['recoverMs']} ms  "
                          f"over {e['overPct']}%  {e['volts']} V", flush=True)
            notes = json.loads(_get("/notes", {"since": nid - 1}))["notes"]
            note = next((x for x in notes if x["id"] == nid), None)
            if note and note["resolved"]:
                if note.get("reply"):
                    print(f"[operator #{nid}] {note['resolution']}: \"{note['reply']}\"")
                break
            if wheel_of(s, args.wheel)["mode"] != "VEL":
                print("wheel left VEL mode: " + s["message"])
                break
            time.sleep(0.1)
    finally:
        send("flyOff", wheel=args.wheel)
        _get("/ack", {"id": nid, "resolution": "cleared"}, retries=1)
    rec = [e["recoverMs"] for e in evs if e["recoverMs"] >= 0]
    print(f"{len(evs)} shots detected in {time.time() - t0:.0f} s; not back at speed: {len(evs) - len(rec)}")
    print(f"recoverMs  {describe(rec)}")
    print(f"dipPct     {describe([e['dipPct'] for e in evs])}")
    print(f"overPct    {describe([e['overPct'] for e in evs if e['overPct'] is not None])}")
    print("Shots closer together than ~0.4 s (300 ms re-arm + recovery) are not scored separately.")
    return EXIT_OK


def _step_summary(rs: list[dict]) -> dict:
    return {"overshoot": statistics.fmean(r["overshootPct"] for r in rs),
            "overshootMax": max(r["overshootPct"] for r in rs),
            "ssMean": statistics.fmean(r["ssErrMean"] for r in rs),
            "ssStd": statistics.fmean(r["ssErrStd"] for r in rs),
            "settled": sum(1 for r in rs if r.get("settleS") is not None), "n": len(rs),
            "settleMax": max((r["settleS"] for r in rs if r.get("settleS") is not None), default=None)}


def cmd_autotune(args) -> int:
    """The 2026-11-12 pollen search as one unattended command (search only - validate on real shots):
    1. ff sweep x2 (must agree within 3%) -> F0 = 32767 / velocity at full power
    2. F closed loop: P=3 I=0 D=0, step 0.5T -> T, secant on steady-state error until |ss| <= 1% of T
       (the open-loop fit was 8% off on 2026-11-12: 13.4 predicted, 12.4 shipped)
    3. grid around it, I=0.1: (5, F), (5, 0.975F), (7, 0.975F), (3, 0.975F); each gets 3 steps at T,
       one each at 0.78T and 1.22T, and --kicks synthetic kicks at T
    4. pass = all settled, mean overshoot <= --max-overshoot, ss sd <= band/3 at T, and the wheel stays
       inside the at-speed band at every target: |ss mean| + 2 ss sd <= band (2026-11-12 shipped
       gains: 1800 -> 12 + 2*10 <= 90, 1400 -> 33 + 2*15 <= 70); recommend the passing candidate
       with the fastest kick recovery."""
    need_clear(args)
    T = args.target
    w0 = wheel_of(state(), args.wheel)
    print(f"autotune {args.wheel} at T={T:.0f} ticks/s, starting gains {w0['pidf']}")
    # 1. feed-forward
    if args.f0:
        f0, vmax = args.f0, None
        print(f"[1] skipped ff sweep, F0={f0}")
    else:
        fs, vmaxs = [], []
        for k in range(args.ff_runs):
            send("ffSweep", wheel=args.wheel, powers=args.powers, hold=args.hold)
            deadline = time.time() + len(args.powers.split(",")) * args.hold + 10
            while True:
                w = wheel_of(gstate(), args.wheel)
                if not w["ffRunning"] and w["ff"]:
                    break
                if time.time() > deadline:
                    raise RobotError("ff sweep did not finish")
                time.sleep(0.25)
            log_trial("ff", {"wheel": args.wheel, "autotune": True, **w["ff"]})
            ff = w["ff"]
            if not ff.get("fFromFull"):
                raise RobotError(f"ff sweep {k + 1} has no full-power level / zero velocity: {ff['levels']}")
            fs.append(ff["fFromFull"])
            vmaxs.append(ff["maxVelAtFull"])
            print(f"[1] ff {k + 1}: levels {[(l[0], l[1]) for l in ff['levels']]}  F_full {ff['fFromFull']}")
        spread = (max(fs) - min(fs)) / statistics.fmean(fs)
        if spread > 0.03:
            raise RobotError(f"ff sweeps disagree by {spread * 100:.1f}% (> 3%): battery or wheel not steady "
                             "(2026-11-12 this was a jam). Find out why before tuning.")
        f0, vmax = statistics.fmean(fs), statistics.fmean(vmaxs)
        print(f"[1] F0 = {f0:.3f} (spread {spread * 100:.1f}%), max {vmax:.0f} ticks/s; T is {T / vmax * 100:.0f}% of max")
        if T > 0.95 * vmax:
            print("    WARNING: T is above 95% of max - no headroom to recover after a shot.")

    def steps(pidf, to, n):
        set_pidf(args.wheel, pidf)
        out = []
        for _ in range(n):
            r = run_step(args.wheel, to, 0.5 * T, 1.5, 3.0)
            log_trial("step", {"wheel": args.wheel, "autotune": True, **r})
            out.append(r)
        send("flyOff", wheel=args.wheel)
        return out

    # 2. F closed loop (P=3, I=0)
    fa = f0
    ssa = statistics.fmean(r["ssErrMean"] for r in steps([3, 0, 0, fa], T, 1))
    print(f"[2] F={fa:.3f}: ss {ssa:+.1f}")
    fb = fa * T / (T + ssa)
    f_best, ss_best = fa, ssa
    for it in range(args.f_iters):
        ssb = statistics.fmean(r["ssErrMean"] for r in steps([3, 0, 0, round(fb, 3)], T, 1))
        print(f"[2] F={fb:.3f}: ss {ssb:+.1f}")
        if abs(ssb) < abs(ss_best):
            f_best, ss_best = fb, ssb
        if abs(ssb) <= 0.01 * T or ssb == ssa:
            break
        fa, ssa, fb = fb, ssb, fb - ssb * (fb - fa) / (ssb - ssa)
        fb = min(max(fb, 0.7 * f0), 1.3 * f0)
    f1 = round(f_best, 2)
    print(f"[2] closed-loop F = {f1} (ss {ss_best:+.1f} ticks/s at P=3 I=0)")

    # 3. grid
    grid = [[5, 0.1, 0, f1], [5, 0.1, 0, round(0.975 * f1, 2)], [7, 0.1, 0, round(0.975 * f1, 2)],
            [3, 0.1, 0, round(0.975 * f1, 2)]]
    band = state()["atSpeedFrac"]
    rows = []
    for pidf in grid:
        main_steps = steps(pidf, T, 3)
        side = {}
        for frac in (0.78, 1.22):
            to = round(frac * T)
            if vmax and to > 0.95 * vmax:
                continue
            side[frac] = steps(pidf, to, 1)[0]
        set_pidf(args.wheel, pidf)
        kicks = []
        for _ in range(args.kicks):
            try:
                e = run_kick(args.wheel, T, 120)
            except NotArmed:
                kicks.append(None)
                continue
            log_trial("kick", {"wheel": args.wheel, "ms": 120, "autotune": True, **e})
            kicks.append(e)
        send("flyOff", wheel=args.wheel)
        st = _step_summary(main_steps)
        good = [k["recoverMs"] for k in kicks if k and k["recoverMs"] >= 0]
        all_ss = [(T, st["ssMean"], st["ssStd"])] + [(round(f * T), r["ssErrMean"], r["ssErrStd"])
                                                     for f, r in side.items()]
        holds = all(abs(m) + 2 * sd <= band * tgt for tgt, m, sd in all_ss)
        ok = (st["settled"] == st["n"] and all(r.get("settleS") is not None for r in side.values())
              and st["overshoot"] <= args.max_overshoot and st["ssStd"] <= band * T / 3
              and holds and len(good) == len(kicks))
        row = {"pidf": pidf, "pass": ok, **st, "side": {str(k): v["ssErrMean"] for k, v in side.items()},
               "kickMean": statistics.fmean(good) if good else None,
               "kickSd": statistics.stdev(good) if len(good) > 1 else None, "kickFails": len(kicks) - len(good)}
        rows.append(row)
        log_trial("autotune-candidate", {"wheel": args.wheel, "target": T, **row})
        print(f"[3] {pidf}: {'PASS' if ok else 'fail'}  overshoot {st['overshoot']:.1f}% (max {st['overshootMax']:.1f})  "
              f"ss {st['ssMean']:+.1f} sd {st['ssStd']:.1f}  side ss {row['side']}  "
              f"kick {row['kickMean'] and round(row['kickMean'])} ms (fails {row['kickFails']})")
    passing = [r for r in rows if r["pass"] and r["kickMean"] is not None]
    if not passing:
        near = [r for r in rows if r["kickMean"] is not None and r["settled"] == r["n"]]
        print("[4] no candidate passed - read the table, adjust one coefficient, re-run steps/kicks by hand.")
        if near:
            c = min(near, key=lambda r: (r["overshoot"], r["kickMean"]))
            print(f"    closest: {c['pidf']} (overshoot {c['overshoot']:.1f}%, side ss {c['side']}, "
                  f"kick {round(c['kickMean'])} ms)")
        send("flyOff", wheel=args.wheel)
        return EXIT_OK
    best = min(passing, key=lambda r: r["kickMean"])
    set_pidf(args.wheel, best["pidf"])
    p = " ".join(str(x) for x in best["pidf"])
    print(f"[4] RECOMMENDED {best['pidf']} (live in the tool now). SEARCH ONLY - validate on real shots:")
    print(f"    mechtune.py ab {args.wheel} --a \"<shipped P I D F>\" --b \"{p}\" --test shots --target {T:.0f} --n 10 --confirmed-clear")
    print(f"    (if shipped cannot hold speed: mechtune.py hold {args.wheel} --a \"<shipped>\" --b \"{p}\" --target {T:.0f} --n 6 --confirmed-clear)")
    return EXIT_OK


def cmd_events(_args) -> int:
    for e in state()["events"]:
        print(json.dumps(e))
    return EXIT_OK


def cmd_pull(args, quiet: bool = False) -> int:
    body = _get("/rec.csv", timeout=10)
    os.makedirs(RUNS, exist_ok=True)
    path = os.path.join(RUNS, f"{args.label}.csv")
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(body)
    rows = [ln.split(",") for ln in body.splitlines() if ln and not ln.startswith(("#", "t,"))]
    dts = sorted(float(r[1]) for r in rows[1:] if r[1])
    if quiet:
        return EXIT_OK
    print(f"saved {path} ({len(rows)} samples)")
    if dts:
        mean = statistics.fmean(dts)
        print(f"loop_hz_true={1000 / mean:.1f}  loop_dt_mean_ms={mean:.2f}  "
              f"loop_dt_p90_ms={dts[int(0.9 * (len(dts) - 1))]:.2f}")
    return EXIT_OK


def cmd_color(args) -> int:
    if args.what == "show":
        c = state()["color"]
        print(f"live r{c['r']} g{c['g']} b{c['b']} a{c['a']} dist {c['dist']}  "
              f"rule red>{c['redOverBlue']}*blue  blue>{c['blueOverRed']}*red  dist<{c['maxDistCm']}cm")
        for lab in c["summary"].get("labels", []):
            print(f"  {lab['label']:10s} n={lab['n']:3d} mean {lab['mean']} min {lab['min']} "
                  f"max {lab['max']} calledRed {lab['redHits']} calledBlue {lab['blueHits']}")
        print(f"  suggest {c['summary'].get('suggest')}")
    elif args.what == "sample":
        if not args.label:
            raise RobotError("color sample needs a LABEL")
        if args.ask:
            ask(args.ask)
        send("colorSample", label=args.label, n=args.n)
        while state()["color"]["sampling"] > 0:
            time.sleep(0.2)
        print(state()["message"])
    elif args.what == "apply":
        print(send("colorSuggestApply")["message"])
    elif args.what == "csv":
        os.makedirs(RUNS, exist_ok=True)
        path = os.path.join(RUNS, time.strftime("color-%Y%m%d-%H%M%S.csv"))
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(_get("/color.csv"))
        print(f"saved {path}")
    return EXIT_OK


def cmd_export(args) -> int:
    s = state()
    if not s.get("live"):
        raise RobotError("export reads the running tool's values; Mechanism Tuner is not running")
    only = set(x.strip() for x in args.only.split(",")) if args.only else None
    splice(s["export"], args.write, only)
    return EXIT_OK


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="command", required=True)

    sub.add_parser("check").set_defaults(fn=cmd_check)
    p = sub.add_parser("state"); p.add_argument("--json", action="store_true"); p.set_defaults(fn=cmd_state)
    p = sub.add_parser("cmd"); p.add_argument("action"); p.add_argument("params", nargs="*"); p.set_defaults(fn=cmd_cmd)
    p = sub.add_parser("ask"); p.add_argument("text"); p.add_argument("--timeout", type=float, default=900); p.set_defaults(fn=cmd_ask)
    p = sub.add_parser("say"); p.add_argument("text"); p.set_defaults(fn=cmd_say)
    sub.add_parser("clear").set_defaults(fn=cmd_clear)

    def wheel(p):
        p.add_argument("wheel", choices=["pollen", "nectar"])

    def clear(p):
        p.add_argument("--confirmed-clear", action="store_true")

    p = sub.add_parser("spin"); wheel(p); p.add_argument("vel", type=float); clear(p); p.set_defaults(fn=cmd_spin)
    p = sub.add_parser("off"); p.add_argument("wheel", nargs="?", choices=["pollen", "nectar", "both"]); p.set_defaults(fn=cmd_off)
    p = sub.add_parser("pidf"); wheel(p)
    for k in "pidf":
        p.add_argument(k, type=float)
    p.set_defaults(fn=cmd_pidf)

    def steppy(p):
        p.add_argument("--from", dest="frm", type=float, default=0.0)
        p.add_argument("--pre", type=float, default=1.5)
        p.add_argument("--sec", type=float, default=3.0)

    p = sub.add_parser("step"); wheel(p); p.add_argument("--to", type=float, required=True); steppy(p)
    p.add_argument("--n", type=int, default=1); clear(p); p.set_defaults(fn=cmd_step)
    p = sub.add_parser("kick"); wheel(p); p.add_argument("--target", type=float, required=True)
    p.add_argument("--ms", type=float, default=120); p.add_argument("--n", type=int, default=10); clear(p); p.set_defaults(fn=cmd_kick)
    p = sub.add_parser("shots"); wheel(p); p.add_argument("--target", type=float, required=True)
    p.add_argument("--n", type=int, default=5); p.add_argument("--timeout", type=float, default=120)
    p.add_argument("--keep-spinning", action="store_true"); clear(p); p.set_defaults(fn=cmd_shots)
    # Starts at 0.5: the pollen wheel did not break away at 0.2 (2026-11-12) and the guard stops a stall.
    p = sub.add_parser("ff"); wheel(p); p.add_argument("--powers", default="0.5,0.6,0.7,0.8,0.9,1.0")
    p.add_argument("--hold", type=float, default=2.0); clear(p); p.set_defaults(fn=cmd_ff)
    p = sub.add_parser("ab"); wheel(p); p.add_argument("--a", required=True); p.add_argument("--b", required=True)
    p.add_argument("--test", choices=["kick", "step", "shots"], default="kick")
    p.add_argument("--metric", default=None, help="kick/shots: recoverMs (default) or dipPct; step: settleS (default), firstInBandS, ...")
    p.add_argument("--target", type=float, required=True); p.add_argument("--ms", type=float, default=120)
    p.add_argument("--n", type=int, default=10); p.add_argument("--timeout", type=float, default=120)
    p.add_argument("--seed", type=int, default=None); steppy(p); clear(p); p.set_defaults(fn=cmd_ab)
    p = sub.add_parser("dircheck"); wheel(p); p.add_argument("--power", type=float, default=0.5)
    p.add_argument("--sec", type=float, default=1.5); p.add_argument("--ask", action="store_true")
    clear(p); p.set_defaults(fn=cmd_dircheck)
    p = sub.add_parser("hold"); wheel(p); p.add_argument("--a", required=True); p.add_argument("--b", required=True)
    p.add_argument("--target", type=float, required=True); p.add_argument("--sec", type=float, default=4.0)
    p.add_argument("--n", type=int, default=6); p.add_argument("--seed", type=int, default=None)
    clear(p); p.set_defaults(fn=cmd_hold)
    p = sub.add_parser("rapidfire"); wheel(p); p.add_argument("--target", type=float, required=True)
    clear(p); p.set_defaults(fn=cmd_rapidfire)
    p = sub.add_parser("autotune"); wheel(p); p.add_argument("--target", type=float, required=True)
    p.add_argument("--powers", default="0.5,0.6,0.7,0.8,0.9,1.0"); p.add_argument("--hold", type=float, default=2.0)
    p.add_argument("--ff-runs", type=int, default=2); p.add_argument("--f0", type=float, default=None)
    p.add_argument("--f-iters", type=int, default=4); p.add_argument("--kicks", type=int, default=5)
    p.add_argument("--max-overshoot", type=float, default=5.0)
    clear(p); p.set_defaults(fn=cmd_autotune)
    sub.add_parser("events").set_defaults(fn=cmd_events)
    p = sub.add_parser("pull"); p.add_argument("label"); p.set_defaults(fn=cmd_pull)
    p = sub.add_parser("color"); p.add_argument("what", choices=["show", "sample", "apply", "csv"])
    p.add_argument("label", nargs="?"); p.add_argument("--n", type=int, default=40); p.add_argument("--ask")
    p.set_defaults(fn=cmd_color)
    p = sub.add_parser("export"); p.add_argument("--write", action="store_true")
    p.add_argument("--only", help="comma list of: " + ", ".join(FILE_KEYS)); p.set_defaults(fn=cmd_export)

    args = ap.parse_args(argv)
    if getattr(args, "command", None) == "hold" and args.seed is None:
        args.seed = random.randrange(1 << 30)
    if getattr(args, "command", None) == "ab":
        if args.metric is None:
            args.metric = "settleS" if args.test == "step" else "recoverMs"
        if args.seed is None:
            args.seed = random.randrange(1 << 30)
    try:
        if args.command in SPINNING and getattr(args, "confirmed_clear", False):
            arm_guard()
        return args.fn(args)
    except TimeoutError as e:
        print(f"TIMEOUT: {e}")
        return EXIT_TIMEOUT
    except KeyboardInterrupt:
        print("interrupted - stopping the flywheels")
        try:
            _get("/cmd", {"action": "flyOff", "wheel": "both"}, retries=2)
        except RobotError:
            pass
        return EXIT_REFUSED
    except RobotError as e:
        print(f"ERROR: {e}")
        # Never leave a wheel spinning because the host script fell over.
        if getattr(args, "command", "") in SPINNING:
            try:
                _get("/cmd", {"action": "flyOff", "wheel": "both"}, retries=2)
            except RobotError:
                pass
        return EXIT_REFUSED
    except KeyboardInterrupt:
        try:
            _get("/cmd", {"action": "stopAll"}, retries=2)
        except RobotError:
            pass
        print("interrupted - stopAll sent")
        return EXIT_REFUSED


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
