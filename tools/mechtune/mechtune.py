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
    python mechtune.py events                      the shot/kick event table
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
          "gatePulse", "intake"}


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


def wait_armed(wheel: str, timeout: float = 8.0) -> dict:
    """Waits until the wheel has sat in its at-speed band long enough for the dip detector."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        s = state()
        if wheel_of(s, wheel)["dipState"] == 1:
            return s
        time.sleep(0.05)
    w = wheel_of(state(), wheel)
    raise RobotError(f"{wheel} never armed (vel {w['vel']} vs target {w['target']}). The wheel "
                     "is not holding inside the at-speed band - check PIDF / band / target.")


def wait_event(wheel: str, after_count: int, timeout: float) -> dict | None:
    deadline = time.time() + timeout
    while time.time() < deadline:
        s = state()
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
        w = wheel_of(state(), wheel)
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
        w = wheel_of(state(), args.wheel)
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
    rng = random.Random(args.seed)
    print(f"A/B {args.test} on {args.wheel}, metric {args.metric}, n={args.n}/arm, seed {args.seed}")
    for block in range(args.n):
        order = ["A", "B"]
        rng.shuffle(order)
        for arm in order:
            set_pidf(args.wheel, arms[arm])
            time.sleep(0.3)
            if args.test == "kick":
                r = run_kick(args.wheel, args.target, args.ms)
            elif args.test == "shots":
                r = run_shot(args.wheel, args.target, block * 2 + order.index(arm) + 1,
                             args.n * 2, args.timeout)
                if r is None:
                    print(f"  block {block + 1} {arm}: shot not detected, dropped")
                    continue
            else:
                r = run_step(args.wheel, args.target, args.frm, args.pre, args.sec)
            v = r.get(args.metric)
            print(f"  block {block + 1} {arm}: {args.metric}={v}  volts={r.get('volts')}")
            log_trial(f"ab-{args.test}", {"wheel": args.wheel, "arm": arm, "pidf": arms[arm],
                                          "block": block, "seed": args.seed, **r})
            if v is not None and not (args.metric == "recoverMs" and v < 0):
                data[arm].append(float(v))
    send("flyOff", wheel=args.wheel)
    print(f"A {arms['A']}: {describe(data['A'])}")
    print(f"B {arms['B']}: {describe(data['B'])}")
    if len(data["A"]) >= 2 and len(data["B"]) >= 2:
        w = welch(data["A"], data["B"])
        print(f"B - A = {w['diff']:+.2f}, 95% CI [{w['ci'][0]:+.2f}, {w['ci'][1]:+.2f}], "
              f"t={w['t']:.2f}, p={w['p']:.3g}, Welch df={w['df'] and round(w['df'], 1)}")
    return EXIT_OK


def cmd_events(_args) -> int:
    for e in state()["events"]:
        print(json.dumps(e))
    return EXIT_OK


def cmd_pull(args) -> int:
    body = _get("/rec.csv", timeout=10)
    os.makedirs(RUNS, exist_ok=True)
    path = os.path.join(RUNS, f"{args.label}.csv")
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(body)
    rows = [ln.split(",") for ln in body.splitlines() if ln and not ln.startswith(("#", "t,"))]
    dts = sorted(float(r[1]) for r in rows[1:] if r[1])
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
    p = sub.add_parser("ff"); wheel(p); p.add_argument("--powers", default="0.3,0.45,0.6,0.75,0.9,1.0")
    p.add_argument("--hold", type=float, default=2.0); clear(p); p.set_defaults(fn=cmd_ff)
    p = sub.add_parser("ab"); wheel(p); p.add_argument("--a", required=True); p.add_argument("--b", required=True)
    p.add_argument("--test", choices=["kick", "step", "shots"], default="kick")
    p.add_argument("--metric", default=None, help="kick/shots: recoverMs (default) or dipPct; step: settleS (default), firstInBandS, ...")
    p.add_argument("--target", type=float, required=True); p.add_argument("--ms", type=float, default=120)
    p.add_argument("--n", type=int, default=10); p.add_argument("--timeout", type=float, default=120)
    p.add_argument("--seed", type=int, default=None); steppy(p); clear(p); p.set_defaults(fn=cmd_ab)
    sub.add_parser("events").set_defaults(fn=cmd_events)
    p = sub.add_parser("pull"); p.add_argument("label"); p.set_defaults(fn=cmd_pull)
    p = sub.add_parser("color"); p.add_argument("what", choices=["show", "sample", "apply", "csv"])
    p.add_argument("label", nargs="?"); p.add_argument("--n", type=int, default=40); p.add_argument("--ask")
    p.set_defaults(fn=cmd_color)
    p = sub.add_parser("export"); p.add_argument("--write", action="store_true")
    p.add_argument("--only", help="comma list of: " + ", ".join(FILE_KEYS)); p.set_defaults(fn=cmd_export)

    args = ap.parse_args(argv)
    if getattr(args, "command", None) == "ab":
        if args.metric is None:
            args.metric = "settleS" if args.test == "step" else "recoverMs"
        if args.seed is None:
            args.seed = random.randrange(1 << 30)
    try:
        return args.fn(args)
    except TimeoutError as e:
        print(f"TIMEOUT: {e}")
        return EXIT_TIMEOUT
    except RobotError as e:
        print(f"ERROR: {e}")
        # Never leave a wheel spinning because the host script fell over.
        if getattr(args, "command", "") in ("spin", "step", "kick", "shots", "ff", "ab"):
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
