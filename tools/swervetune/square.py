#!/usr/bin/env python3
"""Stick-flick steering test - TeleOp's direction changes, on the 2x2.

DIAGNOSTIC. The robot drives a small square with no heading hold (TeleOp has none): forward, left, back, right,
each leg --leg s at --power, the stick snapping 90 deg between legs exactly as a driver flicks it. The host streams
the stick (75 ms, the robot.py drive cadence, 400 ms watchdog) - a flick is a step, so the host rate does not
matter. Each flick is scored per pod:
  align_ms     step -> the wheel is within 10 deg of the new direction (either end, a flip is the same travel)
               and stays there until the next flick; settle5_ms the same at 5 deg
  demand_ms    step -> the pod's own demand (ctgt, after Swerve's demand slew limit) reaches its new direction
               within 2 deg - the part of align_ms that is the mixer, not the pod
  overshoot    largest error past the demand after the pod first reaches it, deg
  wiggle       wheel turning-direction reversals in the 0.6 s after the step (0.5 deg hysteresis)
Arms (setPidf / setMixer, e.g. slew=) are applied before each square, in random order per block. Results append
to current_runs/square.jsonl.

  square.py run --power 0.25 --leg 0.8 --confirmed-floor --label NAME [--arms ARMS.json --restore ARM --n N]
  square.py score runs/<file>.csv.gz [...]
"""
from __future__ import annotations

import argparse
import glob
import json
import math
import os
import random
import statistics
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from robot import EXIT_OK, EXIT_REFUSED, gates, send, state, _get  # noqa: E402
from steerqual import reversals  # noqa: E402
from circle import load, unwrap, premove, apply_arm  # noqa: E402

LOG = os.path.join(HERE, "current_runs", "square.jsonl")
MODE_DRIVE = 9
PERIOD_S = 0.075
# Robot-frame legs (f, s): forward, left, back, right - field +y, -x, -y, +x at heading 90.
LEGS = [(1, 0), (0, 1), (-1, 0), (0, -1)]


def wrap180(a):
    return (np.asarray(a) + 180.0) % 360.0 - 180.0


def score(path: str) -> dict:
    d = load(path)
    t = d["t"]
    cmd = np.hypot(d["cf"], d["cs"])
    ang = np.degrees(np.arctan2(d["cs"], d["cf"]))
    active = (d["mode"] == MODE_DRIVE) & (cmd > 0.02)
    steps = [k for k in range(1, len(t)) if active[k] and active[k - 1]
             and abs(wrap180(ang[k] - ang[k - 1])) > 45]
    dts = d["dt"][active]
    out = {"csv": os.path.relpath(path, HERE), "flicks": len(steps),
           "loop_hz_true": float(1 / np.nanmean(dts)) if len(dts) else float("nan"),
           "volts": float(np.nanmean(d["volts"][active])) if active.any() else float("nan"),
           "clamped_frac": float(np.mean(cmd[active] < 0.98 * np.nanmax(cmd[active]))) if active.any() else float("nan")}
    per = []
    for n, k in enumerate(steps):
        # The window ends before the next flick (or 0.75 s, or the end of driving), whichever is first.
        stop = steps[n + 1] - 1 if n + 1 < len(steps) else len(t) - 1
        end = k
        while end < stop and t[end + 1] - t[k] <= 0.75 and active[end + 1]:
            end += 1
        for i in range(4):
            w = d[f"p{i}_wheel"][k:end + 1]
            ctgt = d[f"p{i}_ctgt"][k:end + 1]
            tt = t[k:end + 1] - t[k]
            final = ctgt[-1]
            # The wheel may finish flipped (180 deg off the demand, drive reversed): same direction of travel.
            tw = final if abs(wrap180(w[-1] - final)) < 90 else final + 180
            e = wrap180(tw - w)
            dm = next((1000 * tt[j] for j in range(len(tt)) if abs(((ctgt[j] - final + 90) % 180) - 90) < 2),
                      float("nan"))
            # aligned: within 10 deg of the new direction and staying there to the end of the window
            outside = [j for j in range(len(tt)) if abs(e[j]) >= 10]
            al = 1000 * tt[outside[-1] + 1] if outside and outside[-1] + 1 < len(tt) else (0.0 if not outside else float("nan"))
            s5 = [j for j in range(len(tt)) if abs(e[j]) >= 5]
            st5 = 1000 * tt[s5[-1] + 1] if s5 and s5[-1] + 1 < len(tt) else (0.0 if not s5 else float("nan"))
            sgn = 1.0 if e[0] >= 0 else -1.0
            ov = float(max(0.0, np.nanmax(-sgn * e)))
            wig = reversals(unwrap(w))
            per.append({"align_ms": al, "settle5_ms": st5, "demand_ms": dm, "overshoot": ov, "wiggle": wig})
    for key in ("align_ms", "settle5_ms", "demand_ms", "overshoot", "wiggle"):
        xs = [p[key] for p in per if p[key] == p[key]]
        out[key] = float(np.mean(xs)) if xs else float("nan")
        out[key + "_p90"] = float(np.percentile(xs, 90)) if xs else float("nan")
    out["unaligned"] = sum(1 for p in per if p["align_ms"] != p["align_ms"])
    return out


def brief(r: dict) -> str:
    return (f"{r['flicks']} flicks | align {r['align_ms']:4.0f} ms (p90 {r['align_ms_p90']:4.0f}, {r['unaligned']} never) | "
            f"settle5 {r['settle5_ms']:4.0f} ms | demand {r['demand_ms']:4.0f} ms | overshoot {r['overshoot']:4.1f} deg (p90 {r['overshoot_p90']:4.1f}) | "
            f"wiggle {r['wiggle']:.2f} | clamped {100 * r['clamped_frac']:.0f}% | {r['loop_hz_true']:.0f} Hz "
            f"{r['volts']:.2f} V")


def drive_square(power: float, leg_s: float) -> None:
    """Streams the four legs. Same safety as robot.py drive: abort on two transport failures or a disarmed box,
    always drive 0 + stop on exit."""
    fails = 0
    try:
        for f, s in LEGS:
            end = time.time() + leg_s
            while time.time() < end:
                try:
                    send("drive", f=round(power * f, 3), s=round(power * s, 3), t=0)
                    st = state(retries=1, timeout=0.5)
                    fails = 0
                    if not st.get("box", {}).get("valid"):
                        print("ABORT: box disarmed mid-drive.")
                        return
                except Exception as e:  # noqa: BLE001 - transport errors only; the watchdog covers a lost packet
                    fails += 1
                    if fails >= 2:
                        print(f"ABORT: two consecutive transport failures ({e}); the watchdog stops the robot.")
                        return
                time.sleep(PERIOD_S)
    finally:
        for action, params in (("drive", {"f": 0, "s": 0, "t": 0}), ("stop", {})):
            try:
                _get("/cmd", {"action": action, **params}, timeout=2.0, retries=4)
            except Exception as e:  # noqa: BLE001
                print(f"WARNING: could not send {action}: {e}")


def run_one(a, label: str, cmds=()) -> dict:
    s = state()
    b = s["box"]
    cx, cy = (b["minX"] + b["maxX"]) / 2, (b["minY"] + b["maxY"]) / 2
    # The square goes +y, -x, -y, +x from its start corner: start half a side down-right of the centre.
    side = a.speed * a.leg
    premove(cx + side / 2, cy - side / 2)
    apply_arm(cmds)   # after the move: see circle.apply_arm
    send("recStart", label=label)
    time.sleep(0.3)
    drive_square(a.power, a.leg)
    time.sleep(0.4)
    send("recStop")
    time.sleep(0.4)
    subprocess.run([sys.executable, os.path.join(HERE, "robot.py"), "pull", label], capture_output=True, text=True)
    cands = sorted(glob.glob(os.path.join(HERE, "runs", f"*_{label}.csv*")))
    if not cands:
        raise SystemExit(f"no recording for {label}")
    return score(cands[-1])


def cmd_run(a) -> int:
    if not a.confirmed_floor:
        print("REFUSED: --confirmed-floor (operator confirmed: on the floor, in the fence, area clear).")
        return EXIT_REFUSED
    code, st, reason = gates(need_box=True)
    if code != EXIT_OK:
        print(f"NOT READY: {reason}")
        return code
    arms = json.load(open(a.arms, encoding="utf-8")) if a.arms else {"live": []}
    results = []
    try:
        for block in range(a.n):
            order = list(arms.items())
            random.shuffle(order)
            for name, cmds in order:
                label = f"{a.label}_{name}_b{block}"
                res = run_one(a, label, cmds)
                rec = {"ts": time.strftime("%Y-%m-%dT%H:%M:%S"), "label": a.label, "arm": name, "block": block,
                       "power": a.power, "leg_s": a.leg, "surface": "tiles", **res}
                with open(LOG, "a", encoding="utf-8") as f:
                    f.write(json.dumps(rec) + "\n")
                results.append(rec)
                print(f"[{name} b{block}] {brief(res)}", flush=True)
    finally:
        if a.restore and a.arms:
            for action, params in json.load(open(a.arms, encoding="utf-8")).get(a.restore, []):
                send(action, **params)
    for arm in sorted({r["arm"] for r in results}):
        rs = [r for r in results if r["arm"] == arm]
        print(f"{arm}: n={len(rs)}  " + "  ".join(
            f"{k} {statistics.mean([r[k] for r in rs]):.1f}" for k in ("align_ms", "settle5_ms", "demand_ms", "overshoot", "wiggle")))
    return EXIT_OK


def cmd_score(a) -> int:
    for p in a.files:
        print(f"{os.path.basename(p)}: {brief(score(p))}")
    return EXIT_OK


def main() -> int:
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("run")
    p.add_argument("--power", type=float, default=0.25)
    p.add_argument("--leg", type=float, default=0.8, help="seconds per leg")
    p.add_argument("--speed", type=float, default=5.5, help="expected in/s at --power (sizes the square)")
    p.add_argument("--label", required=True)
    p.add_argument("--arms", default=None)
    p.add_argument("--restore", default=None)
    p.add_argument("--n", type=int, default=1)
    p.add_argument("--confirmed-floor", action="store_true")
    p.set_defaults(fn=cmd_run)
    p = sub.add_parser("score")
    p.add_argument("files", nargs="+")
    p.set_defaults(fn=cmd_score)
    a = ap.parse_args()
    return a.fn(a)


if __name__ == "__main__":
    sys.exit(main())
