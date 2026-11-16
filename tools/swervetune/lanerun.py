#!/usr/bin/env python3
"""High-speed steering A/B in a straight lane - TeleOp's full-speed case (open loop, the bench DRIVE path).

DIAGNOSTIC. The robot sits at one end of a clear lane facing along it (field fence armed). Each run streams the
stick at +power (out) for --sec, stops, then the next run goes at -power (back), so the robot shuttles in the lane and
never needs re-positioning. Arms (setPidf / setMixer) are applied before each run, in random order per block.
Bench heading hold: OFF by default (pure open loop); --hold keeps it ON. 2026-11-16 field: open loop yawed
25-45 deg per 0.9 s run at 0.8 power, angling the robot into the fence (40-57% of samples clamped) - use --hold there.

Scored per run over the fast part (pose speed > --fast in/s): pod |error| mean / p95 (deg), wheel turning-direction
reversals per second (0.5 deg hysteresis), demand movement per sample p95 (a moving demand = the fence or the mixer,
not the pods), fence-clamped fraction, speed (90th percentile - the pose stream glitches), loop rate, volts. Results: current_runs/lanerun.jsonl.

Safety: refuses to start unless the fence is armed and the pose is inside --xmin/--xmax/--ymin/--ymax; aborts a
run if the pose leaves that box or stalls (< 2 in after 1 s); always ends with drive 0 + stop.

  lanerun.py --confirmed-floor --label NAME --arms ARMS.json --n 4 [--power 0.8 --sec 0.9]
             [--xmin 18 --xmax 92 --ymin 18 --ymax 40]
"""
from __future__ import annotations

import argparse
import glob
import json
import math
import os
import random
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from robot import EXIT_OK, EXIT_REFUSED, gates, send, state, _get  # noqa: E402
from steerqual import reversals  # noqa: E402
from circle import load, unwrap, apply_arm  # noqa: E402

LOG = os.path.join(HERE, "current_runs", "lanerun.jsonl")
PERIOD_S = 0.075


def stop_all():
    for action, params in (("drive", {"f": 0, "s": 0, "t": 0}), ("stop", {})):
        try:
            _get("/cmd", {"action": action, **params}, timeout=2.0, retries=4)
        except Exception as e:  # noqa: BLE001
            print(f"WARNING: could not send {action}: {e} - operator: STOP")


def inside(st, a) -> bool:
    p = st["pose"]
    return a.xmin <= p["x"] <= a.xmax and a.ymin <= p["y"] <= a.ymax


def one_run(a, f: float) -> str | None:
    """Streams f for a.sec. Returns an abort reason or None."""
    st = state()
    x0, y0 = st["pose"]["x"], st["pose"]["y"]
    t0 = time.time()
    fails = 0
    try:
        while time.time() - t0 < a.sec:
            try:
                send("drive", f=round(f, 3), s=0, t=0)
                st = state(retries=1, timeout=0.5)
                fails = 0
            except Exception as e:  # noqa: BLE001
                fails += 1
                if fails >= 2:
                    return f"transport ({e})"
                time.sleep(PERIOD_S)
                continue
            if not st.get("box", {}).get("valid"):
                return "fence disarmed"
            if not inside(st, a):
                return f"left the lane at ({st['pose']['x']:.1f}, {st['pose']['y']:.1f})"
            if time.time() - t0 > 1.0 and math.hypot(st["pose"]["x"] - x0, st["pose"]["y"] - y0) < 2.0:
                return "stall"
            time.sleep(PERIOD_S)
    finally:
        stop_all()
    return None


def recentre(a, timeout_s: float = 6.0) -> None:
    """Follower hold back onto the lane line (y = --yline, heading 0) at the nearest x inside [xmin+24, xmax-24].
    Open-loop yaw otherwise walks the robot out of the lane within a run or two (2026-11-16). Arms go AFTER this:
    pedroStart re-applies the shipped steering statics."""
    s = state()
    p, h = s["pose"], s["heading"]["deg"]
    x = min(max(p["x"], a.xmin + 24), a.xmax - 24)
    dh = ((0 - h + 180) % 360) - 180
    if math.hypot(x - p["x"], a.yline - p["y"]) < 1.0 and abs(dh) < 2.0:
        return
    send("pedroStart", pods="shipped")
    time.sleep(0.6)
    send("pedroHold", dx=round(x - p["x"], 2), dy=round(a.yline - p["y"], 2), dh=round(dh, 2))
    end = time.time() + timeout_s
    try:
        while time.time() < end:
            time.sleep(0.2)
            s = state()
            if math.hypot(x - s["pose"]["x"], a.yline - s["pose"]["y"]) < 1.0 and abs(((0 - s["heading"]["deg"] + 180) % 360) - 180) < 2.0:
                time.sleep(0.3)
                break
    finally:
        send("pedroStop")
        time.sleep(0.4)
    send("setHeadingHold", value="true" if a.hold else "false")


def score(path: str, fast: float) -> dict:
    d = load(path)
    t = d["t"]
    m = np.where((d["mode"] == 9) & (np.hypot(d["cf"], d["cs"]) > 0.05))[0]
    if len(m) < 10:
        return {"error": "no drive samples"}
    tt = t[m]
    v = np.hypot(np.gradient(d["px"][m], tt), np.gradient(d["py"][m], tt))
    v = np.where(v > 120, np.nan, v)   # pose-stream glitches
    fm = v > fast
    out = {"csv": os.path.relpath(path, HERE), "peak_v": float(np.nanpercentile(v, 90)), "fast_s": float(np.sum(np.diff(tt)[fm[1:]])),
           "loop_hz_true": float(1 / np.nanmean(d["dt"][m])), "volts": float(np.nanmean(d["volts"][m])),
           "clamped_frac": float(np.mean(np.hypot(d["cf"][m], d["cs"][m]) < 0.98 * np.nanmax(np.hypot(d["cf"][m], d["cs"][m]))))}
    errs, revs, dem = [], 0, []
    for p in range(4):
        e = d[f"p{p}_err"][m]
        errs.append(np.abs(e[fm]))
        w = unwrap(d[f"p{p}_wheel"][m])
        revs += reversals(w[fm]) if fm.sum() > 3 else 0
        c = d[f"p{p}_ctgt"][m]
        dem.append(np.abs(((np.diff(c) + 90) % 180) - 90))
    ae = np.concatenate(errs) if errs else np.array([np.nan])
    out["err_mean"] = float(np.nanmean(ae)) if ae.size else float("nan")
    out["err_p95"] = float(np.nanpercentile(ae, 95)) if ae.size else float("nan")
    out["rev_per_s"] = revs / max(out["fast_s"], 1e-3) / 4
    out["demand_step_p95"] = float(np.nanpercentile(np.concatenate(dem), 95))
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", required=True)
    ap.add_argument("--arms", required=True)
    ap.add_argument("--n", type=int, default=4)
    ap.add_argument("--power", type=float, default=0.8)
    ap.add_argument("--sec", type=float, default=0.9)
    ap.add_argument("--fast", type=float, default=20.0)
    ap.add_argument("--xmin", type=float, default=18)
    ap.add_argument("--xmax", type=float, default=92)
    ap.add_argument("--ymin", type=float, default=18)
    ap.add_argument("--ymax", type=float, default=40)
    ap.add_argument("--restore", default=None)
    ap.add_argument("--confirmed-floor", action="store_true")
    ap.add_argument("--yline", type=float, default=None, help="re-centre onto this y (heading 0) before every run")
    ap.add_argument("--hold", action="store_true",
                    help="keep the bench heading hold ON (open-loop yaw of 25-45 deg per run otherwise angles the robot into the fence)")
    a = ap.parse_args()
    if not a.confirmed_floor:
        print("REFUSED: --confirmed-floor")
        return EXIT_REFUSED
    code, st, reason = gates(need_box=True)
    if code != EXIT_OK:
        print(f"NOT READY: {reason}")
        return code
    if not inside(st, a):
        print(f"REFUSED: pose ({st['pose']['x']:.1f}, {st['pose']['y']:.1f}) is outside the lane box")
        return EXIT_REFUSED
    send("setHeadingHold", value="true" if a.hold else "false")
    arms = json.load(open(a.arms, encoding="utf-8"))
    sign = 1.0
    results = []
    try:
        for block in range(a.n):
            order = list(arms.items())
            random.shuffle(order)
            for name, cmds in order:
                if a.yline is not None:
                    recentre(a)
                    sign = 1.0 if state()["pose"]["x"] < (a.xmin + a.xmax) / 2 else -1.0
                apply_arm(cmds)
                label = f"{a.label}_{name}_b{block}_{'out' if sign > 0 else 'back'}"
                send("recStart", label=label)
                time.sleep(0.25)
                why = one_run(a, sign * a.power)
                time.sleep(0.6)
                send("recStop")
                time.sleep(0.3)
                subprocess.run([sys.executable, os.path.join(HERE, "robot.py"), "pull", label], capture_output=True, text=True)
                path = sorted(glob.glob(os.path.join(HERE, "runs", f"*_{label}.csv*")))[-1]
                res = score(path, a.fast)
                rec = {"ts": time.strftime("%Y-%m-%dT%H:%M:%S"), "label": a.label, "arm": name, "block": block,
                       "dir": "out" if sign > 0 else "back", "power": a.power, "hold": a.hold, "abort": why, "surface": "tiles", **res}
                with open(LOG, "a", encoding="utf-8") as fh:
                    fh.write(json.dumps(rec) + "\n")
                results.append(rec)
                print(f"[{name} b{block} {rec['dir']}] " + (f"ABORT {why} | " if why else "") +
                      (f"err {res['err_mean']:.1f} p95 {res['err_p95']:.0f} | rev/s {res['rev_per_s']:.1f} | peak {res['peak_v']:.0f} in/s "
                       f"| demand p95 {res['demand_step_p95']:.1f} | clamped {100*res['clamped_frac']:.0f}% | {res['volts']:.2f} V"
                       if "error" not in res else res["error"]), flush=True)
                sign = -sign
                if why and why != "stall":
                    return EXIT_REFUSED
    finally:
        stop_all()
        if a.restore:
            apply_arm(arms.get(a.restore, []))
    for arm in sorted({r["arm"] for r in results}):
        rs = [r for r in results if r["arm"] == arm and "err_mean" in r]
        if rs:
            print(f"{arm}: n={len(rs)} err {np.mean([r['err_mean'] for r in rs]):.1f} p95 {np.mean([r['err_p95'] for r in rs]):.0f} "
                  f"rev/s {np.mean([r['rev_per_s'] for r in rs]):.1f} peak {np.mean([r['peak_v'] for r in rs]):.0f}")
    return EXIT_OK


if __name__ == "__main__":
    sys.exit(main())
