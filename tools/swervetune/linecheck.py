#!/usr/bin/env python3
"""Follower straight-line A/B at speed in a lane - the autos' case (the real Pedro follower, Foresight).

DIAGNOSTIC. Like lanerun.py, the robot shuttles: each run is a pedroLine of +/- --dx (field x) at --power, so it ends
where the next one starts. Arms are pedroPidf / setMixer / setPidf command lists, applied before each run's pedroStart
(pedroReset first, so every arm starts from the installed build). Scored from the recording (FOLLOW samples):
  err_mean/p95   pod |error| while the robot moves > 10 in/s (the operator's "pods swinging")
  turn_rms       the follower's turn command while moving (heading corrections swing every pod)
  hdg_dev_max    largest heading deviation from the start heading, deg
  end_err        distance from the line's end point after it settles, in;  overshoot along the line, in
  duration       command start -> follower not busy, s
Stall guard: < 1 in of progress after 3 s stops the follower. Results: current_runs/linecheck.jsonl.

  linecheck.py --confirmed-floor --label NAME --arms ARMS.json --n 3 [--dx 45 --power 1.0]
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
from circle import load  # noqa: E402

LOG = os.path.join(HERE, "current_runs", "linecheck.jsonl")


def stop_follower():
    for a in ("pedroStop", "stop"):
        try:
            _get("/cmd", {"action": a}, timeout=2.0, retries=4)
        except Exception as e:  # noqa: BLE001
            print(f"WARNING: {a} failed: {e} - operator: STOP")


def run_line(dx: float, power: float) -> tuple[dict, str | None]:
    s0 = state()
    x0, y0, h0 = s0["pose"]["x"], s0["pose"]["y"], s0["heading"]["deg"]
    send("pedroLine", dx=round(dx, 2), dy=0, power=power)
    time.sleep(0.25)
    msg = state().get("message", "")
    if "EFUS" in msg.upper():
        return {}, msg
    t0 = time.time()
    why = None
    while time.time() - t0 < 10:
        time.sleep(0.15)
        s = state()
        if time.time() - t0 > 3.0 and math.hypot(s["pose"]["x"] - x0, s["pose"]["y"] - y0) < 1.0:
            why = "stall"
            stop_follower()
            break
        if not s.get("pedro", {}).get("busy") and time.time() - t0 > 0.8:
            break
    dur = time.time() - t0
    time.sleep(0.8)
    s = state()
    return {"x0": x0, "y0": y0, "h0": h0, "tx": x0 + dx, "ty": y0, "duration": dur,
            "end_x": s["pose"]["x"], "end_y": s["pose"]["y"]}, why


def score(path: str, run: dict, dx: float) -> dict:
    d = load(path)
    t = d["t"]
    m = np.where(d["mode"] == 10)[0]
    if len(m) < 10:
        return {"error": "no follow samples"}
    tt = t[m]
    v = np.hypot(np.gradient(d["px"][m], tt), np.gradient(d["py"][m], tt))
    v = np.where(v > 120, np.nan, v)
    mv = v > 10
    e = np.concatenate([np.abs(d[f"p{p}_err"][m][mv]) for p in range(4)]) if mv.any() else np.array([np.nan])
    hd = np.abs(((d["heading"][m] - run["h0"] + 180) % 360) - 180)
    along = (d["px"][m] - run["tx"]) * (1 if dx > 0 else -1)
    return {"csv": os.path.relpath(path, HERE), "err_mean": float(np.nanmean(e)), "err_p95": float(np.nanpercentile(e, 95)),
            "turn_rms": float(np.sqrt(np.nanmean(d["dct"][m][mv] ** 2))) if mv.any() else float("nan"),
            "hdg_dev_max": float(np.nanmax(hd)), "end_err": math.hypot(run["end_x"] - run["tx"], run["end_y"] - run["ty"]),
            "overshoot": float(max(0.0, np.nanmax(along))), "duration": run["duration"],
            "speed_p90": float(np.nanpercentile(v, 90)), "volts": float(np.nanmean(d["volts"][m]))}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", required=True)
    ap.add_argument("--arms", required=True)
    ap.add_argument("--n", type=int, default=3)
    ap.add_argument("--dx", type=float, default=45.0)
    ap.add_argument("--power", type=float, default=1.0)
    ap.add_argument("--confirmed-floor", action="store_true")
    a = ap.parse_args()
    if not a.confirmed_floor:
        print("REFUSED: --confirmed-floor")
        return EXIT_REFUSED
    code, st, reason = gates(need_box=True)
    if code != EXIT_OK:
        print(f"NOT READY: {reason}")
        return code
    arms = json.load(open(a.arms, encoding="utf-8"))
    sign = 1.0
    results = []
    try:
        for block in range(a.n):
            order = list(arms.items())
            random.shuffle(order)
            for name, cmds in order:
                send("pedroReset")
                time.sleep(0.2)
                for action, params in cmds:
                    send(action, **params)
                    time.sleep(0.15)
                send("pedroStart", pods="shipped")
                time.sleep(0.6)
                label = f"{a.label}_{name}_b{block}"
                send("recStart", label=label)
                time.sleep(0.25)
                run, why = run_line(sign * a.dx, a.power)
                send("recStop")
                time.sleep(0.3)
                if not run:
                    print(f"[{name} b{block}] REFUSED: {why}")
                    return EXIT_REFUSED
                subprocess.run([sys.executable, os.path.join(HERE, "robot.py"), "pull", label], capture_output=True, text=True)
                path = sorted(glob.glob(os.path.join(HERE, "runs", f"*_{label}.csv*")))[-1]
                res = score(path, run, sign * a.dx)
                rec = {"ts": time.strftime("%Y-%m-%dT%H:%M:%S"), "label": a.label, "arm": name, "block": block,
                       "dx": sign * a.dx, "power": a.power, "abort": why, "surface": "tiles", **res}
                with open(LOG, "a", encoding="utf-8") as fh:
                    fh.write(json.dumps(rec) + "\n")
                results.append(rec)
                print(f"[{name} b{block} {'+' if sign > 0 else '-'}] " + (f"ABORT {why} | " if why else "") +
                      (f"pod err {res['err_mean']:.1f} p95 {res['err_p95']:.0f} | turn {res['turn_rms']:.2f} | hdg dev {res['hdg_dev_max']:.1f} "
                       f"| end {res['end_err']:.2f} in, over {res['overshoot']:.2f} | {res['duration']:.1f} s | {res['speed_p90']:.0f} in/s | {res['volts']:.2f} V"
                       if "error" not in res else res["error"]), flush=True)
                sign = -sign
                if why:
                    return EXIT_REFUSED
    finally:
        stop_follower()
        send("pedroReset")
    for arm in sorted({r["arm"] for r in results}):
        rs = [r for r in results if r["arm"] == arm and "err_mean" in r]
        if rs:
            print(f"{arm}: n={len(rs)} " + " ".join(f"{k} {np.mean([r[k] for r in rs]):.2f}" for k in
                                                   ("err_mean", "turn_rms", "hdg_dev_max", "end_err", "overshoot", "duration")))
    return EXIT_OK


if __name__ == "__main__":
    sys.exit(main())
