#!/usr/bin/env python3
"""Scripted-stick drive tests through the COMPETITION TeleOp's drive code ("TeleOp Drive Probe" OpMode).

DIAGNOSTIC. TeleDriveProbe extends tele/TeleOp and only swaps where the sticks come from, so these numbers are
the driver's path: heading hold, follower.manual, the vendored Swerve mixer, the pods (mechanisms OFF). This is
the tool for drivetrain acceptance from 2026-11-17 on; Bring-Up DRIVE / follower lines are proxies that did not
transfer (FINDINGS 2026-11-16).

  teleprobe.py arm --hx 24 --hy 24 --confirmed-floor     # stop area centred on the robot NOW (3x3 tiles: 70.5 in
                                                         # square, 18 in robot -> +/-26 in of centre travel; leave margin)
  teleprobe.py run --label NAME --arms ARMS.json --tests line,strafe,reverse,circle,spin --n 4 --power 0.8 --confirmed-floor
  teleprobe.py reset | status

Arms file: {"shipped": [], "kp45": [["podGains", {"kp": 0.45}]], "nohold": [["setTele", {"name": "HEADING_HOLD", "value": "false"}]]}
Each run: `reset` (shipped values) -> the arm's commands -> recStart -> stick script -> sticks 0 for --settle s ->
recStop -> pull -> score. Arms in random order per block; each test aims toward the side with more room.

Scored (current_runs/teleprobe.jsonl), moving = body speed > 10 in/s:
  dir_err_mean/p95   angle between where the robot went (body velocity) and where the stick pointed, deg
  yaw_dev            largest heading change while the stick asked for no turn, deg (translation tests)
  yaw_rate_rms       rms yaw rate while moving with no turn asked, deg/s
  err_mean/p95       pod |azimuth error| while moving, deg;  rev_per_s  wheel steering reversals per pod-second
  speed_p90, t90     top speed (in/s) and time from stick to 90% of it, s
  stop_s, stop_in    stick release -> speed < 3 in/s: time and distance
  spin_rate          (spin) mean yaw rate in the steady part, deg/s;  drift_in  translation during the spin, in
  guard_frac         share of stick-on samples the stop area zeroed (contaminated if > 0)
  loop_hz_true = 1/mean(dt), loop_dt_p90_ms, volts
Comparisons: every arm vs the first arm in the file, Welch t, 95% CI, p (scipy).
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
from circle import load, unwrap  # noqa: E402
from steerqual import reversals  # noqa: E402

LOG = os.path.join(HERE, "current_runs", "teleprobe.jsonl")
PERIOD_S = 0.05
SOURCE = "TeleDriveProbe"


def sticks(f: float, l: float, t: float) -> None:
    # Exactly once: a retried stick packet that lands late would restart the watchdog.
    _get("/cmd", {"action": "sticks", "f": round(f, 3), "l": round(l, 3), "t": round(t, 3)}, timeout=1.0, retries=1)


def stop_all() -> None:
    for _ in range(3):
        try:
            _get("/cmd", {"action": "stop"}, timeout=2.0, retries=2)
            return
        except Exception as e:  # noqa: BLE001
            print(f"WARNING: stop failed: {e} - operator: STOP")


def probe_state() -> dict:
    s = state()
    if s.get("source") != SOURCE:
        raise SystemExit(f"REFUSED: /state source is {s.get('source')!r}, not {SOURCE} - start 'TeleOp Drive Probe'")
    return s


# ---- stick scripts: t (s) -> (f, l, turn); robot-frame, TeleOp convention ----------------------------------

def room(s: dict, ux: float, uy: float) -> float:
    """Distance the centre can travel along field unit vector (ux, uy) before the stop-area edge."""
    p, b = s["pose"], s["box"]
    d = []
    if ux > 1e-6: d.append((b["maxX"] - p["x"]) / ux)
    if ux < -1e-6: d.append((b["minX"] - p["x"]) / ux)
    if uy > 1e-6: d.append((b["maxY"] - p["y"]) / uy)
    if uy < -1e-6: d.append((b["minY"] - p["y"]) / uy)
    return min(d) if d else float("inf")


def pick_sign(s: dict, rf: float, rl: float) -> float:
    """+1 or -1: which way along the robot-frame direction (rf, rl) has more room."""
    h = math.radians(s["heading"]["deg"])
    ux, uy = rf * math.cos(h) - rl * math.sin(h), rf * math.sin(h) + rl * math.cos(h)
    n = math.hypot(ux, uy)
    ux, uy = ux / n, uy / n
    return 1.0 if room(s, ux, uy) >= room(s, -ux, -uy) else -1.0


def script(test: str, s: dict, a) -> tuple:
    P, T = a.power, a.sec
    if test in ("line", "strafe", "diag"):
        rf, rl = {"line": (1, 0), "strafe": (0, 1), "diag": (math.sqrt(0.5), math.sqrt(0.5))}[test]
        sg = pick_sign(s, rf, rl)
        return (lambda t: (sg * P * rf, sg * P * rl, 0.0)), T
    if test == "reverse":
        sg = pick_sign(s, 1, 0)
        return (lambda t: (sg * P if t < T / 2 else -sg * P, 0.0, 0.0)), T
    if test == "flick":   # 90 deg stick flick mid-run: forward then sideways (toward the roomier side)
        sg, sl = pick_sign(s, 1, 0), pick_sign(s, 0, 1)
        return (lambda t: (sg * P, 0.0, 0.0) if t < T / 2 else (0.0, sl * P, 0.0)), T
    if test == "circle":  # stick direction sweeps at --rate deg/s: the robot drives a circle without turning
        w = math.radians(a.rate)
        return (lambda t: (P * math.cos(w * t), P * math.sin(w * t), 0.0)), T
    if test == "spin":
        sg = 1.0 if random.random() < 0.5 else -1.0
        return (lambda t: (0.0, 0.0, sg * a.turn)), T
    if test == "arc":     # forward with turn: the driver's sweeping corner
        sg = pick_sign(s, 1, 0)
        return (lambda t: (sg * P, 0.0, a.turn)), T
    raise SystemExit(f"unknown test {test}")


def run_script(fn, dur: float, settle: float) -> tuple[str | None, int]:
    s0 = probe_state()
    stops0 = s0["box"].get("stops", 0)
    t0 = time.time()
    fails = 0
    try:
        while True:
            t = time.time() - t0
            if t > dur + settle:
                break
            f, l, tr = fn(t) if t < dur else (0.0, 0.0, 0.0)
            try:
                sticks(f, l, tr)
                fails = 0
            except Exception as e:  # noqa: BLE001
                fails += 1
                if fails >= 2:
                    return f"transport ({e})", 0
            time.sleep(PERIOD_S)
    finally:
        stop_all()
    s1 = probe_state()
    return (None if s1["box"].get("valid") else "stop area disarmed"), s1["box"].get("stops", 0) - stops0


# ---- scoring -----------------------------------------------------------------------------------------------

def score(path: str, test: str) -> dict:
    d = load(path)
    t = d["t"]
    m9 = d["mode"] == 9
    cf, cs, ct = d["cf"], d["cs"], d["ct"]
    cmd = np.hypot(cf, cs)
    on = m9 & ((cmd > 0.02) | ((test in ("spin",)) & (np.abs(ct) > 0.02)))
    idx = np.where(on)[0]
    if len(idx) < 10:
        return {"error": "no stick samples"}
    i0, i1 = idx[0], idx[-1]
    seg = np.arange(i0, i1 + 1)
    bvx, bvy = d["bvx"], d["bvy"]
    # yaw rate from the heading itself (Bring-Up recordings leave bw at 0), deg/s
    hw = unwrap(d["heading"])
    yaw_rate = np.gradient(hw, t)
    spd = np.hypot(bvx, bvy)
    spd = np.where(spd > 150, np.nan, spd)
    moving = np.zeros_like(on)
    moving[seg] = spd[seg] > 10
    out = {"csv": os.path.relpath(path, HERE), "volts": float(np.nanmean(d["volts"][seg])),
           "loop_hz_true": float(1 / np.nanmean(d["dt"][seg])),
           "loop_dt_mean_ms": float(1000 * np.nanmean(d["dt"][seg])),
           "loop_dt_p90_ms": float(1000 * np.nanpercentile(d["dt"][seg], 90))}
    # stop area: stick-on samples whose translation came out zero mid-run show up as cmd ~ 0 inside the segment
    if test != "spin":
        out["guard_frac"] = float(np.mean(cmd[seg] < 0.02))
    # pods
    e = np.concatenate([np.abs(d[f"p{p}_err"][moving]) for p in range(4)]) if moving.any() else np.array([np.nan])
    out["err_mean"], out["err_p95"] = float(np.nanmean(e)), float(np.nanpercentile(e, 95)) if np.isfinite(e).any() else float("nan")
    mt = float(np.nansum(d["dt"][moving]))
    revs = sum(reversals(unwrap(d[f"p{p}_wheel"])[moving]) for p in range(4)) if moving.sum() > 3 else 0
    out["rev_per_s"] = revs / max(mt, 1e-3) / 4
    out["moving_s"] = mt
    if test == "spin":
        st = seg[t[seg] > t[i0] + 0.4]
        out["spin_rate"] = float(np.nanmean(np.abs(yaw_rate[st]))) if len(st) else float("nan")
        out["drift_in"] = float(math.hypot(d["px"][i1] - d["px"][i0], d["py"][i1] - d["py"][i0]))
        return out
    # direction error: body velocity vs the commanded robot-frame translation
    mv = np.where(moving & (cmd > 0.02))[0]
    if len(mv):
        de = np.degrees(np.abs(((np.arctan2(bvy[mv], bvx[mv]) - np.arctan2(cs[mv], cf[mv])) + np.pi) % (2 * np.pi) - np.pi))
        out["dir_err_mean"], out["dir_err_p95"] = float(np.nanmean(de)), float(np.nanpercentile(de, 95))
    if test != "arc":
        out["yaw_dev"] = float(np.nanmax(np.abs(hw[seg] - hw[i0])))
        mvn = moving & (np.abs(ct) < 1.0)   # ct is post-hold: the hold's own correction counts as "no stick turn"
        out["yaw_rate_rms"] = float(np.sqrt(np.nanmean(yaw_rate[mvn] ** 2))) if mvn.any() else float("nan")
    out["speed_p90"] = float(np.nanpercentile(spd[seg], 90))
    reach = np.where(spd[seg] >= 0.9 * out["speed_p90"])[0]
    out["t90"] = float(t[seg[reach[0]]] - t[i0]) if len(reach) else float("nan")
    if test == "reverse":
        half = np.where(np.sign(cf[seg]) != np.sign(cf[i0]))[0]
        if len(half):
            k = seg[half[0]]
            flip = np.where(np.sign(bvx[k:]) == np.sign(cf[k]))[0]
            out["reverse_s"] = float(t[k + flip[0]] - t[k]) if len(flip) else float("nan")
    after = np.arange(i1 + 1, len(t))
    if len(after) > 3:
        still = np.where(spd[after] < 3)[0]
        if len(still):
            k = after[still[0]]
            out["stop_s"] = float(t[k] - t[i1])
            out["stop_in"] = float(math.hypot(d["px"][k] - d["px"][i1], d["py"][k] - d["py"][i1]))
    return out


KEYS = ("dir_err_mean", "dir_err_p95", "yaw_dev", "yaw_rate_rms", "err_mean", "err_p95", "rev_per_s",
        "speed_p90", "t90", "reverse_s", "stop_s", "stop_in", "spin_rate", "drift_in", "loop_hz_true", "volts")


def compare(results: list[dict], base: str) -> None:
    try:
        from scipy import stats
    except ImportError:
        stats = None
    for test in sorted({r["test"] for r in results}):
        rs = [r for r in results if r["test"] == test and "error" not in r]
        arms = sorted({r["arm"] for r in rs}, key=lambda a: (a != base, a))
        print(f"\n== {test} ==")
        for arm in arms:
            ar = [r for r in rs if r["arm"] == arm]
            cells = []
            for k in KEYS:
                v = np.array([r[k] for r in ar if k in r and r[k] == r[k]])
                if len(v):
                    cells.append(f"{k} {v.mean():.2f}±{v.std(ddof=1) if len(v) > 1 else 0:.2f}")
            print(f"  {arm} (n={len(ar)}): " + "  ".join(cells))
        if stats is None:
            continue
        for arm in arms[1:]:
            for k in KEYS[:11]:
                a_ = np.array([r[k] for r in rs if r["arm"] == arm and k in r and r[k] == r[k]])
                b_ = np.array([r[k] for r in rs if r["arm"] == base and k in r and r[k] == r[k]])
                if len(a_) < 2 or len(b_) < 2:
                    continue
                t_, p_ = stats.ttest_ind(a_, b_, equal_var=False)
                va, vb = a_.var(ddof=1) / len(a_), b_.var(ddof=1) / len(b_)
                se = math.sqrt(va + vb)
                dfree = (va + vb) ** 2 / (va ** 2 / (len(a_) - 1) + vb ** 2 / (len(b_) - 1)) if se > 0 else 1
                q = stats.t.ppf(0.975, dfree) if se > 0 else 0
                dm = a_.mean() - b_.mean()
                print(f"  {arm} - {base}  {k}: {dm:+.2f} [{dm - q * se:+.2f}, {dm + q * se:+.2f}] p={p_:.3f}")


# ---- commands ----------------------------------------------------------------------------------------------

def cmd_arm(a) -> int:
    if not a.confirmed_floor:
        print("REFUSED: --confirmed-floor (the operator confirmed the robot is on the floor, centred, area clear)")
        return EXIT_REFUSED
    probe_state()
    send("arm", hx=a.hx, hy=a.hy)
    time.sleep(0.4)
    s = probe_state()
    print(s.get("message"))
    print(json.dumps(s.get("box")))
    return EXIT_OK if s["box"].get("valid") else EXIT_REFUSED


def cmd_status(_a) -> int:
    s = probe_state()
    print(f"message: {s.get('message')}\nbox: {json.dumps(s.get('box'))}\npose: {s['pose']} heading {s['heading']['deg']:.1f}"
          f"\nvolts {s.get('voltage')}  loop {json.dumps(s.get('loop'))}\ntune: {json.dumps(s.get('tune'))}")
    return EXIT_OK


def cmd_reset(_a) -> int:
    probe_state()
    send("reset")
    time.sleep(0.3)
    print(probe_state().get("message"))
    return EXIT_OK


def cmd_run(a) -> int:
    if not a.confirmed_floor:
        print("REFUSED: --confirmed-floor")
        return EXIT_REFUSED
    code, st, reason = gates(need_box=True)
    if code != EXIT_OK:
        print(f"NOT READY: {reason}")
        return code
    probe_state()
    arms = json.load(open(a.arms, encoding="utf-8"))
    base = next(iter(arms))
    tests = a.tests.split(",")
    results = []
    try:
        for block in range(a.n):
            plan = [(arm, test) for arm in arms for test in tests]
            random.shuffle(plan)
            for name, test in plan:
                send("reset")
                time.sleep(0.15)
                for action, params in arms[name]:
                    send(action, **params)
                    time.sleep(0.15)
                s = probe_state()
                fn, dur = script(test, s, a)
                label = f"{a.label}_{name}_{test}_b{block}"
                send("recStart", label=label)
                time.sleep(0.25)
                why, stops = run_script(fn, dur, a.settle)
                send("recStop")
                time.sleep(0.3)
                subprocess.run([sys.executable, os.path.join(HERE, "robot.py"), "pull", label], capture_output=True, text=True)
                path = sorted(glob.glob(os.path.join(HERE, "runs", f"*_{label}.csv*")))[-1]
                res = score(path, test)
                rec = {"ts": time.strftime("%Y-%m-%dT%H:%M:%S"), "label": a.label, "arm": name, "test": test, "block": block,
                       "power": a.power, "sec": a.sec, "abort": why, "stop_area_hits": stops, "surface": a.surface, **res}
                with open(LOG, "a", encoding="utf-8") as fh:
                    fh.write(json.dumps(rec) + "\n")
                results.append(rec)
                show = "  ".join(f"{k} {res[k]:.2f}" for k in KEYS if k in res)
                print(f"[{name} {test} b{block}] " + (f"ABORT {why} | " if why else "") +
                      (f"STOP AREA x{stops} | " if stops else "") + (show or res.get("error", "")), flush=True)
                if why:
                    return EXIT_REFUSED
                time.sleep(a.rest)
    finally:
        stop_all()
        try:
            send("reset")
        except Exception:  # noqa: BLE001
            pass
    compare(results, base)
    return EXIT_OK


def main() -> int:
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("arm")
    p.add_argument("--hx", type=float, required=True)
    p.add_argument("--hy", type=float, required=True)
    p.add_argument("--confirmed-floor", action="store_true")
    sub.add_parser("status")
    sub.add_parser("reset")
    p = sub.add_parser("run")
    p.add_argument("--label", required=True)
    p.add_argument("--arms", required=True)
    p.add_argument("--tests", default="line,strafe,reverse,spin")
    p.add_argument("--n", type=int, default=4)
    p.add_argument("--power", type=float, default=0.6)
    p.add_argument("--sec", type=float, default=0.8)
    p.add_argument("--turn", type=float, default=0.5)
    p.add_argument("--rate", type=float, default=90.0, help="circle: stick sweep, deg/s")
    p.add_argument("--settle", type=float, default=1.2)
    p.add_argument("--rest", type=float, default=0.5)
    p.add_argument("--surface", default="tiles")
    p.add_argument("--confirmed-floor", action="store_true")
    a = ap.parse_args()
    return {"arm": cmd_arm, "status": cmd_status, "reset": cmd_reset, "run": cmd_run}[a.cmd](a)


if __name__ == "__main__":
    sys.exit(main())
