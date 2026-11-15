#!/usr/bin/env python3
"""Stick-circle steering test - TeleOp's smooth-steering case, on the 2x2.

DIAGNOSTIC. The robot holds its heading (the bench heading hold) while the OpMode sweeps the stick vector at a
fixed rate (drive rot=, 2026-11-15), so every pod must track a demand that turns smoothly and continuously -
exactly what a driver sweeping the left stick asks of TeleOp. Same vendored Swerve mixer and CoaxialPod as
TeleOp; the bench pods run the tool calibration (/state gainsShipped says whether that equals Constants).

  circle.py run --power 0.25 --rate 110 --laps 2 --confirmed-floor --label NAME [--arms ARMS.json --n N]
  circle.py score runs/<file>.csv.gz [...] [--plot]

run: before every circle the follower's hold puts the robot where the circle is centred in the fence
(start = centre + r to the right, r = v / omega, v from --speed or the last run), heading 90. Arms are applied
after that move (pod gains go through setPidf / setMixer and rebuild the bench pods), in random order per block.
Results append to current_runs/circle.jsonl.

score, per pod, over the steady part of the circle (drive active, first --skip s dropped):
  err_mean/p95     |pod error| as the pod reports it (p*_err: demand the pod acted on minus wheel), deg
  lag_ms           mean signed error / sweep rate - how far the wheel trails the demand in time
  wiggle_per_lap   wheel turning-direction reversals (0.5 deg hysteresis) per lap; a clean sweep has 0
  tgt_wiggle       the same on the pod's demand (ctgt) - "demand shaking" vs "response shaking"
  jitter_rms       RMS of the wheel angle about its own 0.15 s moving average, deg (high-frequency shake)
  flips            shortest-path flips during the sweep (a clean sweep has 0)
plus clamped fraction (fence shrank the command), heading error RMS, loop_hz_true, volts.
"""
from __future__ import annotations

import argparse
import gzip
import csv
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
from robot import EXIT_OK, EXIT_REFUSED, gates, send, state  # noqa: E402
from steerqual import reversals  # noqa: E402 - same 0.5 deg hysteresis as every older trace

LOG = os.path.join(HERE, "current_runs", "circle.jsonl")
MODE_DRIVE = 9
POD = ["RB", "RF", "LF", "LB"]


# ---- scoring ------------------------------------------------------------------------------------------------

def load(path: str) -> dict[str, np.ndarray]:
    op = gzip.open if path.endswith(".gz") else open
    with op(path, "rt", encoding="utf-8") as f:
        rows = list(csv.reader(l for l in f if not l.startswith("#")))
    head, body = rows[0], rows[1:]
    return {h: np.array([float(r[i]) if i < len(r) and r[i] != "" else np.nan for r in body]) for i, h in
            enumerate(head)}


def unwrap(a: np.ndarray) -> np.ndarray:
    out = a.copy()
    ok = ~np.isnan(a)
    out[ok] = np.degrees(np.unwrap(np.radians(a[ok])))
    return out


def score(path: str, power: float | None = None, rate: float | None = None, skip: float = 0.6) -> dict:
    d = load(path)
    t = d["t"]
    cmd = np.hypot(d["cf"], d["cs"])
    active = (d["mode"] == MODE_DRIVE) & (cmd > 0.02)
    idx = np.where(active)[0]
    if len(idx) < 20:
        return {"error": "no drive samples"}
    t0, t1 = t[idx[0]], t[idx[-1]]
    m = active & (t >= t0 + skip)
    # Sweep rate from the commanded direction itself (robust to what the host asked for).
    ang = unwrap(np.degrees(np.arctan2(d["cs"], d["cf"])))
    sel = np.where(m)[0]
    tt = t[sel]
    rate_meas = float(np.polyfit(tt - tt[0], ang[sel], 1)[0]) if len(sel) > 5 else float("nan")
    laps = abs(rate_meas) * (tt[-1] - tt[0]) / 360.0 if len(sel) > 5 else float("nan")
    p_cmd = float(np.nanmax(cmd[sel]))
    clamped = float(np.mean(cmd[sel] < 0.98 * p_cmd))
    dts = d["dt"][sel]
    out = {
        "csv": os.path.relpath(path, HERE), "rate_meas_dps": rate_meas, "laps": laps, "power": p_cmd,
        "duration_s": float(tt[-1] - tt[0]), "clamped_frac": clamped,
        "loop_hz_true": float(1 / np.nanmean(dts)), "loop_dt_p90_ms": float(1000 * np.nanpercentile(dts, 90)),
        "volts": float(np.nanmean(d["volts"][sel])),
        "heading_err_rms_deg": float(np.sqrt(np.nanmean(((d["heading"][sel] - d["htgt"][sel] + 180) % 360 - 180) ** 2)))
        if "htgt" in d and not np.all(np.isnan(d["htgt"][sel])) else float("nan"),
        # Without the bench heading hold (TeleOp has none) the heading is free: how far it wanders.
        "heading_dev_rms_deg": float(np.sqrt(np.nanmean(((d["heading"][sel] - d["heading"][idx[0]] + 180) % 360 - 180) ** 2))),
        "heading_drift_deg": float(((d["heading"][idx[-1]] - d["heading"][idx[0]] + 180) % 360) - 180),
        "turn_cmd_rms": float(np.sqrt(np.nanmean(d["ct"][sel] ** 2))),
    }
    sign = 1.0 if rate_meas >= 0 else -1.0
    pods = []
    for i in range(4):
        err = d[f"p{i}_err"][sel]
        wheel = unwrap(d[f"p{i}_wheel"][sel])
        tgt = unwrap(d[f"p{i}_ctgt"][sel]) if f"p{i}_ctgt" in d else unwrap(d[f"p{i}_tgt"][sel])
        flip = d[f"p{i}_flip"][sel]
        k = max(3, int(round(0.15 / max(1e-3, np.nanmean(dts)))))
        smooth = np.convolve(wheel, np.ones(k) / k, mode="same")
        core = slice(k, len(wheel) - k)
        jit = wheel[core] - smooth[core]
        lapn = max(laps, 1e-6)
        pods.append({
            "err_mean": float(np.nanmean(np.abs(err))),
            "err_p95": float(np.nanpercentile(np.abs(err), 95)),
            # signed: pod error = target - wheel, so along the sweep a trailing wheel is +sign
            "lag_ms": float(1000 * np.nanmean(err) * sign / abs(rate_meas)) if rate_meas else float("nan"),
            "wiggle_per_lap": reversals(wheel) / lapn,
            "tgt_wiggle_per_lap": reversals(tgt) / lapn,
            "jitter_rms": float(np.sqrt(np.nanmean(jit ** 2))) if len(jit) else float("nan"),
            "flips": int(np.nansum(np.abs(np.diff(flip)) > 0.5)),
        })
    out["pods"] = pods
    for key in ("err_mean", "err_p95", "lag_ms", "wiggle_per_lap", "tgt_wiggle_per_lap", "jitter_rms", "flips"):
        out[key] = float(np.mean([p[key] for p in pods]))
    return out


def plot(path: str, res: dict, skip: float = 0.6) -> str:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    d = load(path)
    t = d["t"]
    cmd = np.hypot(d["cf"], d["cs"])
    active = (d["mode"] == MODE_DRIVE) & (cmd > 0.02)
    idx = np.where(active)[0]
    t0 = t[idx[0]]
    w = (t >= t0 - 0.3) & (t <= t[idx[-1]] + 0.6)
    fig, axs = plt.subplots(4, 1, figsize=(13, 11), sharex=True)
    tt = t[w] - t0
    for i in range(4):
        axs[0].plot(tt, d[f"p{i}_ctgt"][w], ".", ms=2, label=f"p{i} {POD[i]} demand")
        axs[0].plot(tt, d[f"p{i}_wheel"][w], "-", lw=0.8)
        axs[1].plot(tt, d[f"p{i}_err"][w], lw=0.9, label=f"p{i} {POD[i]}")
        axs[2].plot(tt, d[f"p{i}_pwr"][w], lw=0.8, label=f"p{i} {POD[i]}")
    axs[0].set_ylabel("pod angle deg (dots demand, lines wheel)")
    axs[1].set_ylabel("pod error deg")
    axs[1].axhline(0, color="k", lw=0.5)
    axs[2].set_ylabel("turn servo power")
    axs[3].plot(tt, ((d["heading"][w] - 90 + 180) % 360) - 180, label="heading - 90")
    ax2 = axs[3].twinx()
    ax2.plot(tt, cmd[w], color="gray", lw=0.8, label="|stick|")
    axs[3].set_ylabel("heading err deg")
    ax2.set_ylabel("|stick| after fence")
    axs[3].set_xlabel("s from drive start")
    for ax in axs[:3]:
        ax.legend(fontsize=7, ncol=4, loc="upper right")
    fig.suptitle(f"{os.path.basename(path)}: {res['rate_meas_dps']:.0f} deg/s at {res['power']:.2f}, "
                 f"err mean {res['err_mean']:.1f} p95 {res['err_p95']:.1f} deg, lag {res['lag_ms']:.0f} ms, "
                 f"wiggle {res['wiggle_per_lap']:.2f}/lap, jitter {res['jitter_rms']:.2f} deg")
    fig.tight_layout()
    out = path.replace(".csv.gz", "_circle.png").replace(".csv", "_circle.png")
    fig.savefig(out, dpi=100)
    plt.close(fig)
    return out


def brief(r: dict) -> str:
    return (f"{r['rate_meas_dps']:6.0f} deg/s x{r['laps']:.1f} laps p={r['power']:.2f} | err mean {r['err_mean']:5.1f} "
            f"p95 {r['err_p95']:5.1f} | lag {r['lag_ms']:4.0f} ms | wiggle/lap {r['wiggle_per_lap']:.2f} "
            f"(demand {r['tgt_wiggle_per_lap']:.2f}) | jitter {r['jitter_rms']:.2f} | flips {r['flips']:.1f} | "
            f"clamped {100 * r['clamped_frac']:.0f}% | turn {r['turn_cmd_rms']:.2f} hdg dev {r['heading_dev_rms_deg']:.1f} drift {r['heading_drift_deg']:+.1f} | "
            f"{r['loop_hz_true']:.0f} Hz {r['volts']:.2f} V")


# ---- running ------------------------------------------------------------------------------------------------

def premove(x: float, y: float, timeout_s: float = 5.0) -> None:
    s = state()
    p, h = s["pose"], s["heading"]["deg"]
    dh = ((90 - h + 180) % 360) - 180
    if math.hypot(x - p["x"], y - p["y"]) < 0.6 and abs(dh) < 1.5:
        return
    send("pedroStart", pods="shipped")
    time.sleep(0.6)
    send("pedroHold", dx=round(x - p["x"], 2), dy=round(y - p["y"], 2), dh=round(dh, 2))
    end = time.time() + timeout_s
    try:
        while time.time() < end:
            time.sleep(0.25)
            s = state()
            if math.hypot(x - s["pose"]["x"], y - s["pose"]["y"]) < 0.6 and abs(((90 - s["heading"]["deg"] + 180) % 360) - 180) < 1.5:
                time.sleep(0.4)
                break
    finally:
        send("pedroStop")
        time.sleep(0.4)


def apply_arm(cmds) -> None:
    """Arm commands go AFTER the re-centring move: premove builds a Pedro follower (pedroStart), and since
    2026-11-15 every drivetrain build re-applies the shipped steering statics (Constants.applySteeringStatics) -
    an arm applied before it was silently reset (acceptance run accept_*, 'old' arm)."""
    for action, params in cmds:
        send(action, **params)
        time.sleep(0.15)


def run_one(a, label: str, speed: float, cmds=()) -> dict:
    s = state()
    b = s["box"]
    cx, cy = (b["minX"] + b["maxX"]) / 2, (b["minY"] + b["maxY"]) / 2
    r = speed / math.radians(abs(a.rate))
    # Stick starts robot-forward (= field +y at heading 90); a CCW sweep curves left, so the centre is r to the
    # left of the start: start r to the right of the box centre (mirrored for CW).
    premove(cx + (r if a.rate > 0 else -r), cy)
    apply_arm(cmds)
    sec = min(10.0, a.laps * 360.0 / abs(a.rate) + 0.5)
    args = [sys.executable, os.path.join(HERE, "robot.py"), "drive", "--f", str(a.power), "--rot", str(a.rate),
            "--sec", f"{sec:.2f}", "--confirmed-floor", "--rec", label]
    if a.power > 0.30:
        args += ["--cap", str(a.power)]
    p = subprocess.run(args, capture_output=True, text=True)
    import glob
    cands = sorted(glob.glob(os.path.join(HERE, "runs", f"*_{label}.csv*")))
    if not cands:
        raise SystemExit("no recording pulled:\n" + p.stdout[-2000:] + p.stderr[-2000:])
    path = cands[-1]
    tail = [l for l in p.stdout.splitlines() if l.startswith(("ticks=", "end ", "ABORT", "REFUSED", "WARNING"))]
    print("   " + " | ".join(tail), flush=True)
    res = score(path)
    res["r_planned_in"] = r
    return res


def cmd_run(a) -> int:
    if not a.confirmed_floor:
        print("REFUSED: --confirmed-floor (operator confirmed: on the floor, in the fence, area clear).")
        return EXIT_REFUSED
    code, st, reason = gates(need_box=True)
    if code != EXIT_OK:
        print(f"NOT READY: {reason}")
        return code
    arms = {"live": []}
    if a.arms:
        arms = json.load(open(a.arms, encoding="utf-8"))
    speed = a.speed
    results = []
    try:
        for block in range(a.n):
            order = list(arms.items())
            random.shuffle(order)
            for name, cmds in order:
                label = f"{a.label}_{name}_b{block}"
                res = run_one(a, label, speed, cmds)
                rec = {"ts": time.strftime("%Y-%m-%dT%H:%M:%S"), "label": a.label, "arm": name, "block": block,
                       "power_req": a.power, "rate_req": a.rate, "surface": "tiles", **res}
                with open(LOG, "a", encoding="utf-8") as f:
                    f.write(json.dumps(rec) + "\n")
                results.append(rec)
                print(f"[{name} b{block}] {brief(res)}", flush=True)
    finally:
        try:
            send("drive", f=0, s=0, t=0)
            send("stop")
        except Exception:  # noqa: BLE001 - best effort, robot.py drive already stopped
            pass
        if a.restore:
            for action, params in json.load(open(a.arms, encoding="utf-8")).get(a.restore, []):
                send(action, **params)
    summarize(results)
    return EXIT_OK


def summarize(results: list[dict]) -> None:
    keys = ["err_mean", "err_p95", "lag_ms", "wiggle_per_lap", "tgt_wiggle_per_lap", "jitter_rms", "flips",
            "clamped_frac"]
    arms = sorted({r["arm"] for r in results})
    for arm in arms:
        rs = [r for r in results if r["arm"] == arm and "err_mean" in r]
        if not rs:
            continue
        parts = []
        for k in keys:
            xs = [r[k] for r in rs]
            mu = statistics.mean(xs)
            sd = statistics.stdev(xs) if len(xs) > 1 else float("nan")
            parts.append(f"{k} {mu:.2f} (sd {sd:.2f})")
        print(f"{arm}: n={len(rs)}  " + "  ".join(parts))


def cmd_score(a) -> int:
    for path in a.files:
        res = score(path)
        if "error" in res:
            print(f"{os.path.basename(path)}: {res['error']}")
            continue
        print(f"{os.path.basename(path)}: {brief(res)}")
        for i, p in enumerate(res["pods"]):
            print(f"   p{i} {POD[i]}: err mean {p['err_mean']:.1f} p95 {p['err_p95']:.1f}  lag {p['lag_ms']:.0f} ms  "
                  f"wiggle/lap {p['wiggle_per_lap']:.2f} (demand {p['tgt_wiggle_per_lap']:.2f})  jitter "
                  f"{p['jitter_rms']:.2f}  flips {p['flips']}")
        if a.plot:
            print("   plot:", plot(path, res))
    return EXIT_OK


def main() -> int:
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("run")
    p.add_argument("--power", type=float, default=0.25)
    p.add_argument("--rate", type=float, default=110.0, help="stick sweep, deg/s, + = CCW")
    p.add_argument("--laps", type=float, default=2.0)
    p.add_argument("--speed", type=float, default=5.5, help="expected robot speed at --power, in/s (sizes r)")
    p.add_argument("--label", required=True)
    p.add_argument("--arms", default=None)
    p.add_argument("--restore", default=None, help="arm to re-apply at the end (e.g. the shipped one)")
    p.add_argument("--n", type=int, default=1)
    p.add_argument("--confirmed-floor", action="store_true")
    p.set_defaults(fn=cmd_run)
    p = sub.add_parser("score")
    p.add_argument("files", nargs="+")
    p.add_argument("--plot", action="store_true")
    p.set_defaults(fn=cmd_score)
    a = ap.parse_args()
    return a.fn(a)


if __name__ == "__main__":
    sys.exit(main())
