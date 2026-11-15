#!/usr/bin/env python3
"""A/B: small-turn roll-out (vendored Swerve, 2026-11-15) off vs on, on the Foresight BRAKE stage (it ends every
trial with a sub-epsilon translation and a small heading correction at speed - the pod snap on stop).

DIAGNOSTIC. Randomised, interleaved runs; each run = one BRAKE stage (5 trials), recorded. Per trial, in the
0.5 s after the brake onset (|command| from >= 0.25 to < 0.06): pods whose commanded angle (ctgt) moves more than
60 deg while the robot still rolls (> 5 in/s); peak deceleration (from the per-loop pose velocity); plus the
stage's own stopping distance. Run from tools/swervetune:
  rollout_ab.py --confirmed-floor N        # N runs per arm (default 3)
"""
import csv, glob, gzip, json, math, os, random, subprocess, sys, time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
os.chdir(os.path.dirname(os.path.abspath(__file__)))
from robot import script_gates, send, state  # noqa: E402
import foresightbox as fb  # noqa: E402

ARMS = {"off": 0.0, "on": 0.3}


def trials(path):
    with gzip.open(path, "rt") as f:
        rows = list(csv.DictReader(l for l in f if not l.startswith("#")))
    fl = lambda r, c: float(r[c]) if r.get(c) not in (None, "") else float("nan")
    t = [fl(r, "t") for r in rows]
    cmd = [math.hypot(fl(r, "dcf"), fl(r, "dcs")) for r in rows]
    spd = [math.hypot(fl(r, "bvx"), fl(r, "bvy")) for r in rows]
    ctgt = [[fl(r, f"p{i}_ctgt") for r in rows] for i in range(4)]
    out = []
    for k in range(1, len(rows)):
        if not (cmd[k - 1] >= 0.25 and cmd[k] < 0.06):
            continue
        end = k
        while end < len(rows) - 1 and t[end] - t[k] < 0.5:
            end += 1
        snaps = 0
        for i in range(4):
            for j in range(k, end):
                a, b = ctgt[i][j - 1], ctgt[i][j]
                if a == a and b == b and abs(((b - a + 180) % 360) - 180) > 60 and spd[j] > 5:
                    snaps += 1
                    break
        dec = 0.0
        for j in range(k + 2, end):
            dt = t[j] - t[j - 2]
            if dt > 0 and spd[j] == spd[j] and spd[j - 2] == spd[j - 2]:
                dec = max(dec, (spd[j - 2] - spd[j]) / dt)
        out.append(dict(t=t[k], v0=spd[k], snaps=snaps, peak_decel=dec))
    return out


def main():
    argv, _ = script_gates(sys.argv[1:], box_always=True)
    n = int(argv[0]) if argv else 3
    order = ["off"] * n + ["on"] * n
    random.shuffle(order)
    recs = []
    try:
        for k, arm in enumerate(order):
            send("setMixer", rollout=ARMS[arm])
            time.sleep(0.3)
            fb.recenter(axis="forward")
            label = f"rollout_{arm}_{k}"
            send("recStart", label=label)
            time.sleep(0.3)
            rec = fb.run_stage("brake", "forward", 0.6, None, "shipped")
            time.sleep(0.5)
            send("recStop")
            time.sleep(0.5)
            subprocess.run([sys.executable, "robot.py", "pull", label], capture_output=True, text=True)
            path = sorted(glob.glob(f"runs/*_{label}.csv.gz"))[-1]
            tr = trials(path)
            for x, p in zip(tr, rec.get("points", [])):
                x["stop_in"] = p[1]
                x["v_stage"] = p[0]
            r = dict(arm=arm, rollout=ARMS[arm], k=k, path=path, ok=rec.get("ok"), trials=tr, volts=state()["voltage"])
            recs.append(r)
            print(f"{k} {arm:3s} snaps {[x['snaps'] for x in tr]} peak decel {[round(x['peak_decel']) for x in tr]} "
                  f"stop {[round(x.get('stop_in', float('nan')), 2) for x in tr]}", flush=True)
    finally:
        send("setMixer", rollout=0.3)
        with open("runs/rollout_ab.jsonl", "a") as f:
            for r in recs:
                f.write(json.dumps(r) + "\n")
    for arm in ARMS:
        tr = [x for r in recs if r["arm"] == arm for x in r["trials"]]
        if tr:
            print(f"{arm}: trials {len(tr)}, trials with a pod snap {sum(1 for x in tr if x['snaps'])}, mean snaps "
                  f"{sum(x['snaps'] for x in tr) / len(tr):.2f}, peak decel mean {sum(x['peak_decel'] for x in tr) / len(tr):.0f}")


if __name__ == "__main__":
    main()
