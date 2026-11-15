# DIAGNOSTIC (2026-11-15). Run from tools/swervetune: python3 holdkd_ab.py --confirmed-floor N
"""A/B: follower hold shake vs translational kD (0.025 shipped vs 0). Randomised, interleaved. Each arm unit =
two follower holds (axis end, then middle), recorded; scored on drive-power reversals, peak command, battery sag."""
import csv, gzip, glob, json, math, os, random, subprocess, sys, time
sys.path.insert(0, "/Users/andres.alonso29/StudioProjects/RuckusBioBuzz/tools/swervetune")
os.chdir("/Users/andres.alonso29/StudioProjects/RuckusBioBuzz/tools/swervetune")
from robot import script_gates, send, state
import foresightbox as fb

argv, _ = script_gates(sys.argv[1:], box_always=True)
n = int(argv[0]) if argv else 3
arms = ["A"] * n + ["B"] * n
random.seed(int(time.time()))
random.shuffle(arms)
KD = {"A": 0.025, "B": 0.0}
out = []
try:
    for k, arm in enumerate(arms):
        send("pedroPidf", td=KD[arm])
        time.sleep(0.5)
        label = f"holdkd_{arm}_{k}"
        send("recStart", label=label)
        time.sleep(0.3)
        axis = "forward" if k % 2 == 0 else "strafe"
        fb.recenter(axis=axis, tol_in=1.0, tol_deg=2.0, timeout_s=5)
        fb.recenter(axis=None, tol_in=1.0, tol_deg=2.0, timeout_s=5)
        time.sleep(0.5)
        send("recStop")
        time.sleep(0.5)
        r = subprocess.run([sys.executable, "robot.py", "pull", label], capture_output=True, text=True)
        path = sorted(glob.glob(f"runs/*_{label}.csv.gz"))[-1]
        with gzip.open(path, "rt") as f:
            rows = list(csv.DictReader(l for l in f if not l.startswith("#")))
        fl = lambda r, c: float(r[c]) if r.get(c) not in (None, "") else float("nan")
        rev = 0
        for i in range(4):
            d = [fl(r, f"p{i}_dpwr") for r in rows]
            rev += sum(1 for a, b in zip(d, d[1:]) if a * b < 0 and abs(b - a) > 0.3)
        cmd = [math.hypot(fl(r, "dcf"), fl(r, "dcs")) for r in rows]
        cmd = [c for c in cmd if c == c]
        volts = [fl(r, "volts") for r in rows if fl(r, "volts") == fl(r, "volts")]
        s = state()
        rec = dict(arm=arm, kd=KD[arm], k=k, axis=axis, path=path, reversals=rev, peak_cmd=max(cmd) if cmd else None,
                   min_volts=min(volts), n=len(rows), end_pose=(s["pose"]["x"], s["pose"]["y"], s["heading"]["deg"]))
        out.append(rec)
        print(f"{k:2d} arm {arm} kD {KD[arm]:.3f} {axis:7s} reversals {rev:3d} peak cmd {rec['peak_cmd']:.2f} "
              f"min V {rec['min_volts']:.2f}", flush=True)
finally:
    send("pedroReset")
    time.sleep(0.3)
    with open("runs/hold_kd_ab.jsonl", "a") as f:
        for r in out:
            f.write(json.dumps(r) + "\n")
for arm in ("A", "B"):
    rs = [r for r in out if r["arm"] == arm]
    if rs:
        print(f"arm {arm} (kD {KD[arm]}): n={len(rs)} reversals mean {sum(r['reversals'] for r in rs)/len(rs):.1f} "
              f"{[r['reversals'] for r in rs]}  peak cmd {[round(r['peak_cmd'],2) for r in rs]}  min V {[round(r['min_volts'],2) for r in rs]}")
