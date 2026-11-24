#!/usr/bin/env python3
"""Interleaved 4-ball burst A/B on the pollen flywheel (Mechanism Tuner, mechtune.py burst).

Each burst: mechtune burst posts "BURST READY" (operator loads balls, presses Done = GO on /tune), then
"gate OPEN, intake RUNNING". This driver auto-closes that second note AUTO_CLOSE_S after it appears (the 4
pieces fire in ~1-2 s), then scores the pulled trace itself: the tool's per-ball detector is fooled by the
20 ticks/s velocity quantisation.

Score per burst (velocity trace, target T):
  dip        min velocity after the gate opens, % below T
  t_recover  first time below 0.95 T  ->  first time back at >= 0.98 T and staying there 0.3 s (s)
  area       integral of max(0, 0.98 T - v) dt over the burst (ticks/s * s) - speed lost, all balls
"""
import json, os, random, subprocess, sys, time, csv, urllib.request

REPO = "/Users/andres.alonso29/StudioProjects/RuckusBioBuzz"
MT = os.path.join(REPO, "tools/mechtune/mechtune.py")
RUNS = os.path.join(REPO, "tools/mechtune/runs")
BASE = "http://192.168.43.1:8080/tune"
TARGET = float(os.environ.get("TARGET", 1200))
AUTO_CLOSE_S = float(os.environ.get("AUTO_CLOSE_S", 8))
OUT = os.environ.get("OUT", "/private/tmp/claude-502/-Users-andres-alonso29-StudioProjects-RuckusBioBuzz/290a6c21-a58b-4b6d-a267-77d060512b67/scratchpad/bursts_ab.jsonl")


def get(path, **q):
    url = BASE + path + ("?" + urllib.parse.urlencode(q) if q else "")
    with urllib.request.urlopen(url, timeout=3) as r:
        return json.loads(r.read().decode())


def score(path):
    lines = [l for l in open(path) if not l.startswith("#")]
    rows = [r for r in csv.DictReader(lines) if r.get("t") not in ("", None) and r.get("pVel") not in ("", None)]
    t = [float(r["t"]) for r in rows]
    v = [float(r["pVel"]) for r in rows]
    vo = [float(r["volts"]) for r in rows if r.get("volts") not in ("", None)]
    lo, ok = 0.95 * TARGET, 0.98 * TARGET
    i0 = next((i for i, x in enumerate(v) if x < lo), None)
    if i0 is None:
        return {"dip_pct": round(100 * (1 - min(v) / TARGET), 1), "t_recover": 0.0, "area": 0.0, "fired": False,
                "volts_min": min(vo) if vo else None}
    vmin = min(v[i0:])
    t_rec = None
    for j in range(i0, len(v)):
        if v[j] >= ok:
            k = j
            while k < len(v) and t[k] - t[j] < 0.3 and v[k] >= ok - 20.5:
                k += 1
            if k >= len(v) or t[k] - t[j] >= 0.3:
                t_rec = t[j] - t[i0]
                break
    area = 0.0
    for j in range(max(1, i0 - 5), len(v)):
        area += max(0.0, ok - v[j]) * (t[j] - t[j - 1])
    return {"dip_pct": round(100 * (1 - vmin / TARGET), 1), "t_recover": None if t_rec is None else round(t_rec, 3),
            "area": round(area, 1), "fired": True, "volts_min": round(min(vo), 2) if vo else None}


def one(label, pidf):
    before = {n["id"] for n in get("/notes", since=0)["notes"]}
    p = subprocess.Popen([sys.executable, MT, "burst", "pollen", "--target", str(int(TARGET)), "--balls", "4",
                          "--pidf", pidf, "--label", label, "--timeout", "120", "--ready-timeout", "600",
                          "--confirmed-clear"], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    open_seen = None
    while p.poll() is None:
        try:
            for n in get("/notes", since=0)["notes"]:
                if n["id"] in before or n["resolved"]:
                    continue
                if n["text"].startswith("BURST: gate OPEN") and open_seen is None:
                    open_seen = (n["id"], time.time())
            if open_seen and time.time() - open_seen[1] >= AUTO_CLOSE_S:
                get("/ack", id=open_seen[0], resolution="done", reply="auto-closed %.0f s after GO" % AUTO_CLOSE_S)
                open_seen = (open_seen[0], float("inf"))
        except Exception:
            pass
        time.sleep(0.3)
    out = p.stdout.read()
    path = os.path.join(RUNS, label + ".csv")
    s = score(path) if os.path.exists(path) else {"error": "no trace"}
    s.update({"label": label, "pidf": pidf, "target": TARGET, "tool_tail": out.strip().splitlines()[-1:] })
    with open(OUT, "a") as f:
        f.write(json.dumps(s) + "\n")
    print(json.dumps(s), flush=True)
    return s


if __name__ == "__main__":
    arms = json.loads(sys.argv[1])          # {"p45": "45 0.1 0 14.4", ...}
    n = int(sys.argv[2])
    tag = sys.argv[3]
    order = []
    for _ in range(n):
        blk = list(arms)
        random.shuffle(blk)
        order += blk
    print("order:", order, flush=True)
    for i, a in enumerate(order):
        one(f"{tag}-{i+1:02d}-{a}", arms[a])
    subprocess.run([sys.executable, MT, "off"], capture_output=True)
