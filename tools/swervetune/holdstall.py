#!/usr/bin/env python3
"""Scores pedrocheck HOLD trials for the near-target stall and for jolts, per arm, against a base arm.

DIAGNOSTIC, read-only: it re-reads the recordings pedrocheck.py archived (current_runs/pedrocheck.jsonl ->
runs/*.csv.gz) and adds what pedrocheck does not score:
  short_in     signed end error ALONG the step, last 0.5 s (+ = stopped short, - = past the target)
  tail_cmd     |translation command| over the last 0.5 s - a stalled hold keeps commanding kP x error
  tail_speed   in/s over the last 0.5 s (stalled = ~0)
  reversals    pod drive-power sign flips with |change| > 0.3, all four pods - the jolt metric the
               kD fix used (hold_kd_ab.py, 2026-11-15). It also counts shortest-path FLIPS (pod turns
               180 deg, motor sign changes, the wheel pushes the same way), so it overcounts jolts
  force_rev    the jolt metric without flips: per pod, the wheel push vector (drive power x the pod's
               MEASURED angle) turning more than 90 deg in one loop with |change| > 0.3
  pod_travel   degrees of pod azimuth travel, four pods summed (the operator: 'pods move a lot')
  crossings    along-step error crossing zero after first reaching 1 in (hunting), 0.05 in hysteresis
  peak_cmd     largest |translation command|
plus pedrocheck's own residual_in / settle_1in_s / overshoot_in. The start pose is the recording's first
row (pedrocheck starts recording 0.2 s before the hold command, robot at rest).

  holdstall.py LABEL [--base base]
"""
from __future__ import annotations

import argparse
import csv
import gzip
import json
import math
import os
import statistics

from scipy import stats

HERE = os.path.dirname(os.path.abspath(__file__))
LOG = os.path.join(HERE, "current_runs", "pedrocheck.jsonl")


def fl(r: dict, c: str) -> float:
    v = r.get(c)
    return float(v) if v not in (None, "") else float("nan")


def score(rec: dict) -> dict:
    path = rec["csv"] if os.path.isabs(rec["csv"]) else os.path.join(HERE, rec["csv"])
    with gzip.open(path, "rt") as f:
        rows = list(csv.DictReader(l for l in f if not l.startswith("#")))
    t = [fl(r, "t") for r in rows]
    px, py = [fl(r, "px") for r in rows], [fl(r, "py") for r in rows]
    dx, dy = rec["params"].get("dx", 0), rec["params"].get("dy", 0)
    n = math.hypot(dx, dy)
    ux, uy = dx / n, dy / n
    tx, ty = px[0] + dx, py[0] + dy
    short = [(tx - x) * ux + (ty - y) * uy for x, y in zip(px, py)]
    cmd = [math.hypot(fl(r, "dcf"), fl(r, "dcs")) for r in rows]
    spd = [math.hypot(fl(r, "bvx"), fl(r, "bvy")) for r in rows]
    tail = [i for i, ti in enumerate(t) if ti >= t[-1] - 0.5]
    rev = frev = 0
    travel = 0.0
    for p in range(4):
        d = [fl(r, f"p{p}_dpwr") for r in rows]
        rev += sum(1 for a, b in zip(d, d[1:]) if a * b < 0 and abs(b - a) > 0.3)
        wh = [math.radians(fl(r, f"p{p}_wheel")) for r in rows]
        f = [(pw * math.cos(a), pw * math.sin(a)) for pw, a in zip(d, wh)]
        for (ax, ay), (bx, by) in zip(f, f[1:]):
            if ax * bx + ay * by < 0 and math.hypot(bx - ax, by - ay) > 0.3:
                frev += 1
        travel += sum(abs((math.degrees(b - a) + 180) % 360 - 180) for a, b in zip(wh, wh[1:]) if a == a and b == b)
    crossings, armed, side = 0, False, 0
    for s in short:
        if s != s:
            continue
        armed = armed or abs(s) < 1.0
        if not armed:
            continue
        new = 1 if s > 0.05 else (-1 if s < -0.05 else side)
        if side and new != side:
            crossings += 1
        side = new
    m = rec["metrics"]
    return {
        "short_in": statistics.mean(short[i] for i in tail),
        "tail_cmd": statistics.mean(cmd[i] for i in tail),
        "tail_speed": statistics.mean(spd[i] for i in tail),
        "reversals": rev,
        "force_rev": frev,
        "pod_travel": travel,
        "crossings": crossings,
        "peak_cmd": max(c for c in cmd if c == c),
        "residual_in": m.get("residual_in"),
        "settle_1in_s": m.get("settle_1in_s"),
        "overshoot_in": m.get("overshoot_in"),
        "volts": rec.get("volts"),
    }


def ci(xs):
    xs = [x for x in xs if x == x]
    if len(xs) < 2:
        return (xs[0] if xs else float("nan")), float("nan")
    return statistics.mean(xs), stats.t.ppf(0.975, len(xs) - 1) * statistics.stdev(xs) / math.sqrt(len(xs))


def welch(a, b):
    a, b = [x for x in a if x == x], [x for x in b if x == x]
    if len(a) < 2 or len(b) < 2:
        return float("nan"), (float("nan"), float("nan")), float("nan")
    va, vb = statistics.variance(a) / len(a), statistics.variance(b) / len(b)
    se = math.sqrt(va + vb)
    d = statistics.mean(b) - statistics.mean(a)
    if se == 0:
        return d, (d, d), float("nan")
    df = (va + vb) ** 2 / (va ** 2 / (len(a) - 1) + vb ** 2 / (len(b) - 1))
    h = stats.t.ppf(0.975, df) * se
    p = 2 * stats.t.sf(abs(d / se), df)
    return d, (d - h, d + h), p


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("label")
    ap.add_argument("--base", default="base")
    a = ap.parse_args()
    recs = [json.loads(l) for l in open(LOG, encoding="utf-8")]
    recs = [r for r in recs if r.get("label") == a.label and r.get("kind", "").startswith("hold")
            and "metrics" in r and (r.get("params") or {}).get("dh", 0) == 0]
    refused = sum(1 for r in [json.loads(l) for l in open(LOG, encoding="utf-8")]
                  if r.get("label") == a.label and "refused" in r)
    by = {}
    for r in recs:
        by.setdefault(r["arm"], []).append(score(r))
    keys = ["residual_in", "short_in", "tail_cmd", "tail_speed", "settle_1in_s", "overshoot_in", "reversals",
            "force_rev", "pod_travel", "crossings", "peak_cmd"]
    print(f"{a.label}: {len(recs)} scored hold trials, {refused} refused")
    for arm in sorted(by, key=lambda k: (k != a.base, k)):
        ms = by[arm]
        v = [m["volts"] for m in ms if m["volts"]]
        print(f"\n  {arm}: n={len(ms)}  volts {min(v):.2f}-{max(v):.2f}")
        for k in keys:
            mu, hw = ci([m[k] for m in ms])
            line = f"    {k:13s} {mu:7.3f} +/- {hw:.3f}"
            if arm != a.base and a.base in by:
                d, (lo, hi), p = welch([m[k] for m in by[a.base]], [m[k] for m in ms])
                line += f"   vs {a.base}: {d:+.3f} [{lo:+.3f}, {hi:+.3f}] p={p:.2g}"
            print(line)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
