"""Single-pod turn step trials - for an UNSTRAPPED chassis on blocks.

DIAGNOSTIC TOOL (tools/swervetune). SEARCH only, never acceptance (TUNING_TASK.md).

`Bench.step_trial` steps all four pods at once (`pidStepAll`); SWERVE_TASK.md warns that the
reaction can walk an unsecured chassis off its blocks, and the rebuilt pods (2026-11-14) slew
~2.5x faster than the pods that warning was written for. This steps ONE pod per trial with
`pidStep pod=N`, records at loop rate with the same recorder, and scores with the same
`swervebench.score_step`, so its numbers are comparable with the archive.

Each trial: park the pod at the base heading, recStart, step to base +/- step, hold, recStop,
release (`stop`). Pods and step directions are interleaved in a seeded random order so battery
or warm-plant drift cannot favour one pod or one direction.

Usage:
    python podstep.py run TAG [--pods 0,1,2,3] [--n 2] [--step 90] [--hold 4]
                              [--gains kp=0.1,kd=0.01,ks=0.0,kf=0] [--seed 1]
    python podstep.py report FILE.jsonl [...]

Appends one JSON line per trial to tools/swervetune/current_runs/podstep_TAG.jsonl.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import random
import statistics
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import swervebench as sb  # noqa: E402

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "current_runs")
PARK_S = 1.5


def parse_gains(spec: str | None) -> dict:
    if not spec:
        return {}
    out = {}
    for part in spec.split(","):
        k, v = part.split("=", 1)
        out[k.strip()] = float(v) if v.strip().lower() not in ("true", "false") else v.strip().lower() == "true"
    return out


def one_trial(bench: sb.Bench, pod: int, base: float, target: float, hold_s: float, label: str) -> dict:
    st = bench.state()
    volts = st.get("voltage")
    gains = sb._gains_of(st)[pod] if sb._gains_of(st) else {}
    bench.cmd("pidStep", pod=pod, deg=base)
    time.sleep(PARK_S)
    bench.cmd("recStart", label=label)
    time.sleep(0.15)
    bench.cmd("pidStep", pod=pod, deg=target)
    time.sleep(hold_s)
    bench.cmd("recStop")
    time.sleep(0.2)
    bench.cmd("stop")
    csv_text = bench.rec_csv()
    trace = sb.parse_csv(csv_text)
    scored = sb.score_step(trace, expected_step_deg=abs(target - base))
    mine = next((p for p in scored["pods"] if p.get("pod") == pod), {"ok": False, "reason": "pod missing"})
    return {
        "kind": "podstep", "pod": pod, "base_deg": base, "target_deg": target,
        "voltage_before": volts, "gains": gains, "label": label,
        "loop_hz_true": scored.get("loop_hz_true"), "voltage_mean": scored.get("voltage_mean"),
        "score": mine, "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
    }


def cmd_run(a) -> int:
    pods = [int(x) for x in a.pods.split(",")]
    bench = sb.Bench()
    gains = parse_gains(a.gains)
    if gains:
        st = bench.set_pidf(scope="all", **gains)
        print("gains set:", sb._gains_of(st)[0])
    os.makedirs(OUT_DIR, exist_ok=True)
    path = os.path.join(OUT_DIR, f"podstep_{a.tag}.jsonl")
    trials = [(p, d) for p in pods for d in (+1, -1) for _ in range(a.n)]
    random.Random(a.seed).shuffle(trials)
    try:
        for i, (pod, d) in enumerate(trials):
            base, target = (0.0, a.step) if d > 0 else (a.step, 0.0)
            r = one_trial(bench, pod, base, target, a.hold, f"{a.tag}_p{pod}_{i}")
            r["tag"] = a.tag
            with open(path, "a", encoding="utf-8") as f:
                f.write(json.dumps(r) + "\n")
            s = r["score"]
            if s.get("ok"):
                print(f"[{i+1}/{len(trials)}] pod {pod} {base:g}->{target:g}  V={r['voltage_mean']:.2f}  "
                      f"meas={s['step_deg_measured']:.1f} rise={s['rise_10_90_s']:.3f} over={s['overshoot_pct']:.1f}% "
                      f"s1.0={s['settle_1_0']:.2f} ss={s['steady_state_deg']:.2f} pp={s['post_settle_pp_deg']:.2f} "
                      f"rings={s['rings']} flips={s['flip_events']}", flush=True)
            else:
                print(f"[{i+1}/{len(trials)}] pod {pod} {base:g}->{target:g}  NOT SCORED: {s.get('reason')}", flush=True)
    finally:
        try:
            bench.cmd("stop")
        except Exception:
            pass
    print("wrote", path)
    return 0


def _ci(xs):
    n = len(xs)
    if n < 2:
        return float("nan"), float("nan")
    m = statistics.mean(xs)
    h = 1.96 * statistics.stdev(xs) / math.sqrt(n)
    return m - h, m + h


def cmd_report(a) -> int:
    rows = []
    for fn in a.files:
        with open(fn, encoding="utf-8") as f:
            rows += [json.loads(line) for line in f if line.strip()]
    keys = ["rise_10_90_s", "overshoot_pct", "settle_1_0", "steady_state_deg", "post_settle_pp_deg"]
    groups = {}
    for r in rows:
        g = (r.get("tag"), r["pod"])
        groups.setdefault(g, []).append(r)
    for (tag, pod), rs in sorted(groups.items()):
        ok = [r["score"] for r in rs if r["score"].get("ok")]
        volts = [r["voltage_mean"] for r in rs if r.get("voltage_mean")]
        line = f"{tag} pod {pod}: n={len(ok)}/{len(rs)} V {min(volts):.2f}-{max(volts):.2f}" if volts else f"{tag} pod {pod}: n={len(ok)}/{len(rs)}"
        for k in keys:
            xs = [s[k] for s in ok if s.get(k) is not None and not (isinstance(s[k], float) and math.isnan(s[k]))]
            if xs:
                lo, hi = _ci(xs)
                line += f"  {k}={statistics.mean(xs):.3f} [{lo:.3f},{hi:.3f}]"
        line += f"  loose(pp>=2)={sum(1 for s in ok if s.get('post_settle_pp_deg', 0) >= 2.0)}"
        print(line)
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("run")
    p.add_argument("tag")
    p.add_argument("--pods", default="0,1,2,3")
    p.add_argument("--n", type=int, default=2, help="trials per pod per direction")
    p.add_argument("--step", type=float, default=90.0)
    p.add_argument("--hold", type=float, default=4.0)
    p.add_argument("--gains", default=None)
    p.add_argument("--seed", type=int, default=1)
    p.set_defaults(fn=cmd_run)
    p = sub.add_parser("report")
    p.add_argument("files", nargs="+")
    p.set_defaults(fn=cmd_report)
    a = ap.parse_args()
    return a.fn(a)


if __name__ == "__main__":
    sys.exit(main())
