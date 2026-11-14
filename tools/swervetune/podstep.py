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


def cmd_ab(a) -> int:
    """Randomised, interleaved A/B: gains are re-applied before EVERY trial, so drift (battery,
    warm plant) lands on both arms alike. Each arm gets n trials per pod per direction."""
    pods = [int(x) for x in a.pods.split(",")]
    arms = {"A": parse_gains(a.a), "B": parse_gains(a.b)}
    bench = sb.Bench()
    os.makedirs(OUT_DIR, exist_ok=True)
    path = os.path.join(OUT_DIR, f"podstep_{a.tag}.jsonl")
    trials = [(arm, p, d) for arm in "AB" for p in pods for d in (+1, -1) for _ in range(a.n)]
    random.Random(a.seed).shuffle(trials)
    try:
        for i, (arm, pod, d) in enumerate(trials):
            bench.set_pidf(scope="all", **arms[arm])
            base, target = (0.0, a.step) if d > 0 else (a.step, 0.0)
            r = one_trial(bench, pod, base, target, a.hold, f"{a.tag}_{arm}_p{pod}_{i}")
            r["tag"] = a.tag
            r["arm"] = arm
            r["arm_gains"] = a.a if arm == "A" else a.b
            with open(path, "a", encoding="utf-8") as f:
                f.write(json.dumps(r) + "\n")
            s = r["score"]
            if s.get("ok"):
                print(f"[{i+1}/{len(trials)}] {arm} pod {pod} {base:g}->{target:g} V={r['voltage_mean']:.2f} "
                      f"pp={s['post_settle_pp_deg']:.2f} ss={s['steady_state_deg']:.2f} over={s['overshoot_pct']:.1f}%",
                      flush=True)
            else:
                print(f"[{i+1}/{len(trials)}] {arm} pod {pod} NOT SCORED: {s.get('reason')}", flush=True)
    finally:
        try:
            bench.cmd("stop")
        except Exception:
            pass
    print("wrote", path)
    return cmd_abreport(argparse.Namespace(tag=a.tag))


def _welch(xa, xb):
    """B - A difference, Welch 95% CI (normal approx for df >= ~20) and two-sided p."""
    ma, mb = statistics.mean(xa), statistics.mean(xb)
    va, vb = statistics.variance(xa), statistics.variance(xb)
    se = math.sqrt(va / len(xa) + vb / len(xb))
    d = mb - ma
    t = d / se if se > 0 else float("inf")
    p = math.erfc(abs(t) / math.sqrt(2))
    return d, d - 1.96 * se, d + 1.96 * se, t, p


def cmd_abreport(a) -> int:
    path = os.path.join(OUT_DIR, f"podstep_{a.tag}.jsonl")
    with open(path, encoding="utf-8") as f:
        rows = [json.loads(line) for line in f if line.strip()]
    by = {arm: [r for r in rows if r.get("arm") == arm and r["score"].get("ok")] for arm in "AB"}
    for arm in "AB":
        g = next((r.get("arm_gains") for r in rows if r.get("arm") == arm), "")
        print(f"arm {arm}: n={len(by[arm])}  gains {g}")
    volts = [r["voltage_mean"] for r in rows if r.get("voltage_mean")]
    print(f"volts {min(volts):.2f}-{max(volts):.2f}")
    for key, fn in (("post_settle_pp_deg", lambda s: s["post_settle_pp_deg"]),
                    ("|steady_state_deg|", lambda s: abs(s["steady_state_deg"])),
                    ("overshoot_pct", lambda s: s["overshoot_pct"]),
                    ("rise_10_90_s", lambda s: s["rise_10_90_s"])):
        xa = [fn(r["score"]) for r in by["A"]]
        xb = [fn(r["score"]) for r in by["B"]]
        xa = [x for x in xa if not math.isnan(x)]
        xb = [x for x in xb if not math.isnan(x)]
        if len(xa) < 2 or len(xb) < 2:
            continue
        d, lo, hi, t, p = _welch(xa, xb)
        print(f"  {key:22} A {statistics.mean(xa):7.3f}  B {statistics.mean(xb):7.3f}  B-A {d:+.3f} "
              f"[{lo:+.3f}, {hi:+.3f}]  t={t:.2f} p={p:.2g}")
    la = sum(r["score"]["post_settle_pp_deg"] >= 2.0 for r in by["A"])
    lb = sum(r["score"]["post_settle_pp_deg"] >= 2.0 for r in by["B"])
    print(f"  loose (pp >= 2 deg): A {la}/{len(by['A'])} (Wilson upper {wilson_upper(la, len(by['A'])):.2f})  "
          f"B {lb}/{len(by['B'])} (Wilson upper {wilson_upper(lb, len(by['B'])):.2f})")
    return 0


def wilson_upper(k: int, n: int, z: float = 1.96) -> float:
    if n == 0:
        return 1.0
    p = k / n
    denom = 1 + z * z / n
    centre = p + z * z / (2 * n)
    spread = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))
    return (centre + spread) / denom


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


def cmd_summary(a) -> int:
    """One line per cell (tag): loose count, post-settle p-p with 95% CI, mean |ss|, per-pod loose."""
    print(f"{'cell':16} {'n':>5} {'loose':>5}  {'pp mean [95% CI]':>20} {'|ss|':>5} {'rise':>6} {'over%':>6} "
          f"{'s1.0 (n)':>9} {'flips':>5} {'V':>6}")
    for tag in a.tags:
        path = os.path.join(OUT_DIR, f"podstep_{tag}.jsonl")
        with open(path, encoding="utf-8") as f:
            rows = [json.loads(line) for line in f if line.strip()]
        ok = [r["score"] for r in rows if r["score"].get("ok")]
        if not ok:
            print(f"{tag:16} no scored trials")
            continue
        pp = [s["post_settle_pp_deg"] for s in ok]
        ss = [abs(s["steady_state_deg"]) for s in ok]
        s10 = [s["settle_1_0"] for s in ok if not math.isnan(s["settle_1_0"])]
        lo, hi = _ci(pp)
        volts = statistics.mean(r["voltage_mean"] for r in rows)
        print(f"{tag:16} {len(ok):>2}/{len(rows):<2} {sum(p >= 2.0 for p in pp):>5}  {statistics.mean(pp):5.2f} "
              f"[{lo:5.2f},{hi:5.2f}] {statistics.mean(ss):5.2f} "
              f"{statistics.mean(s['rise_10_90_s'] for s in ok):6.3f} "
              f"{statistics.mean(s['overshoot_pct'] for s in ok):6.1f} "
              f"{(statistics.mean(s10) if s10 else float('nan')):5.2f} ({len(s10)}) "
              f"{sum(s['flip_events'] for s in ok):>5} {volts:6.2f}")
        per = []
        for p in sorted({r["pod"] for r in rows}):
            mine = [r["score"] for r in rows if r["pod"] == p and r["score"].get("ok")]
            if mine:
                per.append(f"p{p}: loose {sum(s['post_settle_pp_deg'] >= 2.0 for s in mine)}/{len(mine)} "
                           f"|ss| {statistics.mean(abs(s['steady_state_deg']) for s in mine):.2f}")
        print("    " + "  ".join(per))
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
    p = sub.add_parser("ab", help="randomised interleaved A/B, gains re-applied every trial")
    p.add_argument("tag")
    p.add_argument("--a", required=True)
    p.add_argument("--b", required=True)
    p.add_argument("--pods", default="0,1,2,3")
    p.add_argument("--n", type=int, default=3, help="trials per arm per pod per direction")
    p.add_argument("--step", type=float, default=90.0)
    p.add_argument("--hold", type=float, default=4.0)
    p.add_argument("--seed", type=int, default=7)
    p.set_defaults(fn=cmd_ab)
    p = sub.add_parser("abreport")
    p.add_argument("tag")
    p.set_defaults(fn=cmd_abreport)
    p = sub.add_parser("summary", help="compare cells by tag")
    p.add_argument("tags", nargs="+")
    p.set_defaults(fn=cmd_summary)
    a = ap.parse_args()
    return a.fn(a)


if __name__ == "__main__":
    sys.exit(main())
