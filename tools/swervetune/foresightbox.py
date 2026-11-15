#!/usr/bin/env python3
"""Foresight Tuner stages inside the fence (Swerve Bring-Up FORESIGHT mode, ForesightBoxTuner.java).

DIAGNOSTIC. Identification only - nothing here validates or ships a gain. The output file imports with
`robot.py foresight runs/foresight_box_results.txt --partial` (dry run first), after which TUNING_TASK.md
stage 6 (pedrocheck through the real follower) still has to pass before FORESIGHT_MEASURED is touched.
Max velocity is not measured here (needs the long strip); FORESIGHT_MEASURED must not be set on these alone.

  foresightbox.py --confirmed-floor run headingId
  foresightbox.py --confirmed-floor run decel --axis strafe [--p1 25] [--p2 1.0]
  foresightbox.py --confirmed-floor all            # headingId, headingBrake, decel f/s, translational f/s, brake f/s
  foresightbox.py show                             # merged results so far

Stage parameters (omit for the tuner's defaults; the device scales DECEL/TRANSLATIONAL down to the room):
  headingId      p1 power 0.4, p2 runtime 1.2 s
  headingBrake   p1 max power 1.0, p2 trials 12
  decel          p1 target in/s 30, p2 power 1.0
  translational  p1 power 0.4, p2 runtime 1.2 s
  brake          p1 max power 0.7, p2 trials 5
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from robot import EXIT_OK, EXIT_REFUSED, script_gates, send, state  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
RUNS = os.path.join(HERE, "runs")
LOG = os.path.join(RUNS, "foresight_box.jsonl")
LATEST = os.path.join(RUNS, "foresight_box_latest.json")
RESULTS_TXT = os.path.join(RUNS, "foresight_box_results.txt")
TIMEOUT_S = {"headingId": 15, "headingBrake": 90, "decel": 20, "translational": 15, "brake": 60}
SEQUENCE = [("headingId", None), ("headingBrake", None), ("decel", "forward"), ("decel", "strafe"),
            ("translational", "forward"), ("translational", "strafe"), ("brake", "forward"), ("brake", "strafe")]


def spin_refusal(s: dict) -> str | None:
    """A spin sweeps the robot's circumcircle. With a footprint-aware box (border / field) the centre must
    sit at least that radius + 1 in inside every edge; with a marked (centre) box, 2 in inside."""
    box, pose = s.get("box", {}), s.get("pose", {})
    x, y = pose.get("x"), pose.get("y")
    if x is None or y is None:
        return "no pose"
    edges = min(x - box["minX"], box["maxX"] - x, y - box["minY"], box["maxY"] - y)
    if box.get("kind") == "marked":
        need = 2.0
    else:
        need = math.hypot(box.get("robotL", 18), box.get("robotW", 18)) / 2 + 1.0
    if edges < need:
        return (f"the robot centre is {edges:.1f} in from the nearest box edge; a spin needs {need:.1f} in "
                f"({box.get('kind')} box). Centre the robot in the box.")
    return None


def recenter(heading_deg: float = 90.0, axis: str | None = None, tol_in: float = 2.0, tol_deg: float = 3.0,
             timeout_s: float = 6.0) -> None:
    """Puts the robot back square to the box with the follower's hold (pedroStart / pedroHold / pedroStop - the
    OpMode checks the hold point against the fence minus 6 in). Heading stages (axis None) go to the middle;
    linear stages go to the near END of their axis, so the stage gets the whole box (about 23 in of centre
    travel on the 2x2 instead of 14.5 from the middle). Spins drift the robot and every linear stage ends near
    a wall, so this runs before each stage."""
    s = state()
    b, p, h = s["box"], s["pose"], s["heading"]["deg"]
    cx, cy = (b["minX"] + b["maxX"]) / 2, (b["minY"] + b["maxY"]) / 2
    if axis:
        # Field direction of the stage's axis at heading_deg (strafe = robot left), then the end nearest the
        # robot, 0.5 in inside what the OpMode accepts as a hold point (fence reach + 6 in margin).
        hr = math.radians(heading_deg)
        ux, uy = (math.cos(hr), math.sin(hr)) if axis == "forward" else (-math.sin(hr), math.cos(hr))
        # The OpMode accepts a hold point only 6 in + the robot's half-diagonal inside a border/field fence
        # (pedroPointOk: a path may turn the robot) - so this is as near the end as the follower may go.
        reach = math.hypot(b.get("robotL", 18), b.get("robotW", 18)) / 2 if b.get("kind") != "marked" else 0
        inset = reach + 6.0 + 0.5
        if abs(ux) > abs(uy):
            cx = b["minX"] + inset if p["x"] <= cx else b["maxX"] - inset
        else:
            cy = b["minY"] + inset if p["y"] <= cy else b["maxY"] - inset
    dh = ((heading_deg - h + 180) % 360) - 180
    if math.hypot(cx - p["x"], cy - p["y"]) < tol_in and abs(dh) < tol_deg:
        return
    print(f"    recentre: ({p['x']:.1f}, {p['y']:.1f}, {h:.1f} deg) -> ({cx:.1f}, {cy:.1f}, {heading_deg:.0f} deg)",
          flush=True)
    send("pedroStart", pods="shipped")
    time.sleep(0.6)
    send("pedroHold", dx=round(cx - p["x"], 2), dy=round(cy - p["y"], 2), dh=round(dh, 2))
    deadline = time.time() + timeout_s
    try:
        while time.time() < deadline:
            time.sleep(0.3)
            s = state()
            p, h = s["pose"], s["heading"]["deg"]
            if (math.hypot(cx - p["x"], cy - p["y"]) < tol_in
                    and abs(((heading_deg - h + 180) % 360) - 180) < tol_deg):
                time.sleep(0.5)
                break
    finally:
        send("pedroStop")
        time.sleep(0.4)
    s = state()
    print(f"    recentred at ({s['pose']['x']:.1f}, {s['pose']['y']:.1f}, {s['heading']['deg']:.1f} deg)", flush=True)


def run_stage(stage: str, axis: str | None, p1, p2, pods: str, center: bool = False) -> dict:
    if center:
        recenter(axis=axis)
    s0 = state()
    if s0.get("mode") not in ("IDLE",):
        raise SystemExit(f"REFUSED: Bring-Up is in mode {s0.get('mode')}, not IDLE.")
    if stage in ("headingId", "headingBrake"):
        why = spin_refusal(s0)
        if why:
            raise SystemExit(f"REFUSED {stage}: {why}")
    params = {"stage": stage, "pods": pods}
    if axis:
        params["axis"] = axis
    if p1 is not None:
        params["p1"] = p1
    if p2 is not None:
        params["p2"] = p2
    volts0 = s0.get("voltage")
    print(f"--> {stage}{' ' + axis if axis else ''} {params}  ({volts0} V)", flush=True)
    send("foresight", **params)
    deadline = time.time() + TIMEOUT_S[stage]
    s = s0
    started = False
    last_msg = ""
    try:
        while time.time() < deadline:
            time.sleep(0.25)
            s = state()
            f = s.get("foresight", {})
            msg = s.get("message", "")
            if msg != last_msg:
                print(f"    {msg}", flush=True)
                last_msg = msg
            if f.get("running") or s.get("mode") == "FORESIGHT":
                started = True
                continue
            if "REFUSED" in msg or started or "FAILED" in msg or "ABORTED" in msg:
                break
        else:
            print("    TIMEOUT - stopping", flush=True)
            send("foresightStop")
    except KeyboardInterrupt:
        send("foresightStop")
        raise
    f = s.get("foresight", {})
    rec = {
        "time": time.strftime("%Y-%m-%dT%H:%M:%S"), "stage": stage, "axis": axis, "params": params,
        "surface": "floor", "volts_before": volts0, "volts_after": s.get("voltage"),
        "message": s.get("message"), "phase": f.get("phase"), "results": f.get("results", {}),
        "extras": f.get("extras", {}), "points": f.get("points", []), "t": f.get("t", []), "v": f.get("v", []),
        "pods": params["pods"],
    }
    msg_final = s.get("message", "") or ""
    ok = (f.get("phase") == "done" and f.get("stage", "").lower().replace("_", "") == stage.lower()
          and "REFUSED" not in msg_final)
    rec["ok"] = ok
    os.makedirs(RUNS, exist_ok=True)
    with open(LOG, "a", encoding="utf-8") as fh:
        fh.write(json.dumps(rec) + "\n")
    print(f"    {'DONE' if ok else 'NOT DONE (' + str(f.get('phase')) + ')'}  extras {fmt(rec['extras'])}")
    if rec["points"]:
        print("    points (v, distance): " + ", ".join(f"({a:.2f}, {b:.2f})" for a, b in rec["points"]))
    if ok:
        merge(rec)
    return rec


def merge(rec: dict) -> None:
    latest = {}
    if os.path.exists(LATEST):
        with open(LATEST, encoding="utf-8") as fh:
            latest = json.load(fh)
    # Only the results THIS stage produced (the device's map is cumulative for the OpMode's life).
    produced = stage_results(rec["stage"], rec["axis"])
    for name, value in rec["results"].items():
        if name in produced and value is not None:
            latest[name] = {"value": value, "time": rec["time"], "volts": rec["volts_before"],
                            "extras": rec["extras"], "pods": rec["pods"]}
    with open(LATEST, "w", encoding="utf-8") as fh:
        json.dump(latest, fh, indent=2)
    write_results(latest)


def stage_results(stage: str, axis: str | None) -> set[str]:
    a = axis or "forward"
    return {
        "headingId": {"heading kP"},
        "headingBrake": {"headingBrakingLinearCoefficient", "headingBrakingQuadraticCoefficient"},
        "decel": {"naturalForwardDeceleration" if a == "forward" else "naturalStrafeDeceleration"},
        "translational": ({"forwardTranslational Primary kP", "forwardTranslational Secondary kP", "coast kV",
                           "brake kV"} if a == "forward" else
                          {"strafeTranslational Primary kP", "strafeTranslational Secondary kP"}),
        "brake": {f"{a}BrakingLinearCoefficient", f"{a}BrakingQuadraticCoefficient"},
    }[stage]


def write_results(latest: dict) -> None:
    lines = ["# Foresight box-tuner results (foresightbox.py). Identification only - NOT validated.",
             "# Max velocity NOT measured (needs the long strip). Import: robot.py foresight <this file> --partial"]
    for name, e in latest.items():
        lines.append(f"{name}: {e['value']}")
    with open(RESULTS_TXT, "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines) + "\n")


def fmt(d: dict) -> str:
    return "{" + ", ".join(f"{k}={v:.4g}" if isinstance(v, (int, float)) else f"{k}={v}"
                           for k, v in d.items()) + "}"


def show() -> int:
    if not os.path.exists(LATEST):
        print("no results yet")
        return EXIT_OK
    with open(LATEST, encoding="utf-8") as fh:
        latest = json.load(fh)
    for name, e in latest.items():
        print(f"{name:42s} {e['value']:.6g}   ({e['time']}, {e['volts']} V, pods={e['pods']})")
    return EXIT_OK


def main() -> int:
    if len(sys.argv) > 1 and sys.argv[1] == "show":
        return show()
    if "--on-blocks" in sys.argv:
        print("REFUSED: the Foresight stages measure the robot moving on the floor - not on blocks.")
        return EXIT_REFUSED
    argv, _ = script_gates(sys.argv[1:], box_always=True)
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run")
    r.add_argument("stage", choices=list(TIMEOUT_S))
    r.add_argument("--axis", choices=["forward", "strafe"])
    r.add_argument("--p1", type=float)
    r.add_argument("--p2", type=float)
    r.add_argument("--pods", default="shipped", choices=["shipped", "live"])
    r.add_argument("--recenter", action="store_true", help="hold back to the box centre, facing 90 deg, first")
    a_all = sub.add_parser("all")
    a_all.add_argument("--pods", default="shipped", choices=["shipped", "live"])
    a = ap.parse_args(argv)
    if a.cmd == "run":
        if a.stage in ("decel", "translational", "brake") and not a.axis:
            ap.error(f"{a.stage} needs --axis forward|strafe")
        run_stage(a.stage, a.axis, a.p1, a.p2, a.pods, center=a.recenter)
    else:
        for stage, axis in SEQUENCE:
            rec = run_stage(stage, axis, None, None, a.pods, center=True)
            if rec["phase"] != "done":
                print(f"stopping the sequence: {stage} did not finish ({rec['message']})")
                return EXIT_REFUSED
            time.sleep(1.0)
        show()
    return EXIT_OK


if __name__ == "__main__":
    sys.exit(main())
