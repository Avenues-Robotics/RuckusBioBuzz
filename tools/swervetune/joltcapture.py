#!/usr/bin/env python3
"""Records the automatic motions that jolt, one Bring-Up recording per scenario, for joltplot.py.

DIAGNOSTIC. Needs the PodRecorder drive block (p*_dpwr, dcf/dcs/dct, bvx/bvy/bw - 2026-11-15). Each scenario
prints when each motion starts, relative to the recording, so the operator can say which ones jolted.

  joltcapture.py --confirmed-floor hold brake decel trans    # any subset, in that order (default: these four)
Operator 2026-11-15: jolts happen only while translating, never during heading moves - spin is opt-in.
Scenarios:
  hold   four follower holds (pedroStart / pedroHold / pedroStop): end, middle, other axis end, middle
  brake  Foresight BRAKE stage, forward, max power 0.6 (5 trials)
  spin   Foresight HEADING_BRAKE, 4 trials from full power
  decel  Foresight DECEL forward, twice
  trans  Foresight TRANSLATIONAL forward and strafe at 0.35 power (straight runs, then coast)
"""
from __future__ import annotations

import os
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from robot import EXIT_OK, script_gates, send, state  # noqa: E402
import foresightbox as fb  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))


def stamp(t0: float, what: str) -> None:
    print(f"  t={time.time() - t0:5.1f}s  {what}", flush=True)


def recorded(label: str, body) -> None:
    send("recStart", label=label)
    time.sleep(0.3)
    t0 = time.time()
    try:
        body(t0)
    finally:
        time.sleep(0.8)
        send("recStop")
        time.sleep(0.5)
        subprocess.run([sys.executable, os.path.join(HERE, "robot.py"), "pull", label], check=False)


def hold(t0: float) -> None:
    for axis in ("forward", None, "strafe", None):
        stamp(t0, f"follower hold to the {'middle' if axis is None else axis + ' end'}")
        fb.recenter(axis=axis, tol_in=1.0, tol_deg=2.0, timeout_s=5)


def brake(t0: float) -> None:
    fb.recenter(axis="forward")
    stamp(t0, "BRAKE forward, 5 trials (0.6 -> 0.3)")
    fb.run_stage("brake", "forward", 0.6, None, "shipped")


def spin(t0: float) -> None:
    fb.recenter(axis=None)
    stamp(t0, "HEADING_BRAKE, 4 trials (1.0 -> 0.2)")
    fb.run_stage("headingBrake", None, 1.0, 4, "shipped")


def decel(t0: float) -> None:
    for k in range(2):
        fb.recenter(axis="forward")
        stamp(t0, f"DECEL forward #{k + 1}")
        fb.run_stage("decel", "forward", None, None, "shipped")


def trans(t0: float) -> None:
    for axis in ("forward", "strafe"):
        fb.recenter(axis=axis)
        stamp(t0, f"TRANSLATIONAL {axis} at 0.35")
        fb.run_stage("translational", axis, 0.35, None, "shipped")


SCENARIOS = {"hold": hold, "brake": brake, "decel": decel, "trans": trans, "spin": spin}
DEFAULT = ["hold", "brake", "decel", "trans"]


def main() -> int:
    argv, _ = script_gates(sys.argv[1:], box_always=True)
    names = [a for a in argv if a in SCENARIOS] or DEFAULT
    tag = time.strftime("%H%M")
    for name in names:
        print(f"== {name}", flush=True)
        recorded(f"jolt_{name}_{tag}", SCENARIOS[name])
    s = state()
    print(f"done: pose ({s['pose']['x']:.1f}, {s['pose']['y']:.1f}), {s['voltage']} V, mode {s['mode']}")
    return EXIT_OK


if __name__ == "__main__":
    sys.exit(main())
