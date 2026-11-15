#!/usr/bin/env python3
"""Finds jolts in Bring-Up recordings and graphs motor voltage around them (joltcapture.py's runs).

DIAGNOSTIC. A jolt = a spike in body acceleration (from the Pinpoint velocity, bvx/bvy). For each one it
checks the 0.25 s before it for the usual suspects and plots the whole run plus zooms on the worst jolts:
  - X-LOCK while moving: drive command under the swerve's epsilon (|f,s| < 0.05 and |t| < 0.015) for 0.35 s
    (X_LOCK_ENGAGE_DELAY_S) with the robot still rolling - the pods snap to an X across the motion
  - REVERSAL: a pod's drive power changes sign by more than 0.3 in one loop
  - TARGET JUMP: a pod's commanded wheel angle (ctgt) moves more than 90 deg in one loop
  - VECTOR SWING: the commanded translation direction rotates more than 90 deg in one loop
  - SLOW LOOP: dt over 50 ms
Motor voltage = commanded drive power x battery volts (what the hub's PWM applies).

  joltplot.py runs/<file>.csv [...]        # default: every runs/*jolt_* csv
"""
from __future__ import annotations

import csv
import glob
import gzip
import math
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ACCEL_JOLT = 150.0     # in/s^2 - a full-power start is ~100; a jolt is a step well beyond it
EPS_T, EPS_R, XLOCK_S = 0.05, 0.015, 0.35
POD_NAMES = ["p0 RB", "p1 RF", "p2 LF", "p3 LB"]


def load(path: str) -> dict[str, np.ndarray]:
    opener = gzip.open if path.endswith(".gz") else open
    with opener(path, "rt", encoding="utf-8") as f:
        rows = [r for r in csv.reader(line for line in f if not line.startswith("#"))]
    head, body = rows[0], rows[1:]
    cols = {h: np.array([float(r[i]) if i < len(r) and r[i] != "" else np.nan for r in body]) for i, h in
            enumerate(head)}
    return cols


def deriv(t: np.ndarray, x: np.ndarray, k: int = 2) -> np.ndarray:
    out = np.full_like(x, np.nan)
    for i in range(k, len(x) - k):
        dt = t[i + k] - t[i - k]
        if dt > 0:
            out[i] = (x[i + k] - x[i - k]) / dt
    return out


def analyse(c: dict[str, np.ndarray]) -> dict:
    t = c["t"]
    vx, vy = c["bvx"], c["bvy"]
    ax, ay = deriv(t, vx), deriv(t, vy)
    acc = np.hypot(ax, ay)
    speed = np.hypot(vx, vy)
    f, s, r = c["dcf"], c["dcs"], c["dct"]
    trans = np.hypot(f, s)
    quiet = (trans < EPS_T) & (np.abs(r) < EPS_R)
    # X-lock engages after XLOCK_S of quiet command; flag the samples where it is engaged while rolling.
    xlock = np.zeros(len(t), bool)
    start = None
    for i in range(len(t)):
        if quiet[i]:
            start = t[i] if start is None else start
            xlock[i] = t[i] - start >= XLOCK_S
        else:
            start = None
    xlock_moving = xlock & (speed > 3)
    dpwr = np.vstack([c[f"p{i}_dpwr"] for i in range(4)])
    ctgt = np.vstack([c[f"p{i}_ctgt"] for i in range(4)])
    rev = np.zeros(len(t), bool)
    jump = np.zeros(len(t), bool)
    for i in range(4):
        d = dpwr[i]
        rev[1:] |= (np.sign(d[1:]) * np.sign(d[:-1]) < 0) & (np.abs(d[1:] - d[:-1]) > 0.3)
        dc = np.abs(((ctgt[i][1:] - ctgt[i][:-1] + 180) % 360) - 180)
        jump[1:] |= np.nan_to_num(dc) > 90
    ang = np.degrees(np.arctan2(s, f))
    swing = np.zeros(len(t), bool)
    da = np.abs(((ang[1:] - ang[:-1] + 180) % 360) - 180)
    swing[1:] = (da > 90) & (trans[1:] > EPS_T) & (trans[:-1] > EPS_T)
    slow = c["dt"] > 0.05
    # Jolts: local maxima of |accel| above the threshold, at least 0.3 s apart.
    jolts = []
    order = np.argsort(-np.nan_to_num(acc))
    for i in order:
        if not acc[i] > ACCEL_JOLT:
            break
        if all(abs(t[i] - t[j]) > 0.3 for j in jolts):
            jolts.append(i)
    causes = []
    for i in jolts:
        w = (t >= t[i] - 0.25) & (t <= t[i])
        tags = [name for name, m in (("X-LOCK while moving", xlock_moving), ("REVERSAL", rev),
                                     ("TARGET JUMP", jump), ("VECTOR SWING", swing), ("SLOW LOOP", slow)) if m[w].any()]
        causes.append(tags)
    return dict(t=t, acc=acc, speed=speed, xlock_moving=xlock_moving, rev=rev, jump=jump, swing=swing, slow=slow,
                jolts=jolts, causes=causes, dpwr=dpwr, ctgt=ctgt, trans=trans)


def panels(axs, c, a, lo, hi, marks=True):
    t = a["t"]
    w = (t >= lo) & (t <= hi)
    volts = c["volts"]
    for i in range(4):
        axs[0].plot(t[w], (a["dpwr"][i] * volts)[w], lw=1, label=POD_NAMES[i])
    axs[0].set_ylabel("motor V")
    axs[0].axhline(0, color="k", lw=0.5)
    axs[1].plot(t[w], volts[w], color="tab:red", lw=1)
    axs[1].set_ylabel("battery V")
    axs[2].plot(t[w], c["dcf"][w], lw=1, label="forward")
    axs[2].plot(t[w], c["dcs"][w], lw=1, label="strafe")
    axs[2].plot(t[w], c["dct"][w], lw=1, label="turn")
    axs[2].set_ylabel("command")
    axs[2].axhline(0, color="k", lw=0.5)
    axs[3].plot(t[w], a["speed"][w], color="tab:blue", lw=1, label="speed in/s")
    axs[3].set_ylabel("speed in/s")
    ax3b = axs[3].twinx()
    ax3b.plot(t[w], a["acc"][w], color="tab:orange", lw=1, label="|accel|")
    ax3b.axhline(ACCEL_JOLT, color="tab:orange", lw=0.5, ls="--")
    ax3b.set_ylabel("|accel| in/s^2", color="tab:orange")
    for i in range(4):
        axs[4].plot(t[w], a["ctgt"][i][w], lw=1, marker=".", ms=2, ls="none", label=POD_NAMES[i])
    axs[4].set_ylabel("pod target deg")
    axs[5].plot(t[w], c["dt"][w] * 1000, color="gray", lw=1)
    axs[5].set_ylabel("dt ms")
    axs[5].set_xlabel("t (s)")
    for ax in axs:
        ax.set_xlim(lo, hi)
        xm = a["xlock_moving"]
        # shade X-lock-while-moving spans
        inside = False
        for i in np.where(w)[0]:
            if xm[i] and not inside:
                s0, inside = t[i], True
            if inside and (not xm[i] or i == np.where(w)[0][-1]):
                ax.axvspan(s0, t[i], color="red", alpha=0.12, lw=0)
                inside = False
        if marks:
            for j in a["jolts"]:
                if lo <= t[j] <= hi:
                    ax.axvline(t[j], color="red", lw=0.8, ls=":")
    axs[0].legend(fontsize=6, loc="upper right", ncol=4)
    axs[2].legend(fontsize=6, loc="upper right", ncol=3)


def plot(path: str) -> str:
    c = load(path)
    a = analyse(c)
    base = path[:-len(".csv.gz")] if path.endswith(".csv.gz") else os.path.splitext(path)[0]
    fig, axs = plt.subplots(6, 1, figsize=(14, 13), sharex=True)
    panels(axs, c, a, a["t"][0], a["t"][-1])
    fig.suptitle(f"{os.path.basename(path)} - red dotted: jolts (|accel| > {ACCEL_JOLT:.0f}); red shade: X-lock while moving")
    fig.tight_layout()
    fig.savefig(base + "_timeline.png", dpi=110)
    plt.close(fig)
    top = a["jolts"][:3]
    if top:
        fig, axs = plt.subplots(6, len(top), figsize=(6 * len(top), 13), squeeze=False)
        for k, j in enumerate(top):
            col = [axs[r][k] for r in range(6)]
            panels(col, c, a, a["t"][j] - 0.7, a["t"][j] + 0.5)
            col[0].set_title(f"jolt t={a['t'][j]:.2f}s |a|={a['acc'][j]:.0f}: {', '.join(a['causes'][k]) or 'no tagged cause'}",
                             fontsize=8)
        fig.tight_layout()
        fig.savefig(base + "_jolts.png", dpi=110)
        plt.close(fig)
    dts = c["dt"][c["dt"] > 0]
    print(f"{os.path.basename(path)}: n={len(c['t'])}, loop_hz_true={1 / np.mean(dts):.1f}, dt p90 "
          f"{1000 * np.percentile(dts, 90):.1f} ms, volts {np.nanmin(c['volts']):.2f}-{np.nanmax(c['volts']):.2f}, "
          f"X-lock-while-moving {a['xlock_moving'].sum()} samples, reversals {a['rev'].sum()}, target jumps "
          f"{a['jump'].sum()}, vector swings {a['swing'].sum()}, slow loops {a['slow'].sum()}")
    for k, j in enumerate(a["jolts"][:10]):
        print(f"   jolt t={a['t'][j]:6.2f}s |a|={a['acc'][j]:5.0f} in/s^2 speed {a['speed'][j]:4.1f} in/s  "
              f"<- {', '.join(a['causes'][k]) or 'no tagged cause'}")
    return base


def main() -> int:
    files = sys.argv[1:] or sorted(glob.glob(os.path.join(HERE, "runs", "*jolt_*.csv*")))
    for p in files:
        plot(p)
    return 0


if __name__ == "__main__":
    sys.exit(main())
