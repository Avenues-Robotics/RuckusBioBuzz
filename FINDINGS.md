# Swerve steering quality — findings

Session of 2026-08-16. Every number here is measured unless it says "estimated" or "predicted".
Loop rates are `1 / mean(dt)`. `mean(loopHz)` is never quoted as a rate.

Data sources used: the 31 archived runs in `tools/swervetune/runs/`, and live `/state` reads
from the robot (Swerve Bring-Up running, DRIVE mode, at rest, 12.76 V).

Tools written this session (both diagnostic):
- `tools/swervetune/steerqual.py` — all eleven criteria and the six verification graphs from one
  recorder CSV.
- `tools/swervetune/jumpcause.py` — replays the demand chain host-side and attributes every
  azimuth-setpoint discontinuity to a specific branch of the mixer.

---

## Task 0 — Loop hygiene and the publish() question

### 0.1 The remaining 34–37 ms of `publish()` is CPU, not the Lynx bus — REFUTES the standing hypothesis

Measured live, **robot at rest in DRIVE, zero drive command, pods X-locked, no servo or motor
writes happening**, 60 consecutive `/state` samples over 25 s at 12.76 V:

| field | mean | min | max |
|---|---|---|---|
| `msPublish` | **36.67 ms** | 32.31 | 39.58 |
| `msEncoders` | 2.00 | 1.94 | 2.20 |
| `msHeading` (incl. idle-path current + battery) | 5.42 | 5.00 | 6.00 |
| `msTelemetry` | 0.65 | 0.51 | 0.82 |
| `msMode` | 0.19 | 0.17 | 0.26 |

The task's standing hypothesis was that publish blocks on the Lynx bus behind four servo writes
plus four motor writes. **It cannot be:** there are no actuator writes at all in this sample and
publish still costs 36.7 ms. The IDLE/DRIVE asymmetry recorded on 2026-08-13 is explained by
what `appendPod` used to do — eight live `getPower()` transactions per publish — and that was
already fixed. What remains is pure computation.

The candidate that survives: **`fmt()` is `String.format(Locale.US, "%.4f", v)`**, and one
publish makes roughly a thousand calls to it — about **780 of them from the 260-sample trace
alone** (`appendTraceSeries` emits `t`, `tgt` and `act`), plus ~160 from the four pods and ~60
from the header. At 35 µs per call that is 35 ms, which is the whole measurement.

This also explains the loop-rate spread across the archive: publish runs on a 50 ms timer, so
once the loop period exceeds 50 ms **every** loop pays the publish cost and the slow mode
sustains itself. Human-driven runs (heading hold on → trace full → ~780 extra formats) sit at
22–24 Hz; a scripted box drive with the same code sits at 51 Hz.

| run | loop_hz_true | dt mean | dt p50 | dt p90 | dt p99 |
|---|---|---|---|---|---|
| `mydrive-001` (human, dashboard) | 24.1 | 41.6 | 48.4 | 64.1 | 77.1 |
| `drift-look-001` (human, dashboard) | 22.5 | 44.5 | 51.1 | 66.1 | 79.4 |
| `boxdrive-ebburst-pulsed` (scripted) | 51.1 | 19.6 | 12.1 | 52.9 | 68.6 |
| `slowfix` (scripted crawl) | 34.6 | 28.9 | 32.9 | 51.4 | 58.7 |

**Not yet proven**, and the reason for the deployed instrumentation: per-section timers inside
`publish()` plus a formatter A/B (`setFastFmt`) that can be interleaved inside one session.

### 0.2 Bulk caching — audited, correct on both OpModes

| | mode | cleared |
|---|---|---|
| `DriveTeleOp` | `MANUAL` on every `LynxModule` | once per loop, top of `loop()` |
| `SwerveBringUp` | `MANUAL` on every `LynxModule` | once per loop, top of `serviceLoop()` |

`MANUAL` rather than `AUTO` is deliberate and correct here: DRIVE reads every pod encoder three
times per loop (`readEncoders`, `Swerve`'s avgScaling pass, `CoaxialPod.move`) and `AUTO`
re-fetches a whole bulk packet on a repeated same-channel read.

`cache = 0.01` in the gain set is **not** this. It is `CoaxialPod.servoCachingThreshold` — the
minimum change in servo power before the pod actually writes it. Two unrelated things with one
nickname; both are correct as configured.

Reads that bulk caching does **not** cover, and where they now live:
- `VoltageSensor.getVoltage()` — Lynx ADC transaction. `SwerveBringUp`: idle path only (5 Hz).
  `Swerve.getVoltageNormalized()`: **in the shipped mixer, but dead** — `useVoltageCompensation`
  is `false` (SwerveConstants default, not overridden), so it never runs. Verified, not assumed.
- `LynxModule.getCurrent()` — idle path only.
- Pinpoint (I2C) — gated on `headingInUse() || refreshIdleSensors`.

No blocking sleeps in either loop body. No `hardwareMap.get()` inside either loop.
`telemetry.update()` is called exactly once per telemetry tick in both.

### 0.3 `DriveTeleOp` loop rate — instrumented, not yet measured

Never measured honestly; the logged 75.8 Hz was `mean(1/dt)`. `DriveTeleOp` has no `publish()`
and no recorder, so nothing existed to measure it with. Added `TeleLoopProbe` (diagnostic class,
three lines in the OpMode): 1 ms-bin dt histogram → `1/mean(dt)`, p50/p90/p99/min/max on the
Driver Station, plus the same `PodRecorder` and a `/swerve/state` snapshot so
`drivecapture.py` and every host scorer work against the competition OpMode unchanged.

**Awaiting OPS REQUEST 1.**

---

## Task 1 — Heading lock

### 1.1 There is no closed heading loop in the competition path. At all.

Traced end to end:

```
DriveTeleOp.loop()
  → follower.setTeleOpDrive(forward, strafe, turn, true)
      → VectorCalculator.setTeleOpMovementVectors(...)
          teleopHeadingVector = Vector(magnitude = turn stick, theta = current heading)
  → follower.update()  [manualDrive branch]
      → CustomDrivetrain.runDrive(centripetal, heading, pathing, robotHeading, velocity)
          → calculateDrive(...)  returns  headingPower.dot(Vector(1, robotHeading)) = turn stick
      → Swerve.arcadeDrive(forward, strafe, rotation = turn stick)
```

The right stick is a **rate command straight through the mixer**. There is no setpoint, no
error, no latch, no wrap handling — because there is nothing to wrap. Releasing the stick
commands zero rotation and the robot keeps whatever heading momentum left it with.

The heading hold the prior sessions tuned lives **only in `SwerveBringUp.runDriveMode()`**
(diagnostic): setpoint integrated from the stick at 7 rad/s, ±60° lead cap, a three-phase
release machine (ACTIVE → STOPPING → RESTING), epsilon-bypass trim with 1.2°/0.5° hysteresis,
`headingKp` 1.20 / `headingKd` 0.080, its own copy of the gains. None of it is in `tele/`.

So Task 1's answer is: **build one, then characterise it.** Every heading number below is from
the diagnostic path and describes the bring-up tool, not the robot as it competes.

### 1.2 Heading hold as measured in the bring-up path (`mydrive-001`, 71.9 s, 12.37 V, tiles)

| metric | value |
|---|---|
| \|error\| while translating (>2 in/s) | mean **7.83°**, p95 60.0°, max 60.0°, n=1005 |
| \|error\| at rest | mean **3.65°**, p95 15.07°, max 44.04°, n=608 |
| error vs translation speed | r = **−0.168**, slope −0.126°/(in/s), intercept 9.75° |
| error vs \|turn command\| | r = **+0.903** |

Read the two correlations together: **heading error does not grow with translation speed — it
grows with rotation command.** Error is *worst at crawl* (0–5 in/s: 12.17° mean; >30 in/s:
4.07° mean), which is the opposite of a geometry or wheelbase error and consistent with a
setpoint that is being swept by the stick faster than the chassis can follow.

The p95 and max of exactly **60.00°** are not a coincidence and not a controller property: that
is `HEADING_MAX_LEAD = 60°`, the deliberate cap on how far the setpoint may lead the robot. The
error saturates against the cap during any sustained turn. **Any "steady-state heading error"
number taken while the turn stick is held is measuring the lead cap, not the loop.**

Not yet answered, and needing a purpose-built capture rather than a driving session: overshoot,
oscillation frequency/amplitude, and settling time after the stick returns to zero. The
`headingStep` routine already exists in the bring-up tool for exactly this.

### 1.3 Heading source

goBILDA Pinpoint, read fresh every loop that needs it (`readHeading` → `pinpoint.update()`),
gated so IDLE pays 5 Hz instead of loop rate. Confirmed: **no Control Hub IMU anywhere in the
control path**. Read cost is inside the 5.42 ms `msHeading` figure above, which also carries the
idle-path current and battery reads — the isolated 1.81 ms in CLAUDE.md still looks right.
Yaw drift over a run: not yet isolated (needs a stationary run with the recorder going).

---

## Task 2 — Quantised steering

### 2.1 It is real, it is in the demand, and the pods are innocent

`mydrive-001`, while driving, per pod:

| pod | jump p90 | jump max | jumps >15° | /s | flips/s | wheel path ÷ commanded | wheel rev/s | **commanded rev/s** |
|---|---|---|---|---|---|---|---|---|
| 0 RB | 17.8° | 158.5° | 139 | 2.71 | 0.66 | 1.01 | 4.17 | 4.17 |
| 1 RF | 14.4° | 179.3° | 106 | 2.07 | 0.70 | 0.89 | 4.56 | 3.78 |
| 2 LF | 17.8° | 149.9° | 133 | 2.59 | 0.47 | 1.01 | 4.91 | 3.90 |
| 3 LB | 15.6° | 168.3° | 112 | 2.18 | 0.64 | 1.01 | 4.44 | 3.47 |

**Path ratio ≈ 1.0 and commanded reversals ≈ wheel reversals.** The wheels are faithfully
tracking a demand that is itself reversing 3.5–4.2 times per second. This inverts the
2026-08-13 diagnosis, which had the demand reversing 0.41–0.53/s and blamed a hunting closed
loop. Chasing pod gains cannot fix a shaking setpoint.

### 2.2 Every discontinuity attributed to a line of Java

`jumpcause.py` replays `computeTargets` host-side from the logged applied command. Replay
residual against the logged targets: **mean 0.0001°, max 0.0006°** over 1731 samples — the model
is exact, so the attribution below is arithmetic, not inference.

601 jumps >15° while driving, n=1731 samples, 51.4 s of driving:

| cause | jumps | share | typical size |
|---|---|---|---|
| **rotation epsilon wall** (rotation term switched on/off wholesale) | 252 | 42% | — |
| **X-lock engage/release** (pods park on their own radii: the ±43.5°/±136.5° family) | 113 | 19% | — |
| **unattributed** — heading-PID rotation jitter at low translation | 126 | 21% | — |
| **per-axis deadband** (one translation axis exactly zeroed) | 63 | 10% | — |
| **flip** (deliberate 180°, hysteresis working) | 30 | 5% | 180° |
| **translation epsilon wall** | 17 | 3% | **mean 74–95°, max 179°** |
| **box fence** | **0** | 0% | — |

Ranked against the task's priors:

0. **Box clamp — not the cause here.** Zero clamped samples in this run (detector: after
   `applyBoxLimit` a clamped sample has an exactly-zero *field* axis with the other live).
   But the mechanism the task described is real and the code does clamp **per-axis**:
   `applyBoxLimit` zeroes `vx` or `vy` independently, with a speed-dependent margin
   (4 in + 0.3 s × closing speed) that shrinks as the robot slows — so engaging it both rotates
   the command and chatters. For an axis-aligned box, zeroing the outward normal *is* the right
   sliding behaviour; the defect is that it is a **step**, not a taper. Diagnostic code only.
1. **X-lock — confirmed, 19%.** Present in shipped constants (`ZeroPowerBehavior.X_LOCK`) and
   already carries a 0.35 s engage delay. `DriveTeleOp` inherits exactly the same behaviour.
   No magnitude hysteresis on the epsilon itself.
2. **Input path — the symptom is not a dashboard artifact, but the dashboard has its own copy of
   the bug.** `dashboard.html` `padAxis()` deadbands each axis at 0.06;
   `DriveTeleOp.applyDeadband` deadbands each axis at 0.05. Both **rotate the commanded
   direction** instead of shortening it: 35.5% of driving samples had exactly one translation
   axis zeroed, transitioning 1.44 times per second.
3. **Explicit snapping — none.** No rounding, cardinal-lock or `Math.round` anywhere in the
   demand chain. The 45° family comes from geometry: the chassis is 146.42 × 154.24 mm from
   centre, so the pure-rotation azimuths are ±43.5° / ±136.5°, and pure-translation cardinals
   are 0/90/180/270. Every wholesale switch between "translation" and "rotation" therefore
   lands on the 45° family — which is why the symptom looks like snapping to 45°.
4. **Encoder conversion — not implicated.** Quantisation would appear in `wheel`, and `wheel`
   tracks `tgt` at ratio ≈ 1.0.
5. **`atan2` with no magnitude gate — this is the 21% "unattributed" bucket.** At low
   translation the pod demand direction is `atan2` of (translation + rotation), so its
   sensitivity to the rotation term is ≈ trans/(trans²+rot²). Worked example from the log,
   consecutive loops: f/s/t `−0.077/+0.168/+0.204` → `−0.085/+0.187/+0.121`; translation
   essentially unchanged, rotation moves 0.08, demand jumps **67.4°**. The rotation term is the
   heading PID's output, so at crawl the heading controller's own jitter is what swings the
   pods.
6. **Flip rounding — working correctly.** 30 of 601 jumps, ±10° hysteresis band, compared
   against the continuous target. Flip rate 0.47–0.70/s against a 0.2/s criterion, i.e. still
   over, but these are legitimately-decided flips, not rounding artifacts.

### 2.3 A second per-axis clamp, this one only in the shipped path

`CustomDrivetrain.runDrive` (vendored Pedro, shipped) ends with:

```java
double clampedForward = clampReversePower(translationalVector.getXComponent(), robotVelocity.getXComponent());
double clampedStrafe  = clampReversePower(translationalVector.getYComponent(), robotVelocity.getYComponent());
```

`clampReversePower` caps a component at ±0.2 when it opposes the measured velocity on **that
axis alone**. Clamping one component and not the other rotates the commanded direction — the
exact mechanism the task predicted for the box fence, sitting in the shipped teleop path. It
engages on every deceleration and stick reversal, and releases the moment the velocity component
crosses zero, so it also chatters. **`SwerveBringUp` bypasses it entirely** (it calls
`Swerve.arcadeDrive` directly), which means every measurement in this repo's archive was taken
without it and `DriveTeleOp` may be worse than anything measured so far.

### 2.4 The recorder's `tgt` column had drifted from the mixer

`SwerveBringUp.computeTargets` is a host-side mirror of `Swerve.arcadeDrive`. The mixer's
rotation epsilon moved to 0.015 on 2026-08-15 (commit ced6c13); the mirror kept 0.05. For
`|rotation|` in [0.015, 0.05) the recorder logged a translation-only demand while the pods were
given translation plus rotation: **4.0% of samples in mydrive-001, worst disagreement 15.0°** —
in the one column that is supposed to discriminate "the demand is shaking" from "the response is
shaking". The mirror also lacked the X-lock 0.35 s engage delay.

Fixed both, and added `p{i}_ctgt` — the demand read back out of the pod
(`CoaxialPod.getLastTargetWheelRad()`), which cannot disagree with what the pod acted on. `tgt`
is kept so the archive stays comparable.

### 2.5 Predicted-but-unverified: teleop centripetal correction is dead code

`VectorCalculator.teleopUpdate()` does `velocities.add(v); velocities.remove(velocities.get(velocities.size()-1))`
— which removes the element just added (`Vector` does not override `equals`, and
`PoseTracker.getVelocity()` returns a fresh object each call). So `averageVelocity` stays zero
forever, `curvature` is always NaN, and `getCentripetalForceCorrection()` always returns an empty
vector in teleop. Stock Pedro bug, not a Ruckus patch.

This is load-bearing: if the list ever *did* update, teleop curvature would be finite and
`getCentripetalForceCorrection()` would dereference `currentPath`, which is **null** in
`DriveTeleOp` — an NPE on first movement. Flagged as a risk for the first `DriveTeleOp` run.
Predicted safe; **not verified on hardware.**

### 2.6 The fixes, and what simulation says they buy

Four distortions fixed. Each has its own measurable signature, so one capture can attribute
them separately even though they deploy together.

| # | Where | Side | Fix |
|---|---|---|---|
| 1 | `DriveTeleOp.applyDeadband` | **shipped** | deadband the translation **vector**, rescaled from the band edge |
| 2 | `CustomDrivetrain.clampReversePower` | **shipped** | project onto the direction of travel, scale the whole vector |
| 3 | `Swerve.arcadeDrive` epsilon walls | **shipped** | smoothstep taper across each band instead of a step |
| 4 | `Swerve.arcadeDrive` demand rate | **shipped** | slew limit at 214 °/s, the measured pod slew |
| 5 | `SwerveBringUp.applyBoxLimit` | diagnostic | taper the outward component over 6 in |
| 6 | `dashboard.html padAxis` | diagnostic | same vector deadband as fix 1 |

Simulated over `mydrive-001`'s recorded commands — 51.3 s of real driving replayed through both
mixers. **Physical** (mod-180, so deliberate flips are excluded) consecutive-loop demand change:

| configuration | jump p90 | jumps >15°/s | demand reversals/s |
|---|---|---|---|
| as shipped 2026-08-15 | 19.8° | 2.9 | 4.25 |
| epsilon taper only | 19.8° | 2.9 | 4.37 |
| demand slew 214 °/s only | 13.2° | 0.9 | 3.47 |
| taper + slew 214 °/s | **13.2°** | **0.9** | **3.47** |

Two results worth stating plainly because they are negative:

- **The taper alone does almost nothing.** Epsilon crossings are 3% of jumps and the input
  usually crosses the wall in a single loop, which no taper can smooth. It is kept because it
  removes a genuine discontinuity, not because it moved the number.
- **300 °/s, tried first, is worse than 214 at this loop rate.** It spreads one big jump into
  several 19° steps and the count of violations goes *up* (151 → 174 per pod). Rate limits
  interact with loop period; they are not free.

And the consequence that reorders the work: **criterion 1 is a loop-rate criterion.** A slew
limit is a rate, so a slow loop turns any rate into a big step. 53% of the jumps that survive
the 214 °/s limit in simulation land on loops longer than 70 ms (= 15° at 214 °/s). At 50 Hz
true with a 25 ms p99, the same limit permits 5.4° per loop. **Task 0 must land before Task 2's
criterion can be judged.**

---

## Task 3 — The path

Designed offline, validated offline, **not yet run**.

### 3.1 The envelope, computed before any control point was placed

Box read live from `/state`: x ∈ [−2.08, 48.71], y ∈ [−32.77, 12.87] → **50.79 × 45.65 in**.
Robot footprint is **assumed 18 × 18 in** (FTC legal maximum — the safe bound; nothing in the
codebase records the real footprint, and the dashboard's outline is drawn from pod extents plus
a fixed pixel margin, not a measurement). See ASSUMPTIONS.

| heading mode | clearance needed | centre envelope |
|---|---|---|
| tangential (robot sweeps every orientation) | half-diagonal 12.73 + 2.0 cross-track | **21.34 × 16.19 in** |
| constant (footprint never rotates) | half-width 9.00 + 2.0 cross-track | **28.79 × 23.65 in** |

The follower bench's own 6 in waypoint margin is satisfied with room to spare in both.

### 3.2 Geometry: C2 by construction, not by inspection

A **closed uniform cubic B-spline**, converted span by span to Bézier form
(`b0 = (d0+4d1+d2)/6`, `b1 = (2d1+d2)/3`, `b2 = (d1+2d2)/3`, `b3 = (d1+4d2+d3)/6`). C1 and C2 are
then properties of the representation rather than something hand-placed control points must be
checked for — including at the closing joint, which is where a hand-built loop usually fails.

Measured residuals at the **worst joint of all**, both variants: C0 = 0, C1 ≤ 1.5e-14 in,
C2 ≤ 1.4e-14 in. So: **C2 at every joint, including the wrap-around.** It is parametric C2 from
a uniform spline, and because the spans carry equal parameter speed it is also geometric G2 —
curvature is continuous, which the κ trace shows directly.

| variant | segments | length | min radius | max \|dκ/ds\| |
|---|---|---|---|---|
| tangential | 8 cubics, closed | 53.41 in | 5.24 in | 0.0171 in⁻² |
| constant | 8 cubics, closed | 74.46 in | 8.29 in | 0.0073 in⁻² |

### 3.3 Jerk: Pedro 2.1.2 has no jerk limit, so it is bounded by construction and reported

`PathConstraints` carries end-of-path tolerances and braking behaviour only — Pedro is a path
follower, not a trajectory follower, and there is no jerk parameter to set. Saying otherwise
would be inventing a feature. What was done instead:

- C2 geometry bounds `dκ/ds`, and at constant speed lateral jerk is `v³·|dκ/ds|`.
- Speed is chosen from a **pod-slew budget**: 25% of the measured 214 °/s median pod slew. A pod
  riding its slew limit is open-loop — the PID has already saturated — so tracking error there is
  set by the plant, not by gains.

| variant | speed | lap | max lateral accel | max lateral jerk | max pod azimuth rate |
|---|---|---|---|---|---|
| tangential | 13.00 in/s | 4.11 s | 32.2 in/s² (0.083 g) | 37.6 in/s³ | 53.5 °/s (chassis yaw 142 °/s) |
| constant | 7.74 in/s | 9.62 s | 7.2 in/s² (0.019 g) | 3.4 in/s³ | 53.5 °/s |

Worth noting because it is not obvious: on a constant-curvature arc a **tangential** heading
keeps each pod azimuth *fixed* (at `atan(κ·r_pod)`, ±57.9° at the tightest corner) because the
chassis yaws with the path — the pods only move as curvature changes. A **constant** heading
makes the pod azimuth rotate at exactly `v·κ` the whole way round. That is why the two variants
have such different speed allowances for the same slew budget.

### 3.4 Heading interpolation, per segment, with the reason

- **Tangential variant** — `HeadingInterpolator.tangent` on all 8 segments. A closed loop has no
  "approach", and tangential is what makes a traverse look driven rather than dragged. Linear
  interpolation across a curve fights the translation the whole way; it is not used anywhere here.
- **Constant variant** — `constant(0°)` on all 8 segments. This one exists as the *experiment*:
  with heading fixed, every pod azimuth change comes from the path alone, which isolates Task 2's
  question from the heading loop entirely.

### 3.5 Clamp clearance

The path never approaches the fence. Minimum wall clearance: **12.41 in** (constant variant),
**15.52 in** (tangential) — against a 9.00 in half-width and a 12.73 in half-diagonal. The
tangential variant clears the half-diagonal everywhere; the constant variant clears its own
relevant bound (half-width) with 3.4 in to spare and does not need the half-diagonal because it
never rotates. And the follower path does **not** run through `applyBoxLimit` at all — the bench
validates every control point before anything moves, and a Bézier stays inside its control
points' convex hull, so that check bounds the whole curve.

### 3.6 Visualizer validation

`.pp` loaded into the local Pedro visualizer (translated to the field centre; the visualizer
knows only the 144 × 144 field, so the *envelope* check stays with `pathdesign.py` against the
real box). Result: **8 segments, 74 in, zero wall or obstacle collisions** against the
visualizer's own footprint check. Length agrees with the computed 74.46 in.

One thing the visualizer caught: its time estimate was 19.1 s against the computed 9.6 s,
because the exported `.pp` did not mark the segments as chained, so it profiled eight separate
stop-at-end paths. The `pedroChain` command builds a single `PathChain` and is unaffected — but
the exporter should carry the chaining flag, and until it does the visualizer's time estimate
for these files reads roughly double.

Pedro's `BezierCurve` uses the standard characteristic matrix over the control points, so a
4-point curve is a plain cubic Bernstein curve — identical to the Python that designed it. Read
from `generateBezierCurve`, not run.

---

## Corrections to CLAUDE.md forced by this session

1. §5 gains are stale. Shipped now: `turnKPPerPod` 0.380, `turnKD` 0.022, `turnKSPerPod` **0.022**
   (not 0.035 — that is still the scalar `turnKS`, which the factories no longer use), pulsed
   final approach enabled, and `CoaxialPod.TURN_GAIN_SCHEDULING` on.
2. §5's "a mechanical lubrication pass is pending" is wrong — it happened before 2026-08-13 and
   the gains were re-fitted after it.
3. §7's "the remaining 37.4 ms of publish is unfound" — the Lynx-bus explanation is refuted
   above; the surviving candidate is `String.format`.
4. §4's recorder description (7 global columns) predates the driver-session columns; it is
   14 global + 7 per pod now.
5. §6's "wheel reversals 2.58–4.18/s vs 0.41–0.53/s demand" no longer describes the robot: the
   demand itself now reverses 3.5–4.2/s.
6. The line numbers quoted in the task prompt (`SwerveBringUp.java:2532`, `:2562`, `:2384`) are
   stale; the file has grown to 4060 lines. Current: `runDriveMode` 2234, `computeTargets` 3593,
   `handleGamepad` 3471.

---

## Criteria table, as it stands

"sim" = replayed host-side through the new mixer over recorded commands, not measured on the
robot. "—" = needs a robot run that has not happened.

| # | Criterion | Threshold | Measured baseline | After fixes | Status |
|---|---|---|---|---|---|
| 1 | Setpoint jump between loops | ≤ 15°, no 45° clustering | p90 14.4–17.8°, 2.07–2.71/s over 15°, **55–57% within 5° of a 45° multiple** (22% if uniform) | p90 13.2°, 0.9/s over (sim) | **blocked on Task 0** — a rate limit cannot beat a 77 ms p99 loop |
| 2 | Unintended 180° flips | < 0.2 /s | 0.47–0.70 /s | — | open |
| 3 | Azimuth steady-state error | < 1.0° | 2.65–3.01° | — | open, cause still unknown |
| 4 | 90° step settle to ±2° | < 350 ms | ~647 ms, 65% slew-limited | — | open |
| 5 | Wheel path ÷ commanded path | < 1.3× | **0.89–1.01×** | — | **already met** — and the old 1.7–3.0× figure was wrong |
| 6 | Wheel reversals/s | < 1.0 /s | 4.17–4.91 /s, against a **3.47–4.17 /s demand** | demand 3.47 (sim) | open — but the demand is the target, not the pod loop |
| 7 | Heading error translating | < 3.0° | 7.83° mean (bring-up hold; p95 saturates at the 60° lead cap) | — | open; competition path had no loop until today |
| 8 | Heading error at rest | < 1.0° | 3.65° mean (bring-up hold) | — | open |
| 9 | Cross-track | < 2.0 in | never measured | — | needs the path run |
| 10 | `DriveTeleOp` loop | ≥ 50 Hz true, p90 < 25 ms | **never measured** — instrumented today | — | needs a run |
| 11 | No visible heading oscillation | qualitative + p-p | — | — | needs a run |

## What is left, and what it needs

Everything below is blocked on robot time, in this order:

1. **`setFastFmt` A/B, robot stationary** (~3 min, no motion). Settles the `String.format`
   theory of the 36.7 ms publish and, with it, criterion 10's prerequisite.
2. **`DriveTeleOp` loop histogram** — start the OpMode, drive briefly, read the Driver Station.
   Criterion 10 directly, and it decides whether the loop work is done or has just begun.
3. **One capture per path, before and after**, through `drivecapture.py` against both OpModes.
   Criteria 1–6 and the graphs.
4. **`HeadingHold`'s first run.** It is new, in shipped code, and untested. Criteria 7, 8, 11.
5. **The path run**, via `pedroStart` + `pedroChain`. Criterion 9 and the last graph.

---

## Pedro 3.0.0 migration — declared under rule 9 before any data is taken

2026-09-10, branch `pedro-3.0.0-migration`. **Nothing here was measured on the robot**; the
evidence is host-side (surface N/A, volts N/A). Patch-level detail is in
`third_party/PedroPathing/RUCKUS_PATCHES.md`. Every number in the sections above was taken on
Pedro 2.1.2.

### What is provably unchanged: the pod loop and the mixer

`CoaxialPod` + `Swerve`, 2.1.2 fork (`a6edf00`) against the v3 port, driven side by side on the
host with a shared stepped clock: 1200 steps at 8/10/25 ms, pod demand equal to 2e-13°, servo
power to 6e-15, identical write counts. So criteria 1–6 measured on 2.1.2 still describe the
steering code. Exception, and it is real: at exactly zero input the pod demand direction is now
0° where 2.1.2 gave front pods 180° / back pods 0° (IEEE signed zeros). The pods are released
either way; only the remembered flip state differs, which can change the first flip decision
after a release when the new demand lands 80–100° from the pod.

### `DriveTeleOp` (shipped) — behaviour changes, all unmeasured

1. The 0.2 reverse-power clamp that the 2.x follower applied to every teleop command is gone from
   Pedro 3's `manual()`. It is re-applied in `DriveTeleOp`, same cap, same vector form, but
   against the **previous loop's** body velocity (2.x clamped after the pose refresh): ~10 ms older.
2. Saturated commands budget differently. 2.x's follower treated turn as a vector along the
   heading and scaled translation down before the mixer (forward 1.0 + turn 0.5 → 0.5 / 0.5); v3
   has only the mixer's per-pod normalisation (estimated ~0.68 / 0.34 for the same stick).
   Identical whenever no pod vector exceeds magnitude 1.
3. The Pinpoint is no longer re-origined at construction (2.x wrote 0,0,0), and `resetMode NONE`
   keeps v3 from recalibrating the IMU at every init. The pose-preservation code stays as a guard.
4. `DriveTeleOp`'s loop rate on v3 is **unknown**. New code sits on that path (`ConfigVar.get()`
   validation per read, the v3 `Follower`). Criterion 10's 99.3 Hz is a 2.1.2 number until
   re-measured.

### Bring-up tool (diagnostic) — changes that alter what it measures or accepts

1. `pedroStart activate=` — v3 has no per-term PIDF switches; anything but `all` is refused.
2. Bench `power` now sets Foresight `maxPathSpeed`, a fraction of max achievable **velocity**, not
   motor power. Restored on break-off and `stop()`.
3. `pedroPidf`: `tp/ti/td` replace both Foresight translational controllers with a PID (dropping
   any piecewise controller); `fzpa/lzpa` set natural deceleration to their absolute value;
   `tf`, `dp/di/dd/dt/df`, `cent` have no Foresight counterpart and are ignored, named in the reply.
4. `holdPoint` → v3 `hold(pose)`, which applies no hold scaling.
5. `/state` `pedro.terr` now comes from `Foresight.translationalError()` (distance to the closest
   pose) — the same concept, a different implementation. 0 when the algorithm is not Foresight.
6. `pedroBreak` = `stop()` + `drivetrain.stop()`, so the pods release immediately as in 2.x.
7. The heading-tune bench still reproduces the 2.x heading law (sign feed-forward); Foresight's
   heading path differs (feedback + static FF + a feed-forward split, P only), so bench heading
   gains no longer map one-to-one onto `foresightConfig.headingFeedback`.
8. Bench `SwerveConfig`: `maxPower(1.0)` and `velocity(73.9)` dropped (the mixer already normalises
   to 1). The recorder's `tgt` column and `computeTargets()` are untouched, and the pods still
   receive θ in [0, 2π) - existing traces stay comparable.

### Path following is new and on placeholder constants

Foresight replaces the 2.x PIDF follower. Of its required constants, only natural deceleration
(40 in/s², the 2026-08-14 coast-down) is measured; max velocity, brake coefficients, coast/brake
kV and heading brake are placeholders. `SwerveDrivetrainConstants.requireForesightMeasured()`
stops the autos and the tuner's path tests until the Foresight Tuner has been run. Criterion 9
needs that first. Note the tuner's default drive distances (48 in) exceed the 46 in side of the
practice area and ignore the bring-up safe-area box.

## 2026-10-01 — constants consolidation, BIOBUZZ field fence, Constants export (rule 9 declaration)

No data has been taken with any of this. Declared before first use.

**Shipped (`pedroPathing/`) — values unchanged, structure changed.** `SwerveDrivetrainConstants`,
`MecanumDrivetrainConstants` and the `config.jsonc` selector are folded into one swerve-only
`Constants.java`. Every tuned value moved into a TUNED VALUES block at the top as one declaration
each; the configs read those names. Checked identical by hand against the old file: heading P
1.20, translational 0.26/0/0.025, decelerations 40/40 (now two names; the old single constant fed
both), Pinpoint -5.376/-3.912 REVERSED/FORWARD, per-pod arrays, caching 0.01, pulse
6.0°/0.6°/0.035/20 ms/20°/s/0.10 s, encoder names se1/se0/se3/se2. `SwerveDirectTeleOp` now
builds only the swerve (`createSwerve`) instead of a whole Follower, so it no longer constructs a
`PinpointLocalizer` - one fewer thing that can re-origin the Pinpoint.

**Bring-up tool (diagnostic) — what changes for measurement:**
1. A marked box (corners A/B, `boxSet`) behaves exactly as before: centre-in-rect, same margins,
   same taper; `outwardScale` now delegates to `FenceGeometry.taper`, the identical smoothstep.
2. New `fieldFence` mode adds keep-out clamps and a heading-aware perimeter. Any azimuth sample
   taken while `box.clamped=true` under a field fence is suspect for the same reason as before,
   and now also near the HIVE rails and FLOWERS, not only near walls.
3. `/state` `box` gains `kind`, `robotL`, `robotW`, `keepOutClear`, `keepOuts[]`. The keep-out
   JSON is built once at arming; the per-publish cost is a string append plus one clearance
   computation over 6 polygons. Estimated well under 0.1 ms - NOT measured; check `msPublish`
   against the 1.6 ms baseline on the first session.
4. Tool heading kP is now seeded from `Constants.headingKP` (same 1.20).
5. `export` emits `Constants.java` declarations (was a pod-factory block that no longer matched
   the file). `robot.py constants` splices them by name.

Fence geometry checked on the host: `tools/swervetune/FenceGeometryCheck.java`, 353 assertions
against the real BIOBUZZ rail and FLOWER shapes (normals, inside/outside, taper continuity, a
hull-clear curve that still crosses a rail, push-out). Not yet run on the robot.

## 2026-10-01 (later) — Pedro-validation flow (rule 9 declaration)

No data taken yet. What changes in the bring-up tool (diagnostic) and how it alters measurement:

1. **Recorder:** 7 follower columns appended at the END of every row (`fterr,fherr,fcx,fcy,fch,
   fvel,fbusy`); every existing column keeps its position, so archived CSVs and scorers are
   unaffected. In FOLLOW, pod `err/pwr/flip/ctgt` now come from the follower's pods (previously
   NaN) and `tgt` is blanked (it was the tool's stale mixer mirror). Outside FOLLOW: unchanged.
2. **Statics reset at init, stop and `pedroReset`:** Foresight gains on `Constants.foresightConfig`,
   the mixer (`setMixer`) and the turn-gain schedule go back to the installed build. Before this,
   a `setMixer`/`pedroPidf` survived an OpMode restart (and leaked into any auto run in the same
   app process). Any A/B that relied on a toggle persisting across a restart no longer does.
3. **Follower bench:** `pedroStart` always builds a fresh follower; `pods=live` builds it on the
   tool's calibration. Heading kP for Foresight is now set by `pedroPidf hp`, and export takes
   Foresight's heading kP from there - previously it would have exported the tool's heading-HOLD
   kP, a different control law. Existing comment in Constants already noted the tool's kD 0.080
   was "not yet re-validated" on path following - this is the mechanism that validates it.

Shipped (`pedroPathing/Constants.java`): TUNED block now carries all 17 Foresight Tuner values by
name; translational is the tuner's primary/secondary shape, built as ONE PID while the two are
equal (they are: 0.26/0.26), so the installed behaviour is unchanged. Checked by hand: heading 1.20,
translational 0.26/0/0.025 single PID, coast/brake kV 1/73.9, velocities 73.9, decel 40/40,
quadratic brake 1/(2*40), linear and heading brake 0.

## 2026-10-01 (later still) — border fence, odometry in Constants terms (rule 9 declaration)

Diagnostic only, no data taken. `borderFence` arms a W x H perimeter with the field fence's
semantics and no keep-outs (`box.kind = border`, persisted as a `kind|` line; files written before
it load as `field` if they carry a `robot|` line). `odoConfig` now takes Constants.java's names and
keeps unspecified values (previously offsets applied only as a pair and directions only as a pair),
and it now deletes the hub box file like `resetImu` does - before, a reloaded box after `odoConfig`
was caught only by the origin/witness check. Odometry differences from Constants.java appear in
`errors[]` and `/state` `odo`. The bring-up tool and DashboardDriveTeleOp now take the Pinpoint's
device name from `Constants.pinpointConfig` (was a literal "pinpoint"; same value today).

## 2026-10-10 — Mechanism Tuner and operator banners (rule 9 declaration)

Diagnostic only, no data taken. New OpMode `Mechanism Tuner` (`diagnostics/tuning/`) and
`tools/mechtune/mechtune.py` for flywheels, turrets, gate, scoopula and the intake color sensor;
procedure in `MECHANISM_TUNING_TASK.md`. It never builds the drivetrain.

The only change to the swerve tool: `diagnostics/swerve/dashboard.html` now loads
`/tune/notify.js`, which polls `/tune/notes` once a second from the browser. That is one small
extra HTTP request per second on the RC web server, on NanoHTTPD worker threads; nothing in the
Swerve Bring-Up loop, `/swerve/state` or the recorder changes. Expected effect on any swerve
measurement: none measurable (estimated, not measured). If a loop-rate comparison ever needs to
rule it out, close the /swerve tab and drive from `robot.py`, which never loads the page.

Found while building it (shipped code, not changed here): `gate.openPos/closePos` and
`scoopula.scoopPos/unscoopPos` are all 0, so the gate and scoopula never move; and
`shootingRegression` returns `(int) distance` in inches as the flywheel ticks/s target, a
placeholder far below any shooting speed.

## 2026-10-10 — bring-up: Pinpoint read every loop in DRIVE and FOLLOW (rule 9 declaration)

Diagnostic only (`diagnostics/swerve/SwerveBringUp.java`), no data taken, untested on hardware.
`readHeading` used to call `pinpoint.update()` every loop only in HEADING and in DRIVE with heading
hold on; everywhere else it ran on the 5 Hz idle path. So in FOLLOW, and in DRIVE with heading hold
off, `applyBoxLimit` and the FOLLOW breach backstop clamped on pose, velocity and heading up to
200 ms old - about 9 in of travel at 45 in/s. Both modes now read the Pinpoint every loop
(`poseInUse()`).

Expected effect on measurement:
- DRIVE with heading hold ON (the default, and every DRIVE number in CLAUDE.md §6): no change - it
  already read every loop.
- DRIVE with heading hold OFF: one more Pinpoint I2C read per loop. Compare against `msHeading`
  1.81 ms (isolated read, CLAUDE.md §2) / 5.42 ms (incl. idle-path current and battery, §0.1
  above); expect loop dt to grow by roughly that read (estimated, not measured).
- FOLLOW: one more Pinpoint read per loop on top of the follower localizer's own read inside
  `pedro.update()`. No FOLLOW loop rate is recorded in this file, so there is no pre-change
  baseline; the first `pedrocheck.py` session should record `1/mean(dt)` in FOLLOW and label it
  post-2026-10-10.

## 2026-10-10 — DashboardDriveTeleOp enforces field/border fences (rule 9 declaration)

Diagnostic only, no data taken, untested on hardware. `DashboardDriveTeleOp.loadBox` ignored the
`kind|`, `robot|` and `keepout|` lines, so a saved field/border fence was enforced as a plain
centre-point box (footprint up to ~9 in past the wall, keep-outs dropped). It now parses them like
`SwerveBringUp.loadBox` and clamps exactly like `SwerveBringUp.applyBoxLimit` (heading-aware
centre bounds + `FenceGeometry.clampKeepOut`). A box file it cannot parse, or of an unknown kind,
now refuses all motion (it used to drive unfenced). Expected effect: none with a marked box
(identical clamp); with a field/border fence, more clamping near walls and keep-outs. No session
has driven this OpMode under a field fence, so no earlier data is affected.

## 2026-10-10 — dashboard.html circle test through the single-in-flight sender (rule 9 declaration)

Diagnostic only, untested on hardware. The circle test sent `drive` fire-and-forget every 90 ms;
it now goes through the same single-in-flight sender as the pad and hold-buttons, so at most one
drive request is on the wire and stale intermediates are dropped. Expected effect: none while the
round trip is under 90 ms (same commands, same rate, still robot-frame `foc=0`); under WiFi
latency the robot now sees fewer, fresher commands instead of a backlog flushed late. Circle-test
heading numbers taken before this date on a laggy link are not directly comparable.

## 2026-10-10 — host bench scripts: gates, one-shot motion commands (rule 9 declaration)

Diagnostic only (`tools/swervetune`), untested on hardware. `phase2_plant.py` now sends `drive`
and `rawServo` once (`retries=1`, was 6): on a transport failure a step is now missing from the
trace instead of possibly duplicated late (a duplicate `rawServo` restarts the dwell and bleeds
one staircase power into the next). No change on a clean link. `pedrotune.py` `drive`/`cent`/`all`
now refuse to run: under Pedro 3 their `pedroPidf dp/dd/df/cent` are ignored robot-side, so any
`current_runs/pedrotune.jsonl` rows of kind `drive` or `cent` written after the 2026-10-01 Pedro 3
migration compare identical controllers and should not be read as a gain effect. `trans` now
stops if `pedroStart` did not start the follower (`activate=transheading` is refused on Pedro 3),
so `trans` rows written after that migration also timed a follower that never ran.

## 2026-11-12 — Mechanism Tuner `flyDevice` (rule 9 declaration)

Diagnostic only. New command `cmd flyDevice wheel=pollen device=NAME` re-binds a tuner wheel to
another motor name in the active config, until the OpMode restarts. Reason: on 2026-11-12 the
pollen flywheel is wired to **Control Hub motor port 0** (config name `sm0`), not Expansion Hub
port 3 (`pollenTurret`), and there is no Expansion Hub on the robot, so `check` reported
`pollenTurret` missing. The flywheel is still run by the hub's own velocity PIDF
(`RUN_USING_ENCODER`, `setVelocity`), sampled once per loop with the same bulk read and dip
detector. Expected effect on any flywheel measurement: none (the hub firmware PIDF is the same on
either hub). Estimated, not measured. The gains tuned on port 0 apply to the same motor + wheel on
any port. There are no earlier flywheel trials (`tools/mechtune/trials.jsonl` did not exist), so
no dataset becomes non-comparable.

## 2026-11-12 — Pollen flywheel velocity PIDF (Mechanism Tuner, Part A)

Pollen wheel only, on **Control Hub motor port 0** (`sm0`, via `flyDevice`); no Expansion Hub,
nectar not connected. Robot on its own battery, 13.2 V at start, 11.9-12.7 V during the tests.
Hub velocity PIDF; tool loop 177-235 Hz true (`1/mean(dt)`). Velocity reads in 20 ticks/s steps,
so ripple sd ~10 ticks/s is the resolution floor.

**Incident.** Two 2 s-hold ff sweeps gave F 13.36 vs 14.23 (6.5% apart); the next two 5 s-hold
sweeps ran with the wheel **jammed** (vel 0, 4.5 A at 50% power, battery to 11.9 V), about 60 s of
stall in total. The operator cleared the jam. Those four sweeps are discarded. Stall guard
(vel < 1 with > 3 A for 0.6 s -> stopAll) used for every spin afterwards.

**Plant.** Two clean 5 s-hold sweeps after the fix: v ≈ 2970·p − 390 ticks/s (linear, large
friction offset; wheel stalls below ~20% power from rest), 2582 ticks/s at full power (12.5 V),
current 1.1-1.2 A at every level, each level settled to 98% in 0.13-0.25 s. F from full power
12.65 / 12.73 (0.6% apart). T = 1800 ticks/s (70% of max; no operator target).

**Shipped (500, 0, 0, 11.7) is unstable.** P saturates the output on a one-count error: ~5 Hz
limit cycle between 0 and +380 ticks/s over target, full reverse each cycle, once ran at
−400 ticks/s. Steps never settle (3/3). Never holds the band for the 300 ms the shot detector
needs, so shipped cannot be compared on real shots at all.

**Search** (steps 900→1800 n=3, plus 1400/2200; kicks 120 ms at 1800; overshoot is % of step):

| P | I | D | F | overshoot | ss err @1800 | kick recover | note |
|---|---|---|---|---|---|---|---|
| 1.34 | 0.134 | 0 | 13.4 | 11-12% | +45…+84 | 210 ms | FIRST recipe |
| 3 | 0.134 | 0 | 13.4 | 10-14% | | 192 ms | |
| 3 | 0.5 | 0 | 13.4 | 16-20% | +32 | 203 ms | more I = more windup |
| 3 | 0 | 0 | 13.4 | 9.8% | +77 | — | F too hot at 1800 |
| 3 | 0 | 0 | 12.7 | 0% | −14 | 236 ms | |
| 8 | 0 | 0 | 12.7 | 11.8% | | 212 ms | |
| 5 | 0 | 0 | 12.7 | 5.2% | | ~210 ms | −64 @1400 (4.5%, band edge) |
| 5 | 0.1 | 0 | 12.7 | 5.9-8% | +24 | | |
| 5 | 0.1 | 1 | 12.7 | 10% | +23 | | D hurts |
| **5** | **0.1** | **0** | **12.4** | **3.9-8.2%** | **−12…+8** | **234 ms** | **shipped** |
| 7 | 0.1 | 0 | 12.4 | 7.8-10.2% | −14…+1 | 214 ms | |

The open-loop fit predicted F ≈ 13.4 at 1800; closed loop wanted 12.4. The hub's term scaling
does not match a plant fit made in `setPower` units. Find F closed-loop, with I = 0.

**Acceptance.** Shipped vs (5, 0.1, 0, 12.4): interleaved 4 s holds, in-band fraction 0.084 vs
0.999, n=6/arm, t=116. Real shots, (5, 0.1, 0, 12.4) vs (7, 0.1, 0, 12.4), n=10/arm: 106.3 vs
102.3 ms, B−A −3.9 ms, 95% CI [−10.0, +2.2], p=0.19, so P=7 is not shown better. Shipped in the commit "Flywheel: POLLEN_PIDF 500,0,0,11.7 -> 5,0.1,0,12.4".

**Open.** NECTAR_PIDF still ships 500, 0, 0, 11.7: if that wheel is the same motor it will show
the same limit cycle (expected, not measured). AT_SPEED_FRACTION stays 0.05. Gains are for
this wheel at ~12 V; F is not voltage-compensated on the hub.

### 2026-11-12 — rapid-fire check at 2400 ticks/s (shipped pollen gains 5, 0.1, 0, 12.4)

Operator-fed pollen, free cadence, 289 s, 11.79-12.00 V, wheel on Control Hub port 0. 31 shots
detected: recoverMs 100.0 mean, sd 6.5, 95% CI [97.6, 102.4], range 84-111 (n=30; one shot dipped
only 7.5% and recovered in 28 ms, excluded as a glancing piece, all-31 mean 97.7). Dip 18.1% mean
(15.8-20.0), post-recovery overshoot 1.4% mean (max 3.3%), 0 failures to recover. Closest shot
pairs 0.8-0.9 s apart (7 pairs) each recovered fully. Not measured: shots closer than ~0.4 s
(300 ms re-arm + ~100 ms recovery) are not scored separately, and the operator's own shot count
was not recorded, so missed detections are unknown. 2400 is ~93% of the measured full-power speed
(2582 ticks/s at 12.5 V); gains were searched at 1400-2200, so this extends them upward.

## 2026-11-13 — Mechanism Tuner / mechtune.py streamlining (rule 9 declaration)

Diagnostic only. Built and checked against a host-side simulated `/tune` server; **not yet run on
the robot** (it was off the network). What changes and what it does to measurements:

- **OpMode stall guard.** A wheel driven at ≥ 0.35 power or ≥ 300 ticks/s reading < 40 ticks/s for
  0.6 s, or reading > 100 ticks/s backwards against its target for 0.3 s, calls `stopAll()`.
  Excluded during a kick. Effect on measurements: none unless it trips; a trip ends the test.
  Risk: a wheel that legitimately sits below 40 ticks/s at ≥ 0.35 power for 0.6 s would be
  stopped. On 2026-11-12 the pollen wheel left rest within ~0.6 s at 0.5 power (ff traces), so a
  slow spin-up at 0.35-0.45 power could trip it. `cmd stallGuard on=0` if so.
- **`flyDevice` persists** in `FIRST/mechtune_devices.txt`; `check` prints the bindings. No effect
  on measurements.
- **`ab`**: a trial whose gains never armed the dip detector is now a counted failure for its arm
  (it used to abort the whole A/B); `recoverMs = -1` trials are also counted per arm. The means
  and CIs are computed over the same trials as before (failures were already excluded); the
  failure counts are new output. Past A/B outputs stay comparable.
- **`ff` default powers 0.3,0.45,... → 0.5,0.6,0.7,0.8,0.9,1.0.** `fFromFull` is unaffected;
  `fFromFit` (a slope through the origin) **changes** with the level set: with the friction
  offset measured on 2026-11-12 (v ≈ 2970·p − 390), dropping the low levels raises the fitted
  slope. Compare `fFromFit` only between sweeps with the same `levels`.
- **New commands** `dircheck`, `hold`, `rapidfire`, `autotune` (procedures already used by hand on
  2026-11-12, now logged to `trials.jsonl`). `autotune` is a search only; acceptance is unchanged.

## 2026-11-14 — Phase 0 wiring check after the rebuild (swerve)

Blocks, NOT strapped (operator's call; single-device low-power pulses only), bench PSU 12.6 V /
10 A limit (hub read 12.49 V), Swerve Bring-Up idle, build e631b9a8 + operator WIP. Hub had **no**
`swerve_bringup_cal.txt` or `swerve_field_box.txt`: the tool ran on defaults, so stage 1
calibration starts from scratch.

Method: `rawServo pod=i pow=0.4 sec=1.0` / `pulseMotor pod=i` (0.25 x 0.6 s), one device at a
time, 6-8 s apart; the operator named the corner; every encoder's volts logged from `/state`
at ~8 Hz (mean dt 126 ms). The operator's own dashboard spins agreed with every row.

| corner | drive motor | turn servo | encoder | shipped Constants expected |
|---|---|---|---|---|
| BR | CH motor 3 | CH servo 3 | analog 0 | sm0 / ss0 / se1 |
| FR | CH motor 0 | CH servo 1 | analog 2 | sm1 / ss1 / se0 |
| FL | CH motor 1 | CH servo 2 | analog 3 | sm2 / ss2 / se3 |
| BL | CH motor 2 | CH servo 4 (moved there by the operator; was in another port) | analog 1 | sm3 / ss3 / se2 |

Fix: config names follow Constants' corners (c989990b), `podEncoderNames` -> {se0, se2, se3, se1}
(a78f9a16). Not yet checked: drive *directions*, servo directions (both stage 1 / wireScan).

**Pod slew is not the 214 deg/s in CLAUDE.md.** Open loop at 0.4 power, 1 s: 549, 544, 522 deg/s
(BR, FR, FL; net 593-629 deg each; assumes the encoder is still 1:1 and the default 0-3.3 V
span). 2.5x faster at less than half power means the turn servos or the servo->pod gearing
changed. Asked the operator; unanswered. Until answered, treat every turn gain and the 180 deg
flip coverage (`verifyCoverage`) as unvalidated.

**BL encoder (analog 1) is marginal.** Before the fix it sat at 3.69-3.73 V (above the 3.3 V
span); during reseating it jumped 0 / 1.36 / 2.31 / 3.7 / 4.7 V. After the fix, at rest over 25 s:
sd 7.3 mV, range 42 mV (~4.6 deg p-p) vs sd 0.5-1.9 mV on the other three. Reads, but the splice
is suspect. Old "pod 1 intermittent residual" history was a different index and wiring; no link
claimed.

Also seen: under the OLD apk, FTC Dashboard warned "Error communicating with I2C device(s)
'colorSensor'". Not seen since install; recheck under Mechanism Tuner.

### 2026-11-14 — Phase 0 wiring check, mechanisms (Mechanism Tuner, build a78f9a16, PSU 12.4-12.5 V)

Expansion Hub (address 2) reachable. `motorPort hub=2 port=N power=0.2 seconds=1.5`, one at a time,
operator watching ("all good" to every row):

| EH port | name | ticks in 1.5 s | peak A | operator |
|---|---|---|---|---|
| 0 | intakeLeft | 0 (no encoder) | 0.35 | left roller |
| 1 | pollenTurret (flywheel) | 441, 340 ticks/s | 0.82 | pollen flywheel |
| 2 | nectarTurret | 0 | 0.01 | nothing (not plugged in, placeholder) |
| 3 | intakeRight | 0 (no encoder) | 1.72 | right roller; 5x the left's current, operator says fine |

`intakeTest motor=BOTH power=0.3`, 2 s, with the shipped INTAKE_*_REVERSED flags: rollers
counter-rotate, both pulling inward (operator).

Detected but deliberately NOT moved: pollenTurretServo (limits 0.45-0.55 = +/-18.7 deg are a
guess, not measured), turretGate (open/close marks both unset). Not mounted: nectar flywheel,
nectar turret servo, scoopula. Skipped at the operator's word: colorSensor (unplugged); note the
built-in config types it `<ColorSensor>`, which is the SDK's **Modern Robotics** color sensor,
while an older hub config (Abracadabratestbot.xml) has it as `RevColorSensorV3` - fix the tag
when it is plugged in. Pinpoint: detected, pose/heading ok; directions and offsets need the floor.
SystemConstants.Flywheel.NECTAR_PIDF ships (500, 0, 0, 11.7), not the (5, 0.1, 0, 12.4) the run
prompt assumed.

### 2026-11-14 — Stage 1 calibration on blocks (bring-up tool only; nothing shipped yet)

Blocks, not strapped, PSU (hub 12.48 V). Operator confirmed the pods were **rebuilt**, which
explains the ~530 deg/s open-loop slew at 0.4 power: every turn gain from before is unvalidated.
CR pods rotate continuously (600 deg sweeps), so the 270 deg / 180 deg `verifyCoverage` concern
applies only to the shelved positional variant.

- **wireScan**: BR->se0, FR->se2, FL->se3, BL->se1, all `srvRev=true`, no ambiguity notes.
- **Encoder ranges** (`sweep`, 4 s x 0.45 per pod): BR 0.083-3.301, FR 0.073-3.302, FL 0.145-3.341,
  BL 0.164-3.391 V; spans 3.196-3.229 V (within 1%, so no single glitch inflated a bound). FL/BL
  both sit ~0.07 V higher at each end than BR/FR: looks like a ground offset shared by that pair.
- **Zeros** (`zeroAll`, operator lined wheels up by eye). Two alignments ~10 min apart: FL agreed
  to 2.0 deg; BR/FR/BL differed by 185.8 / 182.3 / 184.3 deg, i.e. the wheel flipped end for end
  plus **2.3-5.8 deg** of by-eye error. Drive direction was set after the second alignment, so the
  flip is absorbed; the few-degree per-pod error is not, and the crawl crab check (`zeroTrim`)
  only removes the common part. A straightedge across the wheels would do better.
- **Drive direction** (`pulseMotor`, operator says which way it would push the robot): BR back, FR
  forward, FL back, BL back -> `setDriveReversed` true on pods 0, 2, 3. Saved.
- Not done (needs the floor / a battery): crawl crab check + `zeroTrim`.

## 2026-11-14 — `podstep.py`: single-pod step trials (rule 9 declaration)

New tool `tools/swervetune/podstep.py`: one pod per trial (`pidStep pod=N`), same recorder, same
`swervebench.score_step`, pods and step directions in seeded random order. Written because the
chassis is on blocks **unstrapped** and the rebuilt pods slew ~2.5x faster than the pods the
"pidStepAll walks the chassis" warning was written for. Expected effect vs the archive's
`pidStepAll` steps: only one servo loads the rail and the chassis feels one reaction, so rise
times may read faster and post-settle motion smaller than an all-four step. Do not compare its
numbers with pre-2026-11-14 `pidStepAll` trials; compare cells within it.

### 2026-11-14 — rebuilt pods: shipped gains, and single-pulse travel (blocks, PSU 12.45 V)

**Shipped gains on the rebuilt pods** (kP 0.38, kD 0.022, kS 0.022 / 2 deg, cache 0.01, pulsed
6 deg / 0.6 deg / 0.035 x 20 ms), podstep, 90 deg steps, n=4 per pod (2 each way): **10/16 runs
loose** (post-settle p-p >= 2 deg); BL one run 82% overshoot with 2 flips; p-p up to 7.1 deg.
Steady-state |err| mostly < 0.5 deg - the problem is hunting after arrival, not the final error.

**Single open-loop pulse travel from rest** (`phase4_pulsecal.one_pulse`, 4 pulses per pod per
cell, 2 each direction): 0.020-0.035 power at 20 ms moves 0.06-0.09 deg mean (~encoder noise);
0.045 x 20 ms 0.29 mean (0.00-1.84); 0.045 x 40 ms 0.77 mean (0.07-3.68). Per pod at 0.045 x 40:
BR 0.39, FR 0.45, FL 0.76, BL 1.47 deg mean. Travel near breakaway is stick-slip: the same pulse
moves nothing or several degrees. The shipped 0.035 x 20 ms pulse is now below breakaway, and no
single pulse lands reliably inside a 0.6 deg tolerance.

### 2026-11-14 — rebuilt pods: turn-gain SEARCH on blocks (podstep, single pod, PSU 12.45 V)

SEARCH only - TUNING_TASK acceptance (pedrocheck on the floor) is not possible today (no battery).
90 deg steps, n=16 per cell (4 per pod, 2 each way), cells run back to back:

| cell | loose (pp>=2) | post-settle pp mean [95% CI] | mean abs ss |
|---|---|---|---|
| shipped (pulsed 6/0.6/0.035x20ms, kP .38 kD .022 kS .022) | 10/16 | 3.05 [1.86, 4.23] | 0.41 |
| pulsed OFF, kP .38 kD .022 kS .022 | 1/16 | 0.57 [0.24, 0.91] | 0.73 |
| pulsed OFF, kP .19 kD .011 kS .022 | 0/16 | 0.36 [0.17, 0.54] | **3.58** (stops short) |
| pulsed 0.045x20ms, tol 1.0 deg | 6/16 | 2.05 [0.59, 3.51] | 0.55 |
| pulsed OFF, kP .50 | 0/16 | 0.31 [0.09, 0.52] | 0.88 |
| pulsed OFF, kS .035 | 0/16 | 0.39 [0.18, 0.60] | 0.65 |
| pulsed OFF, kD .035 | 1/16 | 0.96 [-0.19, 2.10] | 0.60 |

**Randomised interleaved A/B** (`podstep.py ab`, gains re-applied before every trial, seed 41),
A = shipped, B = pulsed OFF + kS 0.035 (rest shipped), n=24 per arm (3 per pod per direction),
12.45 V throughout:
- post-settle p-p: A 2.35, B 0.41 deg; B-A **-1.94 [-2.82, -1.07]**, t=-4.34, p=1.4e-5
- loose runs: A 12/24, B 1/24 (Wilson upper 0.20)
- abs steady state: A 0.38, B 0.54; B-A +0.16 [-0.02, +0.34], p=0.074 (not shown worse)
- overshoot, rise 10-90: no difference (p >= 0.35); B worst-case abs ss 1.13-1.36 deg on 3 runs.

Reading: on the rebuilt (faster, stickier) pods the pulsed final approach causes the hunting -
the pulse is below breakaway until it isn't (stick-slip, see pulse travel above). Candidate for
the floor: **pulsed approach OFF (ships as pulseBandDeg = 0: CoaxialPod enters pulsed mode only
when |err| < pulseBandRad), kS 0.035**, kP/kD unchanged. Tool live gains set to B and saved.
Caveats: single-pod steps on blocks only, no drive load, unstrapped chassis, PSU not battery,
n=24/arm. Must be validated with pedrocheck --pods live on the floor before export.

Shipped today from stage 1 only: podZeroDeg, podMinV, podMaxV, podDriveReversed (4 commits,
4417f950..cc1b1ff7). Gains NOT shipped.

### 2026-11-14 — rebuilt pods ON THE FLOOR (parked steering), podstep, PSU 12.47-12.48 V

Operator moved the robot to the floor (surface type not stated - tiles assumed, unconfirmed) and
asked for the gains to be retuned for floor friction; the blocks result is only a baseline.
Parked steering is the regime these gains own: `TURN_GAIN_SCHEDULING` gives full authority at
zero drive power and scales to 0.24 once rolling/swinging, and the pulsed branch is gated off
above 0.08 drive power.

Floor is far harder than blocks. 8-16 trials per cell (2-4 per pod), back to back:

| cell | loose | pp mean | abs ss |
|---|---|---|---|
| shipped (pulsed 6/0.6/0.035x20) | 6/8 | 6.15 | 2.54 |
| blocks winner B (no pulse, kS .035/2) | 4/8 | 4.36 | 3.62 |
| no pulse kS .06/2 | 3/8 | 5.28 | 1.28 |
| no pulse kS .09/2 | 5/8 | 13.47 | 0.74 |
| no pulse kS .06/5 | 2/8 | 2.10 | 1.78 |
| no pulse kS .08/6 | 2/8 | 2.03 | 1.50 |
| no pulse kS .06/8 | 4/8 | 3.01 | 3.38 |
| kS .08/6 + parked deadband (pband 2.5 / ptol 2.0) | 3/16 | 1.39 | 1.71 |
| kS .08/6 + parked deadband (pband 3.5 / ptol 3.0) | 3/16 | 1.27 | 1.73 |
| kS .08/6 + parked deadband (pband 5.5 / ptol 5.0) | 3/16 | 1.40 | 3.61 |

Left pods only (FL, BL), kS .08/6: kD .010 / .022 / .035 -> 4/8, 5/8, 6/8 loose; kP .30 -> 4/8.

**Mechanism (trace podstep fl_db3 trial 14, FL 90->0):** the pod swings +/-4 deg at ~10 Hz while
the commanded power is ZERO: the controller pushes (0.07) and twists the tyre without moving the
contact patch, and the twist springs back the moment power drops. So parked precision on the
floor is bounded by tyre torsional compliance (~4 deg here), and near-zero-error gain makes it
worse: kS*tanh(err/band) has slope kS/band at zero, 0.035/2deg = 1.0 per rad, already 2.6x kP;
widening the band (2 -> 5-6 deg) is what cut the chatter. No cell removed it entirely.
**FL (pod 2) was loose in about half its runs in EVERY cell** - check that pod mechanically
(tyre, belt, gear lash) before tuning around it.

**Floor A/B, randomised interleaved** (`podstep.py ab floor_ab_shipped_vs_db3`, seed 121, gains
re-applied every trial), n=24 per arm (3 per pod per direction), 12.47-12.48 V PSU:
A = shipped; B = kP .38, kD .015, kS .08 / 6 deg, pulsed band 3.5 / tol 3.0 / 0.045 x 20 ms.
- post-settle p-p: A 4.96, B 1.40 deg; B-A **-3.57 [-5.41, -1.73]**, p=0.00014
- abs steady state: A 3.68, B 1.70 deg; B-A **-1.98 [-3.38, -0.59]**, p=0.0054
- rise 10-90: A 0.311, B 0.216 s; B-A -0.095 [-0.175, -0.014], p=0.021; overshoot n.s.
- loose: A 17/24, B 6/24 (B: BR 3/6, FR 0/6, FL 3/6, BL 0/6)
Robot creep over ~25 min of floor steering: < 0.4 in x/y, heading within ~1 deg (Pinpoint).

**Candidate F = arm B is the floor search winner.** Tool live gains set to F and saved;
`robot.py constants` dry run carries exactly turnKD/KS/KSBand per pod + pulseBand/Tolerance/Power
(tools/swervetune/runs/constants_20261114_143756.java). **NOT SHIPPED**: TUNING_TASK accepts gains
only on pedrocheck (real Follower, driving), which needs a battery - today is PSU-tethered.
Rolling side unvalidated: kS .08 scales to ~.019 at SCHED_FLOOR .24 when rolling, but the schedule
(floor, velocity leg 22 deg/s) was sized for the old 214 deg/s pods.
Supersedes the blocks candidate (pulsed off, kS .035) for the floor.

### 2026-11-14 — odometry push test: Pinpoint frame is rotated 90 deg from the robot's front

Battery (13.3 V), FTC tiles, Swerve Bring-Up idle, operator pushing by hand inside a 2x2-tile area;
pose logged from `/state` at ~7 Hz; moves converted to the robot frame with the Pinpoint's own
heading at the start (-91.5 deg; heading changed < 3 deg per push).

| operator push | expected (robot frame) | Pinpoint measured |
|---|---|---|
| forward 1 tile | forward +23.6 | forward -2.2, **left -22.7** (two stages, 5.3 + 17.4) |
| right 1 tile | left -23.6 | **forward -26.5**, left -2.0 |
| return, first leg | | left +27.3 (laptop Wi-Fi dropped 14:46:54, rest lost) |

measured (forward, left) = (true left, -true forward): a 90 deg rotation, not a sign error, so
`pinpointX/YPodReversed` cannot fix it. Most likely the X and Y pod cables are swapped on the
Pinpoint (or the pods are mounted rotated). The wheel zeros and drive directions follow the
operator's front, so drive and odometry frames currently disagree by 90 deg - nothing that
uses the pose (fence, Pedro, TeleOp field-centric) can be trusted until this is fixed.
Hand pushes are not a scale test (26.5 vs 23.6 in is push error). Asked the operator to check
the Pinpoint ports; the operator's new offsets (-2.84 / -5.988, uncommitted) depend on which pod
is X and will need re-deriving.

### 2026-11-14 (evening) — odometry fixed: X/Y cables swapped, X direction flipped, offsets validated

Battery 13.2–13.9 V, FTC tiles, 2x2-tile area, Swerve Bring-Up, border fence 47.16 x 47.16 in armed
for every drive (start pose = operator: back-right tile, facing up -> (35.37, 11.79) at 90 deg,
tile centre assumed, about +/-2 in). Candidate F gains live in the tool.

**Power.** The robot lost power completely at ~17:51:50: both hubs rebooted, logcat
`HubPowerCycle: Expansion Hub 2 regained power after a complete power loss`. Operator: the battery
connection is sensitive; re-seated. The 14:46 "laptop Wi-Fi drop" in the first push test was most
likely the same thing (the robot's AP vanished). A sudden disconnect = suspect power first.

**Cables.** Operator swapped the Pinpoint X and Y pod cables (the 90 deg rotation above).

**Drive breakaway (rebuilt pods, tiles).** f = 0.12 for 1.5 s and f = 0.20 for 1.0 s did not move
the robot at all (pose and IMU heading unchanged to 0.02 in / 0.02 deg); battery current rose
+0.37 A and +1.3 A, which is what four stalled motors draw at those duties, and the IMU showed no
yaw, so it is breakaway friction, not pods fighting (operator: "it went forward" at 0.30).
f = 0.30 moves it. **ROBOT_CONTROL §3.5's 0.12 smoke-test power does not move this robot.**
Also seen at f = 0.12: pod 0 parked 15 deg off its 90 deg target with 0.044 turn power -
TURN_GAIN_SCHEDULING cuts authority to 0.24 at drive power >= 0.10 and floor friction wins.
Traces: runs/20261114-1800{40,129}_fwdcrawl{1,2}.csv.gz.

**Directions** (one change, one test each):

| drive | odo flags | operator saw | Pinpoint (field, heading ~90) | verdict |
|---|---|---|---|---|
| f 0.30, 0.8 s | xRev **true**, yRev false | forward | dy **-4.1** in | X reads backward |
| f 0.30, 1.0 s | xRev **false**, yRev false | forward | dy **+9.8**, dx +1.0 | X correct |
| s 0.30, 0.8 + 1.0 s | xRev false, yRev false | left (asked for it) | dx -1.8, then **-9.0** (dy +0.8) | Y correct |

Traces: runs/20261114-180420_fwdcrawl3, -180907_fwd4, -1809{20,33}_left{1,2}.csv.gz.

**Offsets** (operator-measured -2.84 / -5.988 in, uncommitted until now). Turn-in-place fits of
p(theta) = c + [[sin, cos], [-cos, sin]](a, b), where a = xPodOffset_cfg - true and
b = true - yPodOffset_cfg (in/rad), per trace own c:

| trace | turn | a | b | rms in |
|---|---|---|---|---|
| rot1..rot4 (four 90 deg, start/stop each) | +91, -90, -94, +92 | +1.02, -0.46, -1.16, +0.46 | -1.36, -0.19, +0.23, +0.03 | 0.04–0.17 |
| spinccw (continuous) | +421 | +0.70 | -0.05 | 0.27 |
| spincw (continuous) | -435 | -0.57 | +0.32 | 0.22 |
| **joint, both spins** | | **+0.06** | **+0.14** | 0.51 |

An offset error gives the same (a, b) in both directions; a flips sign with direction here, so the
per-direction term is real robot motion (scrub while pods lag in the turn), not geometry. The joint
fit's correction is < 0.14 in on both offsets - inside the scrub - so **the operator's offsets are
kept**. Net pose change after a full turn: 1.3 in (+421 deg), 0.5 in (-435 deg); that is the
robot actually walking, which the orbit criterion (<= 0.6 in per 45 deg) cannot separate from
offset error on this drivetrain. Formal SEs are optimistic (autocorrelated residuals).
Scale (taped line, 1%) NOT tested; goBILDA 4-bar pod type unchanged; hand pushes were within
+/-12% of a tile (push error), so no gross pod-type error.
Traces: runs/20261114-180958_rot1, -1810{58}/-1811{02,06}_rot{2,3,4},
-1811{53}/-181202_spin{ccw,cw}.csv.gz. Fit: rotfit (scratch), model above.

### 2026-11-14 (evening) — crab check: zeroTrim -3.9 deg kept; the rest is rolling steering error

Battery ~13.0 V, FTC tiles, border fence, candidate F gains live, odometry as committed (7edafdba..).
Six 0.8 s drives at f = +/-0.30 (3 back + 3 forward, interleaved) along the field y axis
(heading 87-96 deg) per cell. Crab = robot-frame direction of travel minus commanded, from the
recorded pose after a 0.25 s settle (window ~0.55 s, ~5-6 in); + = CCW. scratch crab.py.

| cell | crab mean [95% CI] | sd | fleet steering err (wheel - tgt) | yaw per run |
|---|---|---|---|---|
| calibration as shipped | **-3.88 [-5.30, -2.46]** | 1.35 | -1.4 | +0.1..+2.7 |
| zeroTrim -3.9 | **-1.28 [-1.78, -0.78]** | 0.48 | -4.6 | -0.4..+0.6 |
| zeroTrim -3.9 -1.3 | -1.30 [-1.39, -1.21] | 0.09 | -4.8 | -0.4..+0.1 |

The first trim moved the crab +2.6 deg (Welch t ~ 4.5, n = 6/6) and cut the yaw while driving;
the second trim moved it 0.0 - **reverted, total trim -3.9 kept** (tool cal on the hub; ships
with the next `robot.py constants` export as podZeroDeg). Criterion (< 1 deg) NOT met.

Why the second trim did nothing: while rolling the pods do not track. Per-pod mean steering
error over the drives (trimmed cells): BR +2 to +3, FR -5 to -7, FL -3 to -4.5, **BL -11.5 deg**.
TURN_GAIN_SCHEDULING cuts turn authority to SCHED_FLOOR 0.24 by 0.10 drive power; at F's gains
that is ~0.02-0.04 servo power for a 5-10 deg error - below floor friction - so the pods stay
roughly where they were when the drive started (IDLE leaves the servos limp) and a 1.3 deg zero
change is invisible. The floor (0.24) was sized 2026-08-15 for the OLD 214 deg/s pods, where full
authority limit-cycled while rolling. Next: SEARCH the floor on the rebuilt pods (setPidf floor=),
then accept through pedrocheck.
Traces: runs/20261114-1817*_crab{B,F}{1,2,3}, -1818*_trim{B,F}*, -1819*_trim2{B,F}*.csv.gz.

### 2026-11-14 (evening) — SCHED_FLOOR search on the rebuilt pods: 1.0 hunts; 0.24 vs 0.50 unresolved

Battery ~13.0 V, tiles, border fence, candidate F, zeroTrim -3.9. Three arms (setPidf floor=),
3 blocks, arm order shuffled per block (seed 1114), each arm = one back + one forward 0.8 s drive
at 0.30 per block (n = 6 drives/arm). Metrics after a 0.25 s settle (scratch rollmetrics.py):
mean |wheel - tgt| (fleet), its sd within the drive (shake), wheel direction reversals/s.

| floor | mean abs err | err sd (shake) | reversals/s | crab |
|---|---|---|---|---|
| 0.24 (shipped) | 7.0 +/- 2.3 | 5.0 +/- 4.2 | 2.9 +/- 3.0 | -0.6 +/- 1.8 |
| 0.50 | 5.0 +/- 2.1 | 6.0 +/- 3.1 | 3.8 +/- 1.9 | -0.9 +/- 1.3 |
| 1.00 (no schedule) | 8.6 +/- 1.2 | **11.1 +/- 1.2** | **9.3 +/- 2.8** | +1.2 +/- 1.1 |

**Floor 1.0 limit-cycles while rolling** (as on the old pods, 2026-08-14) - unscheduled is out.
0.24 vs 0.50 is inside the noise at n = 6. Contaminated: the robot yawed to ~82 deg over the run and
the fence clamped 3-7 of 7 ticks in blocks 2-3 (per-axis clamping rotates the commanded vector, so
targets move mid-drive). Block 1 alone (6 deg off-axis less, 0-3 clamps): 0.24 frozen (err sd
0.1, 0 reversals, |err| 5.2), 0.50 tracking (|err| 2.3-3.2). Needs a bigger clean area or
pedrocheck to settle. Floor restored to 0.24. Traces runs/20261114-18*_fl{024,050,100}_*.csv.gz.

### 2026-11-14 (evening) — pedrocheck hold/turn in the 2x2, shipped pod gains vs candidate F

`pedrocheck.py suite --pods live --surface tiles --only hold,turn --step 6 --n 3 --arms
current_runs/arms_shipped_vs_F.json` (label pc2x2_ShipVsF), battery 12.9 -> 12.8 V, tiles, border
fence 47.16 in, zeroTrim -3.9, SCHED_FLOOR 0.24 both arms. hold+-x / hold+y were mostly REFUSED by
the fence (6 in margin in a 2x2 - nothing moved), so hold is hold-y (n=3/arm) + 3 hold+x.

| arm / trial | heading overshoot deg | heading residual deg | settle 2 deg | translational residual in |
|---|---|---|---|---|
| shipped / turn+ (90) | 10.5 +/- 2.4 | 4.9 +/- 1.1 | never | 0.41 |
| shipped / turn- | 12.6 +/- 3.8 | 5.4 +/- 2.9 | never | 0.25 |
| F / turn+ | 9 +/- 15 | 4.5 +/- 7.4 | once (b1) | 0.39 |
| F / turn- | 11.7 +/- 7.7 | 4.7 +/- 2.0 | never | 0.17 |
| shipped / hold-y | - | 3.3 | - | 0.32 (1 FAIL 0.515) |
| F / hold-y | - | 1.1 | - | 0.43 (1 FAIL 0.671) |

(mean +/- 95% CI half-width, n = 3; the script's own summary.) **No pod-gain difference is
claimable at n = 3** (rule 2 / TUNING_TASK stage 4 want n >= 10). **Both arms fail every turn**:
9-15 deg overshoot, 4-6 deg residual, never inside 2 deg. That is follower-level, not pod gains:
Pedro's heading kP 1.20 turns a 5 deg (0.087 rad) residual into ~0.10 of rotation demand, below
the ~0.2-0.3 drive power this drivetrain needs to break loose on tiles (FINDINGS above), so the
robot stops short and stays there. The Foresight constants are still placeholders
(maxAchievable*Velocity = PLACEHOLDER 73.9). Pod tracking while Pedro drives: pod_err_p95
29-57 deg in both arms (turn transitions included).
**Process gap:** TUNING_TASK stage 6 (Foresight Tuner, operator-run AutoTune) cannot run as written -
the tuner was removed with Sloth 2026-11-01 (branch autotune-sloth, CLAUDE.md platform table).
Heading kP is still reachable through stage 7 (`pedroPidf hp=` arms), but turns need the 2x2.
Data: current_runs/pedrocheck.jsonl (label pc2x2_ShipVsF), runs/20261114-183*_pc2x2_ShipVsF_*.

### 2026-11-14 (evening) — 1x4 strip: odometry scale exact; SCHED_FLOOR 0.24 vs 0.50 n=10; crab +0.6 deg

Battery 12.8 V, FTC tiles, a 1x4-tile strip with clear floor >= 3.5 in beside each long edge
(operator), border fence 30.5 x 94.32 in (the strip + that margin; a 23.58-wide fence is refused by
the 12 in travel rule and pedro targets need 6 in + half-diagonal, so the FOLLOWER CANNOT RUN in a
strip - drive commands only). Start pose: operator tape, back bumper 3 in from the strip's back
edge, centred, facing down it -> (15.25, 12.0) at 90.

**Scale.** One f 0.30 drive for 3.5 s: odometry 48.13 in (dx +1.1, yaw +1.3 deg). Operator taped
the back bumper at the predicted 51.1 in from the back edge: "correct, precise". Scale error not
resolvable at tape precision (estimated < 0.5%); goBILDA 4-bar pod type confirmed.

**SCHED_FLOOR**, randomised interleaved blocks (seed 11142), each arm one 2.5 s back + one 2.5 s
forward run at 0.30 per block, 5 blocks: n = 10 drives/arm, ~30 in each, metrics after 0.4 s
(scratch rollmetrics.py; crab_t = travel direction minus the pods' mean target vector, immune to
the fence's lateral clamping, which modified 30/31 ticks because the robot sits inside the side
margin).

| metric | 0.24 (shipped) | 0.50 | diff [95% CI] | p |
|---|---|---|---|---|
| mean abs steering err, deg | 4.67 +/- 2.25 | 3.42 +/- 1.52 | -1.25 [-3.06, +0.56] | 0.15 |
| err sd within run (shake) | 5.27 +/- 3.49 | 3.90 +/- 2.89 | -1.37 [-4.40, +1.65] | 0.34 |
| wheel reversals /s | 1.99 +/- 2.05 | 3.31 +/- 2.98 | +1.33 [-1.09, +3.74] | 0.25 |
| crab_t, deg | +0.60 +/- 0.92 | +1.05 +/- 1.08 | +0.44 [-0.50, +1.39] | 0.32 |
| distance in 2.5 s, in | 27.3 +/- 1.4 | 28.7 +/- 1.1 | +1.4 [+0.2, +2.6] | 0.011 |

No steering metric differs at n = 10; 0.50 leans lower error but more reversals; distance is the
one significant metric of seven (treat as suggestive). **Floor stays 0.24** (it is a vendored
RUCKUS PATCH constant - changing it needs more than this). Per-pod |err| at 0.24: BR 3.7, FR 4.9,
FL 4.1, BL 6.0 deg - long runs track far better than the 0.55 s windows in the 2x2 (7 deg).

**Crab with zeroTrim -3.9, long runs: +0.60 deg [-0.06, +1.26]** (0.24 arm, n = 10) - inside the
< 1 deg criterion on the point estimate. The 2x2's -1.3 came from 0.55 s windows in which the pods
had not converged. The trim is accepted (crab -3.9 -> +0.6). The robot crept +4.6 in sideways
over ~640 in of shuttling (heading hold, not crab).
Traces: runs/20261114-1843*_scale1, runs/20261114-18*_sf{024,050}_*.csv.gz.

## 2026-11-14 — mechtune.py `--rec-each` on `shots` / `ab` (rule 9 declaration)

Diagnostic only. With `--rec-each PREFIX`, each shot trial is wrapped in its own Mechanism Tuner
flight-recorder trace: wait until the wheel holds the band, `recStart`, the unchanged `run_shot`,
0.6 s more, `recStop`, pull to `tools/mechtune/runs/PREFIX_...csv`. Reason: FlyRecorder holds 6000
samples (~60 s at the tuner's 85-105 Hz) and never wraps, so one trace cannot span a 20-shot A/B,
and the Phase 2 portfolio figure needs the velocity trace of every shot. Expected effect on the
measurement: none on control (the hub runs the velocity PIDF; the OpMode only samples); recording
adds one array write per loop, and the CSV render runs on the web thread between shots, while the
wheel idles at target. Not measured. Without the flag nothing changes, so earlier shot A/Bs stay
comparable.

## 2026-11-14 — Pollen flywheel PIDF (Phase 2): P 5 -> 10, real-shot recovery -36 ms

Pollen wheel on **Expansion Hub motor port 1** (`pollenTurret`; was Control Hub port 0 on 11-12,
no saved `flyDevice` on the hub), robot on battery 12.42-12.71 V, on tiles, Mechanism Tuner on build
d0fca369 + operator WIP. Clear confirmed in the opening banner. Nectar flywheel not mounted. Tool
loop 85-150 Hz true. Operator had just plugged in the flywheel encoder.

- **Direction** (`dircheck --ask`): +0.5 power -> +1167 ticks/s, operator: shooting direction.
- **Plant** (`ff` x2): 2656 / 2650 ticks/s at full power, 12.6 V; F_full 12.34 / 12.36, F_fit 12.61 /
  12.57 (0.3%). 3% faster than 11-12 (2582 at 12.5 V). **T = 1860** = 70% of max (no operator
  target).
- **Baseline, starting gains (5, 0.1, 0, 12.4)**: step 930 -> 1860 n=5 settle 0.21 s (0.19-0.22),
  overshoot 2.0% (0-3.9), ss sd 10.7; kick 120 ms n=10 recover 256 +/- 26 ms, dip 24.7%; real shots
  n=4 (one not fed in time) **165.4 +/- 3.4 ms**, dip 28.5%.
- **Search** (`autotune`): closed-loop F 12.43, recommended (5, 0.1, 0, 12.43) = the shipped gains;
  its grid stops at P = 7. Recovery is not saturated at P = 5 (P*err ~ 2600 of 32767 on top of
  F*T ~ 23000), so P was pushed up by hand: **P = 10** step overshoot 4.8% (n=3), kick 163 +/- 9 ms
  (n=4); **P = 15** overshoot 11.8% (fails <= 5%), kick 157 ms. (A first attempt at these probes ran
  on (5, 0.1, 0, 12.43) because `pidf` got one quoted argument; that data is labelled by its pidf in
  trials.jsonl.)
- **Acceptance**, `ab --test shots --n 10 --seed 1114`, randomised interleaved, operator-fed pollen:
  A (5, 0.1, 0, 12.4) 163.4 +/- 14.8 ms (n=9) vs B (10, 0.1, 0, 12.4) **127.1 +/- 10.4 ms** (n=10),
  **B - A = -36.3 ms, 95% CI [-49.0, -23.6], t = -6.12, p = 2.5e-5**, 0 failures each. Dip 29.3 vs
  28.4%, post-shot overshoot 1.67 vs 1.61% (max 3.2 vs 2.2%). Steps at P = 10, n = 11 in all:
  overshoot 4.1% mean (2.0-8.3), settle ~0.14 s, ss sd 10.1 (shipped arm same session n=8: 1.0%,
  0.2 s, 10.5). Survey A thresholds (CI < 0, overshoot <= 5%, sd <= band/3) are guesses.
- **Shipped** c1a9e9fa: POLLEN_PIDF (10, 0.1, 0, 12.4); deployed; Mechanism Tuner init shows it.
  AT_SPEED_FRACTION stays 0.05: ripple sd ~10 ticks/s (0.5% of T) would allow a tighter band, but no
  exit-velocity measurement says it helps.
- **Figure**: docs/portfolio/flywheel/ (PNG, SVG, per-shot CSV, traces, script, README).
- Not measured: whether the higher P changes shot-to-shot exit velocity; cadence closer than ~0.5 s
  (one A/B trace caught two pieces ~470 ms apart); NECTAR_PIDF still (500, 0, 0, 11.7), unmounted.

## 2026-11-14 — Turret end stop: shipped limits 0.003-0.997, and the bench tools now respect them

Operator: with the servo at 0.5 the pollen turret faces straight and the mechanism's limits are the
Axon MAX+'s own travel ends; on the full 500-2500 us PWM range the servo "sometimes skips over its
limit and breaks the mechanism" (an end overshoot crossing the servo's encoder wrap, as far as the
symptom shows - not measured). In Mechanism Tuner's limits tab (free mode, slider) the operator drove
it to 0.0 and 1.0 (-/+186.9 deg) at 19:39; the hub then dropped off the network. Shipped:
POLLEN_TURRET_MIN/MAX 0.45/0.55 -> 0.003/0.997 (002acbc2, b11d7613), the operator's 1-degree stop
(1.1 deg of turret), range 371.6 deg >= 360 + the 10 deg unwind hysteresis. Overshoot size at an
end is NOT measured; up to 0.018 per side keeps 360 deg if 0.003 is too little.

**Rule 9 declaration (diagnostic tools, no measurement depends on them):** `diagnostics/TurretEndStop`
clamps raw servo commands to a turret servo to its shipped limits. `servoTest` (drove its servo in
INIT, default 0.0 on `pollenTurretServo`, and sent 0 at stop) and `servoRangeTest` (default 0.0,
full PWM) now default to 0.5 and clamp; `servoTest` no longer sends a turret to 0 at stop;
`turretServoTest` clamps; Mechanism Tuner's free mode (limits tab) stays 0.003 off each end.
servoRangeTest's "travel at POSITION 1" reading on a turret servo is now taken at 0.997.
`turretTrackTest` and PreMatchSystemCheck already went through the shipped clamp / center.

### 2026-11-14 — the 1-degree stop was not enough: pollen turret stops widened to 0.018 / 0.982

The operator reported the turret skipped again on the 0.003 build (ab4c1de7). Mechanism Tuner's
state showed its last command was turretPos 0.003 (-185.8 deg), the stop itself; the OpMode was
stopped at once so the hub failsafe released the servo. Root cause (arithmetic, not measured): the
servo covers ~359 deg of its 360 deg encoder over 500-2500 us (servoRangeTest 2026-11-01: 323.0 deg
at 600-2400, x 2000/1800 = 358.9), so a commanded end sits ~0.6 deg from the wrap; 0.003 adds only
1.1 deg. With 355/370 gearing, 360 deg of turret needs 345.4 deg of servo, so the margin cannot
exceed 6.7 deg per side without losing 360 deg: shipped 0.018 / 0.982 (360.3 deg, ~7 deg from the
wrap), and the tools' end stop (TurretEndStop.END, Mechanism Tuner free mode) moved to 0.018 too.
If a full-speed arrival still skips there, 360 deg is not reachable at full servo speed: either
slow the approach near the ends in systems/Turret.java or give up a few degrees of coverage.

## 2026-11-15 — Candidate F accepted and shipped; the follower's hold jitter; battery power losses

Tiles, 2x2 border fence (47.16 in, operator pose: bottom-right tile facing up, then the tile cross
facing up), battery 12.2-12.5 V. `pedrocheck.py suite --pods live --only hold,turn --step 4 --n 10
--arms current_runs/arms_shipped_vs_F.json`, randomised interleaved. Four runs were cut short:
three by the robot **losing power** (the battery jolted loose; each time the hub had just rebooted,
`adb shell uptime` 2 min, which looked like a laptop Wi-Fi drop because the Mac then auto-joined
another network) and one stopped on the operator's word (below). Pooled labels pc2x2_n10b/c/d,
24 shipped + 14 F trials (scratch pc_compare.py, Welch, normal-approx p):

| metric | shipped | F | diff [95% CI] | p |
|---|---|---|---|---|
| hold: settle to 1 in, s | 0.95 +/- 0.23 (n=16) | 0.80 +/- 0.13 (n=10) | -0.15 [-0.29, -0.01] | 0.03 |
| hold: overshoot, in | 0.24 +/- 0.54 | 0.04 +/- 0.09 | -0.21 [-0.48, +0.06] | 0.13 |
| hold: heading residual, deg | 2.06 +/- 1.63 | 1.61 +/- 1.43 | -0.46 [-1.65, +0.74] | 0.45 |
| hold: pod err p95 (mean of pods), deg | 47.2 +/- 10.4 | 40.2 +/- 11.1 | -7.0 [-15.5, +1.6] | 0.11 |
| threshold failures | 13/24 | 5/14 | | |

Turns n = 2-4 per direction: inconclusive (both arms overshoot 3-13 deg and stop 2-7 deg short).
No metric worse for F. With the floor podstep A/B (2026-11-14, n=24/arm: post-settle pp -3.57 deg
p=0.00014, loose 17/24 -> 6/24), **F is accepted and shipped** (e49de336..8b2778ac): kD 0.015,
kS 0.080, kS band 6.0, pulse band 3.5, tolerance 3.0, power 0.045. CLAUDE.md section 5 updated.
`--pods shipped` acceptance after deploy is still to run (turns only - see the jitter below).

**Hold jitter (follower, both arms).** The operator stopped the run: "jolting extremely". Traces of
the hold trials show the follower's commanded wheel direction (`ctgt`) jumping 120-160 deg every
~0.15 s as the robot nears the hold point (e.g. 133 -> 260 -> 98 -> 246 deg), every pod swinging at
~0.5 turn power and flipping its drive sign: 3.4-5.1 flips/s per pod in hold trials today, 3.9/s
yesterday (step 6) - the same, so not something new under the robot (operator asked to check
anyway; battery since zip-tied). Turns: ~1 flip/s. It is Pedro's translational correction near
zero error on a swerve whose drive needs ~0.3 power to break loose: small corrections do nothing,
the error grows, the correction vector swings around the target. TeleOp's follower.manual() has
no position hold, so driving is unaffected; autos will show it at every hold / path end. Not fixed.
Candidates (untested): a translation deadband below which pods keep their angle, or a smaller
heading of the hold vector change per update (Swerve.demandSlewDegPerSec is sized for the old
214 deg/s pods and does not appear to limit ctgt here).
Data: current_runs/pedrocheck.jsonl, runs/*pc2x2_n10[bcd]*.

## 2026-11-15 — TeleOp shakedown: no-DS driving fixed; Pinpoint heading froze at exactly 0.0 once

**Driving TeleOp with no Driver Station.** FTC Dashboard 0.5.1's gamepad override does not work with
an iterative OpMode on SDK 12 (`OpMode.internalPreUserCode` re-copies the DS gamepad data into
`gamepad1` before every `loop()`, wiping Dashboard's write; momentary presses slip through, held
sticks never do - a 0.5 stick for 0.8 s moved TeleOp's pose 0.00 in). Its client also only re-sends
on change (+150 ms), so a held stick is zeroed by its own 500 ms watchdog. Fixed tool-side with
`diagnostics/tuning/GamepadBridge` (77ec1202): the /tune Driver Station tab posts the pad to
`/tune/gamepad`, injected through `newGamepadDataAvailable` with a 400 ms watchdog. TeleOp now
drives. TeleOp loop rate in this shakedown: 49 Hz median of 256-sample windows (31-56), dt mean
19.3 ms, p90 27.3 ms - the first measurement of TeleOp's loop rate (battery 12.9-13.2 V).

**Turret fixes found in the shakedown (shipped):** POLLEN_TURRET_REVERSED true (5e5074f9: the turret
turned with the robot); Shooter RESET keeps aiming through the 1.0 s unwind settle (dd88da25: driver
yaw rate while turning 102 deg/s median, 167 p90, 478 max, so the frozen aim landed 100+ deg past).

**Pinpoint heading froze at exactly 0.0.** After the 11:17 redeploy TeleOp reported heading 0.0 in
all 856 telemetry samples (11:18-11:19) while x/y traced circles as the robot spun; Swerve Bring-Up
read the same (`heading.deg` 0.0, `rate` 0.0 - a live IMU shows ~0.004 deg of noise at rest). The
turret, which aims from the heading, stopped turning. `resetImu` (Pinpoint resetPosAndIMU, robot
still) brought it back: -0.0039 at rest, +90.08 after an operator's ~90 deg CCW hand turn. Cause
unknown - it worked across the previous redeploys and the battery swap; it froze across an app
restart with the hub powered. **Competition risk:** TeleOp and the autos have no check for a frozen
heading (SwerveBringUp has one, `headingStuckSeconds`); a frozen heading breaks turret aiming and the
zone check silently. Not yet addressed.

## 2026-11-15 — rapid fire at 2400: the wheel cannot keep up; F voltage-compensated; heading froze again

**First 4-ball burst** (`mechtune.py burst`, gate open + intake feeding, (10, 0.1, 0, 12.4) at 2400 ticks/s,
battery 11.9 V before / 11.4-11.6 V during): contacts at 1.00, 1.12, 1.20, 1.60 s, wheel speed at contact
2300 / 2120 / 1840 / 2400 (-4 / -12 / -23 / 0%), back in band 1.82 s. Balls 2-3 hit before the wheel
recovered, so one burst spans ~23% of exit-speed spread. During recovery the hub output is saturated
(F x T = 29,760 of 32,767 before any P term), so P/I/D cannot speed it up; recovering 1500 -> 2400 took
~200 ms at the limit. One shot only (n=1), plus an accidental single shot earlier: drop 35.8%, back in band
461 ms (127 ms at 1860 on the same gains). The burst scorer counts one ball per excursion below the band,
so merged contacts show as one - read the trace for back-to-back balls.

**F at 2400.** Closed-loop search (P=3, I=0) -> F 12.2; but with the production P/I, F 12.2 settles -43
ticks/s vs -21 for the shipped 12.4 (n=4/4 interleaved, 12.6 V), so F stays 12.4. The -2.5% before the
burst was the battery (11.9 V): **F is now voltage-compensated** (f8e1b825: F x 12.6 V / battery V,
smoothed, 4 Hz). On the robot, TeleOp telemetry (1cffa83a): wheel 1180-1200 / 1200 at idle, F scale 0.995
at 12.66 V and 1.000 at 12.62 V - working as designed; the low-battery benefit is not measured yet.

**Mechanical (operator asked; estimates, not measured):** 24:20 drives the wheel at 1.2x the motor; at
2400 ticks/s the motor is at ~91% of its loaded max (2650), leaving ~9% of stall torque to recover. Each
ball takes ~47-60% of the wheel's kinetic energy (27-37% speed drop). Advice given: add inertia (back-to-
back consistency) and gear up to ~1.6-1.7x (motor near 65% of max, ~2.5-3x the accelerating torque at the
wheel); the motor-side target then scales (same wheel speed at 1.6x = 2400 x 1.2 / 1.6 = 1800 ticks/s).

**Pinpoint heading froze at exactly 0.0 a second time**, again right after an app restart (adb install,
11:58; 156 TeleOp samples all 0.0, Bring-Up 0.0 / rate 0.0). Not caused by TeleOp (PinpointLocalizer
resetMode NONE does nothing to the IMU at init). resetImu in Bring-Up fixed it again (-0.013 at rest;
TeleOp then 0.0035-0.0056). Not every restart does it (11:12 was fine). The Pinpoint's device-status flags
are not read anywhere - next step to find the cause; TeleOp/autos still have no frozen-heading check.

### 2026-11-15 — operator confirmation: RT shooting, 2400 stopgap, turret stops, unwind aiming

Operator, after testing on the robot: TeleOp's right-trigger shoot (2b10d3e2), the flat 2400 ticks/s
shooting speed (c4aae62d), the pollen turret stops 0.018 / 0.982 (9680018e, 805f74b5) and keeping the aim
through the unwind settle (dd88da25) "have all been tested, it works". No numbers were recorded for these
checks (operator observation). The --pods shipped turns-only pedrocheck on the installed F gains
(pc2x2_shippedF_turns, n=3/direction, 12.5 V) failed every turn on the follower's heading (overshoot
5.4-14.8 deg, residual 3.3-6.3 deg) exactly like both arms before - a follower-level issue, not the pods.

## 2026-11-15 — Auto Dry Run: RED auto on blocks with a simulated pose - the gate never opens

**New diagnostic** (bfc477b3 shipped hook, aca55e68 tool): `Auto Dry Run (on blocks, sim pose)` runs
RedSoloAuto unmodified (paths, events, shooter, intake, gate, turret, scoopula on the real hardware) with a
kinematic SimLocalizer in place of the Pinpoint, integrating the follower's own commanded DrivePowers.
Pose on /field. It bypasses requireForesightMeasured() for the simulated follower only (the real autos still
refuse to run: FORESIGHT_MEASURED is false).

**First run diverged** (pose 1e135 in within 1.5 s; NaN reached the turret servo, the SDK refused it and
stopped the OpMode - no hardware harm). Cause: the sim took DrivePowers unclipped; Foresight's braking term
grows with v^2, so an ideal integrator ran away. Fixed in the tool: powers go through the vendored Swerve
mixer's limits (translation clipped to 1, pod vectors normalised by the largest), and a non-finite pose is
refused and stops the dry run.

**Second run** (robot on blocks, no pieces, 12.60 -> 12.25 V, RED, MOVE_PODS true): all 14 steps ran in
order, the auto reached Done at 52.4 s, stop() clean. Mechanism results (real code on real hardware):

| | result |
|---|---|
| Shoot steps (SHOOT_TIME 1.5 s) | **4/4 timed out with the gate shut** - shooter never left TRACK/RESET |
| Shooter during Shoot | pollen aimed (aimP) in all 4; **nectar never aimed** in any; pollen wheel 2400-2460 at step end |
| Intake | never ran (it runs only while the shooter is in SHOOT) |
| Turret unwinds (RESET) | 9 during the auto; Shoot step 3 began mid-unwind (~0.7 s of its 1.5 s lost) |
| Scoopula | down 8.09 s / up 16.83 s, down 33.50 s / up 38.37 s - as the callbacks place them |

Cause of the shut gate: Shooter fires only with BOTH turrets aimed and BOTH wheels at speed; the nectar side
is not mounted (turret limits 0.45-0.55 unmeasured guess, wheel absent). Predicted before the run, confirmed.

**Path finding, mechanism in code, risk on the robot plausible but not measured:** chain2 is a loop whose end
(10.2, 10.3) is 2.1 in from its start (9.9, 8.2). BezierCurve.closestParameter seeds from a GLOBAL coarse
search (t = 0, 0.1 .. 1.0), and Foresight ends a path once closestT >= 0.975 (parametricTConstraint 0.025).
In the sim the robot arrived 12 in past POINT_1 and the follower declared chain2 done after 0.39 s, the loop
never driven. A real robot reaching POINT_1 ~1 in north of the line would be nearer t = 1 than t = 0 too.

**Sim-only, not a prediction:** path-end overshoot 2.8-17.6 in, lateral wander up to ~12 in (chain5, chain8),
and the 52.4 s total (> the 30 s auto period) come from an invented chassis model (vMax = the 73.9 in/s
placeholder, 0.15 s lag) against placeholder Foresight constants. Do not read path timing from this tool.

**Third run, NECTAR_MOUNTED = false** (a10797e1; on blocks, no pieces, 12.54 -> 12.12 V): 14/14 steps,
Done at 55.3 s. **2/4 Shoot steps opened the gate** (was 0/4):

| Shoot step | gate opened | gate open | what happened |
|---|---|---|---|
| 1 (start) | +0.28 s, wheel 2380 | 1.18 s | wheel 0 -> 2380 inside 0.28 s from rest |
| 2 (POINT_3) | never | 0 | aim lost at +0.27 s while the sim robot slid 14 in past POINT_3, then unwinding to the end |
| 3 (POINT_5) | never | 0 | began mid-unwind (as in run 2); the unwind took 1.35 s of the 1.5 s window |
| 4 (POINT_7) | +0.31 s, wheel 2280 | 1.23 s | |

12 unwinds in the auto. Along chain5 the heading is fixed at 270 deg, so those unwinds come from the hive
bearing sweeping as the robot translates - path geometry the real robot shares; WHEN they land relative to
a Shoot step depends on path timing, which this tool cannot predict. Step 2's lost aim rides on the sim's
overshoot (run 2 held aim at the same step). Either way **a fixed 1.5 s Shoot step has no margin for an
unwind (~1.0-1.35 s each).** Whether the intake actually fed during the open windows is not in this log
(its `intake=` field is the auto's Intake event flag, not the motor) - operator observation pending.

**Fourth run, pre-aim + Shoot waits for the shot** (680b33c4; on blocks, no pieces, 12.42 -> 11.93 V; RED):

| | run 3 (live aim, fixed 1.5 s) | run 4 (pre-aim, wait for shot) |
|---|---|---|
| unwinds in the auto | 12 | **5** (2 at path starts while pre-aiming, in transit; 3 live) |
| Shoot steps that fired | 2/4 | **3/4** - start +0.29 s, POINT_3 +0.39 s (was never), POINT_7 +0.45 s |
| gate open per firing step | 1.18 / 1.23 s | 1.19 / 1.20 / 1.21 s (SHOOT_OPEN_TIME 1.2 s) |
| POINT_5 Shoot step | never (1.5 s) | **never - ran to the 3.0 s cap**, RESET twice |

POINT_5 is the wrap geometry the host model flagged (hive 179.9 deg robot-relative, 0.1 deg margin) plus the
sim's hold drift (x 55-75 in during the step): no aiming code can fix it, the shooting heading has to change
(model: P5 at 180 deg -> 98 deg margin; BLUE 0 deg). Until then this step costs the full 3.0 s cap instead of
1.5 s. The live unwind at 40.84 s came when the follower called chain6 done 11 in short of POINT_6 (sim
path-end error) and live tracking resumed. n = 1 run per arm on a model chassis: indicative, not a measurement
of the real auto.

**Fifth run, new paths** (78a32da3: chain2 split, POINT_5 180 deg, POINT_3 0 deg; BLUE mirrored; on blocks, no
pieces; RED 12.22 -> 11.75 V, BLUE 11.81 -> 11.59 V):

| | run 4 RED (old paths) | run 5 RED | run 5 BLUE (first ever) |
|---|---|---|---|
| Shoot steps that fired | 3/4 | **4/4** | **4/4** |
| gate first open | +0.29 / +0.39 / never / +0.45 s | +0.27 / +0.51 / +0.51 / +0.38 s | +0.28 / +0.56 / +0.43 / +0.41 s |
| gate open per step | 1.19-1.21 s | 1.20-1.25 s | 1.20-1.24 s |
| unwinds | 5 (3 live) | **3, all pre-aim in transit** | 4 (3 pre-aim, 1 live at a sim path end) |
| chain2 (corner loop) | 0.77 s | 1.10 s | 1.71 s |
| Done at | 57.4 s | 49.7 s | 49.7 s |

POINT_5 now fires (+0.51 s) - the heading change, as the host turret model predicted (98 deg margin). The
loop is driven instead of skipped. Every Shoot step's gate-open time sits at SHOOT_OPEN_TIME (1.2 s) and
no step hit the 3 s cap. Path timing and "Done at" remain sim-only (the visualizer estimates 26.9 / 27.0 s).
Not checked by any of this: whether chain3's new heading still lets the scoopula collect.

## 2026-11-15 — Foresight Tuner ported into Swerve Bring-Up (FORESIGHT mode), for the 2x2

**New diagnostic tooling (rule 9).** The Foresight Tuner was deleted with Sloth (cf5dcee9); CLAUDE.md said it
lived on branch `autotune-sloth`, which does not exist (fixed, c7e15a79). Its stages are ported, fits and
result names unchanged, into `diagnostics/swerve/ForesightBoxTuner.java`, run by Bring-Up's new FORESIGHT mode
on a fresh follower (shipped pods by default), driven by `tools/swervetune/foresightbox.py` (floor + box gates;
results to `runs/foresight_box_*`, importable with `robot.py foresight --partial`).

Ported: heading ID (kP), heading braking, forward/strafe deceleration, forward/strafe translational (kP
primary/secondary, coast/brake kV), forward/strafe braking. NOT ported: max velocity (full power over 48 in -
needs the long strip). Changes against the tuner, all for the box:
- linear stages drive toward the side of their axis with more room and end the drive on room (predicted coast /
  brake distance + 3 in), since the fence cannot clamp a coasting robot; leaving the fence aborts the stage;
- DECEL lowers its 30 in/s target and TRANSLATIONAL its 0.4 power / 1.2 s to fit the room, and say so - the
  translational K is then identified at lower power (linear motor model assumed);
- BRAKE's heading hold clamps +/-0.3 (the tuner's [-0.3, +1.0] looked like a typo).

Host check (ForesightBoxTunerCheck.java, simulated swerve, 30 x 30 in box): heading kP 8.04 vs 8.33 true, decel
34.9 / 35.7 vs 35, K 79.7 / 79.9 vs 80, tau 0.174 / 0.181 vs 0.18; no excursion outside the box. Not yet run on
the robot. These are identification numbers - pedrocheck (TUNING_TASK.md stage 6) still decides.

**First run on the robot, 2026-11-15** (tiles, 2x2 border fence 47.16 in, robot centred facing up, shipped pods
= candidate F, battery 13.3 -> 12.9 V). Data: tools/swervetune/runs/foresight_box.jsonl; candidate set
runs/foresight_box_candidateA.txt. Two tool fixes on the way, both before any data was kept:
TRANSLATIONAL no longer scales itself down to power 0.2 (the drive sits in a static deadband there - heading
spun 0.16 rad/s at 0.2 vs 2.6 at 0.4), it runs the given power and FAILS if the room cuts it before a plateau
(5 % last-15 vs previous-15); foresightbox recentres linear stages to the end of their axis (hold points must be
6 in + half-diagonal inside the fence: ~19 in of run instead of 14.5 from the middle).

**[CORRECTED below - not established] Pinpoint heading froze a third time** right after an app restart (91.1944 deg on 20 of 20 reads over 4 s, rate
exactly 0.0); resetImu cleared it (reads flicker in the 4th decimal again) and the fence was re-armed on the
operator's pose (robot unmoved since a closed-loop return to it, within 0.2 in / 1.2 deg). Still unguarded in
TeleOp and the autos.

| constant | now (placeholder) | measured | n | note |
|---|---|---|---|---|
| heading kP | 1.20 | **7.72** (6.53, 7.89, 8.73) | 3 | K 6.45-6.61 rad/s per power; tau 0.127-0.173 s carries the spread |
| heading braking lin / quad | 0 / 0 | **0.0505 / 0.0199** | 1 x 12 trials | 78-102 deg to stop from 7.6-7.9 rad/s; power 0.2 barely turns |
| natural decel fwd | 40 | tuner mean dv/dt 101.6 (SD 8.5); **v^2/2d 88.0 (SD 15.3)** | 3 | coast with pods released, from 22-24 in/s |
| natural decel strafe | 40 | 85.1 (SD 6.6); **63.8 (SD 10.4)** | 3 | |
| fwd translational kP prim / sec | 0.26 / 0.26 (kD 0.025) | **0.496 / 0.183** | 2 | K 42.2 / 45.3 in/s per power at 0.35; the 0.4 run was cut by room |
| strafe translational kP prim / sec | 0.26 / 0.26 | **0.399 / 0.148** | 2 | K 38.8 / 53.6 |
| coast / brake kV | 1/73.9 | 1/K = 0.0228 / 0.0194 | 2 | NOT recommended: v ~ 94 (power - 0.19) from 0.35 vs 0.4 runs (2 points) - a single kV is speed-dependent |
| fwd / strafe braking | 0 / 1/80 | 0.0203 / 0.0016; 0.0001 / 0.0027 | 1 x 5 trials | NOT recommended: <= 31 in/s, same-speed trials stop 0.83 vs 3.11 in (the tuner's brake is never weaker than the heading hold, so heading noise brakes) |
| max velocity | 73.9 | not measured (deadband model hints ~76) | - | needs the long strip |

Why distance-based decel: Foresight's coast model is v^2 = v0^2 - 2ad (a distance average); the tuner's
mean dv/dt is a time average and comes out higher when the robot slows hardest at the end (released pods
scrubbing), i.e. it would predict shorter coasts than happen. **Import trap:** Constants derives
forward/strafeBrakeQuadratic as 1/(2 * naturalDeceleration), so importing 88 / 64 would halve the predicted
stopping distance - the braking change held back above. Candidate A must pin those at 1/(2*40).
Not imported, nothing shipped. Next: candidate A through pedrocheck (stage 6) against the current constants.

## 2026-11-15 — "Violent jolts" under automatic motion: the follower hold's kD (fixed); a pod snap on stop (open)

Operator: the robot jolts violently sometimes when moved automatically (follower holds, Foresight stages), never
in TeleOp, and never during heading moves - only translating. Tiles, 2x2 border fence, 13.0 -> 12.8 V.

**Tool change (rule 9):** PodRecorder gained 10 columns appended after the follower block (existing columns keep
their positions): p0..p3_dpwr (drive power commanded - x volts = motor voltage), dcf/dcs/dct (the DrivePowers the
vendored Swerve was handed), bvx/bvy/bw. FORESIGHT is now recorded like FOLLOW. joltcapture.py / joltplot.py
(motor voltage per pod, battery, command, speed/accel, pod targets, dt). Bring-Up bug found on the way: FORESIGHT
read the pose on the 5 Hz idle path (staircase speed in the recordings), so its fence check and room planning ran
up to 200 ms stale - now every loop (poseInUse).

**1. Hold oscillation - FIXED (translationalKD 0.025 -> 0, shipped).** In every follower hold the forward/strafe
command spiked to 4-8x full scale and flipped sign every 1-2 loops; drive motors swung +10 V / -12 V every 20-50
ms; battery sagged to 11.0-11.4 V. Foresight's hold drives from a PROJECTED pose (pose + v^2 brake
displacement), which jumps with velocity noise every loop; the carried-over Pedro 2 kD differentiates it.
A/B through the real follower (pedroPidf td=), n=10 per arm, randomised interleaved, two holds per unit:

| | kD 0.025 (shipped) | kD 0 |
|---|---|---|
| drive-power reversals (> 0.3, sign change) per unit | 77.7 (SD 15.6) | 2.3 (SD 3.0) |
| peak command | 5.1 | 1.28 |
| min battery V | 11.36 | 11.73 |
| hold end error | 0.22 in | 0.34 in |

Difference 75.4 reversals, 95% CI [64.8, 86.0], Welch t = 15.0. Data: runs/hold_kd_ab.jsonl, runs/*holdkd_*.
TeleOp never runs the hold (follower.manual), which is why it never jolted. Every auto ends each path in this
hold, so this was in every auto. Not yet verified on the deployed build.

**2. Pod snap on stop - OPEN (vendored Swerve).** In the BRAKE stage recordings (runs/*jolt_brake_1409*): the
translation command drops under the Swerve epsilon (0.05) while a small heading correction stays above the
rotation epsilon (0.015, a RUCKUS patch); every pod is then steered to its rotation tangent within one loop
(e.g. all at 270 deg -> 20/70/110/160 deg) - the demand slew limiter skips jumps the pod resolves by flipping -
while the robot is still rolling at 21-23 in/s; speed 23 -> 0 in one step. Spins never show it (the pods are
already on the tangents) - matching the operator's report. Seen in the Foresight BRAKE stage by design of its
brake command; can also occur at a path end. Candidate vendored fix: while translation was active within the last
~0.3 s, treat a sub-0.05 rotation as zero for steering (hold the travel direction), so heading corrections at
rest still act. Needs its own test; not changed.

Also [CORRECTED below - not established]: Pinpoint heading froze a 4th time after an app restart (85.2568 deg, 20/20 reads); resetImu cleared it.

**Verified on the deployed build** (9559cf8a installed, gainsShipped true, no pedroPidf override - exactly what an
auto runs; 12.89 -> 12.83 V): 6 two-hold units, drive-power reversals 1, 1, 3, 0, 0, 0 (mean 0.8, was 77.7),
peak command 1.12-1.13, hold end error 0.10-0.28 in. Data: runs/*holdinstalled_*, runs/hold_kd_ab.jsonl
(arm installed_kd0). [CORRECTED below - not established] Heading froze a 5th time on that app restart (88.7488 deg, 20/20) - every restart today.

## 2026-11-15 — Pinpoint "freeze" #3-#5 were not freezes; IMU reset guard at init (shipped)

**Correction.** The three "freezes" logged today (after app restarts: 91.19, 85.26, 88.75 deg) were called on 20
bit-identical heading reads at rest. That test cannot tell a frozen Pinpoint from a still one: with the new
`/state` pinpointDev block, right after the next app restart the device reported READY, loop 846-863 us (1.17 kHz),
firmware v4, and the raw heading changed only in the 7th digit between runs of 2-7 identical reads; a 93 deg
spin then tracked (89.1 -> -177.9 deg) with no reset. So today's resets were unnecessary (harmless), and the
claims above are withdrawn. The two TeleOp freezes from earlier sessions (heading exactly 0.0, rate 0.0, while
DRIVING) remain real and unexplained.

Driver reading (SDK 12.0 GoBildaPinpointDriver): on a read it judges corrupt (NaN, a jump past the threshold,
or loopTime < 1 on v1/v2 firmware) it keeps the OLD value and flags FAULT_BAD_READ - so a new driver object after
an app restart that is fed bad reads holds its initial 0.0, which fits the old freezes (exactly 0.0, rate 0.0).
Not confirmed; the pinpointDev block (status, loopTime, version, encoders, raw heading, same-read count) is there
to catch the next one.

**Guard (shipped: helpers/PinpointGuard, tele/TeleOp, auto/Red|BlueSoloAuto - hand-wired):** a freeze cannot be
detected at rest, so at init - the robot must stand still anyway - every competition OpMode resets position +
IMU (resetPosAndIMU, the reset that cleared the old freezes), waits for READY (>= 0.35 s, 2 s timeout), then
re-applies its pose (TeleOp: the auto's stored pose; autos: START_STEP, again in start()). TeleOp also rumbles
gamepad 1 and shows a WARNING when the heading does not change by a single bit for 0.6 s while the driver turns
the stick past 0.4. The shared PinpointConfig keeps resetMode NONE: Bring-Up rebuilds its follower mid-session.
Verified: TeleOp INIT and the auto's init (through Auto Dry Run RED) both logged "PinpointGuard: OK (IMU reset,
READY after 0.35 s)". Not yet exercised against a real freeze.

Also measured on the way: heading ID without a reset gave K 4.02 rad/s per power, kP 13.8 (vs K 6.45-6.61 in the
three runs earlier today) - the turning plant is not stationary; pedrocheck, not one run, has to decide kP.

## 2026-11-15 — Pod snap on stop: small-turn roll-out tested - it stops the snap, not the jolt (vendored, default OFF)

Hypothesis (from the jolt recordings): a stop with a small heading correction still running sends every pod to
its rotation tangent while the chassis rolls, and that scrubs it to the violent stop. Vendored Swerve patch:
within 0.3 s of translation, a turn input under 0.05 is treated as zero (pods released holding the travel
direction). A/B on the Foresight BRAKE stage (rollout_ab.py; setMixer rollout=0 vs 0.3; 3 runs x 5 trials per
arm, randomised interleaved; tiles, 2x2 fence, 12.8 V). Stops inside the patch's condition (|translation| < 0.05,
0.015 <= |turn| < 0.05):

| | roll-out off (n = 9) | roll-out on (n = 6) |
|---|---|---|
| pod rotation in the 0.3 s after the stop | 13-100 deg, mean 59 | 1-25 deg, mean 11 |
| turn-servo power, same window | ~0.14 | ~0.03 |
| peak deceleration 0-0.3 s | mean 183 in/s^2 | mean 199 in/s^2 |
| speed at +0.3 s | 0.1-1.4 in/s | 0-0.3 in/s |
| stopping distance | 0.6-4.6 in | 0.5-4.4 in |

The patch does what it says, but the stop is no gentler: the robot stops from ~27 in/s within 0.3 s in both arms.
The abruptness is the rebuilt drivetrain's own coast deceleration (85-100 in/s^2 measured today, 40 on the old
pods), not the pod snap - hypothesis refuted. (The ctgt column could not show this: a released pod still records
its demand; pod rotation came from the wheel angle.) Patch kept in the fork, **default 0 = upstream behaviour**,
switchable for later (setMixer rollout=); RUCKUS_PATCHES.md row added. The operator's violent jolts were the hold
oscillation (kD, fixed above).

## 2026-11-15 — Foresight heading controller shipped (candidate B): turns stop overshooting

Candidate A (the whole box-tuner set) through pedrocheck, one smoke block vs shipped: turns transformed
(overshoot 9.1 / 20.3 -> 0.1 / 0.8 deg, residual ~4 -> 0.01-0.1 deg) but holds failed residual (0.57-0.86 vs
0.25-0.32 in): its near-target translational kP 0.18 (< 0.26 shipped) cannot push through the ~0.19-power drive
deadband. So candidate B = shipped translational/decel/braking + candidate A's heading only (kP 7.7173, heading
braking 0.0505 / 0.0199).

pedrocheck suite shipped vs candB, --only hold,turn --step 4, 5 blocks randomised interleaved, tiles, 2x2 fence,
12.78 -> 12.53 V (label pc2x2_candB; hold+x refused in blocks 3-4 - the robot had drifted ~1 in toward +x, the
hold point fell outside the fence margin, nothing moved):

| | shipped | candB |
|---|---|---|
| turn overshoot +90 / -90 deg | 9.46 +/- 6.5 / 8.03 +/- 5.9 | 0.50 +/- 0.36 / 0.67 +/- 0.79 |
| turn residual deg | 3.71 +/- 2.1 / 4.48 +/- 3.5 | 0.16 +/- 0.15 / 0.20 +/- 0.22 |
| turn settle to 2 deg | mostly never | 1.04 / 1.05 s |
| heading residual in holds | 1.42-1.43 deg | 0.12-0.24 deg |
| hold position residual | 0.32-0.34 in | 0.34-0.38 in (inside the CIs) |

Shipped (2be016e0, robot.py foresight --partial --write). **Acceptance on the deployed build** (gainsShipped true,
no overrides, 3 blocks, 12.69 -> 12.48 V, label pc2x2_candB_accept): turn overshoot 0.17 / 1.43 deg, residual
0.47 / 0.40 deg, settle 1.11 / 1.12 s (3 of 6 turns over the 1.0 s guess-threshold: 1.33, 1.56, 1.03 s); holds
residual 0.34-0.41 in (1 of 12 at 0.527 > 0.5), heading 0.10-0.25 deg. FORESIGHT_MEASURED stays false: max
velocity, the forward/strafe braking model and kV are not measured. Open: translational gains that beat the drive
deadband near the target (candidate A's 0.18 secondary is too weak; 0.26 leaves 0.3-0.4 in).

## 2026-11-15 — Near-target translational gain (2x2): a higher secondary kP fixes the hold stall; NOT shipped

**Why holds stop 0.3-0.4 in short.** Every acceptance hold (pc2x2_candB_accept, 12 holds) ends with the robot
stopped (speed 0.00 in/s) while the follower still commands 0.06-0.14 = 0.26 x the remaining error, and no pod
above 0.18 power: it is stalled under the drive breakaway (~0.19 power, box tuner), whose band at kP 0.26 is
+/-0.19/0.26 = +/-0.7 in. Follower.hold(pose) is unscaled (holdPointTranslationalScaling 0.45 applies only to
hold(pose, true), which nothing here calls). The follower also brakes on a projected pose (pose + v|v| x
1/(2 x 40)); the box measured 88 / 64 in/s^2 of coast-down, so that projection is ~2x too far and the robot starts
braking early (held back: the braking model needs path speeds, the long strip).

**A/B** (`pedrocheck suite --only hold --step 4`, arms in current_runs/arms_hold_secondary.json: shipped vs
secondary kP tfs=tss 0.40 vs 0.55, primary 0.26 above 2.5 in unchanged). 8 blocks randomised interleaved, 32 holds
per arm, 0 refused, tiles, 2x2 border fence 47.16 in, fresh battery 13.55 -> 12.79 V, label pc2x2_hold2nd. Scored
by tools/swervetune/holdstall.py (new, diagnostic, read-only: re-reads the archived recordings). Diff vs shipped,
95% CI, Welch:

| | shipped 0.26 | 0.40 | 0.55 |
|---|---|---|---|
| hold residual (mean \|err\|, last 0.5 s) | 0.324 +/- 0.041 in | 0.198: -0.127 [-0.173, -0.080] p=1e-6 | 0.109: -0.216 [-0.262, -0.170] p=2e-12 |
| stopped short along the step | 0.192 in | 0.115: -0.077 [-0.153, -0.001] | 0.064: -0.128 [-0.194, -0.063] |
| settle to 1 in | 0.836 s | 0.732: -0.104 [-0.183, -0.025] | 0.684: -0.151 [-0.215, -0.087] |
| overshoot / zero crossings at the target | 0.026 in / 0.19 | n.s. / n.s. | n.s. / n.s. |
| push reversals per hold (wheel push vector > 90 deg, \|change\| > 0.3) | 2.9 | 6.6: +3.7 [+1.5, +5.9] p=0.001 | 7.1: +4.2 [+1.7, +6.6] p=0.001 |
| pod azimuth travel per hold, 4 pods | 1151 deg | 1283: +133 [-53, +318] n.s. | 1406: +255 [+97, +414] p=0.002 |

Smoke block before it (11.9-12.1 V, n=4/arm): kP 0.85 reached 0.07 in but doubled pod travel (1714 deg) and
reversals (11.8), and was dropped. The operator saw the pods "move a LOT" on the left/right holds during it.

**What the extra reversals are:** 225 of 226 (0.55 arm) happen at a pod FLIP, 0.25-0.67 s into the hold, robot at
~9 in/s - the braking moment, when the projected pose crosses the target and the pods flip to push back. Same size
as shipped (median |power| after 0.28 vs 0.25, braking clamped at maxBrakingPower 0.2 on the translation part);
none in the final creep, no hunting. Not the kD jolt (77 full-voltage reversals per hold pair), but more pods
flipping at every stop. A heading-gain explanation for the pod travel was checked and is not supported: the
shipped-vs-candB A/B earlier today (heading kP 1.2 vs 7.7, same holds) had pod travel 789 vs 823 deg, p=0.63.

**Not shipped, because the benefit does not reach the robot's real use:**
- Autos do not wait for the hold: Foresight drops busy 100 ms after a hold starts (timeoutConstraint 100 ms,
  calculateHold), and RedSoloAuto/BlueSoloAuto advance on !isBusy(), so a 0.3 in stall costs no auto time.
- Shooting holds: the turret aims from the measured pose, so 0.3 in of position error is not an aim error.
- Below 2.5 in is where almost all of a PATH's cross-track correction happens, so the secondary kP is really the
  path-following gain - untestable in the 2x2 (hold points must sit 18.7 in inside the fence).
- The pre-stated guard (no significant rise in reversals or pod travel) failed for both arms.
0.40 and 0.55 go to the long-strip session as path candidates (pedrocheck on the real .pp paths: cross-track error,
end error, pod travel), next to the braking-model change that may remove the early braking at its source.

## 2026-11-15 — TeleOp steering: schedule floor 0.30, demand-rate feed-forward, slew 300 (shipped)

Operator: "our driver needs a responsive and smooth drivetrain ... make it go in a circle while maintaining heading
... record the pod headings for any swerving or wiggling". Two new diagnostic tests on the 2x2 (tiles, border fence
47.16 in, 0.25 drive power, ~7 in/s):
- **stick circle** (`circle.py`, `drive rot=`): the OpMode rotates the stick vector every loop, a perfectly smooth
  sweep, 110 deg/s (and 220); scored per pod: |error|, lag, wheel reversals per lap (0.5 deg hysteresis, steerqual),
  jitter, flips.
- **stick-flick square** (`square.py`): forward/left/back/right, 0.7 s legs, 90 deg flicks; per flick and pod: time
  to within 10 deg (and 5) of the new direction and staying, overshoot, the mixer's own demand time.

**The bench heading hold confounded the first circles.** With it on, the turn command was 0.14 RMS (peaks 0.34)
against 0.25 translation at ~1.3 Hz: each pod's demand was tilted tens of degrees, opposite pods opposite ways
(pod errors uncorrelated, r ~ 0), and heading rocked +/-3 deg. TeleOp has no heading hold, so every scored run has
it off (`setHeadingHold value=false`); the demand then reverses 0 times per lap and heading wanders ~1-3 deg RMS.

**What was wrong (shipped, circle 110 deg/s):** 14 deg mean / 30 deg p95 pod error, ~100 ms behind the stick, ~25
wheel reversals per lap - a stall-and-jump staircase. Inside the 20 deg error gate a rolling pod's whole turn output
is scaled by the schedule floor 0.24 (sized 2026-08-15 on the OLD 214 deg/s pods); ~15 deg of error then commands
~0.04, under breakaway, so the pod waits until 20+ deg and jumps. Floor 1.0 (no schedule) instead oscillates at
~3 Hz, the August finding: the feedback cannot both track a turning demand and stay stable with ~60 ms of loop
dead time (estimated from the flick traces: the wheel starts ~60 ms after the demand).

**Fix 1 - demand-rate feed-forward** (vendored CoaxialPod, commits 53a2d34c / 4590e6b2): turn power +=
kV x d(target)/dt, so the pod gets the sweep's push up front and feedback only corrects the rest.
A/Bs (circle 110 deg/s, randomised interleaved, 12.2-12.7 V):

| arm (vs shipped) | n | mean err | lag | reversals/lap |
|---|---|---|---|---|
| shipped (floor 0.24, no FF) | 5-6 | 13.7-14.2 deg | 100-105 ms | 23-26 |
| floor 0.50, no FF | 6 | 13.8 | 121 | 14.3 |
| floor 0.50, kV 0.04 | 5 | 4.8 | 9 | 16.3 |
| floor 0.35, kV 0.04 | 5 | 4.6 | 28 | 7.2 |
| floor 0.30, kV 0.05 | 5 | 4.5 | 3 | 7.2 |
kV 0.08 leads (pods ahead of the stick by ~90 ms). The left pods (2 LF, 3 LB) wobble 3-6x more than the right at
equal gains, whether flipped or not (encoder noise equal, ~0.05 deg) - a lower floor calms them.

**Fix 2 - flicks.** Swerve's demand slew limiter ramps every flick at exactly the slew rate; fed forward, that ramp
overshot 27-41 deg (vs 17-20 shipped). Pod kD 0.04-0.06 damped it but moved the rolling wobble to ~4.6 Hz on the
left pods (circle reversals: kD 0.015 / 0.03 / 0.045 / 0.06 -> 9.6 / 16 / 25 / 28 per lap; feed-forward tau 15 vs
50 ms n.s.). So the feed-forward now ignores demand rates at or above a threshold (`demandRateMaxRadS`, vendored,
tunable): just under the slew rate, flicks go to the feedback and sweeps keep the feed-forward. Square, n=4/arm:
kD 0.015 + slew 300 + FF only below 240 deg/s: align 373 vs 563 ms shipped (-190 [-229, -151]), overshoot 8.6 vs
16.6 deg (-8.0 [-11.6, -4.4]); slew 400 / threshold 320 overshot 22.8.

**Shipped** (Constants TUNED VALUES, applied by `createSwerve` -> `applySteeringStatics`, so TeleOp and the autos
both run them; pod kD unchanged 0.015): `turnScheduleFloor 0.30`, `turnDemandRateKV 0.045`, `turnDemandRateTauS
0.015`, `turnDemandRateMaxDegS 240`, `demandSlewDegPerSec 300` (was 214). Re-applying them on every drivetrain
build also fixes a leak: these statics live for the whole app process, so a bench session's live values used to
carry into the next TeleOp.

**Acceptance on the deployed build** (gainsShipped + staticsShipped true, installed vs the old values set live,
n=3/arm interleaved, 11.94-12.00 V loaded, label accept2_*): circle 110 deg/s error 14.0 -> 5.1 deg (-8.9
[-10.3, -7.5]), p95 29.7 -> 10.7, lag 105 -> 27 ms, reversals 27.5 -> 6.9 per lap (-20.6 [-25.9, -15.4]), jitter
5.1 -> 3.3 deg; flicks align 538 -> 358 ms (-179 [-262, -97]), settle to 5 deg 578 -> 377 ms, overshoot 19.4 -> 12.8
(n.s.). The 220 deg/s circle on the installed build: 6.8 deg mean error, 5.5 reversals/lap (n=3).

**Rule 9 / harness note:** the first acceptance (label accept_*) is void for the 'old' arm. circle.py / square.py
applied arm settings before the re-centring move, whose pedroStart builds a drivetrain - which now re-applies the
shipped statics - so 'old' ran the new values (it scored 6.0 deg / 10.6 reversals). Arms now go after the move
(`apply_arm`). Every A/B before this build is unaffected (createSwerve did not touch the statics then). The same
applies to live tuning on the bench: any pedroStart resets slew / floor / feed-forward to Constants.

Open: TeleOp's own loop rate is unmeasured (the bench ran 52-65 Hz here); the driver's real stick inputs, faster
than 240 deg/s sweeps, and full-speed driving were not tested - the 2x2 caps speed at ~7-10 in/s. Pedro paths
(autos) now run these statics too; validate on the long strip with the rest of Foresight.

## 2026-11-16 — Competition field: corner reset point measured, max velocity 52 in/s (shipped), field-fence lessons

Surface: competition field tiles. Battery 13.1 V fresh -> 11.3 V loaded at the end. Field fence (`fieldFence`, walls +
6 keep-outs from field/biobuzz_field.json) armed for every move - its first use on a real field.

**Incident (my error):** "move the robot to the center" was the centre of the AUDIENCE wall, not of the field. I armed the
fence at (70.75, 70.75); the robot was really at (70.75, 8.6), back on the wall, so a 40 in "down" follower line pushed
it into the wall for 15 s at 0.4 (no damage reported). The fence could not help - it trusted the wrong pose. A stall
guard (stop if < 1 in of progress after 2-3 s) was added to the move script used for the rest of the session. Lesson:
confirm the operator's placement against the field drawing before arming a fence.

**Corner reset point (SystemConstants.TeleOp.RELOCALIZE_X/Y):** pose set at the audience-wall centre seam (x 70.75 by
eye, y 8.561 = back on the wall), follower to (22, 22), turned to face the red wall, pushed into the corner by the
operator 3 times: x 10.71 / 11.10 / 11.11 = **10.97 +/- 0.23 in**, y 9.67 / 9.57 / 9.84 = **9.69 +/- 0.14 in**; heading
178.6-178.9 (Pinpoint drift since the set, the walls hold 180). Was 8.561 / 8.561 (square-robot guess). Accuracy ~0.5
in from the start placement. NOTE: this change was committed inside f055ee7e (the flywheel-hold commit), whose message
does not mention it.

**Max velocity (Constants, f677b2e8):** open-loop forward runs, recorded (runs/*vmax_*): 0.47 power -> ~27.5 in/s,
0.80 -> 44, 1.00 -> ~50-53 (loaded battery). 73.9 -> **52**, strafe assumed equal (not measured), coast/brake kV 1/52.
Route review at 52 in/s: red 27.7 s, blue 27.8 s of 30. Acceptance on the deployed build, follower lines +/-45 in at
power 1.0 along the audience band (n=4, accept_vmax_lines): end 0.09-0.53 in short, no overshoot, 3.3-4.9 s, no clamp.

**Field-fence lessons:** keep-out margins grow with speed (4 in + 0.3 s x v + 6 in taper, robot = half-diagonal circle),
so a lane 4 in from a flower (x = 22 along the red wall) or 10 in from a hive rail (the centre corridor) clamps fast
runs; the per-axis clamp then ROTATES the command and all four pods swing together (the operator's "pods swinging
like crazy" at 0.8 in the corridor: pod errors -38 -> +38 deg in unison, i.e. the demand moved). The clean fast lane is
the audience band along x at y ~25-29 (rails 13 in away, flower only near x 90). Bench heading hold must be off for
open-loop runs (it fought the drive, +/-0.2-0.4 turn). Pods held by `pidStepAll` hunt while the robot is pushed
by hand - release them (`pidHold`) for pushes.

**Not done (needs more field time):** forward/strafe braking and natural decel at speed (still 40 in/s^2 - the follower
brakes early, peak only 23-30 in/s on 45 in lines), strafe max velocity, the real auto paths through pedrocheck (they
run into corners the fence will not allow). FORESIGHT_MEASURED stays false. Flywheel 35:21 target (6d190513) held at
24:20 (f055ee7e) until the gears are swapped.

## 2026-11-16 — Lunch field session: high-speed floor leg (vendored + shipped); heading / translational kP NOT changed

Competition field, FTC tiles, fresh battery (12.97 V rest, 11.6-12.1 V loaded in runs). Fence armed from the
audience-wall centre seam (70.75, 8.56, 90 deg). Lane: audience band, y 26, heading 0. All runs in
current_runs/{lanerun,linecheck,turncheck}.jsonl and runs/*lunch_*.

**Open-loop pods at speed: the gain schedule starves them.** At 0.8 drive (lanerun.py, bench DRIVE, heading hold off)
the robot yawed 25-45 deg per 0.9 s run. The trace shows why: with a fixed 90 deg demand, pods parked 5-19 deg off
target (p0 +19, p1 -15, p2 -6, p3 -9) commanding **0.02-0.05 power** - inside the 20 deg error gate the schedule scales the
whole turn effort (kS included) to the 0.30 floor, which lands inside the CR servo's own deadband. Non-parallel wheels ->
scrub + yaw. So the planned A/B (floor 0.30 vs 0.20 vs 0.12) was the wrong direction and was replaced by 0.30 / 0.60 /
1.00 (schedule off). Scored on unclamped samples > 20 in/s, per-run steady offset (|mean err| per pod) and oscillation
(sd err per pod), n=8 runs/arm, randomised blocks:

| floor | steady offset | oscillation sd | pod |err| mean |
|---|---|---|---|
| 0.30 (shipped) | 5.5 deg | 7.9 | 8.8 |
| 0.60 | 2.9 | 9.2 | 7.6 |
| 1.00 | 4.0 | 14.9 | 12.5 |

Circle guard (circle.py 110 deg/s, 0.25 drive, n=2/arm): a flat 0.60 **doubled the wiggle, 23.3 vs 10.0 reversals/lap**
(sd 0.5 / 0.15), err 6.4 vs 5.1 deg. So the floor now depends on drive power: **vendored CoaxialPod
setScheduleHighSpeed (RUCKUS PATCH)** ramps the floor from SCHED_FLOOR to SCHED_FLOOR_HIGH between 0.35 and 0.70 drive
power; off (upstream-equivalent single floor) while floorHigh <= 0. Bench: setPidf floorhi / fhs / fhf; pedroReset and
staticsShipped include it. A/B with the new code path, f30 vs fh60 (0.30 -> 0.60), n=6/arm: offset -1.7 deg [-4.7, +1.2]
p=0.24, yaw -6.7 deg/s p=0.14. Pooled with the flat-0.60 arm (identical floor at 0.8 drive), n=14/arm: **steady offset
3.0 vs 5.1 deg, -2.1 [-3.5, -0.8], p=0.003** (Mann-Whitney 0.002); oscillation not significantly changed.
**Shipped:** Constants.turnScheduleFloorHigh 0.60, start 0.35, full 0.70 (applySteeringStatics).

Acceptance on the installed build: circles unchanged by construction (0.25 drive is below the ramp) and measured so -
110 deg/s err 5.5 deg, 6.6 wiggle/lap; 220 deg/s err 8.3, 6.2/lap (n=2 each). Lane n=6: steady offset **4.7 deg** - did NOT
reproduce the A/B's 3.0. Weaker evidence (not interleaved, loaded 11.67 V vs ~11.9 in the A/B) but recorded as it is:
the high-speed benefit is real in interleaved data and unconfirmed on the shipped build.

**Follower heading kP (linecheck.py, +/-45 in lines at power 1.0, n=3/arm):** 7.7 -> 2.5 cut turn rms 0.45 -> 0.15 and line
duration 3.86 -> 2.52 s (ranges do not overlap), heading deviation unchanged (7.7 / 7.6 deg). But +/-90 deg in-place turns
(follower hold, n=3-6/arm): settle to +/-2 deg 1.00 s at 7.7 vs **1.46 at 2.5 (+0.46 [+0.21, +0.72], p=0.01)** and 2.5
parks 1.5 deg short; 4.0 +0.23 s p=0.09; 5.5 +0.05 s p=0.66 but lines only 3.46 -> 3.15 s (n=4). **Kept 7.7**: the plan's
guard (turns must not get worse) rules out 2.5, and 5.5's line gain is marginal.

**Follower lines: pods at ~35 deg mean error on every arm** - a 2-4 Hz lateral / heading limit cycle (robot lateral
velocity +/-15 in/s, heading +/-5-8 deg, follower strafe command flipping sign at ~+/-0.8 every few samples) with only
0.02-1.8 in of cross-track error. Translational kP 0.26 -> 0.13 / 0.08 (hp 2.5): pod err 35.8 -> 31.5 / 29.9, faster
lines (2.6 -> 2.0 s) but end error 0.33 -> 0.71 in and small overshoots; not the root, **kept 0.26**. Bench FOLLOW loop
was 30-31 ms (~33 Hz) - phase lag at this frequency is a candidate cause, unmeasured. Open, autos only (TeleOp's manual
drive has no translational or heading feedback).

**Tooling (diagnostic):** lanerun.py --hold (bench heading hold on; it bang-bangs +/-1.0 at speed - do not use) and
--yline (follower re-centre onto the lane before every run - without it open-loop yaw walks the robot out of the lane in
1-2 runs). Not run: braking / decel at speed (cut for time).

## 2026-11-16 — Straight driving: the yaw is the drive wheels, not the steering; TeleOp heading hold added (shipped, UNTUNED)

Fresh battery (12.99 V rest), corner start (9.7, 8.56, 90), lane y 26. Open-loop 0.8-power straight runs on the
shipped build (lanerun straight_base, n=6): heading change -20 to +13 deg per 0.8 s run, 7-44 deg/s at speed, the
direction varying run to run - and in one run +26 deg/s with all four pods within 4 deg of target. Pod angle errors do
not predict the yaw (signs and sizes uncorrelated across runs), so the cause is on the drive side (wheel speed /
traction differences, worst accelerating from rest); the battery was not it. No steering gain can fix that, so the fix
is closed loop:

**TeleOp heading hold (SystemConstants.TeleOp.HEADING_HOLD, shipped).** Left stick translating + right stick centred ->
hold the heading captured when the turn stick was released (after HOLD_CAPTURE_S 0.25 s, or once |w| < 30 deg/s, so a
turn coasts out and is kept). Shooter lock's controller shape (P on heading projected by AIM_BRAKE_*), HOLD_KP 3.0,
HOLD_MAX_TURN 0.5. Off at rest, under the turn stick and in SHOOTER LOCK; cleared on relocalize. GUESSED gains.

First driver session (TeleOp, ~4.5 min, pose from /field/state at ~13 Hz, current_runs/teleop_hold_drive1.jsonl):
10 fast segments (> 20 in/s, >= 0.5 s). Heading in them swings +/-8-20 deg at ~2 Hz with small net change (-0.5,
+2.9, -2.0, +3.4, +2.8 deg in five of them) - a fishtail, not a drift. Without stick inputs logged it cannot be
separated from the driver's own steering; the driver's report was not received before the robot went offline. If it
is the hold, the next step is HOLD_KP 3.0 -> 1.5 (Dashboard-editable live). The follower shows the same 2-4 Hz
heading/lateral cycle at heading kP 7.7 (lunch entry above), so heading feedback + pod lag at speed is the suspect.

## 2026-11-17 — Driver verdict on the 2026-11-16 tune, and a measurement path that is TeleOp itself (diagnostic + shipped hooks)

**Verdict.** The driver drove TeleOp after the 2026-11-16 tune and called it horrible (a loose screw was found and
fixed afterwards; it does not explain the result away). That result belongs to the tune: it was searched and
accepted on proxies - Bring-Up open-loop shuttles, follower lines, stick circles - none of which is the driver's
path (tele/TeleOp: sticks -> heading hold -> follower.manual -> vendored Swerve), and the TeleOp heading hold
shipped untuned without a driver check. Next session (3x3 tiles, field centre): full retune from scratch,
measured on the path below, accepted only after the driver drives it.

**New tool (rule 9 - changes what is measured).** `diagnostics/swerve/TeleDriveProbe` ("TeleOp Drive Probe")
extends `tele/TeleOp` and swaps only the stick source (`driveSticks()`), so the hold, the lock / field-centric
stages, `follower.manual`, the mixer and the pods are the competition code. Differences from TeleOp: mechanisms and
driver 2 off (no flywheel in drive tests - so its loop rate is NOT the competition loop rate); a stop area
(`arm hx hy`, centred on the pose at arm time) that ZEROES outward translation inside 4 in + 0.30 s x speed of an
edge - a stop, not a taper, and scorers report the zeroed share as `guard_frac`; `TeleLoopProbe`'s recorder and
`/swerve/state`; live setters (`podGains`, `pulse`, `sched`, `ff`, `mixer`, `setTele`) and `reset` to the build's
values. Host: `tools/swervetune/teleprobe.py` (line / strafe / diag / reverse / flick / circle / spin / arc stick
scripts, randomised interleaved arms, Welch 95% CI vs the first arm). Metrics: stick-vs-actual direction error,
yaw drift with no turn asked, pod error and wheel reversals, top speed and t90, stop time / distance, reversal
time, spin rate, loop_hz_true. Scorer checked offline on the lunch run `straight_base_shipped_b4_out`
(yaw rate rms 25.4 deg/s, matching the 7-44 deg/s reported then); NOT yet run on the robot.

**Shipped change, behaviour-neutral.** `tele/TeleOp` gains the hooks the probe needs: `driveSticks()` (returns
gamepad1), `mechanismsEnabled()` (true), read-only accessors, and the mechanism / telemetry blocks moved into
private methods unchanged. Competition behaviour is identical; not re-measured.

No number from the old proxies carries over as a baseline: every metric above starts from its first TeleDriveProbe run.

## 2026-11-17 — 3x3 retune through TeleOp's drive code: motor swap found, heading hold kP 3.0 -> 1.0 (shipped), schedule floor kept

All numbers from `TeleDriveProbe` (tele/TeleOp's own drive code, mechanisms off), 3x3 tiles in the field centre,
stop area +/-22 in, randomised interleaved arms, `teleprobe.py` scoring; volts 11.6-12.9 under load (logged per run in
`tools/swervetune/current_runs/teleprobe.jsonl`, labels base1, fixchk*, ab1-ab5). Loop rates here are the PROBE's
(no mechanisms) - 48-75 Hz true - and are not the competition loop rate.

1. **Encoder pairing re-checked** (operator: "servo encoders got switched"). Bring-Up wire scan: ss0->se0, ss1->se2,
   ss2->se3, ss3->se1, all unambiguous, servo directions as shipped - identical to Constants.podEncoderNames, no change.
2. **FL / FR drive motors were swapped** (operator; config fixed in bd924818). Before: CCW spin 15-17 deg/s with every
   drive power the same sign, strafes 148-163 deg off the stick with 45-77 deg yaw. After: spins 121-130 deg/s both ways.
3. **Shipped heading hold (kP 3.0, max 0.5) was the fishtail.** Lines n=4/arm: yaw-rate rms 57.0 vs 16.3 deg/s with the
   hold off (-40.7 [-52.3, -29.1], p<0.001), pod error 19.7 vs 4.0 deg, top speed 25 vs 33 in/s; strafes the same.
   A ~3 Hz limit cycle: hold turn +/-0.27-0.50, heading +/-3-5 deg, pods lagging their swung targets by up to 50 deg.
4. **Hold off, the robot still drifts 3-14 deg per 0.8 s run, sign varying run to run** - pods park 4-8 deg off target at
   speed (non-parallel wheels). Schedule floor 0.7 / 1.0 did NOT fix it (n.s.) and floor 1.0 added wheel reversals
   (strafe +8.4/s, p=0.004): floor stays 0.30 / high 0.60.
5. **Hold kP 1.0, max turn 0.25 (shipped).** Pooled over ab3+ab4 at 0.6 power, n=30 vs 31 runs: drift 3.6 vs 7.7 deg
   (-4.1 [-5.8, -2.4], p<0.001), wobble 19.2 vs 17.6 deg/s (n.s.), reversals +1.6/s (p=0.010). kP 1.8 oscillates
   (lines +27 deg/s, p=0.025); a 0.15 s brake lookahead oscillates at kP 0.6 and 1.0 (strafe +24 / +39 deg/s, p<=0.001).
   At 0.9 power (ab5, n=4-5): reversals drift 4.0 vs 9.4 deg (p=0.038), strafe/line drift n.s., line wobble +14 deg/s
   (p=0.089) - the hold is weaker evidence at full speed. HEADING_HOLD is Dashboard-live; the driver decides.
6. **X-lock at rest: no significant effect** on any metric except reversal t90 -0.05 s with it off (p=0.036). Kept.

Not done this session: pod turn gains (kP/kD/kS) - pods held 4-8 deg mean while driving with the hold off, not
retuned; full-power (1.0) runs do not fit the 3x3 (50+ in/s, 7-10 in stops trip the stop area). Driver check pending.

## 2026-11-18 — Heading hold kP 1.0 -> 0.6 (shipped); pod kS flat. (Pod kP/kD rows VOID - see correction)

All from `TeleDriveProbe` (TeleOp's own drive code, mechanisms off) on FTC tiles, a **2x6-tile strip** (stop area
+/-56 x +/-13 in, robot facing along the strip), 0.6 power, 1.0 s runs, randomised interleaved arms, fresh battery
13.17 V resting at start, 11.9-12.8 V under load across the session (per run in `current_runs/teleprobe.jsonl`,
labels podsmoke, kp1, kskd1, wob1, k06c, k06post). Tests: line, strafe, diag, spin.

**Tool change (diagnostic, declared per rule 9):** `teleprobe.py` start points in a strip (one side > 1.5x the
other) turn the robot first so each test's first direction lies along the long axis (strafe at -90 deg, diag at
-45 deg); in a square area nothing changes (heading 0). Spin runs now score pod error over the whole stick-on
segment (it was NaN: no translation, so no "moving" samples). Earlier datasets keep their meaning.

> **CORRECTION (same day): items 1 and 2's kP / kD arms never changed the gains.** TeleDriveProbe's `podGains`
> called `PIDFController.setCoefficients(new ...)`, and `run()` re-reads the supplier every loop (Constants builds
> the controller with the PIDFCoefficients object as its own supplier; `SwerveBringUp.java:809` already warns), so
> the new object was discarded one loop later. Every kp/kd/ki arm in kp1, kskd1, ki1, ki2 ran the SHIPPED gains -
> which is why they were flat. kS arms were real (`setStaticFriction`). Found because kI 15 changed nothing: pod
> power vs error had the same slope as kI 0. Fixed (set the fields in place), verified on the robot (kpcheck:
> power-vs-error slope 0.31-0.36 /rad at kP 0.20, 0.98-1.03 at kP 0.80). Re-run as kp2 onward.

1. **[VOID - gains unchanged, see correction] Pod kP 0.30 / 0.38 / 0.50 (kp1, n=4 per test per arm): no consistent effect.** kP 0.30 cut spin pod error
   (-2.9 [-4.3, -1.6] deg, p=0.003) but raised line direction error p95 (+5.9 [+0.7, +11.0], p=0.032); 0.50 better
   nowhere. Kept 0.38.
2. **kS 0.05 / 0.11 (real), kD 0 / 0.03 (VOID, unchanged) vs shipped (kskd1, n=3-4 per test per arm): flat.** Pooled pod error 6.45-6.72 deg
   on every arm (shipped 6.51); "kD 0 diag speed -1.9 in/s (p=0.025)" was shipped-vs-shipped noise. Kept kS 0.08.
3. **The variation that matters is bimodal.** 40% of line/strafe runs (all arms ran the shipped pod gains) fell
   into the ~3 Hz heading limit cycle (yaw-rate rms 30-58 vs 3-15 deg/s, pod error 10-16 vs 3-4 deg, hold turn
   swinging +/-0.15-0.2, pods +/-30 deg); diag runs never did (0/14). Yaw-rate sign / lag checked: Pinpoint omega
   tracks d(heading)/dt (r = 0.82-0.93, ~2 samples late), so the hold's lookahead is not inverted.
4. **Mixer demand slew 300 (shipped) vs 550 vs off (wob1): did not fix it** - 4/12 wobbly runs each vs 6/12.
   Unchanged.
5. **Heading hold kP 0.6 fixes it (shipped).** wob1 + k06c pooled, line + strafe: wobbly runs 1/31 vs 14/32 at kP
   1.0 (Fisher p=0.0002); yaw-rate rms 11.2 vs 25.7 deg/s (-14.5 [-21.0, -8.1], p=0.0001); pod error 4.0 vs 7.4 deg
   (p=0.001); heading drift per run 3.6 vs 4.4 deg (-0.8 [-1.7, +0.1], p=0.07 - not worse); stop distance +0.9 in
   (p=0.001, the wobbling runs were slower). The confirmation alone (k06c): 0/19 vs 8/20 (p=0.003).
   Yesterday kP 1.0 wobbled 3/11 at 11.6-12.7 V: the cycle is bistable and likelier on a fresh battery, so a
   tired-battery A/B under-reports it.
6. **Post-deploy check (k06post, shipped build, n=5 per test): 0/10 wobbly**, yaw-rate rms 10.0 deg/s mean (max
   24.5), pod error 4.1 deg, 12.0 V under load.

Open: full power (0.9-1.0) not re-checked at kP 0.6; driver check on the real TeleOp pending; wheel reversals
still 7-10 per pod-second at 0.5 deg hysteresis while driving straight (pulsed-approach / kS dither - not moved by
any gain tested).

## 2026-11-18 (afternoon) — Full power keeps hold kP 0.6; pod turnKP 0.38 -> 0.28 for steering jitter (shipped); kI rejected

Same strip and tool as the morning entry, after the `podGains` fix (7ad78c28). Volts under load 11.4-12.4.

1. **Full power (fs1, power 1.0, 0.8 s, run-in 80 in; valid - hold gains go through setTele).** n=13-15 per arm
   pooled over line / strafe / reverse, 10 of 57 runs touched the stop area (reported with and without; same
   verdict): kP 1.0 wobbles more than 0.6 (yaw-rate rms 36.3 vs 23.0 deg/s, +13.3 [+2.9, +23.8], p=0.018; pod error
   +2.8 deg, p=0.009). kP 0.4 and hold OFF are not different from 0.6 (p >= 0.45). At full power the hold buys no
   drift (7.5 vs 7.8 deg per run). ~25% of full-power runs wobble on EVERY arm, hold off included: that residue is
   not the hold.
2. **Cruise jitter metric** (`/tmp` analysis, to be folded into teleprobe): after speed first reaches 90% of its
   p90, until stick release - per-pod |err| mean, err sd (oscillation), and wheel reversals > 2 deg per pod-second.
   The scorer's rev_per_s (0.5 deg hysteresis, whole run) is dominated by the 0.3 s ring after the X-lock -> drive
   step and by sub-degree dither.
3. **What the jitter is** (k06post traces): drive-wheel scrub pushes pods off target at cruise, and the scheduled
   turn effort (x ~0.5 at 0.6 drive power) is near the stiction level: one pod sits 5 deg off on -0.048 power,
   another saw-tooths 1 -> 9 deg at ~10 Hz.
4. **kP (kp2 + kp28ki pooled, n=30 vs 32, line/strafe/diag):** kP 0.28 vs 0.38: cruise reversals > 2 deg 1.04 vs
   2.30 /pod-s (-1.26 [-2.26, -0.26], p=0.015); oscillation sd 0.54 +/- 0.32 vs 1.21 +/- 1.72 deg (p=0.036; median
   0.45 vs 0.61, Mann-Whitney p=0.11); cost: cruise offset +0.90 deg [+0.38, +1.42] (p=0.001), spin pod error ~+2
   deg (p~0.1 in each round); whole-run pod error unchanged (+0.07, p=0.83). kP 0.20 parks +1.6 deg (p=0.0004);
   kP 0.80 oscillates (yaw 47-54 deg/s, pod error 14 deg, kpcheck n=2). **Shipped turnKPPerPod 0.28** (TeleOp
   path only - NOT re-validated on pedrocheck / the Follower, so autos carry an unmeasured change).
5. **kI rejected.** kP 0.28 + kI 6 (band 15 deg, reset 3 deg, limit 0.10): offset -0.80 deg (p=0.015) but
   oscillation median 1.67 vs 0.48 deg (Mann-Whitney p=0.0003), 3 wobbly runs vs 1. The integral fights the scrub
   with a lag and becomes the oscillator.
6. **Post-deploy (kp28post, shipped build, robot reports kp 0.28; n=12 translation + 4 spin, 0 stop-area, 12.46 V
   rest / 11.7 V under load):** cruise oscillation sd 0.59 +/- 0.36 deg, offset 2.86 deg, reversals > 2 deg 1.71
   /pod-s, yaw-rate rms 10.1 deg/s - in line with the kP 0.28 arm (0.54 / 3.12 / 1.04). Driver check pending.
