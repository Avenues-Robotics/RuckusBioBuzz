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
