# Ruckus PedroPathing Patch Notes

This folder is vendored into the main `RuckusBioBuzz` git repository. It is not a nested git checkout or submodule.

## Upstream

Official source:

```bash
https://github.com/Pedro-Pathing/PedroPathing.git
```

The main repository should have a remote named `pedro-upstream`:

```bash
git remote add pedro-upstream https://github.com/Pedro-Pathing/PedroPathing.git
git remote set-url --push pedro-upstream DISABLED
```

The vendored Gradle metadata currently reports PedroPathing version **`3.0.0`** — upstream tag
`v3.0.0` (`bb27610`, 2026-09-09), vendored verbatim in `1af2762` on `pedro-3.0.0-migration`, with
the patches below re-applied on top. Everything from "Complete inventory vs. v2.1.2" down is the
history of the 2.1.2 fork these were ported from.

## Local patches on v3.0.0

Search the tree for `RUCKUS PATCH`. Each entry names the upstream 3.0.0 behaviour it changes.

| Where | Patch | Upstream 3.0.0 behaviour it changes |
|---|---|---|
| `build.gradle.kts`, `core/`, `revhub/build.gradle.kts` | Dokka, Spotless and `kotlin("android")` removed | Dokka broke variant selection across the composite build (2026-08-09 entry below) |
| `revhub/build.gradle.kts` | `compileSdk` 35 → 34 | platform 35 is not installed; TeamCode compiles at 34 |
| `gradle/libs.versions.toml` | FTC 11.2.0 → 12.0.0 (was 11.1.0 until 2026-10-05), AGP 8.7.3 → 8.7.0, `annotations` 23.0.0 restored | compile against exactly what the app ships (a mismatch makes Android Studio flag Pinpoint enum types in `Constants` as errors); `:telemetry` needs `annotations` |
| `settings.gradle.kts`, `telemetry/**` | local `:telemetry` module kept | not part of upstream. Its only TeamCode user was the 2.x `Tuning.java`, now gone: removable |
| `core/.../control/PIDFController.java` (+ `PIDFCoefficients`, `PIDFCoefficientSupplier`) | the 2.1.2 classes, **retained** with their integral patches, now `implements Controller` | upstream deleted them. v3's `PIDController` has no integral clamp, band or reset threshold, and skips D when dt ≤ 1 ms. Every pod turn gain was measured on this arithmetic |
| `revhub/.../CoaxialPod.java` | every 2.1.2 patch (kS tanh, pulsed approach + stall ladder, gain scheduling, flip hysteresis, output slew, derivative-on-measurement, median filter off, instrumentation); `getTurnPIDF()`; `setMotor/ServoCachingThreshold` write through to the config | stock v3 pod: sign-relay kF only, no hysteresis, no schedule |
| ″ | **released pods end the pulse episode** (2026-10-10): while `ignoreAngleChanges`, no pending pulse or coast, stall history cleared, fire power back to base; `wasLastMovePulsed()` reads false | none upstream (v3 has no pulses) — fixes an interaction between two patches: the ladder kept "firing" into a servo forced to 0, counted every phantom pulse as stalled, and reached the 0.13 cap in ~0.5 s of release, so the first real pulse after it fired at the cap. Host sim only, untested on hardware |
| ″ | `PULSE_MAX_POWER` comment corrected (2026-10-10): it justified 0.13 by "podMinV max 0.106", which is pod 3's analog-encoder minimum *voltage*; measured breakaway is 0.050 on tiles. Value unchanged | none — comment only |
| `revhub/.../Swerve.java` | 2.1.2 mixer: rotation ε 0.015, smoothstep taper, demand slew 214 °/s, X-lock engage delay 0.35 s, single voltage read | stock v3 mixer |
| ″ | **`stop()` re-sends each pod its last demand** (2026-10-10): `move(lastCommandedTheta[i], 0, true)`; a pod never commanded gets its current heading converted to wheel space (`wheelThetaOf`, the inverse of `adjustThetaForEncoder`) | **upstream bug:** v3 passes `pod.getAngle()` — the encoder-frame angle — where `move()` takes a wheel-space theta. Released, so the servo ignores it, but the pod records it as its demand (recorder `tgt`), re-decides the flip state against it (which breaks the next 80–100° tie) and feeds it to the turn PID's error history. `Follower` calls `stop()` every IDLE loop. Untested on hardware |
| ″ | **released pods get exactly 0 drive** (2026-10-10): `release ? 0 : magnitude × avgScaling` | restores stock v3's (and pre-taper 2.1.2's) zero drive on released pods. With the taper the sub-epsilon magnitude reached the motor while the servo was released, and the drive cache (0.05) forces a write only at exactly 0, so a stale nonzero power (host sim: up to 0.085 on entry, 0.034 held for a whole 1 s IGNORE_ANGLE_CHANGES hold) sat on unsteered wheels until X-lock 0.35 s later, or indefinitely under IGNORE_ANGLE_CHANGES. Untested on hardware |
| ″ + `CoaxialPod.java` | **demand slew limiter skips only real flips** (2026-10-10): a > 90° demand jump bypasses the limiter only if the pod will change its flip state for it — `CoaxialPod.wouldChangeFlip()`, sharing `decideFlip()` with `move()`, hysteresis and memory included; other `SwervePod`s keep the quarter-turn rule | none upstream (v3 has neither limiter nor hysteresis) — fixes an interaction between two patches: the limiter assumed every > 90° jump flips, the pod flips only past 100° and holds its state inside 80–100°, so a 90–100° jump on an unflipped pod was neither limited nor flipped: a ~95° step at full authority (host sim: servo −1.0 on the jump step, now a 214 °/s ramp). Untested on hardware |
| `revhub/.../CoaxialPod.java` | **demand-rate feed-forward** (2026-11-15): `raw += kV x d(target)/dt`, the rate taken from the unflipped target, low-passed (tau, default 0.05 s), a single-call jump over 900 deg/s counted as a step (0); added after the gain schedule, off while released. Robot-wide runtime `setDemandRateFeedforward(kV, tau)` (bench: `setPidf ffkv= fftau=`). **Default OFF (kV 0) = upstream output, bit for bit** | **changes upstream 3.0.0 behaviour when kV > 0:** the stock turn loop is feedback only, so a pod tracking a turning demand trails it by the error that makes the push - stick circle on the 2x2 (circle.py, 110 deg/s sweep, 0.25 drive, no heading hold): 10-15 deg mean error, 46-127 ms lag, a 3 Hz staircase/limit cycle. `demandRateMaxRadS` (default 900 deg/s, tunable): rates at or above it count as steps, so the shipped 240 deg/s leaves Swerve's slew-limited FLICK ramps (300 deg/s) to the feedback - fed forward they overshot 27-41 deg. **Shipped 2026-11-15** from Constants (kV 0.045, tau 0.015 s, max 240 deg/s; applySteeringStatics) |
| `revhub/.../CoaxialPod.java` | **high-speed floor leg** (2026-11-16): the gain schedule's floor ramps from `SCHED_FLOOR` to `SCHED_FLOOR_HIGH` between `SCHED_HIGH_START` (0.35) and `SCHED_HIGH_FULL` (0.70) of drive power. Robot-wide runtime `setScheduleHighSpeed(floorHigh, start, full)` (bench: `setPidf floorhi= fhs= fhf=`). **Default OFF (floorHigh <= 0) = the single floor** | the schedule itself is a Ruckus patch (upstream 3.0.0 has no gain schedule); this changes it only above 0.35 drive. Why: at 0.8 drive a 0.30 floor left pods 5-19 deg off target commanding 0.02-0.05 power (servo deadband), wheels non-parallel, robot yawing 25-45 deg/run; 0.60 at speed cut the steady offset 5.1 -> 3.0 deg (p=0.003, n=14/arm) while a flat 0.60 doubled slow-circle wiggle. **Shipped 2026-11-16** from Constants (0.60, 0.35, 0.70; applySteeringStatics). FINDINGS 2026-11-16 lunch |
| `revhub/.../Swerve.java` | **small-turn roll-out** (2026-11-15): within `smallTurnRolloutS` (0.3 s, runtime `setSmallTurnRolloutS`, 0 = off) of the translation input last being ≥ ε (0.05), a turn input under ε but over the rotation ε (0.015) is treated as zero — the pods are released holding the travel direction, as the X_LOCK engage delay already does for zero input; afterwards small turns steer again (heading hold at rest unchanged) | **changes upstream 3.0.0 behaviour:** any turn over the rotation ε steers the pods at once. With a heading correction still running at a stop, every pod went to its rotation tangent within one loop (the slew limiter skips flips) while the chassis rolled at 21–23 in/s and was scrubbed to a dead stop (jolt recordings 2026-11-15, Foresight BRAKE stage). `computePodPowers` (maxScaling) is not changed — stateless. **Default OFF (0):** A/B 2026-11-15 (rollout_ab.py, BRAKE stage) - pod rotation after a stop 59 -> 11 deg, but the stop is no gentler (peak decel 183 vs 199 in/s^2, n = 9/6); kept switchable, not shipped on |
| ″ | **rotation geometry kept at 2.1.2's** `atan2(x, −y)` with CCW-positive turn | **upstream bug:** v3's `atan2(−x, −y)` with negated turn mirrors the rotation field for x-forward/y-left offsets (CCW 0.5: v3 LF 133.5°, true tangent 226.5°) — not a rigid rotation |
| ″ | pod directions carried beside the `Vector2D`s | **upstream bug:** `Vector2D` is cartesian, so the zero-magnitude X-lock vector hands every pod θ = 0 |
| ″ | θ normalised to [0, 2π) before `move()` | 2.1.2's polar `Vector` did; keeps the recorder's `tgt` column comparable |
| ″ | **translation priority** for manual drive: `setTranslationPriority(f)` cuts rotation only as far as needed to keep fraction `f` of the translation when a pod saturates; applied only by `drive(powers, true)`, default 0 = off, TeleOp sets 0.75 | stock v3 divides every pod by the largest, so full stick + full turn keeps 54% of translation on this chassis |
| ″ | zero-power behaviour written only on change | v3 re-writes it on every `drive()` call |
| ″ | `stop()` invalidates that cache (`appliedZeroPowerBehavior = null`) after floating the pods | none in effect — it keeps the cache from changing upstream's behaviour. v3's `stop(brake)` writes BRAKE/FLOAT and then calls `setToFloat()` on every pod anyway (so the pods end FLOAT even for `stop(true)`); without the reset the cache would still say BRAKE and the next manual `drive()` would never re-apply it. Cost: in `Follower` IDLE (`stop()` every loop) the zero-power write pattern is upstream's, 8 writes per loop |
| ″ | `computePodPowers` (what `maxScaling` plans with) uses the mixer's geometry, rotation ε 0.015 and smoothstep taper; X-lock omitted (it only yields zero vectors) | v3 deletes sub-ε translation and rotation (shared ε 0.05) and uses its mirrored rotation geometry. **Known limitation, not fixed:** `maxScaling` solves `|a + t·b| = 1` assuming pod vectors are linear in `DrivePowers`, but ε handling is applied to `current` and `delta` separately. A sub-ε `delta` is shrunk by the taper (< 1), so `b` is understated and λ over-granted — Foresight hands a small heading delta (< 0.015) or translation delta (< 0.05) more than fits, and `applyDrive`'s normalise-by-largest then scales every term down to pay for it. The excess pod magnitude is bounded by ε (0.015 rotation, 0.05 translation). Upstream is worse in the same direction (it zeroes such deltas, `b = 0`, λ = 1 regardless of saturation, and does so up to 0.05 for rotation too). Candidate fix: build `computePodPowers` without ε handling for `maxScaling`. Not applied — it changes Foresight's allocation on every path and nothing here can validate it off the robot. Foresight is not measured on this robot yet (`Constants.FORESIGHT_MEASURED = false`), so if it is taken, take it before that tuning run, not after |
| `core/.../utils/Control.java`, `algorithm/ForesightPowerAllocator.java` | `clampBrakingPower(Vector2D, Vector2D, max)` — direction-preserving | **upstream regression:** v3 clamps braking per axis again, the bug the 2.1.2 CustomDrivetrain patch fixed (rotates the command 30–45°) |
| `core/.../paths/curves/Curve.java` | `pathCompletion` = `1 − remaining/length` | **upstream bug:** default returned the fraction *remaining*; `Line`/`CompoundCurve` inherit it, so `linear`/`longLinear` headings ran backwards, `PiecewiseInterpolator` switched in reverse, `Foresight.completion()` counted down |
| `core/.../follower/Follower.java` | **update() timing** (2026-11-24): `static long[] lastUpdateNs` = ns in `localizer.update()`, the algorithm's calculate, and `drivetrain.drive/stop` on the last `update()`; read by the autos' loop-time log | none - timing only, no control path reads it |
| `revhub/.../Swerve.java` | **`lastCommand()`** (2026-11-24): returns the last forward / strafe / rotation `applyDrive` used (plain fields) | none - read-only accessor; `debug()` has the same values but reads every drive motor's power off the hub |

**Equivalence evidence for the pod + mixer port** (2026-09-10, host-side, not on the robot): the
2.1.2 fork at `a6edf00` and this tree were driven side by side through identical inputs against
identical simulated plants (the robot's real zeros, voltage ranges, per-pod gains and pulse
settings), each in its own classloader, with `System.nanoTime()` replaced by a shared stepped
clock. 1200 steps — X-lock, snap-out, diagonal + turn, spin, both epsilon walls, strafe flips,
flip hysteresis, 400 steps of random driving with releases, pulsed crawl, park — at 8, 10 and
25 ms: pod demand agrees to 2e-13°, servo power to 6e-15, write counts identical. The one real
difference is the demand direction at exactly zero input: 2.1.2's polar `Vector` derived it from
IEEE signed zeros (front pods 180°, back pods 0°), v3 reports 0°. Pods are released either way
and the loop error is identical; only the remembered flip state differs. The vector braking clamp
matched a transcription of the 2.1.2 function on 10⁶ random cases to 8.9e-16.

## History: the 2.1.2 fork

**Base verified 2026-09-10:** the tree vendored in `9761b95` was fingerprinted against every
upstream commit since 2025-06 (line endings normalised — the raw checkout differs from upstream
in every file on CRLF alone). Closest tag is `v2.1.2` (`96df977`). The `pedro-upstream` remote
described above does **not** exist in this clone; the check used a scratch clone of upstream.

### Complete inventory vs. `v2.1.2` (as of `a6edf00`)

The dated entries below cover only some of these. Everything that differs from the tag:

| File | Change | Dated entry |
|---|---|---|
| `core/build.gradle.kts` | Dokka removed | 2026-08-09 |
| `core/.../control/PIDFController.java` | integral clamp, sign-change reset threshold, integral band, `updateErrorWithDerivative` | 2026-08-11, 08-12 |
| `core/.../drivetrain/CustomDrivetrain.java` | reverse-power clamp applied **along the direction of travel** instead of per axis (per-axis capping rotated the command 30–45° on swerve) | **none** |
| `core/.../follower/FollowerConstants.java` | `stuckTValue = 0.8` → `stuckTValueLow = 0.1`, `stuckTValueHigh = 0.8` (present since vendoring) | **none** |
| `ftc/.../drivetrains/CoaxialPod.java` | kS `tanh` term, pulsed approach, derivative-on-measurement, instrumentation getters | 2026-08-12 |
| ″ | **flip hysteresis** ±10° around the 90° flip boundary | **none** |
| ″ | **turn gain scheduling** — drive-power and pod-velocity legs, 20° error gate, floor 0.24, runtime setters `setScheduleTuning` / `setScheduleRamp` | **none** |
| ″ | measured pod velocity (seam-wrapped), `setTurnSlewPerUpdate` output slew limit, pulse stall-escalation ladder + drive-power gate, `getLastTargetWheelRad`, `getLastDrivePower`, median-of-3 encoder filter (compiled **off**) | **none** |
| `ftc/.../drivetrains/Swerve.java` | rotation epsilon 0.015 (translation keeps 0.05); smoothstep **epsilon taper**; per-pod **demand slew limit** 214 °/s (skipped past 90°); **X_LOCK engage delay** 0.35 s; voltage-compensation fix (stock discarded the `times()` result) | **none** |
| `gradle/libs.versions.toml` | AGP plugin 8.7.3 → 8.7.0 (match the root build) | **none** |
| `settings.gradle.kts`, `telemetry/**` | `:telemetry` module added — `SelectableOpMode` etc., used by `TeamCode/.../Tuning.java`. Not part of the upstream PedroPathing repo | **none** |
| `README.md`, `gradlew` (mode bit) | cosmetic | — |

## Local Patches

### 2026-08-12 — Derivative on measurement, and a pulsed final approach, on `CoaxialPod`

- **Files changed:** `ftc/src/main/java/com/pedropathing/ftc/drivetrains/CoaxialPod.java`,
  `core/src/main/java/com/pedropathing/control/PIDFController.java`
- **Reason:** two attempts at the residual that a continuous loop cannot reach. Both are
  implemented, both default to off, and **both are currently negative results** — kept because they
  are cheap to re-test and expensive to re-derive.

  **`setDerivativeOnMeasurement(boolean)`** plus `PIDFController.updateErrorWithDerivative`.
  Differencing the error assumes a stationary setpoint; a pod's steps by up to 90° and jumps
  discontinuously at the ±180° flip, and at 120 Hz a 90° step alone drives `kD = 0.010` to 1.96 of
  output, saturating the clamp. Derivative-on-measurement removes that kick and is identical while
  the setpoint holds. It did **not** permit more damping: kD 0.040 rings 16×, 0.070 and 0.110 are
  unusable. The reason is structural — for a plant `1/(s(1+sT))` the PD zero cancels the lag pole at
  `kD = kP·T`, so with the measured T = 42 ms the useful range is already covered. A randomised grid
  confirmed the optimum sits at 1.7–2.4× that value, above it rather than below, because the 39 ms
  of *pure transport delay* needs additional lead that pole cancellation does not account for.

  **`setPulsedApproach(...)`.** The pod has no creep regime: static friction breaks to kinetic and it
  goes from stopped to tens of degrees per second with nothing between, so any continuous controller
  with the authority to correct a small error commits to several degrees of travel before feedback
  (39 ms transport + 42 ms velocity lag) can act. A bounded open-loop pulse sidesteps the stability
  question entirely. Where it works it is decisive — 11 of 16 pod-runs parked at ≤0.34° with
  peak-to-peak ≤0.34° and *exactly zero* holding power. It is blocked by the pulse quantum: at the
  SDK's default 20 ms servo PWM frame the shortest possible pulse travels 0.06–2.85° at 0.040 power,
  already more than the whole 1.0° error budget, and travel is not a repeatable function of duration
  (pod 0 moves 2.85° at 20 ms and 1.80° at 45 ms), so the scatter is intrinsic breakaway variability
  rather than something a calibration can remove. Halving the frame period halves the quantum but
  not necessarily the scatter, which is the thing to judge a retest on.

- **Upstream issue:** none filed.
- **How to remove:** search for `RUCKUS PATCH`. Both are inert at their defaults, so removal is
  deleting the setters, the two `if` blocks in `move()` and `updateErrorWithDerivative`.

### 2026-08-12 — Continuous static-friction term and instrumentation on `CoaxialPod`

- **Files changed:** `ftc/src/main/java/com/pedropathing/ftc/drivetrains/CoaxialPod.java`
- **Reason:** `kF` is not a feed-forward on a swerve pod. `move()` feeds
  `turnPID.updateFeedForwardInput(getTurnDirection(...))`, which is exactly ±1, so the F term is
  `±kF` — a relay, constant in magnitude and switching sign at the target, zeroed inside 2°.

  Measured on this drivetrain (robot on blocks, 12.9–13.3 V, `pidStepAll` 90°, n≥12 pod-runs each):

  - Open-loop breakaway power is **0.025–0.050** depending on pod and direction (measured by
    applying each power from rest on a staircase; see `tools/swervetune/phase2_plant.py`).
  - Shipped `kF = 0.005` is therefore **5–10× below breakaway** — the relay does nothing at all,
    and the pod parks where `kP·e` falls under breakaway, i.e. `0.035/0.300 = 0.117 rad ≈ 6.7°`.
    Measured residual was 3–6°, matching.
  - Raising `kF` to 0.035 (≈ breakaway) *does* fix the residual — |steady-state| fell 3.52° → 1.25°
    — but produced a mean of **15.8 error sign changes per step** and a **42.5° post-settle
    peak-to-peak**. That is the relay limit-cycling, and it is why the previous session's table
    records rings rising monotonically with kF (0.005→2.7, 0.011→4.3, 0.022→7.7, 0.035→13.7).

  So the fix is not more kF, it is a term with the same authority away from the target and no
  switching discontinuity at it. `setStaticFriction(power, bandRadians)` adds
  `power · tanh(error / band)` to the controller output. With `kS = 0.035`, `band = 2°`,
  `kP = 0.20`, `kD = 0.020`, `kF = 0`: |steady-state| **0.51° mean**, post-settle peak-to-peak
  0.34°, resting servo power RMS 0.0115 — below breakaway, so the pod is genuinely at rest.

  Also added, instrumentation only with no control effect: `getLastTurnPower()`,
  `getLastErrorRad()` and `wasLastMoveFlipped()`. Scoring a step needs the error the PID actually
  acted on — after the ±180° flip is resolved — and the power actually *written*, which output
  caching makes different from the controller's output. Reconstructing either outside the pod means
  duplicating `move()`'s wrap and flip logic and watching it drift.

  A no-op at `staticFrictionPower = 0`, which is the default and every stock configuration.

- **Upstream issue:** none filed. Worth proposing: the sign-only feed-forward is a genuine design
  problem for any friction-dominated servo axis, not just this robot.
- **How to remove:** if upstream adds a real static-friction term, drop `setStaticFriction` and use
  theirs. Search the file for `RUCKUS PATCH`. After removing, re-measure breakaway and confirm the
  replacement is continuous through zero error — a relay will limit-cycle here.

### 2026-08-12 — Make the integral term actually usable in `PIDFController`

- **Files changed:** `core/src/main/java/com/pedropathing/control/PIDFController.java`
- **Reason:** the 2026-08-11 patch below clears `errorIntegral` on any error sign change. Near the
  target the error sign flips on sensor noise every few loops, so the accumulator was being cleared
  continuously in exactly the situation it was added for. The measured encoder noise floor here is
  **σ = 0.042–0.054°** against a 0.1125° read granularity, so zero crossings at rest are constant.

  Two additions, both no-ops at their defaults:

  1. `setIntegralResetThreshold(t)` — a sign change only clears the accumulator if the error was
     larger than `t` on one side of the crossing. Default 0 reproduces the previous behaviour.
  2. `setIntegralBand(b)` — only accumulate while `|error| < b`, holding the accumulator at zero
     outside. Conditional integration: an integral meant to break stiction at the target should not
     wind up across a 90° slew where the proportional term already saturates the output. Default
     infinity reproduces the previous behaviour.

  Both are reachable on a pod through `CoaxialPod.setTurnIntegralSettings(...)`, added for this,
  since the controller is constructed inside that class.

- **Upstream issue:** none filed.
- **How to remove:** search for `RUCKUS PATCH`; the shared `accumulateIntegral()` helper contains
  all of the changed behaviour and reverts to the stock two lines at default settings.

### 2026-08-11 — Make the integral term safe to use in `PIDFController`

- **Files changed:** `core/src/main/java/com/pedropathing/control/PIDFController.java`
- **Reason:** The swerve pods park 2–4° short of target on carpet and stop dead. `CoaxialPod.move()`
  zeroes the feed-forward inside 2°, and P alone there is far below breakaway, so nothing remains to
  close the gap. Raising kF makes it worse — the pod breaks free, overshoots and stops on the far
  side. An integral term is the correct fix, but stock `PIDFController` could not carry one safely:
  `errorIntegral` accumulates without bound, is never clamped, and nothing calls `reset()` during
  normal operation, so any non-zero I eventually saturates the servo.

  Two changes, both no-ops while I is 0 — which is every stock Pedro config:

  1. `run()` clamps the integral **contribution** to `integralLimit` (default 0.25 of normalised
     output). Clamping the contribution rather than the raw sum keeps the bound meaningful for any I.
  2. `updateError()` and `updatePosition()` zero `errorIntegral` when the error changes sign.
     Accumulation from the approach is stale once the target is crossed, and carrying it through a
     swerve pod's 180° flip would drive hard the wrong way.

- **Upstream issue:** none filed. Arguably worth proposing upstream — unbounded integral with no
  reset path is a latent trap for anyone who sets I.
- **How to remove:** if upstream adds its own anti-windup, drop both patches and use theirs. Search
  the file for `RUCKUS PATCH`. Verify afterwards that a pod with I set does not creep or saturate
  when held off-target.

### 2026-08-09 — Remove Dokka from `:core` so its classes reach the APK

- **Files changed:** `core/build.gradle.kts`
- **Reason:** With Dokka applied, `:core` exposes consumable configurations whose attributes are
  `DGP~`-prefixed strings rather than Gradle's typed attributes. When the Android app consumed
  `:core` transitively (via `ftc`'s `api(project(":core"))`) across the composite build, Gradle
  selected the variant `dokkaHtmlPublicationPluginApiOnlyConsumable~internal` instead of
  `runtimeElements`. That variant carries the Dokka documentation-plugin classpath, not the code
  jar, so **no `com.pedropathing` core class was packaged into the APK** — verified with `dexdump`:
  `follower`, `geometry`, `control`, `math` were all absent while `com.pedropathing.ftc.*` was
  present.

  The symptom was a Robot Controller crash loop: Panels' Configurables plugin scans `@Configurable`
  classes at startup, and `getDeclaredFields()` on `Tuning` threw
  `NoClassDefFoundError: Lcom/pedropathing/follower/Follower;`, taking down the whole app process
  every ~40 seconds. That killed FTC Dashboard connections and made every Pedro OpMode unusable.

  Removed the `org.jetbrains.dokka` plugin, its `dokkaPlugin` dependency, the `dokkaJar` task and
  the `docs(dokkaJar)` deployer entry. Generated documentation has no value for a vendored copy.

- **Upstream issue:** none filed; this is a Dokka Gradle Plugin v2 variant-selection interaction
  with composite builds, not a PedroPathing source bug.
- **How to remove:** once DGP no longer publishes those configurations for `java-library` consumers
  (or if Pedro is consumed from Maven instead of `includeBuild`), restore the four removed pieces
  from upstream `core/build.gradle.kts`. After any change here, confirm with:

  ```bash
  ./gradlew :TeamCode:dependencyInsight --configuration debugRuntimeClasspath --dependency core
  ```

  The selected variant must be `runtimeElements`, not a `dokka*` one.

When local Pedro changes are added, list them here with:

- files changed
- reason for the patch
- upstream issue or PR if one exists
- how to remove the patch once upstream includes it

## Upstream Update Workflow

Fetch upstream tags and branches without touching the working tree:

```bash
git fetch pedro-upstream --tags
```

Review upstream changes before applying them:

```bash
git log --oneline v3.0.0..pedro-upstream/main
git diff --stat v3.0.0..pedro-upstream/main
```

Because Pedro is vendored under `third_party/PedroPathing`, do not merge `pedro-upstream/main` directly into the robot repo. Apply upstream changes into the vendored prefix from the recorded base tag, then compile TeamCode before committing.

Recommended update shape, replacing `main` with a newer tag if you want a tagged release instead:

```bash
git diff --binary v3.0.0..pedro-upstream/main -- . \
  | git apply --3way --directory=third_party/PedroPathing
```

That only works for incremental releases. 2.1.2 → 3.0.0 was a rewrite (165 files, modules
renamed): it was migrated by vendoring the new tag verbatim in its own commit, then re-applying
each patch as its own commit. Do the same for the next major version.

If upstream changes conflict with local patches, resolve the files in `third_party/PedroPathing`, update this note, then run the normal TeamCode Gradle compile. After a successful update, replace the base tag in this file with the new upstream tag or commit SHA.
