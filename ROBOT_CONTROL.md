# Robot control runbook — connecting Claude to the swerve robot

How a Claude Code session connects to the physical robot, checks it is safe, drives it and
records data through the **Swerve Bring-Up** OpMode and its HTTP server. The same steps, in the
same order, every session.

**Precedence.** `CLAUDE.md` wins over this file. Rules 1, 2, 3, 6, 8 and 11 of `CLAUDE.md` §8,
and the deploy gate in rule 5, are the safety and evidence floor, and nothing here overrides
them. This file is only the *how*. If the code disagrees with this file, the code wins: fix this
file in the same commit.

Everything here is **diagnostic tooling** (`diagnostics/swerve/`, `tools/swervetune/`). Driving
through the bring-up OpMode uses the *tool's* saved gains (`/sdcard/FIRST/swerve_bringup_cal.txt`
on the hub), not `pedroPathing/Constants.java`. `DriveTeleOp` has no HTTP path, and nothing here
controls it.

---

## 0. Who does what

| You (Claude) can | Only the operator can |
|---|---|
| `./gradlew :TeamCode:assembleDebug`, then install it (`adb install -r`, relaunch the RC activity) | Confirm the robot is placed safely before an OpMode that moves on START |
| Read `/swerve/state` and pull `/swerve/rec.csv` | Power the robot and swap batteries |
| Send `/swerve/cmd` commands, once the gates in §3 pass | Put the robot on or off blocks, strap it down |
| Stop motion: `robot.py stop`, and `robot.py estop` as a last resort | Install / start by hand when adb or the dashboard can't reach the hub |
| Init + start OpModes with `ftcdash.py "<OpMode>"` (CLAUDE.md rule 5) | |
| Write `FINDINGS.md`, `RUN_STATE.md`, commits | Turn pods by hand, mark box corners, confirm the floor is clear |
| | Press **STOP** on the Driver Station. This is the real emergency stop |

`tools/swervetune/ftcdash.py` is a WebSocket client that inits, starts and stops OpModes through FTC
Dashboard. Since 2026-11-15 (CLAUDE.md rule 5) you install and start OpModes yourself - but never one
that moves on START (an auto) without the operator confirming the robot is placed for it, and
`robot.py estop` still uses it to stop.

Never call `GET /swerve/restart`: it restarts the whole Robot Controller app. Never call
`/swerve/config?write=1` or `?builtin=…`: they change the active hardware config. Both need the
operator to ask for them by name.

**One agent on the robot.** Do not fan robot work out to subagents or parallel workflows. There is
one `/state`, one recorder and one operator, and they can't be shared. Offline analysis can be
parallel; commands cannot.

---

## 1. Network

- The robot serves everything at **`http://192.168.43.1:8080/swerve`**, on the Control Hub's own
  WiFi (SSID `FTC-####`). The laptop running Claude Code must be joined to that network.
- **The hub's access point has no internet, and Claude Code needs internet.** The laptop needs a
  second route out, such as Ethernet, a USB-tethered phone or a second WiFi adapter, with
  `192.168.43.0/24` reachable over the robot WiFi. If the session goes silent right after the
  operator switches WiFi, this is why.
- Override the host with `SWERVE_HOST=ip:port` (read by `swervebench.py` and `robot.py`), for
  example for a phone Robot Controller.
- The hub WiFi **drops for 1–2 s several times an hour**. `swervebench._get` retries 6× at 0.75 s.
  `robot.py` retries reads but sends motion commands **once** (see §5).
- The web routes are registered when the **app** starts, not the OpMode. So there are three states:
  - Timeout or connection refused: the app is not running, or you are not on the network.
  - `{"live":false,…,"message":"Run the Swerve Bring-Up OpMode."}`: the app is up but the OpMode
    is not running, or not publishing within 1500 ms.
  - `live:true`: the OpMode is running. Check `started` next.

---

## 2. The front door: `tools/swervetune/robot.py`

Use this instead of hand-written `urllib`/`curl` snippets. Every rule below is built into it.
Standard library only; run it from `tools/swervetune/` (Python 3.12 on this laptop). It works the
same from PowerShell or the Bash tool.

```
python robot.py check                      # the gates; exit code says which failed
python robot.py state [--json]             # compact snapshot / raw JSON
python robot.py cmd ACTION k=v ...         # one command + what the OpMode said about it
                                           #   motion actions: + --confirmed-floor | --on-blocks
python robot.py drive --f 0.15 --sec 1.5 --confirmed-floor [--s S] [--t T] [--cap 0.30] [--rec LABEL]
python robot.py pull LABEL                 # recorder CSV -> runs/, prints loop_hz_true
python robot.py stop                       # drive 0,0,0 then stop -> IDLE (OpMode keeps running)
python robot.py estop                      # STOP the OpMode via FTC Dashboard (last resort)
```

| `check` exit code | Meaning | What you do |
|---|---|---|
| 0 | READY: live, started, pose ok, box armed, pose inside box | Proceed per §3 |
| 2 | Unreachable | Network (§1), or the app is restarting after a deploy. Ask the operator |
| 3 | `live=false` | OPS REQUEST: start `Swerve Bring-Up` |
| 4 | INIT, not started | OPS REQUEST: press START. **Motion commands in INIT are dropped, and `/cmd` still says ok** |
| 5 | No pose | Pinpoint not reading. Stop and report; no translation is possible |
| 6 | Box not armed, or pose outside it | OPS REQUEST: re-mark the box (§3.4) |
| 7 | `robot.py` refused, or the OpMode reported a failure | Read the printed message |

For bulk experiments, script against `swervebench.Bench` (`step_trial`, `set_pidf`, `rec_csv`,
`parse_csv`) or the existing drivers (`boxdrive.py`, `drivecapture.py`). **Grep the `Bench` API
before writing a script, and run one trial as a smoke test before a background batch.** A
background A/B once died on `b.step(...)`, which does not exist; the method is `b.step_trial`.

---

## 3. Session start — every time, in this order

### 3.1 Read state

`CLAUDE.md` → `RUN_STATE.md` (if present; it is a hypothesis, not a fact) → `ASSUMPTIONS.md`
→ the tail of `FINDINGS.md`.

### 3.2 `python robot.py check`

Record what it prints. **Do not trust `RUN_STATE.md` over this output.**

### 3.3 Pre-flight OPS REQUEST — one message, not drip-fed

**Deliver OPS REQUESTs as dashboard banners** when the operator is at the robot rather than the
laptop: `python tools/mechtune/mechtune.py ask "OPS REQUEST 3 - ..."` shows it on `/swerve` (and
`/tune`) with a beep, waits, and prints their reply (`ready 12.6 V`). The board is app-level, so
it works under Swerve Bring-Up. It only carries messages; every gate below still applies.

Send this before the **first** motion of the session. Repeat the starred items after every deploy
or battery swap.

```
OPS REQUEST <n> — pre-flight
1. Robot on: FTC tiles / blocks (say which). If blocks: chassis strapped to them,
   hands, cables and gamepad lead clear of all four wheels.
2. Battery strapped, leads clear of wheels. Reply with resting volts (want >= 12.5 V).
3.* Turn each pod by hand through its travel - report any bind, grind, play or debris.
4.* Start "Swerve Bring-Up" (Diagnostics) on the Driver Station and press START (not just INIT).
5.* Encoder liveness: with nothing commanded, turn each pod by hand one at a time and tell
   me which pod you are turning - I watch its angle in /state.
6.* If on tiles: 3 ft clear around the 51 x 46 in taped box. Box check (see below).
7. Dashboard tab: do NOT press "Drive with gamepad" while I am driving - only one sender.
8. Driver Station within reach, STOP visible.
Reply "ready" + volts + surface.
While you do that I'll be: <the offline work you will actually do>.
```

**Encoder liveness (item 5).** Poll `python robot.py state` while the operator turns each pod.
Every pod's `wheel=` must move. A pod whose angle holds still has a dead encoder. The CR loop then
sees a constant error and holds full command on that servo indefinitely, and the 5 Hz rail-current
total will not show it. **STOP: no steering commands until it is fixed.**

### 3.4 The safe-area box

Facts from the code (`SwerveBringUp.java`, `applyBoxLimit`, `loadBox`, frame check in the heading
read):

- It is saved on the hub in `/sdcard/FIRST/swerve_field_box.txt` together with a *witness pose*,
  and reloaded when the OpMode inits. **It can survive a redeploy.**
- It is **discarded automatically** in three cases. Read `box.valid` and `message` after every
  restart, because nothing else tells you:
  - At reload, the pose does not match the witness within **1 in and 2°** (or there is no
    witness). The witness is written when the box is armed and refreshed at the OpMode's
    `stop()` and on `setPose`, so this catches a Pinpoint power cycle, any auto/TeleOp that set
    a start pose or drove, any OpMode that builds a Pedro `Follower`, and a robot moved by hand.
    `message` carries the discarded box's coordinates.
  - Odometry jumps faster than the robot can move (it was picked up or dragged).
  - `resetImu`, `odoConfig` or `boxClear` is sent.
- It lives in the **dead-reckoned pose frame**. "Armed" does not mean "correct": yaw drift slides
  and rotates it. CLAUDE.md rule 6 still requires re-marking after a reflash or pose reset, and
  re-reading `/state` before motion. A box that reloads as valid does **not** exempt you from the
  box check.

**Box check** (every session, before the first floor motion): the operator places the robot on
the corner-A tape, square to the box. You read `x, y` from `robot.py state`. Repeat at corner B.
PASS if both are within **2 in** of the box corners and the box is 51 × 46 ± 2 in. On a fail, do
the re-mark procedure; do not drive on a box that failed. Log each pair in `FINDINGS.md` as a
drift measurement.

**Re-mark procedure — order matters:**
1. Robot on corner-A tape, square to the box, +x along the 51 in side.
2. **Reset pose**. This clears the box, so it must come *before* marking.
3. **Mark corner A**.
4. Operator drives to the diagonally opposite mark, holding heading.
5. **Mark corner B**.
6. The operator replies with the x, y at B.

Then run `robot.py check`; it must exit 0.

#### 3.4.1 The BIOBUZZ field fence (on a real field, instead of marking corners)

On a full BIOBUZZ field the fence comes from the field itself, not from two marked corners:

- **Rect** = the perimeter (0..141.5 in both ways), or a sub-region the operator names. Its
  meaning changes: it is the **wall**, and the robot's footprint (heading-aware, 18 × 18 in by
  default) must stay inside it, not just its centre.
- **Keep-outs** = every obstacle in `TeamCode/.../field/biobuzz_field.json` - the two HIVE base
  rails and the four FLOWERS. That file is the visualizer's obstacle source too
  (`tools/pedro-visualizer/src/config/defaults.ts` imports it), so the fence and every path drawn
  in the visualizer agree by construction. Edit obstacles there and nowhere else.
- Keep-outs get the same clamp as the walls - 4 in + 0.30 s × speed toward it, 6 in smoothstep
  taper - along the obstacle's own normal, against a circle of the robot's half-diagonal (any
  heading). Only the component pointing at the obstacle is removed; the robot slides past.
- Follower commands (`pedroLine`, `pedroCurve`, `pedroChain`, `pedroHold`) are refused if the
  sampled path brings the robot within 6 in of a keep-out. A Bezier's control-point hull bounds
  it against a box but not against a rail in the middle, so curves are sampled, not hull-checked.
- The pose frame becomes the **visualizer's**: x = 0 the red wall, y = 0 the audience wall,
  inches. `fieldFence` writes the robot's measured field pose into the Pinpoint.

**Arming procedure:**
1. Operator places the robot and **measures** its field pose (tile seams are 23.58 in apart), and
   reports x, y, heading. You do not estimate it.
2. `python robot.py cmd fieldFence x=… y=… headingDeg=… --pose-from-operator`
   (optional `minX= minY= maxX= maxY=` for a sub-region, `robotL= robotW=` for the real footprint).
   Or the FIELD panel's **Arm BIOBUZZ field fence**.
3. Operator confirms the dashboard drawing - rails, FLOWERS, robot - matches the real field.
4. `robot.py check` must exit 0. It now also fails if the robot overlaps a keep-out.

The witness/frame checks, the odometry-jump discard and rule 6 apply unchanged: `resetImu`,
`odoConfig` and `boxClear` clear a field fence exactly as they clear a marked box.

#### 3.4.2 Border fence (any floor, no obstacles)

`borderFence width=W height=H x=… y=… headingDeg=… [robotL robotW]` (or the FIELD panel's
fence selector → **Border only**). A plain W × H in perimeter with the same wall semantics as the
field fence (the robot's footprint stays inside, heading-aware) and no keep-outs. The operator
picks a corner as (0, 0): x runs along the width edge, y along the height edge, and the pose is
the robot centre's measured distance from those two edges plus its heading from +x. `robot.py`
requires `--pose-from-operator`. `/state` `box.kind` is `border`. `pedrocheck.py path` refuses a
border fence (`.pp` paths are in BIOBUZZ field coordinates); `pedrocheck.py suite` runs on any
fence.

### 3.5 Smoke test before any real run

After "ready":
1. `python robot.py check` → 0.
2. On tiles, drive at the lowest meaningful power for about 1 s and confirm the pose moved the way
   you expected: `python robot.py drive --f 0.30 --sec 1.0 --confirmed-floor`. (This said 0.12
   until 2026-11-14: on the rebuilt pods 0.12 and 0.20 do not break the drivetrain loose on tiles
   at all - stalled-motor current, zero motion - and 0.30, the default cap, moves ~10 in/s.
   FINDINGS 2026-11-14 evening. A smoke test that does not move proves nothing.)
3. Only then run the real experiment.

Pass `--confirmed-floor` **only** if this session's operator reply said the robot is on the floor.
That is CLAUDE.md rule 8, and the flag exists so you have to assert it every time - on `drive`,
on every motion action through `cmd` (`headingStep`, `pulseMotor`, `pedro*`, ...), and on
`pedrocheck.py`. For steering
checks with the wheels in the air, pass `--on-blocks` instead. It skips the box and pose gates,
since the pose cannot move, and is valid only after the operator has confirmed the chassis is
strapped and hands and leads are clear. You must pass exactly one of the two flags.

---

## 4. HTTP API reference

Every call is a plain **GET** with query parameters. There is no JSON body, no headers and no
auth.

| Route | Returns |
|---|---|
| `/swerve` | dashboard HTML |
| `/swerve/state` | JSON snapshot: the **last published** one (20 Hz default) |
| `/swerve/cmd?action=NAME&k=v…` | always `{"ok":true,"live":…}`. **It only queues.** The verdict appears later in `/state.message` |
| `/swerve/rec.csv` | the recorder, CSV. Slow to render: use a 30 s timeout |
| `/swerve/config`, `/swerve/restart` | hardware config / app restart. **Operator-only** (§0) |

Commands are queued and drained on the OpMode loop. **Poll `/state` for identity, never for
timing.** Wait for a changed `message`, an echoed value or `rec.label == your label`; never
"sleep 0.2 s and read". The known failure was a `/state` read straight after `recStart` that still
showed the previous `recording:false`, so every capture chunk ended after one sample. Any
parameter `pod=N` also **changes the selected pod** as a side effect. An unknown action sets
`message` to `UNKNOWN COMMAND: "…"` and adds an entry to `errors`.

### Commands (the switch is `SwerveBringUp.handleCommand`, about line 2730; grep `case "`)

**Motion.** These are refused before START, and `robot.py` sends each of them once, with no
retry. `robot.py cmd` gates every one exactly like `robot.py drive`: exactly one of
`--confirmed-floor` / `--on-blocks` (§3.5), and the box gate unless `--on-blocks` (the `pedro*`
commands need the box either way). `pedrocheck.py` likewise needs `--confirmed-floor` with
`--surface tiles` (or `--on-blocks` with `blocks`):

| Action | Params | Notes |
|---|---|---|
| `drive` | `f s t` in [-1,1], `foc=0\|1`, `rot=` deg/s | Robot frame unless `foc=1`. `rot` (2026-11-15) rotates the (f, s) vector by that rate every loop, OpMode-side - a smooth stick sweep (`robot.py drive --rot`). Enters DRIVE mode. **400 ms watchdog.** Use `robot.py drive`, never a one-shot |
| `pidStep` / `pidStepAll` | `deg` | **Absolute** wheel heading, not a relative step. One pod (the selected one) / all four |
| `rawServo` | `pod pow sec` | Open loop; `sec` ≤ 5. A duplicate restarts the dwell timer |
| `nudge` / `spinServo` | `pod`, `dir` | Jog the selected pod |
| `headingGoto` / `headingStep` | `deg` | Relative robot rotation, held |
| `wireScan`, `sweep`, `pulseMotor`, `autoTune` | — | Bring-up routines. **On blocks, strapped** |
| `pedroStart` | `activate=all` `pods=shipped\|live` | Needs pose **and** an armed box. Always builds a fresh follower. `shipped` (default) = `Constants.createFollower` as installed; `live` = same follower and shipped `SwerveConfig`, pods from the tool's live calibration |
| `pedroLine` / `pedroHold` / `pedroCurve` | `dx dy power` / `dx dy dh` / `d power` | Refused if any target or control point is within 6 in of the box edge, or the path comes within 6 in of a keep-out. `dh` = heading change, deg, done by Foresight |
| `pedroChain` | `pts=x,y;…\|…` (2-12 points per segment) `head=tangent\|reverseTangent\|constant:deg\|linear:a:b\|param:a:b:curve` `power` | `param` = the visualizer's / exported auto's t^curve interpolation (Pedro's `linear` is on distance and differs) |
| `pedroPidf` | `hp` `tp` `tfp tfs tsp tss` `ti td` `fzpa lzpa` | Foresight gains on the shared config; discards the follower. `hp` is Foresight's heading kP - NOT `setHeadingPidf` |
| `pedroReset` | - | follower gains, mixer and schedule back to the installed build (also done at OpMode init and stop) |
| `pedroStop` | — | → IDLE |

**Stop and idle:** `stop` (all servos and motors off → IDLE), `pidHold` (release the pod).

**Tuning and toggles.** Absent params are left unchanged; an unparseable value falls back to the
current one:

| Action | Params |
|---|---|
| `setPidf` | `kp ki kd kf ks ksband cache slew kilimit kiband kireset dom pulsed pband ptol ppow pms pcoast`, `scope=all` (else the selected pod), plus robot-wide `floor velstart gate ramp` |
| `setHeadingPidf` | `hkp hkd hkf` (session-scoped, not saved) |
| `setHeadingHold`, `setXLock` | `value=true\|false` (omit it to toggle) |
| `setMixer` | `taper=true\|false`, `slew=<deg/s>` |
| `setPublishHz` | `value` (clamped 1–200). Default 20. Lower it during recorded runs, restore it after |
| `setFastFmt`, `setFastCurrent` | measurement-tool toggles. **Declare per CLAUDE.md rule 9 before using them for data** |
| `select` | `pod` |
| `focRef` | capture field-forward |

**Recorder:** `recStart label=…`, `recStop`. Since 2026-10-01 every row ends with follower
columns `fterr,fherr,fcx,fcy,fch,fvel,fbusy` (empty outside FOLLOW), and in FOLLOW the pod
`err/pwr/flip/ctgt` columns come from the FOLLOWER's pods (`tgt` is blank there).

**Validating through Pedro: `tools/swervetune/pedrocheck.py`** (`suite`, `path`, `report`) - see
`TUNING_TASK.md`. The tool's own step/heading/drive tests are search tools; acceptance is
`pedrocheck.py --pods shipped` on the installed build.

**Pose and box.** These **destroy or rewrite the fence**; `robot.py cmd` makes you pass
`--clears-box-ok`:

| Action | Effect |
|---|---|
| `resetImu` | Pinpoint pose + IMU reset. **Clears the box** |
| `odoConfig` | `xPodOffset yPodOffset` (in) `xPodReversed yPodReversed` (true/false), or `restore=1` - Pinpoint geometry in Constants.java's terms; unspecified values keep their live value (legacy `fy sx fdir sdir` still accepted). Resets the pose. **Clears the box**. Live values in `/state` `odo{…,shipped}`; any difference from Constants.java is also an `errors[]` entry, and `export` carries them |
| `boxClear` | disarms the box |
| `boxMark` `corner=0\|1` | operator-side action, normally done from the FIELD panel |
| `boxSet minX minY maxX maxY` / `setPose x y headingDeg` | recovery only. Nothing verifies them against the mat, so the operator must confirm |
| `fieldFence x y headingDeg [minX minY maxX maxY robotL robotW]` | writes the pose and arms the BIOBUZZ field fence (§3.4.1). `robot.py` requires `--pose-from-operator` (also for `setPose`) |
| `borderFence width height x y headingDeg [robotL robotW]` | writes the pose and arms a plain border fence (§3.4.2) |

**Calibration.** These write the hub cal file. Only do them on purpose, and say so: `zeroPod`,
`zeroAll`, `zeroTrim deg`, `setEncoderReversed`, `setDriveReversed`, `setServoReversed`,
`setLabel`, `setRange`, `save`, `reload`, `export`.

**The output of a tuning session is `Constants.java`.** `export` renders everything the tool
holds - per-pod turn gains, zeros, analog ranges, drive directions, encoder names, servo caching,
pulse settings, heading kP, translational PID, coast-down decelerations, Pinpoint offsets and
directions - as one declaration per line, each matching a declaration in the TUNED VALUES block
at the top of `pedroPathing/Constants.java`. Then:

    python robot.py constants            # export, save to runs/constants_<ts>.java, show the diff
    python robot.py constants --write    # splice by name; provenance comments are untouched

Anything the tool holds that Constants.java cannot express (a non-zero kI, per-pod pulse
settings, a pod relabelled to a different corner, heading kD) comes back as a WARNING and is not
written. `--write` changes **shipped** code: build, then commit each value with its evidence.
`--from FILE` re-splices a saved export without the robot.

### `/state` fields you will actually use

`live`, `started`, `mode` (IDLE / DRIVE / PID / FOLLOW / …), `busy`, `voltage`, `message`,
`errors[]`, `xLock`,
`heading{ok,deg,targetDeg,hold,…}`, `pose{ok,x,y,vx,vy}` (inches, in/s),
`box{valid,minX,minY,maxX,maxY,marked0,clamped,kind,robotL,robotW,keepOutClear,keepOuts[]}`
(`kind` is `marked` or `field`; `keepOutClear` is the robot circle's clearance to the nearest
keep-out, in, `null` with none),
`pedro{active,job,busy,x,y,h,terr}`, `rec{recording,runId,samples,overflowed,label}`,
`timing{encoders,heading,mode,publish,telemetry,publishHz,pub{…}}`,
`pods[]{i,label,hasEnc,volts,wheelDeg,tgtDeg,cmdPower,kp,kd,ks,…}`.

The `errors[]` entries about **gain divergence** from `Constants` are
deliberate, not faults.

---

## 5. Driving

- **Use `robot.py drive`.** It checks the gates and caps each axis (default **0.30**, the
  SWERVE_TASK speed cap; going above 0.50 needs `--allow-fast` and a stated reason). It re-sends
  `drive` every **75 ms** against the **400 ms** watchdog and reads `/state` each tick. It
  **aborts after two consecutive transport failures** or if the box disarms, and it **always**
  sends `drive 0,0,0` then `stop` on exit, including on Ctrl-C.
- **Never retry a `drive` command.** A retried packet arrives late with stale stick values. Losing
  a packet is harmless, because the watchdog covers it.
- **One sender at a time.** If the dashboard's "Drive with gamepad" is active, it and your script
  both command the robot. `stop` sent while any loop is still streaming gets overridden within
  about 60–150 ms. Stop your own loop first, then send `stop`.
- A watchdog trip zeroes the input. With `xLock` on, zero input parks the pods in an **X**, which
  is a ±45° pattern. **Discard any recorded chunk in which the watchdog fired**, because it fakes
  the "45° snapping" symptom. The `message` says `Drive watchdog tripped`. A backgrounded browser
  tab trips it too.
- **The fence (`applyBoxLimit`) clamps commanded *velocity* per field axis, not position.**
  - The margin is 4 in plus 0.30 s × the outward velocity.
  - The outward component is tapered with a smoothstep over the last 6 in.
  - Per-axis clamping **does rotate** the velocity vector near a wall; the taper only makes that
    rotation continuous. So any azimuth trace from a tick with `box.clamped=true` is suspect.
    `robot.py drive` counts those ticks.
  - The clamp does not kill momentum, so the robot coasts past the margin. **The tape is the
    barrier; the clamp is not.**
- `f`/`s` are robot-frame. The sign of `t` is **not recorded here**. Before relying on it, verify
  it with a short, low turn (`--t 0.15 --sec 0.5`) and log the result.
- A box armed with no pose refuses all translation. That is a "No pose" problem, not a driving
  bug.
- The Pedro follower (`pedroStart` …) needs the box and refuses any target within 6 in of an edge.
  The Foresight tuner's default 48 in distance does **not** fit the 46 in side of the practice box.

---

## 6. Recording data

- The recorder holds **3000 samples, one per loop, and stops when full; it does not wrap.** That
  is about 30 s at 90 Hz. Longer runs need chunking: `drivecapture.py <label>`, stopped by
  creating `tools/swervetune/runs/DRIVE_STOP`. On PowerShell use
  `New-Item tools/swervetune/runs/DRIVE_STOP`; `touch` does not exist there.
- Pattern: `recStart label=X` → confirm `rec.label == X` and `rec.recording` in `/state` → run →
  `recStop` → `robot.py pull X` (or `drive --rec X`, which does all of this).
- CSV columns: `t,dt,volts,loopHz,mode,servoMa,batteryMa,heading,htgt,px,py,cf,cs,ct`, then per
  pod `pN_v,pN_wheel,pN_tgt,pN_err,pN_pwr,pN_flip,pN_ctgt`. `tgt` against `wheel` separates
  "demand is shaking" from "response is shaking".
- **Loop rate is `1/mean(dt)`.** Report `loop_hz_true`, `loop_dt_mean_ms` and `loop_dt_p90_ms`.
  Never report `mean(loopHz)` unless the word "inflated" sits next to it. `robot.py pull` prints
  the correct figures.
- Record `mode`, `xLock`, `heading.hold`, `publishHz` and the full toggle vector with every result.
  Loop behaviour differs between IDLE and DRIVE, so measure in the mode you are making claims
  about.
- Every result needs **surface, volts, n and spread** (CLAUDE.md rule 7). If the operator has not
  given them, write `surface=UNKNOWN` / `volts=UNKNOWN` and ask in the next batched survey.
- Battery: do not start a new chunk below **11.8 V loaded**. Do not split an A/B arm across a
  battery swap.

---

## 7. Stopping and aborting

Escalation ladder, fastest safe option first:

1. **Your own script:** Ctrl-C or kill its loop. `robot.py drive` then sends zero and `stop`.
2. **`python robot.py stop`:** `drive 0,0,0` + `stop` → IDLE. The OpMode stays up.
3. **`python robot.py estop`:** STOP_OP_MODE through FTC Dashboard (port 8000). This kills the
   OpMode, so `/state` goes to `live:false`. The operator must restart it, and you must re-run
   §3.2–3.4 before any motion.
4. **Operator presses STOP on the Driver Station.** This is the real emergency stop. Ask for it
   the moment 1–3 fail or the robot is doing something you did not command.

**Stop on your own and ask for STOP** in either of these cases:
- A pod's measured azimuth does not respond while you are commanding it to move. That is a stalled
  CRServo or a dead encoder.
- `/state` fails twice in a row while a drive command is outstanding.

**"ABORT" from the operator:**
- Stop sending to `/swerve/cmd` and to every bench script immediately.
- Report the last command you sent and when you sent it.
- Send nothing motion-capable until the operator says **"re-arm"**. "ready", a survey answer or
  silence is not a re-arm.
- Discard any recorder chunk that spans the abort, and say how many you discarded.

---

## 8. Deploys

1. `./gradlew :TeamCode:assembleDebug` must pass. Commit.
2. Batch: a deploy is expensive, so put experiments behind runtime `/cmd` toggles so one install
   carries many A/B arms. Anything in `Constants.java` is compiled in, so changing it costs a deploy.
3. `ftcdash.py stop` (saves the box witness), `adb -s 192.168.43.1:5555 install -r
   TeamCode/build/outputs/apk/debug/TeamCode-debug.apk`, relaunch
   `com.qualcomm.ftcrobotcontroller/org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity`,
   wait for the dashboard, `ftcdash.py "Swerve Bring-Up"` (CLAUDE.md rule 5). Anything physical the
   deploy needs (battery, placing the robot) goes in one OPS REQUEST; if the hub is unreachable, the
   operator installs and starts it.
4. Once it is running:
   - `robot.py check`.
   - Read `box.valid` and `message`. The box may have reloaded, or been discarded by the frame
     check.
   - Re-do encoder liveness and the box check (the starred items).
   - Set `on_robot_build` in `RUN_STATE.md`.
   - **Session-scoped settings reset on restart:** heading PIDF, `xLock`, `headingHold`, mixer
     taper/slew, schedule tuning, PWM range and enable, publish Hz. Re-apply them and log it
     before measuring.
5. Before attributing a result to a code change, confirm `on_robot_build == last_commit`.

---

## 9. Gotchas that have already cost robot time

1. **The `/state` read that follows a command is stale.** It is the last published snapshot, so
   poll for identity (§4).
2. **`/cmd` always says ok.** Motion commands sent in INIT are silently dropped with a
   `Press START` message. Unknown actions do nothing except set `UNKNOWN COMMAND`. Read
   `message`.
3. **A deploy or restart kills the OpMode.** A client that checks liveness in its constructor
   (`Bench()`) throws before any retry loop runs. Wrap the construction itself in the retry.
4. A `/state` fetch that has exhausted its retries must not be indexed like a full dict. Guard it.
5. **The bring-up OpMode *can* drive**, through the dashboard's browser-gamepad path
   (`dashboard.html`, drive section), which the Driver Station handler doesn't show. Read
   `dashboard.html` before saying what the tool can or cannot do.
6. **`stop` is not "stop the OpMode"**; it only goes to IDLE. Stopping the OpMode is `estop` or the
   DS. Starting it is always the operator's job.
7. **Surface and volts go in the "ready" reply, before motion,** not after the runs. One A/B once
   had to be labelled after the fact.
8. **`SwerveBringUp.java` is about 215 KB and `dashboard.html` about 73 KB.** Never read either
   whole: grep, then read a window. Line numbers quoted in older notes (`:2532`, `:2562`,
   `:2384`) are stale; grep for the symbol.
9. **The Foresight tuner's default distance (48 in) is longer than the box (46 in side).**
10. **Running `DriveTeleOp`, or any Pedro-`Follower` OpMode, re-origins the Pinpoint.** Expect the
    saved box to be discarded, or worse, to be silently shifted, the next time bring-up starts.
11. Background jobs: smoke-test one trial first, then launch the batch in the background and work
    offline. Don't poll-sleep while it runs.

---

## 10. Record-keeping

- `RUN_STATE.md` at repo root. The schema is in `SWERVE_TASK.md`: `on_robot_build`,
  `opmode_running`, `surface`, `battery_v`, `box_armed`, `open_ops_request`. Rewrite it before
  every OPS REQUEST and after every operator reply.
- `FINDINGS.md` gets every result with its evidence. `ASSUMPTIONS.md` gets every default you took.
- Commit per finding. **No `Co-Authored-By:` trailer** (CLAUDE.md rule 11).
