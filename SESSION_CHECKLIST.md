# Session checklist — swerve + shooter tuning day

One page, so the session runs on **one opening pass at the robot** instead of a banner every few
minutes. Written 2026-11-13 from the 2026-11-12 pollen-flywheel session (what cost time is in
`FINDINGS.md`) and a read of `TUNING_TASK.md`, `ROBOT_CONTROL.md`, `SWERVE_TASK.md`, `robot.py`,
`pedrocheck.py`. `CLAUDE.md` §8 still applies in full; nothing here relaxes a gate, it only
front-loads them.

---

## A. Before the robot is powered (laptop, 5 min)

- [ ] Laptop on the robot Wi-Fi **and** a second route to the internet (`ROBOT_CONTROL.md` §1).
- [ ] `adb connect 192.168.43.1:5555` → `adb devices` says `device`, not `offline`.
      On 2026-11-12 Wi-Fi adb refused twice and cost two banners. **Keep a USB-C cable to the
      Control Hub plugged in as the fallback.**
- [ ] `git status` clean or deliberately dirty; `git log -1` is what you mean to deploy.
- [ ] Batteries: all charged, labelled. Resting ≥ 12.5 V to start a swerve A/B
      (`ROBOT_CONTROL.md` §6); swap below 11.8 V loaded, **never mid-arm**.
- [ ] Bench power supply for everything stationary (see **Power plan** below): set to
      **12.5 V**, current limit as high as it goes (check it covers the flywheel's stall current —
      estimated ~9 A for one motor at 12 V, not measured).

## Power plan (decided 2026-11-13)

The robot runs on the bench supply while it is **not driving**, and on a battery once it drives.
Every switch between them is a power cycle: the app restarts (Claude re-starts the OpMode), the
Pinpoint resets (any box/fence is discarded), and the standing clearance is re-asked once. So:

1. **Supply phase, one block, no switching:** deploy, shooter (C), swerve stage 1 **on blocks**
   (wire scan, zeros, `setRange`, `pidStep` search). Constant volts make these A/Bs cleaner
   (yesterday's battery drifted 13.2 → 11.8 V and the shipped arm sagged it further).
2. **One switch to battery**, bundled into one banner: battery in, robot on the field at a
   measured pose, clearance, `fieldFence`. Then everything that moves the robot.
3. Battery swaps after that only at stage boundaries, each bundled with the fence re-arm.

Caveats: hub velocity F is **not voltage-compensated**, so a flywheel tuned at a constant
12.5 V will run slightly faster on a fresh 13+ V battery and slower late in a match (estimated;
the size is unmeasured). Finish the shooter with **one `rapidfire` on battery** and record both
volts. Swerve gains are accepted only on `pedrocheck` on the battery anyway.

## B. The opening banner (one pass, operator at the robot)

Claude posts **one** `ask` with every line below and its default; the operator answers what they
want to change and presses Done once. Anything not answered takes the default.

**Physical, done once now:**
1. Robot on **blocks, strapped** (swerve stage 1) or **on tiles** — say which. Hands and leads clear.
2. Battery volts at rest.
3. Each pod turned by hand through its travel, naming the pod, while Claude watches `/state`
   encoder liveness (`ROBOT_CONTROL.md` §3.3).
4. Flywheel(s): which **motor port and encoder port** each is on, and whether the wheel turns
   freely by hand. (2026-11-12: pollen was on Control Hub port 0 = `sm0` while the config said
   Expansion Hub port 3. `cmd flyDevice wheel=pollen device=sm0` now persists on the hub.)
5. 3 ft clear around the box; Driver Station STOP within reach.

**Decisions (defaults in bold):**
6. Shooter target speed per wheel, ticks/s — **70% of measured max** if unknown.
7. Swerve area — **the real field, field fence** (`fieldFence`; stage 8 `.pp` paths in scope).
8. Robot footprint — **18 × 18 in**.
9. Path power — **0.5**.
10. Deploy budget — **3**.
11. Foresight Tuner — **skip** (it no longer exists on master, see D1).

**Standing clearance:** the operator's "clear" in this banner covers the whole session for both
mechanisms. It is re-asked **only** after: a battery swap or supply↔battery switch, a physical change the operator reports,
a stall-guard trip, or an operator ABORT ("re-arm" needed, `ROBOT_CONTROL.md` §7). A chat "stop",
a finished command, or a redeploy alone do **not** void it. OpModes that move on START (autos,
Pedro tuners, `PreMatchSystemCheck`) still each need their own "clear" banner naming them
(`CLAUDE.md` rule 5) — ask for those **in the opening banner** too, as "I will start X at stage N".

## C. Shooter (Mechanism Tuner, `mechtune.py`) — ~15 min per wheel

| Step | Command | Operator |
|---|---|---|
| start | `ftcdash.py "Mechanism Tuner"`, `mechtune.py check` (shows device bindings, guard) | — |
| wiring | `cmd flyDevice wheel=W device=NAME` if not on the configured port (persists) | — |
| direction | `dircheck W --ask --confirmed-clear` — encoder sign checked automatically, one yes/no | 1 answer |
| search | `autotune W --target T --confirmed-clear` — ff ×2, closed-loop F, 4-candidate grid, recommendation (~4 min) | none |
| validate | `ab W --a "<shipped>" --b "<rec>" --test shots --target T --n 10 --confirmed-clear` | feed 20 pieces |
| …if shipped can't hold speed | `hold W --a "<shipped>" --b "<rec>" --target T --n 6 --confirmed-clear` | none |
| ship | `export --only flywheel` → `--write` → build → commit one value per commit → deploy → `state` shows it | — |
| demo | `rapidfire W --target T --confirmed-clear` — logs every shot, Done stops it | fire at will |

Built in since 2026-11-12: the OpMode's **stall guard** stops everything if a driven wheel reads
< 40 ticks/s for 0.6 s, or runs backwards against its target for 0.3 s; the host aborts with the
reason. `ab` counts "never held speed" as a failure per arm instead of aborting. `ff` now starts
at 50% power.

## D. Swerve (`robot.py`, `pedrocheck.py`) — decide these **before** the session

1. **Foresight Tuner is gone.** `TUNING_TASK.md` stages 2 and 6 send the operator to
   `pedroPathing/Tuning.java` (port 10158); it was removed 2026-11-01 with Sloth and lives only on
   branch `autotune-sloth`, which brings back Sloth and its broken live config. Default: **skip
   stage 6**, keep shipped Foresight constants, validate with `pedrocheck` only.
2. **Real field → field fence, not the 51 × 46 box. First use ever.** `fieldFence x y headingDeg`
   (`ROBOT_CONTROL.md` §3.4.1) has **never run on the robot** (`CLAUDE.md` §4). Pick a **home
   spot**: a tile corner at least **45 in** from every wall and keep-out in
   `field/biobuzz_field.json` on every side (the suite reaches 24 in from home along +x/+y and
   18 in elsewhere; follower targets need 6 in + the 12.7 in half-diagonal of an 18 × 18 robot,
   so 24 + 18.7 ≈ 43 in; arithmetic from the code, rounded up). Tape it, measure its field
   x, y (visualizer frame: x = 0 red wall, y = 0 audience wall) and heading **once**. The operator
   then confirms the dashboard drawing matches the real field before anything moves, and the first
   drive is a crawl at the 0.30 cap with a hand on STOP. With that room the default suite
   (`--line 24 --curve 18`) fits; the box-geometry problem only applied to the practice box.
3. **Stage 8 is in scope:** `pedrocheck.py path X.pp --pods shipped` (needs the field fence, which
   we now use). The auto itself still needs its own "clear" banner naming it (rule 5; autos have
   no fence).
4. **Every power cycle discards the fence** (battery swap or supply↔battery switch resets the
   Pinpoint). Because the home spot is taped and measured, re-arming is: robot on the tape,
   `fieldFence <same x y h> --pose-from-operator`, operator glances at the drawing. Bundle it into
   the swap banner. See **Power plan**.
5. **Stage 2 (odometry) clears the fence every iteration** (`odoConfig` needs `--clears-box-ok`),
   but its ±45° check rotations need one. Same fix: return to the taped home spot and re-issue
   `fieldFence` with its measured pose, no new measurement needed.
6. **Restarting Bring-Up resets session settings** (heading PIDF, xLock, mixer, publish Hz,
   `ROBOT_CONTROL.md` §8.4). Batch code changes so one deploy carries many experiments.
7. `--pods shipped` acceptance: `pedrocheck` only **warns** if the shipped flags are false and does
   not check `on_robot_build`; Claude must check `RUN_STATE.md` `on_robot_build == last_commit`
   itself. `RUN_STATE.md` does not exist yet: stage 0 creates it.

Who does what, swerve, in order: **[supply]** blocks for wire scan / zeros / `pidStep` search →
**[switch to battery, one banner]** robot on the taped home spot, `fieldFence`, drawing check →
crawl at 0.30 → **unattended** `pedrocheck suite --pods live` A/Bs → export, deploy, fence re-arm,
`--pods shipped` → stage 8 `.pp` paths → TeleOp drive by the operator.

## E. Docs that disagree with the code (fix when convenient — local edits pending in some)

- `ROBOT_CONTROL.md` §3.3 item 4 and §9.6 still say the operator starts the OpMode; rule 5 says Claude.
- `SWERVE_TASK.md` says any deploy invalidates the box; `CLAUDE.md` §4 says it can survive via the witness.
- `ROBOT_CONTROL.md` §3.5: `--on-blocks` does **not** skip the box gate for `pedro*` or any `pedrocheck` run.
- `SWERVE_TASK.md` is stale in places: `DriveTeleOp`, the Pedro 2.1.2 API, the solved 37.4 ms
  publish cost, the old kS-in-band claim, the already-refuted cache sweep, `/command` instead of `/swerve/cmd`.
