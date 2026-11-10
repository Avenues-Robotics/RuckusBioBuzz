# Mechanism Tuner

Diagnostic only — never ships. Tunes everything except the drivetrain (that is Swerve Bring-Up).

1. Join the robot WiFi, open **http://192.168.43.1:8080/tune**.
2. On the Driver Station run **Mechanism Tuner** (Diagnostics group), INIT, START.
   Nothing moves before START; everything stops if no browser or host script has talked to the
   robot for 30 s.

| Tab | What it does |
|---|---|
| Start here | Status, an ordered checklist of every value this tool tunes (live status: not set / changed / shipped, ticks kept in the browser), what is tuned elsewhere, message history from Claude |
| Start here → Robot configuration | Lists every robot configuration (built-in and hub files), shows the active one, and **Activate & restart** switches it — the same two commands the Driver Station sends (activate, restart robot). Works with no OpMode running (`/tune/configs`, `/tune/configActivate?name=`) |
| Flywheels | Live velocity chart and a zoomed speed-error chart (entry and hold bands, gate-open shading), spin / open loop, hub velocity PIDF (Write / Write to both / Shipped / Copy, sweep F, FIRST starting point, "not written" flag), step test with a rule-of-thumb verdict, synthetic kick, feed-forward sweep, **Step trials** (every step overlaid with its gains, pulled from `/tune/rec.csv`), shot/dip events and **Recovery by gain set**, recorder, Claude prompt |
| Shooting | `shooter.java`'s gate logic run with tool values (entry band, hold band, min open time) and fed by the shipped intake; gate-window table; zone-entry test for `IDLE_FRACTION` |
| Turrets & servos | Turret center / reversed / gear ratio against the exact `turret.java` formula, mount offsets; gate and scoopula positions with Mark buttons |
| Turret limits | Per turret: jog (allowed past the limits) and mark the wiring limit on each side and the start position (`*_TURRET_MIN/MAX/START`); every other turret move on the dashboard is held inside the marked limits |
| Intake & color | Live raw RGB, what the shipped `ColorCheck` rule calls it, intake run through the shipped `intake` class, `REVERSE_BRAKE_MS`, labelled samples and a ratio + distance-gate suggestion (`/tune/color.csv`) |
| Swerve | Whether Swerve Bring-Up is running, link to `/swerve`, Claude prompt |
| Regression | Pick alliance + hive (AUTO = shipped rule by robot y, or force UP/DOWN), Pinpoint pose (settable), turrets tracking the hive; per wheel: set a speed, shoot, **Record** (target ticks/s at the turret-to-hive distance), linear or degree 2/3 least-squares fit with R²/RMSE/residuals, **Save fit** → export group `regression` → `modules/ShootingRegression.java`. Samples persist on the hub in `FIRST/mechtune_regression.csv` (`/tune/regression.csv`) |
| Export | Every value next to what the code had at init, changed rows highlighted, the `--only` command for the changed files |

Step trials and shot/kick history are kept in the browser's localStorage (per viewer, not on the
robot), so they survive an OpMode restart but not a different laptop. The shoot sequence does not
model aiming or the zone: the turrets stay put.

Starting values are read from the shipped statics at init. Intake thresholds and
`REVERSE_BRAKE_MS` are applied to the shipped statics for the run so the shipped intake logic uses
them, and restored on STOP. Nothing
reaches competition code except through `tools/mechtune/mechtune.py export --write`.

**Operator banners.** `GET /tune/notify?text=...&level=action|info|warn&await=ack|none|shot:pollen|shot:nectar`
posts a note; `/tune/notes` lists them; `/tune/ack?id=N&reply=...&resolution=done|skipped` answers one.
Registered at app start, so they work under any OpMode. `notify.js` renders them on any page that
includes it (this one and `/swerve`). `await=shot:<wheel>` notes close themselves when that
flywheel's dip detector logs a shot. Host side: `mechtune.py ask` / `say` / `clear`.

Files: `MechanismTuner.java` (OpMode), `TuningHub.java` (state, command queue, notes),
`TuningWebApp.java` (routes), `FlyRecorder.java` (6000-sample trace, does not wrap),
`tuning.html`, `notify.js`. Procedure for a Claude session: `MECHANISM_TUNING_TASK.md`.
