# Ruckus Bio Buzz Pedro Visualizer

This is a local copy of the Pedro Pathing visualizer:

https://github.com/Pedro-Pathing/Visualizer

Local customization:

- The export menu includes `TeamCode Auto`.
- That exporter generates an FTC OpMode in `org.firstinspires.ftc.teamcode.auto`.
- Generated autos use this repo's `PathStep` helper for the start and endpoint poses.
- The sequence editor supports `Path`, `Wait`, and `Event` items. `Add Event` creates a timed `Shoot` event by default.
- The control panel supports named pose variables that can be assigned to the start pose or path endpoints.
- The starting point has an editable `Heading` in degrees (literal or expression) — the pose the robot is placed at, which `follower.setPose` receives. Turning from it to whatever the first path needs is timed like any other rotation, so a start heading that does not match the first path shows its real cost. A tangential start point has no heading of its own and follows the first path until a value is entered.
- The control panel supports path variables that store a reusable copy of a selected path chain and can insert that stored path back into the route.
- Endpoints assigned to pose variables still allow editable heading mode, linear start heading, and heading curve while the pose controls the final position/heading.
- Individual paths can be duplicated, selected groups of paths can be wrapped in repeat loops, and path chains can be looped as a group.
- Repeat loops and `if` blocks hold paths, waits and events in one ordered list, so a pause or a mechanism trigger can sit between two paths inside a loop and run on every pass. Drag any of them in by its handle.
- Number variables store reusable numeric constants for repeat counts, waits/events, path speed, and event marker values. TeamCode export emits them as Java constants.
- Each path has a `Path Speed` scale from `0.05` to `1.0`. Pedro 3 has no per-path power cap, so TeamCode export applies it as Foresight's `maxPathSpeed` for that path (`path.with(maxPathSpeed.at(speed))`) — a fraction of the robot's max achievable **velocity**, which is also how the time estimate has always used it. At `1.0` nothing is applied.
- Each path can define parallel event markers. Pedro 3 dropped PathBuilder's parametric, temporal and pose callbacks, so the generated auto carries a small `AutoPath` runtime that keeps Pedro 2's trigger rules per path of a chain: parametric fires once the path's distance completion reaches the marker (or the follower leaves that path), temporal fires that many ms after the path began and is dropped if the path ends first, pose fires once the curve parameter passes the point nearest the pose. Optional timed finish handling is unchanged.
- The control panel has a telemetry readout that follows playback, showing the current path state, pose, path progress, path speed, active parallel events, and the next queued event marker.
- Visualization settings can show event pins on the field, per-segment path length/time labels, a 30-second autonomous countdown overlay, an optional blue-to-red velocity gradient based on the trapezoidal motion profile, and estimated swerve module angles on robot previews.
- Undo/redo history keeps recent recovery snapshots in localStorage across reloads.
- Browser project files are stored in IndexedDB, with one-time migration from the old localStorage file blob.
- Linear heading paths include an editable heading curve graph. Values above `1.0` shift more of the turn toward the end of the path.
- The control panel includes `Mirror X` and `Mirror Y` actions for switching alliances by flipping path coordinates and headings on either field axis.
- Paths are checked against the obstacles and the field walls using the robot's own footprint at the heading it holds. Path rows show `Hits <thing>` or the clearance in inches, the field draws the robot where it is in trouble, and the export warns before the auto reaches a match. The chip is a button that moves a control point, an endpoint or the starting point until the robot clears.  The robot drawn on the field for a collision sits where the robot FIRST touches, and the span is named after what it touches first — the deepest sample can be a long way further along the path (and the nearest thing there can be something else entirely), which made a collision look like it happened much later than it does. `ClearanceSpan` carries `contactDistance` / `contactPose` / `contactFootprint` alongside the worst sample, and the export warning quotes first contact. Overlap depth also counts crossing edges: a robot straddling a thin obstacle has no vertex inside it and none of its inside it, which used to report 1e-6 in of overlap — "just touching" for a robot driving clean through a HIVE rail. Waits are checked too: `checkClearance` takes `stationaryPoses`, built from the timeline's wait events, and measures the pose the ROBOT holds through each one against the same obstacles, walls and midline, reporting it against the Wait (a chip on the Wait row, and its own outline on the field) instead of against a path. The heading model turns the ROBOT while it drives rather than in place, so a Wait holds whatever pose the previous path ended on and that pose is usually already reported by the path's last sample — a stationary pose whose pose was already flagged is dropped rather than counted twice, so this mostly earns its keep where nothing else lands: a Wait before any path, or a route with no paths at all.
- `Settings → AUTO Midline` (off by default) keeps the ROBOT on its own half during AUTO. BIOBUZZ Competition Manual V1 G402 makes TILE columns A-C the red side and D-F the blue, and says navigating into the opposing side during AUTO "is a risky gameplay strategy that may be seen as STRATEGIC" — so the line is the middle TILE seam, x = 70.75 in visualizer coordinates, and the setting picks which half is yours. It is measured like a wall: against the ROBOT's whole footprint at the heading it holds, so a ROBOT turned broadside crosses at a different point than one driving nose-first, and it obeys the same safety margin. Crossings read `Crosses the midline at <n>in`, the `Fix` search treats them like any other clearance problem (the report and the search share one options object, so a fix it calls clear really is clear), and the export warning names G402. The rule is a competition rule, not physics: nothing stops you planning across it with the setting off.
- **Pollen Pickup** (`Add → Pollen`) is a sequence step that uses TeamCode's `PollenDetectionPipeline`: look for POLLEN, drive to the best clump in a search zone, run the intake, and return to where the step started so every later path still starts where it was planned from. The plan comes from one module, `src/utils/pollenVision.ts`: a pinhole camera model (mount forward/left/height, tilt, yaw, FOV) projects pixels onto the TILES and back (round trip exact to 1e-12 px, tested), the camera's ground footprint, the range at which a single ball still passes MIN_AREA, the step's legs and times, and its findings. The editor draws the camera view from where each step starts (solid out to that range, dashed beyond), the zone, a draggable POLLEN marker and the approach / intake / return legs; the row shows expected and worst-case time, a live preview of where the POLLEN lands in the camera image, and warnings when it is out of view, too small to detect, across the midline, or reads as several balls because `One ball` area is calibrated at another range. The legs are animated, charged to the time estimate at the expected search length, and walked by the collision checker like paths. `Settings → Vision` holds the camera model and every pipeline tunable; `measured` stays false until someone measures the mount, and every pickup and the exported auto say so. The TeamCode export generates the runtime step: a VisionPortal with a `PollenDetectionPipeline.Config` from those settings, the same projection in Java, zone and midline filtering, an approach that re-plans whenever the estimate moves more than `retargetInches` (at the default mount a 1° tilt error is 6.7in at 60in but 1.3in at 24in, so closing in fixes it), blind pickup or skip on timeout, and the return leg. Top level only: a repeat loop's fixed slots cannot hold legs that depend on what the camera sees. Verified by a generated auto compiling in TeamCode and by driving the built UI in headless Edge over DevTools (load a .pp, expand a row, drag the marker, open Settings); not yet run on the robot.
- **Camera pick** and **Turn first** (path row checkboxes, saved on the line as `cameraPick` / `turnFirst`) preview what TeamCode's `auto/modules/CameraPick.java` and `runTurnStep` do on the robot. Camera pick: partway down the path the robot reads ONE camera frame, picks the biggest pollen clump against the end wall and is re-routed into it. The editor has no camera, so the clump is a yellow marker dragged along the wall (`previewX`); the path is cut where it passes the point nearest (path end x, `pickY`) - the Java's pose callback - and replaced by CameraPick.slamPath's cubic (from -> (from.x, from.y + 0.45 dy) -> (end.x, end.y - 0.35 dy) -> end, end = (clamp(previewX, xMin, xMax), wallY), heading held), and the next path starts from that end with its own control points. The field draws the camera wedge from the pick (`hfovDeg`, `cameraForward` - both estimates, red when the clump is outside it), the allowed stretch of wall (green), the planned rest of the path dashed (what runs when nothing is picked) and the pick point. Turn first turns in place at the path's start to its end heading, timed from the angular velocity plus a 0.15 s settle (capped at the Java's 1.2 s timeout), then holds it; it also cuts the chain before the path. One function, `expandRouteEdits` in `src/utils/cameraPick.ts`, turns both into plain lines and sequence items (the picked drive is a synthetic line appended at the END of the array, so every original index still means the same path; the turn is a wait item with `turnToDeg`), and everything that times, draws, animates or checks the route reads that - App's field, playback and clearance, the header time, the Telemetry tab, and the MCP review - while the editors keep the real `lines`. The TeamCode export reads the real lines and generates the glue the hand-written Red/BlueSoloAuto carry: `CameraPick` field + `new CameraPick(hardwareMap, Alliance.<RED|BLUE>)`, the `addPoseCallback(..., cameraPick.pickY(), this::decideChainNPick, 0.5)`, `decideChainNPick` / `followPickStep` / `buildChainM(Pose start)` for the path after it, reset in `start()`, `close()` in `stop()`, a telemetry line, and a `runTurnStep(<deg>)` case before a Turn first path. One camera pick per auto (CameraPick holds one set of constants); top-level paths only. Still hand-wired after an export, as before: AutoMechanisms, PinpointGuard, the AutoConstants step durations and the Shoot `eventDone` hook. The export now also imports `auto.modules.PathStep` (moved there in 0c24c00b; generated autos did not compile without it).
- Number fields drag: hover a literal for an `ew-resize` cursor and drag sideways to adjust it, with Shift for coarse and Alt for fine. Fields driven by a variable do not drag — scrub the variable itself in the Variables tab.
- `Settings → Motion Parameters` can read the robot's Pedro 3 `ForesightConfig` straight out of TeamCode, showing each value, what it changes it from, and which `c.<field>.set(...)` it came from — plus what Pedro does not measure and which values TeamCode still carries as placeholders, so the remaining guesses stay visible.
- The TeamCode exporter validates generated autos, can download a `.java` file, and can save directly to `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/` while running from Vite. Direct saves run `:TeamCode:compileDebugJavaWithJavac` and restore the previous file if the generated Java does not compile.
- `Settings → Field Map` offers **BIOBUZZ Field (2026-2027)**, the default-list entry above DECODE. `public/fields/biobuzz.webp` is **Pedro Pathing's own** field image, taken byte-for-byte from Pedro-Pathing/Visualizer at `e976a746` ("the field is buzzing!" + "Fix biobuzz field orientation"), 1080². It replaced a 4096² render this repo generated from the Competition Manual V1; that generator (`scripts/gen_biobuzz_field.py`) was removed so running it could not silently overwrite the upstream image — it is in git history. Upstream ships no BIOBUZZ obstacles, so the BIOBUZZ preset in `src/config/defaults.ts` is this repo's, re-measured off Pedro's image (0.13 in a pixel) so the outlines sit on what it draws; the DECODE goals were synced to upstream's outlines at the same commit. Orientation is the manual's and upstream's alike: audience at the bottom, red ALLIANCE on the left, so x = 0 is the red wall and y = 0 the audience wall. BIOBUZZ is the default field map, as upstream's is. Because every settings change saves the whole object, a browser that ran an earlier build has `decode.webp` stored whether anyone chose it or not, so settings schema 1.1.0 moves a pre-1.1.0 `decode.webp` to BIOBUZZ once; the next save writes 1.1.0, so picking DECODE afterwards sticks. Any other saved field is left alone. A `.pp` whose settings carry no field map is also reviewed against BIOBUZZ by the MCP now.
- Field obstacles follow the field map. `getDefaultShapes(fieldMap)` returns that field's preset — DECODE's two goals, or BIOBUZZ's HIVE Frame and four FLOWERS — and Into The Deep, Centerstage and custom images start with no obstacles rather than another season's. Picking a different field map in `Settings → Field Map` swaps the obstacles with it; if you have edited them, it asks first and cancelling keeps yours. Loading a project never swaps them: only a user pick reaches the handler, so a saved route keeps the obstacles it was drawn against. The BIOBUZZ preset is what is in the way at drive height, not everything the field has. Almost none of the HIVE Structure is: the bottom of a HIVE is 25.5 in above the TILES (Fig 9-10) and an FTC ROBOT is at most 18 in tall, so a ROBOT drives under the CELLS, under the crossbar at 43.95 in, and under the Frame legs climbing to it — the centre of the FIELD is a crossing, not a wall. The obstacle is the two base rails on the floor (~2 in wide, 38.95 in long, 49.46 in apart outside to outside), and nothing else of the HIVE. The FLOWERS are obstacles all the way down: their lower ring sits on the TILES.
- `mcp/` is an MCP server that reviews a `.pp` route from a Claude chat: collisions, the AUTO midline rule, the time estimate against the 30 s AUTO period, per-path lengths and times, and the route warnings. Two tools, `review_path` (a file path or the file's contents, with optional `alliance`, `margin`, `field`, `step`, `autoSeconds`) and `list_fields`. It is stdio JSON-RPC with **no dependencies** — no MCP SDK — and it does not reimplement any analysis: esbuild bundles the visualizer's own modules into it, so a review in a chat is the review the editor would give. Two modules were extracted to make that true: `src/utils/projectFile.ts` (the loader the editor now reads files through, so there is one normalization, not two) and `src/utils/pathReview.ts` (the review itself, and its text formatting). Build with `npm run build:mcp` or `./gradlew :PedroVisualizer:pathMcp`, which prints the Claude Code and Claude Desktop config with the absolute path filled in; `mcp/dist/` is gitignored on purpose, since a stale bundle would silently disagree with the editor. A browser chat on claude.ai cannot run a local stdio server — Claude Desktop or Claude Code can. See `mcp/README.md`. Note on the gradle tasks: `npmInstall` runs `npm ci`, which DELETES `node_modules` before reinstalling and cannot delete rollup's native module while a dev server has it loaded — it dies half way with `EPERM -4048` and leaves the tree in pieces. Nothing in the everyday path depends on it any more: `npmBuild`, `npmDev`, `visualizer` and `npmBuildMcp` depend on `npmEnsureDeps`, which runs the incremental `npm install` and only when `node_modules/.bin` is missing. Use `npmInstall` for a deliberate clean reinstall, with the dev server stopped.

Path chaining:

- Consecutive paths are followed as one Pedro path, `Paths.path(p1, p2, ...)`.
  Pedro 3's Foresight brakes only for the end of it: when it would start
  braking on a path that is not the last, it skips to the next one instead
  (`ForesightConfig.pathSkip`, on by default) — so the robot drives straight
  through every waypoint inside the chain instead of decelerating and settling
  on each one. (Pedro 2 got the same result from `PathChain`'s
  `DecelerationType.LAST_PATH`.)
- The time estimate profiles a chain as one accelerate/cruise/decelerate move,
  so interior paths are timed at cruise speed. Four collinear paths take 4.3s
  chained versus 8.0s stopping at each.
- A chain is cut only where the robot genuinely has to stop: a wait or event
  step follows, a repeat loop starts its next pass, a block ends, the next path
  uses a different speed, a path can be switched off at runtime, or `Stop at
  end` is ticked on the path. `buildChainRuns` owns those rules and both the
  estimate and the exporter read them, so the auto stops exactly where the
  editor says it will.
- Each path row shows `Chains on` or `Stops`, the field marks every full stop
  (Settings → Stop Points), and Telemetry counts them. Stops are what cost time
  now, so they are the thing worth seeing.

Turn model:

- The robot never stops to turn. Not even at the head of a chain:
  `follower.follow` starts driving and correcting heading in the same command,
  so a stationary rotation was always fiction.
- Turning still costs time, because driving and turning draw on one motor-power
  budget — the drivetrain mixes the pathing and heading vectors and normalizes
  them into wheel power. It is charged as a cap on speed, applied where the
  turning happens: sweeping the heading `dθ` per inch travelled means the robot
  can go no faster than `aVelocity ÷ dθ/ds`, and with the budget shared the
  cruise speed drops to `1 ÷ (1/maxVelocity + coupling · (dθ/ds) / aVelocity)`.
- `Settings → Turn Coupling` (0..1, default 1) sets how much of the budget the
  two share, so it can be calibrated against the robot instead of guessed. At 1
  the per-inch costs of driving and turning add — a 180° sweep over 40 in halves
  the cruise speed to 20 in/s, exactly saturating the drivetrain. At 0 only the
  hard spin-rate limit applies.
- The heading rate is read along the path, so an S-curve on a tangential path or
  a heading curve that front-loads its turn slows the robot where the turning
  actually is. It is smoothed over half the robot's width, because a robot
  blends a heading change over roughly its own length rather than snapping to it
  at a point.

Cornering:

- Speed through a curve is capped at `sqrt(grip × radius)`: holding radius `r`
  at speed `v` needs `v² / r` sideways, and past what the wheels give the
  follower cannot hold the line. `Settings → Cornering Grip` is that limit in
  in/s², measurable by driving an arc until the robot slides.
- A chain is profiled numerically rather than as a trapezoid — the curve is
  sampled, each sample capped by its own radius, then a forward pass keeps
  acceleration in range and a backward pass leaves room to brake for what is
  coming. On a straight it reproduces the trapezoid exactly.
- Grip is a property of the wheels, so the per-path speed scale does not touch
  it. A radius tighter than half the robot is treated as pivoting rather than
  cornering, since sampled geometry reports a near-zero radius at a hard join
  between two paths and the turn model already covers that cost.

Steady speed:

- A Bezier's parameter is not proportional to its arc length — with control
  points bunched to one side the curve can cover ground twenty times faster at
  one end than the other. Every path carries an arc-length table so a distance
  maps back to the right parameter; advancing the parameter in proportion to
  distance instead made the robot visibly surge and stall along a path it
  should cross at a steady speed.
- The profile is built from samples spread evenly along the curve itself, not
  at evenly spaced parameters and not by interpolating the chords between them.
  Curvature read across three nearly-touching or unevenly spaced points is
  mostly rounding error, and a noisy curvature reading becomes an oscillating
  speed cap.
- Distances, chain offsets and arc-length tables all live on one ruler. Mixing
  a chord-measured length with a sampled one makes the position drift and then
  snap back at every path boundary.
- Within a profile interval the robot holds a constant acceleration, so time
  maps to distance as a quadratic. Interpolating linearly held the speed flat
  across each interval and stepped it at every boundary — smooth acceleration
  chopped into stairs.
- Turning is charged as a cap on speed where the turning happens, not as a
  stretch applied to a whole path. Stretching per path made the speed jump
  wherever two chained paths turned by different amounts.

Velocity gradient:

- Coloured from the same numbers as the time estimate, never from the path on
  its own. Two things that are invisible from a single path decide the speed:
  the profile belongs to the whole PathChain, so a path in the middle of one is
  driven at cruise rather than ramping up and back down; and turning slows the
  path it happens on. Both come from the timeline, so the colours cannot
  disagree with the clock.

Heading continuity:

- A path states a heading *goal*; where two paths meet that goal can jump — a
  tangential path after a linear one picks up wherever its curve points. The
  robot cannot jump, so it rotates toward the new goal at `aVelocity` while it
  keeps driving. `headingAlongPath` blends over that catch-up window, and the
  animated robot, the swept-area preview and the onion layers all read it, so
  the heading is continuous everywhere.
- The head of a chain never needs a catch-up: the robot turns in place there and
  that rotation is timed.
- A path row shows `Heading jump N°` when its goal does not pick up where the
  previous path left off. On a linear path the chip is a button that restarts it
  at the incoming heading; on a tangential path the geometry decides, so it is
  only a warning. The exporter warns above 20°.
- Switching a path to linear seeds its start heading from the heading the robot
  arrives with, so the common way of creating a jump no longer does.

Robot picture from CAD (`src/utils/cadRobot.ts`):

- `Settings → Robot Configuration → Build from CAD` reads an STL (binary or
  ASCII) or OBJ export and renders a top-down picture of the real robot. Both
  formats are parsed here rather than pulling in a mesh library.
- Pick the model's up axis and sign, spin it in the plane until the robot's
  forward points right — the field draws the image that way at heading 0 — and
  say what units the CAD is in.
- The view basis is chosen so `right × up` equals the axis being looked down,
  which keeps the render from coming out mirrored and putting every asymmetric
  feature on the wrong side.
- Faces are painted back to front and shaded by how squarely they face the
  camera, so the top deck reads bright and the sides fall away.
- `Use image and size` also sets the robot dimensions from the projected
  footprint. `rWidth` is the extent along forward (the image's horizontal
  axis), `rHeight` is across.
- A real export runs to hundreds of thousands of triangles, so the work is split
  in two. Reading, parsing, centring and per-face normals all happen once at
  upload behind a progress bar; changing the axis or dragging the rotation only
  re-projects what is already prepared.
- The redraw is a scanline rasteriser with a depth buffer, not a painter's-order
  pass. Sorting every face and making a canvas call per triangle was what made
  dragging crawl; rasterising needs no sort at all, costs what the covered
  pixels cost rather than what the mesh weighs, resolves overlapping decks
  exactly, and leaves no seams between neighbouring triangles.
- Buffers are reused between frames, and a drag renders at draft resolution with
  the supersampled version landing once the controls stop moving. On a 19 MB,
  400,000-triangle STL that is a first preview at ~110 ms and a rotation drag at
  ~30 fps.

Clearance (`src/utils/clearance.ts`):

- A path is a curve through the middle of the robot, and the middle of the robot
  fits places the robot does not. What is checked is the footprint, planted every
  0.75in along the route and measured against the obstacles and the field walls.
- It is checked at the heading the robot actually holds, taken from the same
  heading model the animation uses. That is the whole point: an 18in robot clears
  a 20in gap square-on and cannot fit it at 45°, because its diagonal is 25.5in.
  Two of the three problems on the first real auto this ran against were exactly
  that — clearances that only exist because the robot is turned.
- `Settings → Safety Margin` is how close the robot may come before a path is
  flagged. Inside the margin is amber, overlapping is red, and both are drawn on
  the field as the robot's own shape at the worst point, so the problem is a
  picture rather than a number.
- Overlap is measured as how far past the boundary the intrusion reaches, not as
  the distance between outlines. Once two shapes cross, that distance is zero
  however deep the robot is buried, so a path driven straight through an obstacle
  would report as just touching.
- Samples are spread evenly along each curve. Stepping the curve parameter would
  sample densely at one end and stride past a whole obstacle at the other, for
  the same reason the speed profile cannot use it either.
- Where one path ends and the next begins is one pose, not two, so it is checked
  once. Counting it twice would report a single tight corner as two problems. A
  branch starts somewhere else, so the join has to actually line up.
- A CAD upload also stores the robot's real outline, and clearance uses it
  instead of the bounding rectangle. A robot with a corner intake sweeps a
  hexagon, and the rectangle claims material where there is only air, so gaps it
  genuinely fits through would read as collisions. The outline is the convex hull
  of the projected footprint, reduced to the extreme point of each thin column
  and row first because a real export has over a million vertices — a reduction
  that cannot shrink the shape, and what it could shave off is added back. It is
  stored with the size it was measured at, so editing the robot's dimensions
  rescales it rather than silently checking the wrong robot.
- The chips on the path rows, the shapes on the field, the Telemetry readout and
  the export warnings all read one report, so they cannot disagree.

Fix collision issues:

- The chip on a flagged path is a button that moves a waypoint on X and Y until
  the robot clears. It searches outward for the smallest move that works, since
  the smallest change is the one least likely to undo whatever the path was
  placed for.
- Handles are tried in order of what they cost to accept, and the first that
  reaches the safety margin wins. A **control point** only bends the curve
  between two waypoints the robot still hits, so it is nearly free and goes
  first; an **endpoint** changes where the robot parks, usually a scoring
  position; the **previous path's endpoint** goes last, because it edits a
  different path than the button that was pressed. A collision partway along a
  path is normally a control-point problem and out of an endpoint's reach
  entirely.
- The first stretch of a path is pinned to wherever the previous path left the
  robot, so a collision there is not really that path's at all: its own endpoint
  and control points are downstream of the problem and none of them can move it.
  That is why the button has to be able to reach back. On the first real auto
  this ran against, three of the five collisions were at a hand-off like that,
  and the button reported it could do nothing while pressing the *other* path's
  button fixed them.
- When nothing works, the message names the thing that would have to change
  rather than shrugging: which path hands the robot over, whether that endpoint
  is locked or comes from a pose variable, or that the gap is simply tighter than
  the robot. A path handed a bad pose by a pose-variable endpoint says so —
  that position is owned by the variable, and no amount of moving points here
  will shift it.
- The starting point has its own button, labelled `fix start position only` so it
  is clear it moves where the robot is staged and nothing else. It is a separate
  button rather than one more handle on the path button. It is judged on where the robot is *staged*, not on the whole
  first path, because a path that drives through a goal is that path's problem
  and re-staging cannot fix it. Mixing the two objectives made the search
  re-stage the robot to gain an inch at the start while leaving the path driven
  into a wall. The rest of the path still has a veto: a move that makes it worse
  scores as that worse number. It also reaches further, 24in against 14in, since
  a robot parked inside a goal has to come most of its own width to get out.
- The search spirals rather than pushing away from the obstacle, because a
  handle governs the shape of the whole curve: the direction that clears a
  problem in the middle of a path is often not the direction away from the thing
  being hit.
- The path that follows starts where this one ends, so both are measured on
  every endpoint candidate. A move that traded one collision for another would
  not be a fix, and none is offered in that case.
- Candidates are scanned against a coarsely sampled route and the winner is then
  re-measured at full resolution; anything that does not survive that is
  discarded. Scanning at the real resolution made one click take most of a
  second. A fix also has to be worth making — it must move at least 0.02in, buy
  at least 0.05in, and actually end up clear. Without those the search happily
  reported nudging a waypoint half an inch to go from 0.99in to 0.99in, and
  reshaping a path by a foot to end up still overlapping.
- Clearing the obstacle and reaching the safety margin are separate outcomes and
  the button says which it got. It aims for the plateau rather than for bare
  contact, since stopping at the first move that merely gets off the obstacle
  leaves the robot at 0.00in.
- A locked point or one driven by a pose variable is left alone — the pose
  variable owns the position, so writing x/y would not stick — and the chip stays
  a plain label there. Where nothing can help, the button says so instead of
  moving the path for nothing.
- It is one edit on the undo stack.

Fix all collisions:

- One button in the step toolbar walks everything currently flagged, worst
  first, applying the best fix for each until nothing more improves. The
  per-path buttons each fix one thing; a chain of hand-offs would otherwise have
  to be clicked through one at a time. On the first real auto this ran against
  it reported `6 moves — everything clears by 1in`, from five collisions.
- Targets are re-derived after every applied fix rather than planned up front:
  moving one waypoint moves the start of the path after it, so what is still in
  trouble changes as it goes. A target that cannot be fixed is not retried, since
  its geometry has not changed and retrying would spin.
- Poses are tried before the paths bound to them. A path whose endpoint a pose
  owns cannot be fixed on its own, so going path-first would only burn a round
  marking it unfixable.
- It yields to the page between rounds. Each round runs a few hundred candidate
  measurements and would otherwise freeze the frame.

Pose variables:

- A point bound to a pose takes its position from the variable, so the path it
  sits on cannot move it — writing x/y on the point is overwritten on the next
  resolve. The pose itself is a handle, and the Variables tab shows a fix button
  on any pose something flagged depends on.
- Moving a pose moves everywhere it is used, so every bound path and the path
  after each of them is measured on every candidate. A move that clears one and
  buries another is not offered.
- The fix is written to the variable, not to the points, and the usual sync
  carries it to everything bound to it.

Dragging numbers:

- The drag lives on the field's **label** — the `X:`, `Y:`, `Start:`, `Deg:` next
  to the box — not on the box. Hover a label for an `ew-resize` cursor and drag
  sideways. Shift is ten times coarser for crossing the field, Alt ten times
  finer for the last hundredth. Typing a coordinate is fine for a first guess and
  bad for the tenth, and the field redraws as the drag goes, so a position can be
  dialled in by eye instead of by retyping.
- On the input it would have had to tell a click for the caret apart from the
  start of a drag, and a stray few pixels while clicking would nudge the value.
  A label has no other job, so there is nothing to disambiguate: labels drag,
  boxes type.
- A label whose field holds an expression goes inert — no cursor, no drag. The
  field is owned by whatever variable it names, and writing a number over it
  would silently cut the link; the variable itself drags in the Variables tab,
  and everything reading it follows.
- `scrubbable` in `src/utils/scrub.ts` is a Svelte action, so a label attaches it
  in one line with the same value and callbacks its input already gets. The
  labels `ExpressionInput` renders itself carry it too, which is what covers the
  Variables tab. Headings ask for a bigger step: at the coordinate default a full
  turn would need a 7200px drag.
- The value is computed from where the press started rather than accumulated, so
  a drag out and back lands exactly where it began instead of drifting by the
  rounding of every frame between. It commits once at the end, so a drag is one
  entry on the undo stack rather than one per frame.

Tuned constants from TeamCode (`src/utils/teamcodeConstants.ts`):

- `Settings → Motion Parameters → Load from TeamCode` reads the robot's own
  Pedro 3 `ForesightConfig` instead of asking anyone to guess it. Foresight is
  configured with physical quantities — `maxAchievableForwardVelocity`,
  `naturalForwardDeceleration`, an optional `maxAccelerationConstraint` — and
  they sit two directories away from the visualizer.
- Values are read from `c.<field>.set(<argument>)` in the config lambda. An
  argument that names a `static final double` in the same file is resolved
  through it, which is how the constants files keep one number in one place.
- **Placeholders are never imported.** Until the Foresight Tuner has been run,
  TeamCode carries placeholder constants (named `PLACEHOLDER_*`) and
  `FORESIGHT_MEASURED = false`. The dialog shows the flag and lists each
  placeholder, with its value, under "still hand-tuned" — replacing one guess
  with another dressed as a measurement would be worse than leaving it.
- History: under Pedro 2 the first auto this ran against had `maxVelocity` at
  40 in/s against a configured 73.9 and `maxDeceleration` at 30 in/s² against
  197.1, and the estimate went 30.99s to 25.07s. Both of those Pedro 2 figures
  turned out to be unmeasured (73.9 dates from the constants skeleton, before
  the pods existed) or wrong (the robot's measured coast-down is 40 in/s², not
  197.1), which is exactly why placeholders are now reported, not applied.
- The constants come from `TeamCode/.../pedroPathing/Constants.java`, the one
  (swerve-only) constants file since 2026-10-01; `config.jsonc` and the mecanum
  file are gone.
  The forward speed is the cap; the sideways speed and coast-down are shown as
  context only, since paths are profiled at one speed.
- Comments are stripped before anything is read. These files park alternates
  behind `//`, and picking one up would silently import a number the robot is
  not using.
- Nothing is applied until it is shown: the dialog lists each value, what it
  changes it from, and the Java call it came from.
- What Pedro does *not* measure is listed just as explicitly: no turn-rate
  limit (the heading brake coefficients describe how far a spin carries on, not
  how fast it can go), nothing matching this tool's turn coupling, no cornering
  limit, and no acceleration limit unless `maxAccelerationConstraint` is set.
- `naturalForwardDeceleration` is flagged as a **floor**. It is how fast the
  robot slows with no power applied; Foresight brakes actively on top of that,
  so the robot stops at least this hard and the estimate leans pessimistic.
  (Under Pedro 2 the same quantity was shown here as a ceiling — it never was.)
- The endpoint only exists under the dev server, since that is the only place
  the repo is on disk to read from, and the dialog says so if it is missing.

Onion layers:

- The bodies drawn along the route are for judging clearance by eye, so they use
  the robot's real outline when CAD has given us one rather than a bounding box.
  Drawing a box around a robot whose shape is known would show the wrong shape
  for the one job they have, and would disagree with both the picture on the
  field and the clearance check.
- `getVisibleOnionLayers` takes the playback position as an argument instead of
  reading it from the component. Svelte works out what a reactive block depends
  on from the names appearing in it, and reading `percent` inside the helper left
  the blocks that draw the bodies with no dependency on it: with `Next point
  only` on they froze on whichever path was current when something else last
  changed, while the swerve overlay — whose block does mention the playback
  position — kept following the robot. The two drifted apart on screen.
- The geometry and the filter are separate reactive steps, because only the
  filter depends on playback. Walking the route to place every body is the
  expensive half and it was being redone on every animation frame for a result
  that had not changed — at a 1in spacing that alone blocked the main thread for
  37ms a frame, 55ms with swerve modules on, which is 18fps before anything else
  on the page gets a turn. Split, and with the Two.js bodies rebuilt only when
  the visible set actually changes, the same scene costs 4-5ms a frame, the same
  as having them switched off.
- Svelte treats any assignment of an object as a change, so the block that builds
  the bodies still runs every frame; what stops it allocating a few thousand
  anchors for an identical picture is an explicit guard on the layer array's
  identity, the selected path, the colour and the field scale.
- Spacing is measured along the route, so the honest check is how many bodies
  each path gets for its length. Straight-line distance between consecutive
  bodies is shorter wherever the route doubles back, which is not a spacing
  error — on the first real auto this ran against, three joins reverse and read
  as 0.17in, 0.75in and 2.08in gaps against a 3in spacing while every path's
  count was exact.

Waits and events inside a loop or an `if`:

- A repeat loop or an `if` block holds an ordered list of members that can be
  paths, waits or events. It used to hold a bare list of path ids, so a pause
  between two paths inside a loop was not representable at all — which is why
  dragging one in did nothing. Files written before this are converted on load,
  and the old field stays on the type as deprecated so a project saved months
  ago still opens.
- A wait inside a three-pass loop happens three times, in its place in the
  order: `path, wait, path` becomes
  `path wait path  path wait path  path wait path`. A loop that cannot run
  spends none of them.
- In the generated Java a hold takes a slot in the loop's `AutoPath[]` with a
  null path, and a parallel `long[]` carries its duration; `followRepeatStep`
  reads a null path as "hold here" rather than something to drive. An event
  member fires the same generated method a top-level one would. Verified by
  compiling the output against the real SDK.
- Only paths are reconciled against `lines`. A wait carries its own data, so
  there is nothing for it to have gone stale against, and dropping one there
  would quietly delete a step from inside a loop.

Route invariants (`src/utils/sequence.ts`):

- `buildRoute` is the single walk of the step list. The field drawing, the time
  estimate, the animation, the overlays and the exporters all read the route
  from it, so they cannot disagree about order, start points, or which steps run.
- Every path appears in the step list exactly once. `reconcileSequence` drops
  steps whose path is gone and appends any path the list forgot, so a path can
  never sit in the editor while being invisible to the field and the exporters.
- Adjacent `if` blocks are one if / else-if chain: every branch starts where the
  chain is reached, and only the first branch whose condition holds runs.
- A step whose `Enabled if` is false costs no time and does not move the robot,
  matching what the generated Java does. It is still drawn, faded.
- A parallel event with a zero duration fires once. The generated
  `startParallelEvent` finishes it immediately, so the field and telemetry show
  it as a momentary trigger rather than as active until the end of auto.

- State effects (States tab): shoot, roll and flip are assigned per state name
  (an event or wait, a path event marker, a Pollen Pickup phase, or Driving) and
  stored in `settings.stateVfx`, so they save with the .pp. They are a pure
  function of time into the state, so scrubbing shows the exact frame, and roll
  and flip are fitted to whole turns so the robot lands flat. When states
  overlap the most specific one with an effect plays. They are editor-only
  visuals: nothing reaches the exporters, and GIF export does not include them.
  Shoot aims at the nearest goal in `SCORING_POINTS` (BIOBUZZ: the four HIVE
  CELLS), not at where the robot faces; old files saying "fire" play as shoot.

Run locally:

```powershell
npm install
npm run dev -- --host 127.0.0.1 --port 5173
```
