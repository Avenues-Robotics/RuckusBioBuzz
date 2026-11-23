package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;

public final class SystemConstants {
    private SystemConstants() {}

    @Config
    @Configurable
    public static class TeleOp {
        public static double TRANSLATION_PRIORITY = 0.75;
        // Field-oriented drive: the left stick moves the robot in the DRIVERS' frame - stick forward = away from
        // your own alliance wall (red +x, blue -x), whichever way the robot faces - using the odometry heading.
        // Right after an auto or a corner relocalize that frame is exact; otherwise point the robot away from the
        // drivers and press driver 1 OPTIONS to re-zero it (drive frame only - odometry and aim are untouched).
        // false = robot-centric (stick forward = the robot's front), the behaviour before 2026-11-15.
        public static boolean FIELD_CENTRIC = false;
        // Driver 2 aim corrections (helpers/AimOffsets): one d-pad left/right press moves the aim point this far
        // sideways (in, coach's perspective, judged from the last shot); one up/down press adds this to the
        // flywheel target (ticks/s, the regression's y-intercept). GUESSES - set them from real shots.
        public static double AIM_STEP_IN = 1.0;
        // ticks/s at the MOTOR: 18 at 35:21 is the same wheel-speed step 25 was at 24:20 (25 x 1.2 / 1.667).
        public static double SPEED_STEP = 18;   // 35:21 installed 2026-11-16 (was 25 at 24:20)
        // Shooter lock (driver 2 OPTIONS): turret straight ahead, the robot's heading aims instead. P on a
        // projected heading (heading + w*LIN + w|w|*QUAD), the shape and the values of Foresight's heading
        // controller (Constants.headingKP / headingBrake*, measured 2026-11-15). NOT tested in TeleOp.
        public static double AIM_KP = 7.7173;
        public static double AIM_BRAKE_LINEAR = 0.050501;
        public static double AIM_BRAKE_QUADRATIC = 0.019853;
        public static double AIM_MAX_TURN = 0.8;
        /** |right stick x| above this turns the robot by hand even in lock mode. */
        public static double AIM_STICK_OVERRIDE = 0.10;
        // Relocalize (driver 1 holds SHARE/BACK this long): the pose is set to this RED-frame pose, rotated
        // 180 deg about the field centre for blue. A corner, so two walls fix x, y and heading at once (operator,
        // 2026-11-15). Red: the red-wall / audience-wall corner, robot FACING the red wall (heading 180) with its
        // LEFT side on the audience wall - the side the auto start uses (START_STEP y 8.561 = centre to the left
        // side). Blue: the far corner of the blue wall, facing the blue wall, left side on the far wall. Corners are
        // clear at drive height (field/biobuzz_field.json: the nearest FLOWER is 43.6 in along each wall).
        // MEASURED 2026-11-16 on a competition field: pose set at the audience-wall centre seam (x 70.75 by eye, back
        // on the wall, y 8.561), driven/pushed into this corner 3 times, odometry read each time: x 10.71 / 11.10 /
        // 11.11 (10.97 +/- 0.23), y 9.67 / 9.57 / 9.84 (9.69 +/- 0.14). Was 8.561 / 8.561 (square-robot guess).
        // Accuracy ~0.5 in (the start's seam placement); a tape measure of the robot would confirm it.
        // Heading hold (2026-11-16): with the left stick translating and the right stick centred, the robot holds the
        // heading it had when the turn stick was let go. Why: on a competition field, open-loop 0.8-power straight
        // runs yawed 7-44 deg/s on a fresh battery, direction varying run to run, with the pods within 4 deg of target
        // in some of them - the drive wheels (speed / traction), not the steering, so no pod gain fixes it.
        // Same controller shape as the shooter lock (P on heading projected by the braking lookahead), gentler gain:
        // heading corrections swing every pod. Off at rest (no pod jitter standing still), off under the turn stick
        // (above HOLD_STICK_DEADBAND) and in SHOOTER LOCK. The target is captured HOLD_CAPTURE_S after the stick is
        // released, or earlier once the robot turns slower than HOLD_CAPTURE_RATE_DEG_S, so it keeps the heading the
        // driver turned to instead of snapping back.
        // Gains measured 2026-11-17 through TeleOp's own drive code (TeleDriveProbe, 3x3 tiles, 11.6-12.7 V,
        // randomised interleaved, FINDINGS): the old kP 3.0 / max 0.5 drove a ~3 Hz heading limit cycle - yaw-rate
        // rms 57 vs 16 deg/s with the hold off, pod error 19.7 vs 4.0 deg (lines, n=4/arm, p<0.001). kP 1.8 and any
        // longer brake lookahead (AIM_BRAKE_LINEAR 0.15) also oscillate. kP 1.0 / max 0.25 at 0.6 power: heading drift
        // per 0.8 s run 3.6 vs 7.7 deg with the hold off ([-5.8, -2.4], p<0.001, n=30/31), wobble not different
        // (+1.6 [-3.1, +6.2] deg/s), wheel reversals +1.6/s (p=0.010). At 0.9 power mixed (reversals -5.4 deg drift,
        // p=0.038; lines n.s., wobble +14 deg/s p=0.089). Driver decides on/off (HEADING_HOLD is Dashboard-live).
        // kP 1.0 -> 0.6 (2026-11-18, 2x6-tile strip, fresh battery 12.0-13.2 V): kP 1.0 still fell into the ~3 Hz
        // limit cycle in 14 of 32 line/strafe runs at 0.6 power (yaw-rate rms > 25 deg/s, pods +/-30 deg), kP 0.6 in
        // 1 of 31 (Fisher p=0.0002); wobble 11.2 vs 25.7 deg/s (p=0.0001), pod error 4.0 vs 7.4 deg (p=0.001), heading
        // drift not worse (3.6 vs 4.4 deg, p=0.07). Bistable, and likelier on a fresh battery (3/11 at kP 1.0 on
        // 11.6-12.7 V yesterday). Mixer slew 550 or off did not fix it (4/12 each). Pod kS: flat.
        // (A same-day "pod kP/kD flat" note here was void: the probe's gain setter never reached the pods.)
        public static boolean HEADING_HOLD = true;
        public static double HOLD_KP = 0.6;
        public static double HOLD_MAX_TURN = 0.25;
        public static double HOLD_STICK_DEADBAND = 0.05;
        public static double HOLD_DRIVE_DEADBAND = 0.05;
        public static double HOLD_CAPTURE_S = 0.25;
        public static double HOLD_CAPTURE_RATE_DEG_S = 30.0;
        public static double RELOCALIZE_HOLD_S = 0.5;
        // Robot CENTRE when backed into red's audience corner, facing up the field (+y): robot 15 in wide x 18 in
        // long (operator, 2026-11-23), left side on the red wall (x = 0), back on the audience wall (y = 0).
        // Blue mirrors through the field centre (TeleOp.relocalizePose). Assumes the pose origin is the robot's
        // geometric centre (the Pinpoint pod offsets are measured from it).
        public static double RELOCALIZE_X = 7.5;
        public static double RELOCALIZE_Y = 9.0;
        public static double RELOCALIZE_HEADING_DEG = 90;
        /** Turret stays parked this long after BACK is released, so it does not swing into the wall. */
        public static double RESET_TIMER_SEC = 2;
    }

    @Config
    @Configurable
    public static class Cache {
        public static boolean VELOCITY_WRITES = true;
        public static boolean SERVO_WRITES = true;
        public static double REFRESH_MS = 250;
        public static double COLOR_READ_INTERVAL_MS = 50;
        public static double TELEMETRY_INTERVAL_MS = 100;
    }

    @Config
    @Configurable
    public static class Shooter {
        // Is the NECTAR shooter (turret + flywheel) mounted and wired? false (2026-11-15, it is not): automatic
        // shots (an auto's Shoot event, TeleOp's in-zone shot) need the pollen side alone aimed and at speed.
        // true: both sides must be. Auto Dry Run 2026-11-15: with the nectar side required, 4/4 auto Shoot steps
        // timed out with the gate shut. (Which shots are automatic is Shooter's autoFire, not this flag.)
        public static boolean NECTAR_MOUNTED = false;
        public static double MIN_GATE_OPEN_S = 0.3;
        public static double HOLD_SPEED_FRACTION = 0.20;
    }

    @Config
    @Configurable
    public static class Flywheel {
        public static boolean POLLEN_FLYWHEEL_REVERSED = false;
        public static boolean NECTAR_FLYWHEEL_REVERSED = false;
        public static double AT_SPEED_FRACTION = 0.05;
        public static double IDLE_FRACTION = 0.5;
        // Feed-forward voltage compensation (systems/Flywheel.compensateF): F x (F_NOMINAL_VOLTS / battery V).
        // 12.6 V is where POLLEN_PIDF F = 12.4 held 2400 within 0.9% (2026-11-15, n=4 steps); at 11.9 V the same
        // gains sat 2.5% low. Battery volts smoothed over F_COMP_TAU_S, re-applied every F_COMP_UPDATE_MS.
        public static boolean F_VOLTAGE_COMP = true;
        public static double F_NOMINAL_VOLTS = 12.6;
        public static double F_COMP_TAU_S = 1.0;
        public static double F_COMP_UPDATE_MS = 250;
        // POLLEN (10, 0.1, 0, 12.4) -> (45, 0.1, 0, 12.75), 2026-11-18, after the operator added flywheel inertia
        // (35:21 gearing, target 1728 ticks/s). Mechanism Tuner, tiles, volts per result. ff with 5 s holds (2 s
        // holds no longer reach steady state): F_full 13.66 / 13.61 at ~11.8 / 11.76 V = 12.75 at F_NOMINAL_VOLTS
        // (the tool sends F raw; this class compensates). Hold A/B at 1728, 11.6 V, n=6 interleaved: shipped in
        // band 0.19 of the time (never armed), (45, 0.1, 0, 13.6) 1.00 (+0.81 [+0.62, +1.00], p=0.0001). Steps
        // 864 -> 1728: P 45 rise 1.0 s (motor-saturated: P 70 no faster), overshoot 1.5%, ss error ~0 (P 25: -18
        // to -30). Real shots, 12.5-13.1 V, F 12.31 (= 12.75 compensated), n=10/arm interleaved, 0 failures each:
        // P 45 98.7 +/- 42.6 ms, P 70 85.4 +/- 27.3 ms (-13 [-47, +21], p=0.42 - P 70 not shown better).
        // Synthetic kicks no longer separate gains (400 ms disable dips the heavier wheel only ~5%).
        public static PIDFCoefficients POLLEN_PIDF = new PIDFCoefficients(45, 0.1, 0, 12.75);
        public static PIDFCoefficients NECTAR_PIDF = new PIDFCoefficients(500, 0, 0, 11.7);
    }

    @Config
    @Configurable
    public static class Turret {
        // Robot frame {forward, left}, in (PredictiveAiming.leadPose). Pollen is ~6 in BEHIND centre on the
        // centre line (operator, 2026-11-20) - TeleOp.aimTurn's shooter lock assumes that. {0, -5.91} (right of
        // centre) was tried 2026-11-18 and is wrong. Nectar not mounted: 3.22 forward is unconfirmed.
        public static double[] nectarTurretOffset = {3.22, 0};
        public static double[] pollenTurretOffset = {-5.91, 0};
        public static double POLLEN_TURRET_CENTER = 0.5;
        public static double NECTAR_TURRET_CENTER = 0.5;
        public static double RESET_POSE = 0.8;

        // Limits, servo units; every turret command (tracking, unwind, start) is clamped to them.
        // POLLEN, 2026-11-14 (operator): with the servo at 0.5 the turret faces straight, and the
        // mechanism's own limits ARE the servo's travel ends. The Axon MAX+ travels ~359 deg of
        // its 360 deg encoder over 500-2500 us (323.0 deg measured at 600-2400 on 2026-11-01,
        // servoRangeTest, x 2000/1800), so each end is only ~0.6 deg from the encoder wrap: an
        // overshoot past it reads as the far end and drives through the mechanism (it has broken
        // it, twice). A 1-deg stop (0.003) was NOT enough - it skipped at 0.003 on 2026-11-14.
        // 0.018 is the most that keeps 360 deg of turret: 360 deg needs 360 x 355/370 = 345.4 deg
        // of servo, leaving (358.9 - 345.4)/2 = 6.7 deg per side -> ~7 deg from the wrap.
        // Range 0.018-0.982 = 360.3 deg; an unwind (UNWIND_HYSTERESIS_DEG 10) lands 10.3 deg inside.
        // Overshoot at the ends is still NOT measured. If this skips too, 360 deg is not reachable
        // at full servo speed: slow the approach near the ends, or give up a few degrees.
        public static double POLLEN_TURRET_MIN = 0.018;
        public static double POLLEN_TURRET_MAX = 0.982;
        // NECTAR: not mounted on 2026-11-14 - still the narrow +/-18.7 deg guess, NOT measured.
        public static double NECTAR_TURRET_MIN = 0.45;
        public static double NECTAR_TURRET_MAX = 0.55;
        public static double POLLEN_TURRET_START = 0.5;
        public static double NECTAR_TURRET_START = 0.5;
        public static boolean POLLEN_TURRET_REVERSED = true;
        public static boolean NECTAR_TURRET_REVERSED = false;
        public static double[] POLLEN_TURRET_GEAR_RATIO = {355, 370};
        public static double[] NECTAR_TURRET_GEAR_RATIO = {355, 370};
        public static double SERVO_RANGE_DEG = 358.6;
        public static double PWM_LOWER_US = 500;
        public static double PWM_UPPER_US = 2500;
        public static double UNWIND_HYSTERESIS_DEG = 10;
        public static double UNWIND_SETTLE_S = 1.0;
        // Aimed = the turret is modelled as arrived: a model position chases the command at SPEED_DEG_S
        // (turret deg/s) and aimed waits until it is within AIM_TOL_DEG. 360 is NOT measured - it is the
        // speed UNWIND_SETTLE_S already assumes (a ~360 deg unwind in 1 s). Nothing senses the turret.
        public static double SPEED_DEG_S = 360;
        public static double AIM_TOL_DEG = 3;
        // Live tracking aims from the heading HEADING_LEAD_MS ahead (measured turn rate), for the servo's lag
        // behind a turning robot: loop + 20 ms PWM frame + the Axon's own response. 50 is ESTIMATED, not
        // measured; 0 turns it off. At 100 deg/s of turn, each 10 ms here moves the aim 1 deg.
        public static double HEADING_LEAD_MS = 50;
    }

    @Config
    @Configurable
    public static class Gate {
        public static double openPos = 0.347;
        public static double closePos = 0.5;
    }

    @Config
    @Configurable
    public static class Intake {
        public static double INTAKE_FEED_POWER = 1.0;
        /** Flips BOTH motors: which way is "intake" vs "reject". */
        public static boolean INTAKE_REVERSED = false;
        /** Per-motor flips so the two rollers pull the same way (they fight if one is wrong). Mechanism Tuner, Intake direction tab. */
        public static boolean INTAKE_LEFT_REVERSED = false;
        public static boolean INTAKE_RIGHT_REVERSED = true;
        public static double REVERSE_BRAKE_MS = 60;
    }

    @Config
    @Configurable
    public static class ColorCheck {
        public static double RED_OVER_BLUE = 0.93;
        public static double BLUE_OVER_RED = 1.62;
        public static double MAX_DIST_CM = 6.7;
    }

    @Config
    @Configurable
    public static class Scoopula {
        public static double scoopPos = 0;
        public static double unscoopPos = 0;
    }

    @Config
    @Configurable
    public static class Camera {
        /** TeleOp opens the webcam + pollen pipeline at init. Off = no camera thread at all. */
        public static boolean ENABLED = true;
        /** Looked up in the active robot config first; else the first webcam attached to the hub is used. */
        public static String WEBCAM_NAME = "Webcam 1";
        /** Capture size. PollenDetectionPipeline's area filters (minArea, singleBallAreaPx) assume 640 x 480. */
        public static int WIDTH = 640;
        public static int HEIGHT = 480;
        /** The webcam is mounted on its side: frames are turned this far CLOCKWISE (0/90/180/270) before detection,
         *  so the feed and the steering error are upright. 90 from the first feed 2026-11-23 (text read bottom-up). */
        public static int ROTATE_DEG = 90;
        /** Horizontal field of view of the UPRIGHT (rotated) frame, deg - with ROTATE_DEG 90 that is the camera's
         *  vertical FOV. 43 is ESTIMATED (Logitech C920 class), not measured: check with a ball at a known offset. */
        public static double HFOV_DEG = 43;
        /** Camera lens on the robot, {forward, left} in from the robot centre. ESTIMATED (front, centre line). */
        public static double[] CAMERA_OFFSET = {9, 0};
        /** Field page feed (/field, Camera tab): frames encoded per second, width, JPEG quality. Not in branch 2. */
        public static double STREAM_FPS = 10;
        public static double STREAM_WIDTH = 320;
        public static double JPEG_QUALITY = 60;
    }

    @Config
    @Configurable
    public static class PredictiveAiming {
        /** Zone check + shooter lock lead (predictPose / leadPose). Shoot-on-the-move aiming has its own below. */
        public static double PREDICT_MS = 150;
        // Turret shoot on the move (Turret.sotmLead): lead t = SOTM_FEED_MS + tof(d) ms, tof(d) = SOTM_TOF_MS[0]
        // + SOTM_TOF_MS[1]·d + ... (d = turret-to-goal, in). FEED = decision to ball leaving the shooter (gate,
        // feed); tof = ball flight. Defaults = the old flat 150 ms, which does NOT hit (operator 2026-11-23).
        // Neither part is measured yet - tune on the field.
        // OFF 2026-11-23 (operator: not worth having for now): turret and shooter lock aim from where the robot
        // IS, no velocity lead. The zone check still uses PREDICT_MS. Heading lead (Turret.HEADING_LEAD_MS, the
        // servo lagging a turn) is not SOTM and stays on.
        public static boolean SOTM_ENABLED = false;
        public static double SOTM_FEED_MS = 150;
        public static double[] SOTM_TOF_MS = {0, 0};
        public static double[] CENTER = {0, 0};
    }

    @Config
    @Configurable
    public static class HivePosition {
        // x = centre of each alliance's HIVE cells, measured 2026-11-20 off the field drawing (Pedro's
        // biobuzz.webp, 0.13 in/px, the image field/biobuzz_field.json was measured from): red cells span
        // x 47.8-67.7, blue 73.9-93.8, symmetric about the field centre. 45 / 72 (2026-11-18) were the red
        // rail and the gap between halves. UP_POS / DOWN_POS are shared by both alliances, but the drawing
        // is point-symmetric: red's cells span y 51.9-95.4, blue's 46.2-89.6 (see FINDINGS 2026-11-20).
        public static double xred = 64;
        public static double xblue = 92;
        public static double UP_POS = 90;
        public static double DOWN_POS = 52;
        public static double HALF_Y = 71;
        /** The aimed hive end switches only this far past HALF_Y (in): a lead pose near the line flicked it. */
        public static double HALF_Y_HYSTERESIS_IN = 4;
    }

    @Config
    @Configurable
    public static class Zone {
        public static double FIELD_SIZE = 141.5;
        public static double ROBOT_LENGTH = 18;
        public static double ROBOT_WIDTH = 18;
        public static double fieldx = 141.5;
        public static double fieldy = 141.5;
        public static int yOffsetNectar = 0;
        public static int yOffsetPollen = 0;
        public static int xQuality = 10;
        public static int yQuality = 10;
        public static int[][] mapNectar = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
        // Red frame: row y (audience -> far wall), column x (red wall -> blue wall); blue is turned 180 deg.
        // 2026-11-23: repainted by the operator on Mechanism Tuner's Zone map tab (over the field drawing), 10 x 10
        // cells of 14.15 in: the audience-wall row (y 0-14) full width, and y 14-28 for x 14-113.
        public static int[][] mapPollen = {{1, 1, 1, 1, 1, 1, 1, 1, 1, 1}, {0, 1, 1, 1, 1, 1, 1, 1, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
    }

    @Config
    @Configurable
    public static class FieldView {
        public static double PUBLISH_INTERVAL_MS = 50;
    }
}
