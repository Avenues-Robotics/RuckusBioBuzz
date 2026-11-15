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
        // Is the NECTAR shooter (turret + flywheel) mounted and wired? false (2026-11-15, it is not): an auto's
        // Shoot event fires on the pollen side alone (aimed + at speed), and TeleOp shoots ONLY on the right
        // trigger. true: both sides must be aimed and at speed - the original rule, in autos and TeleOp (TeleOp
        // then also fires by itself in the zone). Auto Dry Run 2026-11-15: with the nectar side required, 4/4
        // auto Shoot steps timed out with the gate shut.
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
        public static PIDFCoefficients POLLEN_PIDF = new PIDFCoefficients(10, 0.1, 0, 12.4);
        public static PIDFCoefficients NECTAR_PIDF = new PIDFCoefficients(500, 0, 0, 11.7);
    }

    @Config
    @Configurable
    public static class Turret {
        public static double[] nectarTurretOffset = {3.22, 0};
        public static double[] pollenTurretOffset = {-5.91, 0};
        public static double POLLEN_TURRET_CENTER = 0.5;
        public static double NECTAR_TURRET_CENTER = 0.5;
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
    }

    @Config
    @Configurable
    public static class Gate {
        public static double openPos = 0.347;
        public static double closePos = 0.482;
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
    public static class PredictiveAiming {
        public static double PREDICT_MS = 150;
        public static double[] CENTER = {0, 0};
    }

    @Config
    @Configurable
    public static class HivePosition {
        public static double xred = 58;
        public static double xblue = 84;
        public static double UP_POS = 90;
        public static double DOWN_POS = 52;
        public static double HALF_Y = 71;
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
        public static int xQuality = 5;
        public static int yQuality = 10;
        public static int[][] mapNectar = new int[yQuality][xQuality];
        public static int[][] mapPollen = new int[yQuality][xQuality];
    }

    @Config
    @Configurable
    public static class FieldView {
        public static double PUBLISH_INTERVAL_MS = 50;
    }
}
