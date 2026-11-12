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
        public static PIDFCoefficients POLLEN_PIDF = new PIDFCoefficients(500, 0, 0, 11.7);
        public static PIDFCoefficients NECTAR_PIDF = new PIDFCoefficients(500, 0, 0, 11.7);
    }

    @Config
    @Configurable
    public static class Turret {
        public static double[] nectarTurretOffset = {3.22, 0};
        public static double[] pollenTurretOffset = {-5.91, 0};
        public static double POLLEN_TURRET_CENTER = 0.5;
        public static double NECTAR_TURRET_CENTER = 0.5;
        public static double POLLEN_TURRET_MIN = 0.0;
        public static double POLLEN_TURRET_MAX = 1.0;
        public static double NECTAR_TURRET_MIN = 0.0;
        public static double NECTAR_TURRET_MAX = 1.0;
        public static double POLLEN_TURRET_START = 0.5;
        public static double NECTAR_TURRET_START = 0.5;
        public static boolean POLLEN_TURRET_REVERSED = false;
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
        public static double openPos = 0;
        public static double closePos = 0;
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
