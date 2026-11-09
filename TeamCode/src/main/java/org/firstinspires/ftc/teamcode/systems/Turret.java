package org.firstinspires.ftc.teamcode.systems;

import org.firstinspires.ftc.teamcode.SystemConstants;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import org.firstinspires.ftc.teamcode.helpers.Alliance;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.helpers.HivePosition;
import org.firstinspires.ftc.teamcode.modules.Cache;
import org.firstinspires.ftc.teamcode.modules.PredictiveAiming;

public class Turret {
    // Tunables (centers, limits, gear ratios, offsets, PWM, unwind): SystemConstants.Turret.

    private final PredictiveAiming predictor;
    private Alliance alliance;

    private final TurretServo pollenTurret;
    private final TurretServo nectarTurret;
    private final HivePosition hivePosition;

    private boolean pollenAimed;
    private boolean nectarAimed;

    public class aimer {
        public double theta;
        public double distance;

        public aimer(double theta, double distance) {
            this.theta = theta;
            this.distance = distance;
        }
    }

    public static void configurePwm(Servo servo) {
        if (servo instanceof ServoImplEx) {
            ((ServoImplEx) servo).setPwmRange(new PwmControl.PwmRange(SystemConstants.Turret.PWM_LOWER_US, SystemConstants.Turret.PWM_UPPER_US));
        }
    }

    public Turret(HardwareMap hardwareMap, PredictiveAiming predictor, HivePosition hivePosition, Alliance alliance) {
        this.predictor = predictor;
        this.alliance = alliance;
        pollenTurret = new TurretServo(hardwareMap.get(Servo.class, "pollenTurretServo"),
                limit(SystemConstants.Turret.POLLEN_TURRET_START, SystemConstants.Turret.POLLEN_TURRET_MIN, SystemConstants.Turret.POLLEN_TURRET_MAX));
        nectarTurret = new TurretServo(hardwareMap.get(Servo.class, "nectarTurretServo"),
                limit(SystemConstants.Turret.NECTAR_TURRET_START, SystemConstants.Turret.NECTAR_TURRET_MIN, SystemConstants.Turret.NECTAR_TURRET_MAX));
        this.hivePosition = hivePosition;
    }

//in form dist, theta
    public void update(){
        pollenAimed = pollenTurret.track(aimPollen().theta, SystemConstants.Turret.POLLEN_TURRET_CENTER, SystemConstants.Turret.POLLEN_TURRET_REVERSED, SystemConstants.Turret.POLLEN_TURRET_GEAR_RATIO,
                SystemConstants.Turret.POLLEN_TURRET_MIN, SystemConstants.Turret.POLLEN_TURRET_MAX);
        nectarAimed = nectarTurret.track(aimNectar().theta, SystemConstants.Turret.NECTAR_TURRET_CENTER, SystemConstants.Turret.NECTAR_TURRET_REVERSED, SystemConstants.Turret.NECTAR_TURRET_GEAR_RATIO,
                SystemConstants.Turret.NECTAR_TURRET_MIN, SystemConstants.Turret.NECTAR_TURRET_MAX);
    }

    public boolean unwinding(){
        return pollenTurret.unwinding() || nectarTurret.unwinding();
    }
    public aimer aimNectar() {
        return aimFrom(predictor.leadPose(SystemConstants.Turret.nectarTurretOffset));
    }

    public aimer aimPollen() {
        return aimFrom(predictor.leadPose(SystemConstants.Turret.pollenTurretOffset));
    }

    public boolean pollenAimed() { return pollenAimed; }
    public boolean nectarAimed() { return nectarAimed; }

    private aimer aimFrom(Pose turret) {
        Pose target = hivePosition.target(predictor.leadPose(), alliance);
        double dx = target.x() - turret.x();
        double dy = target.y() - turret.y();
        double dist = Math.hypot(dx, dy);
        double theta = AngleUnit.normalizeRadians(Math.atan2(dy, dx) - turret.heading());
        return new aimer(theta, dist);
    }

    private static final class TurretServo {
        private final Servo servo;
        private double position = Double.NaN;
        private long settleUntilNanos;
        private final boolean cacheable;
        private final double pwmLower, pwmUpper;
        private final Cache cache = new Cache();

        TurretServo(Servo servo, double start) {
            this.servo = servo;
            configurePwm(servo); // SDK default 600-2400 us only reaches ~90% of the Axon's travel
            cacheable = servo instanceof ServoImplEx;
            pwmLower = SystemConstants.Turret.PWM_LOWER_US;
            pwmUpper = SystemConstants.Turret.PWM_UPPER_US;
            position = start;
            write(start);
        }

        private void write(double p) {
            long key = Cache.servoKey(p, servo.getDirection() == Servo.Direction.REVERSE, pwmLower, pwmUpper);
            if (cache.changed(key, SystemConstants.Cache.SERVO_WRITES && cacheable, SystemConstants.Cache.REFRESH_MS)) {
                servo.setPosition(p);
            }
        }

        boolean track(double theta, double center, boolean reversed, double[] gearRatio, double min, double max) {
            double lo = Math.max(0, Math.min(min, max)), hi = Math.min(1, Math.max(min, max)); //tolerate swapped limits
            min = lo;
            max = hi;
            double range = SystemConstants.Turret.SERVO_RANGE_DEG * gearRatio[1] / gearRatio[0]; //turret degrees across the full servo travel 0-1
            double turn = 360.0 / range; //one full turret revolution in servo position units
            double last = Double.isNaN(position) ? center : position; //last commanded position or center on the first call
            double base = center + (reversed ? -1 : 1) * Math.toDegrees(theta) / range; //servo position that points at theta
            double follow = base + Math.round((last - base) / turn) * turn; //same aim shifted by whole turns to the copy closest to last
            double overshoot = Math.max(min - follow, follow - max); //how far follow is past a limit; <= 0 means it is reachable
            boolean aimed = false;

            if (overshoot <= 0) {
                position = follow;
                aimed = true;
            } else if (overshoot <= SystemConstants.Turret.UNWIND_HYSTERESIS_DEG / range) {
                position = limit(follow, min, max);
            } else {
                double unwound = follow - Math.signum(follow - (min + max) / 2) * turn;
                if (unwound >= min && unwound <= max) {
                    position = unwound;
                    settleUntilNanos = System.nanoTime() + (long) (SystemConstants.Turret.UNWIND_SETTLE_S * 1e9);
                } else {
                    position = limit(follow, min, max);
                }
            }
            write(position);
            return aimed && System.nanoTime() >= settleUntilNanos;
        }

        boolean unwinding() {
            return System.nanoTime() < settleUntilNanos;
        }

    }

    static double limit(double position, double min, double max) {
        double lo = Math.max(0, Math.min(min, max)), hi = Math.min(1, Math.max(min, max));
        return Math.max(lo, Math.min(hi, position));
    }

}
