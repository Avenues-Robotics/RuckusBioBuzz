package org.firstinspires.ftc.teamcode.systems;

import org.firstinspires.ftc.teamcode.SystemConstants;


import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.modules.Cache;
import org.firstinspires.ftc.teamcode.modules.ShootingRegression;

public class Flywheel {

    private final ShootingRegression regression;

    private final DcMotorEx pollenFlywheel;
    private final DcMotorEx nectarFlywheel;
    private final VelocityWriter pollenWriter;
    private final VelocityWriter nectarWriter;
    private double pollenTarget;
    private double pollenCommand;
    private double nectarTarget;
    private double pollenVelocity;
    private double nectarVelocity;
    private final VoltageSensor battery;
    private double filteredVolts = Double.NaN;
    private double appliedFScale = 1.0;
    private long lastCompNs;
    /** TeleOp driver 2 (helpers/AimOffsets): ticks/s added to both regression targets - their y-intercept. */
    private double speedOffset;
    public Flywheel(HardwareMap hardwareMap, Turret aiming) {
        regression = new ShootingRegression(aiming);
        battery = hardwareMap.voltageSensor.iterator().hasNext() ? hardwareMap.voltageSensor.iterator().next() : null;

        pollenFlywheel = hardwareMap.get(DcMotorEx.class, "pollenTurret");
        nectarFlywheel = hardwareMap.get(DcMotorEx.class, "nectarTurret");

        pollenFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        nectarFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pollenFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        nectarFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        setVelocityPidf(pollenFlywheel, SystemConstants.Flywheel.POLLEN_PIDF);
        setVelocityPidf(nectarFlywheel, SystemConstants.Flywheel.NECTAR_PIDF);
        pollenWriter = new VelocityWriter(pollenFlywheel);
        nectarWriter = new VelocityWriter(nectarFlywheel);
    }
    private static void setVelocityPidf(DcMotorEx flywheel, PIDFCoefficients pidf) {
        setVelocityPidf(flywheel, pidf, 1.0);
    }

    private static void setVelocityPidf(DcMotorEx flywheel, PIDFCoefficients pidf, double fScale) {
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new com.qualcomm.robotcore.hardware.PIDFCoefficients(pidf.P, pidf.I, pidf.D, pidf.F * fScale));
    }

    /**
     * Voltage-compensated feed-forward. The hub's velocity F is a fixed fraction of full power per tick/s, so
     * the same F gives less speed on a lower battery: 2026-11-15 the pollen wheel sat 2.5% under 2400 at
     * 11.9 V and 0.9% under at 12.6 V on the same gains. Every F_COMP_UPDATE_MS this re-applies the PIDF with
     * F x (F_NOMINAL_VOLTS / battery volts), battery volts smoothed with time constant F_COMP_TAU_S, only when
     * the factor moved by more than 0.5%, clamped to [0.85, 1.25] against a bad reading.
     */
    private void compensateF() {
        if (!SystemConstants.Flywheel.F_VOLTAGE_COMP || battery == null) return;
        long now = System.nanoTime();
        double dtS = lastCompNs == 0 ? 0 : (now - lastCompNs) / 1e9;
        if (lastCompNs != 0 && dtS * 1000 < SystemConstants.Flywheel.F_COMP_UPDATE_MS) return;
        lastCompNs = now;
        double v = battery.getVoltage();
        if (!(v > 6)) return;   // no reading / brownout: keep the last factor
        double tau = Math.max(1e-3, SystemConstants.Flywheel.F_COMP_TAU_S);
        filteredVolts = Double.isNaN(filteredVolts) ? v : filteredVolts + (v - filteredVolts) * Math.min(1, dtS / tau);
        double scale = Math.max(0.85, Math.min(1.25, SystemConstants.Flywheel.F_NOMINAL_VOLTS / filteredVolts));
        if (Math.abs(scale - appliedFScale) > 0.005 * appliedFScale) {
            setVelocityPidf(pollenFlywheel, SystemConstants.Flywheel.POLLEN_PIDF, scale);
            setVelocityPidf(nectarFlywheel, SystemConstants.Flywheel.NECTAR_PIDF, scale);
            appliedFScale = scale;
        }
    }

    public void setSpeedOffset(double ticksPerSecond) {
        speedOffset = ticksPerSecond;
    }

    public void update(boolean inZone){
        compensateF();
        double scale = inZone ? 1 : SystemConstants.Flywheel.IDLE_FRACTION;
        pollenTarget = Math.max(0, regression.shooterspeedpollen() + speedOffset);
        nectarTarget = Math.max(0, regression.shooterspeednectar() + speedOffset);
        pollenCommand = pollenTarget * scale;
        pollenWriter.set(pollenCommand, SystemConstants.Flywheel.POLLEN_FLYWHEEL_REVERSED);
        nectarWriter.set(nectarTarget * scale, SystemConstants.Flywheel.NECTAR_FLYWHEEL_REVERSED);
        pollenVelocity = pollenFlywheel.getVelocity();
        nectarVelocity = nectarFlywheel.getVelocity();
    }
    public double pollenVelocity() { return pollenVelocity; }
    /** What the wheel is commanded right now: the shooting target, or IDLE_FRACTION of it out of the zone. */
    public double pollenCommand() { return pollenCommand; }
    /** F multiplier currently applied by the voltage compensation (1.0 when off). */
    public double fScale() { return appliedFScale; }
    public boolean atSpeedPollen() { return atSpeedPollen(SystemConstants.Flywheel.AT_SPEED_FRACTION); }
    public boolean atSpeedNectar() { return atSpeedNectar(SystemConstants.Flywheel.AT_SPEED_FRACTION); }
    public boolean atSpeedPollen(double fraction) { return atSpeed(pollenVelocity, pollenTarget, fraction); }
    public boolean atSpeedNectar(double fraction) { return atSpeed(nectarVelocity, nectarTarget, fraction); }

    private static boolean atSpeed(double velocity, double target, double fraction) {
        return target > 0 && Math.abs(velocity - target) <= fraction * target;
    }

    private static final class VelocityWriter {
        private final DcMotorEx motor;
        private final Cache cache = new Cache();

        VelocityWriter(DcMotorEx motor) {
            this.motor = motor;
        }

        void set(double ticksPerSecond, boolean reversed) {
            motor.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            if (cache.changed(Cache.velocityKey(ticksPerSecond, reversed),
                    SystemConstants.Cache.VELOCITY_WRITES, SystemConstants.Cache.REFRESH_MS)) {
                motor.setVelocity(ticksPerSecond);
            }
        }
    }
}
