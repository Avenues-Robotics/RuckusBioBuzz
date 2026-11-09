package org.firstinspires.ftc.teamcode.systems;

import org.firstinspires.ftc.teamcode.SystemConstants;


import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.ShootingRegression;

public class Flywheel {

    private final ShootingRegression regression;

    private final DcMotorEx pollenFlywheel; // Should a Flywheel have two Flywheels?  Of should you make two instances of Flywheel in your OpMode?  All I see is duplicate code here, that's a good case for making two instances of this one class.
    private final DcMotorEx nectarFlywheel;
    private double pollenTarget;
    private double nectarTarget;
    private double pollenVelocity;
    private double nectarVelocity;
    public Flywheel(HardwareMap hardwareMap, Turret aiming) {
        regression = new ShootingRegression(aiming);

        pollenFlywheel = hardwareMap.get(DcMotorEx.class, "pollenTurret");
        nectarFlywheel = hardwareMap.get(DcMotorEx.class, "nectarTurret");

        pollenFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        nectarFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pollenFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        nectarFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        setVelocityPidf(pollenFlywheel, SystemConstants.Flywheel.POLLEN_PIDF);
        setVelocityPidf(nectarFlywheel, SystemConstants.Flywheel.NECTAR_PIDF);

    }
    private static void setVelocityPidf(DcMotorEx flywheel, PIDFCoefficients pidf) {
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new com.qualcomm.robotcore.hardware.PIDFCoefficients(pidf.P, pidf.I, pidf.D, pidf.F));
    }
    public void update(boolean inZone){
        pollenFlywheel.setDirection(SystemConstants.Flywheel.POLLEN_FLYWHEEL_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        nectarFlywheel.setDirection(SystemConstants.Flywheel.NECTAR_FLYWHEEL_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        double scale = inZone ? 1 : SystemConstants.Flywheel.IDLE_FRACTION;
        pollenTarget = regression.shooterspeedpollen();
        nectarTarget = regression.shooterspeednectar();
        pollenFlywheel.setVelocity(pollenTarget * scale);
        nectarFlywheel.setVelocity(nectarTarget * scale);
        pollenVelocity = pollenFlywheel.getVelocity();
        nectarVelocity = nectarFlywheel.getVelocity();
    }
    public boolean atSpeedPollen() { return atSpeedPollen(SystemConstants.Flywheel.AT_SPEED_FRACTION); }
    public boolean atSpeedNectar() { return atSpeedNectar(SystemConstants.Flywheel.AT_SPEED_FRACTION); }
    public boolean atSpeedPollen(double fraction) { return atSpeed(pollenVelocity, pollenTarget, fraction); }
    public boolean atSpeedNectar(double fraction) { return atSpeed(nectarVelocity, nectarTarget, fraction); }

    private static boolean atSpeed(double velocity, double target, double fraction) {
        return target > 0 && Math.abs(velocity - target) <= fraction * target;
    }
}
