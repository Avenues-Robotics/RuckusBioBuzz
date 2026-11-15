package org.firstinspires.ftc.teamcode.diagnostics.tests;

import org.firstinspires.ftc.teamcode.diagnostics.TurretEndStop;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * DIAGNOSTIC ONLY. Measures a positional servo's real travel (position 0 to 1) with the Control
 * Hub IMU instead of a protractor.
 *
 * <ol>
 * <li>POSITION = 0. Point the horn at a fixed mark. Set ZERO = true (resets IMU yaw).</li>
 * <li>POSITION = 1. Turn the ROBOT until the horn points at the same mark again.</li>
 * <li>Travel = 360 - |yaw| (the horn turned S, the robot undid it by turning 360 - S the short
 *     way). Valid for travel above 180 deg, which is visibly the case for a turret servo.</li>
 * </ol>
 * Logcat tag servoRangeTest every 200 ms: position, yaw, implied travel, pulse range.
 *
 * <p>FULL_PWM: the SDK drives servos at 600-2400 us by default (PwmControl.PwmRange defaults,
 * RobotCore 12.0.0); Axon rates its travel over 500-2500 us. FULL_PWM = true sets 500-2500.
 * Measured 2026-11-01 at the default 600-2400: 323.0 deg (n=2).
 */
@Config
@Configurable
public class servoRangeTest extends LinearOpMode {
    public static String SERVO_NAME = "pollenTurretServo";
    public static double POSITION = 0.5; // was 0.0 = the pollen turret's travel end (TurretEndStop)
    public static boolean ZERO = false;
    public static boolean FULL_PWM = true;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Servo servo = hardwareMap.get(Servo.class, SERVO_NAME);
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();
        long lastLog = 0;
        Boolean appliedFull = null;
        while (!isStopRequested()) {
            if (ZERO) {
                imu.resetYaw();
                ZERO = false;
            }
            if (appliedFull == null || appliedFull != FULL_PWM) {
                ((ServoImplEx) servo).setPwmRange(FULL_PWM
                        ? new PwmControl.PwmRange(500, 2500) : PwmControl.PwmRange.defaultRange);
                appliedFull = FULL_PWM;
            }
            servo.setPosition(TurretEndStop.clamp(SERVO_NAME, POSITION));
            double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double travel = 360 - Math.abs(yaw);
            telemetry.addData("position", POSITION);
            telemetry.addData("imu yaw deg", yaw);
            telemetry.addData("travel deg (360-|yaw|, valid at POSITION 1)", travel);
            telemetry.update();
            long now = System.currentTimeMillis();
            if (now - lastLog >= 200) {
                lastLog = now;
                Log.i("servoRangeTest", String.format(java.util.Locale.US,
                        "pos=%.3f yaw=%.2f travel=%.2f pwm=%s", POSITION, yaw, travel,
                        FULL_PWM ? "500-2500" : "600-2400"));
            }
            sleep(20);
        }
    }
}
