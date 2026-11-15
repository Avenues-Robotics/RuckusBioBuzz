package org.firstinspires.ftc.teamcode.diagnostics.tests;

import org.firstinspires.ftc.teamcode.diagnostics.TurretEndStop;

import org.firstinspires.ftc.teamcode.SystemConstants;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.systems.Turret;

/**
 * DIAGNOSTIC ONLY. Drives the pollen turret servo (Control Hub servo port 5,
 * "pollenTurretServo" in res/xml/ruckusbiobuzz.xml) from FTC Dashboard.
 *
 * Open http://192.168.43.1:8080/dash, Configuration -> turretServoTest, edit POSITION (0..1).
 * The servo is driven at turret.configurePwm's 500-2500 us range, so positions match what
 * systems/turret commands. With SWEEP on it ping-pongs between SWEEP_MIN and SWEEP_MAX every
 * SWEEP_PERIOD_S seconds instead. Drives during INIT too.
 */
@Config
public class turretServoTest extends LinearOpMode {
    public static String SERVO_NAME = "pollenTurretServo";
    public static double POSITION = 0.5;
    public static boolean SWEEP = false;
    public static double SWEEP_MIN = 0.25;
    public static double SWEEP_MAX = 0.75;
    public static double SWEEP_PERIOD_S = 2.0;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Servo servo;
        try {
            servo = hardwareMap.get(Servo.class, SERVO_NAME);
        } catch (IllegalArgumentException e) {
            while (!isStopRequested()) {
                telemetry.addData("ERROR", "no Servo named \"" + SERVO_NAME + "\" in the active config");
                telemetry.addLine("Select the ruckusbiobuzz config on the Driver Station.");
                telemetry.update();
                sleep(200);
            }
            return;
        }
        Turret.configurePwm(servo);

        long t0 = System.nanoTime();
        while (!isStopRequested()) {
            double cmd;
            if (SWEEP) {
                double t = (System.nanoTime() - t0) / 1e9;
                boolean high = ((long) Math.floor(t / Math.max(0.1, SWEEP_PERIOD_S))) % 2 == 1;
                cmd = high ? SWEEP_MAX : SWEEP_MIN;
            } else {
                cmd = POSITION;
            }
            cmd = TurretEndStop.clamp(SERVO_NAME, Math.max(0, Math.min(1, cmd)));
            servo.setPosition(cmd);

            telemetry.addData("servo", SERVO_NAME + " (port " + servo.getPortNumber() + ")");
            telemetry.addData("mode", SWEEP ? "SWEEP" : "POSITION");
            telemetry.addData("commanded", cmd);
            telemetry.addData("approx deg from 0", cmd * SystemConstants.Turret.SERVO_RANGE_DEG);
            telemetry.addData("state", isStarted() ? "RUNNING" : "INIT (already driving)");
            telemetry.update();
            sleep(20);
        }
    }
}
