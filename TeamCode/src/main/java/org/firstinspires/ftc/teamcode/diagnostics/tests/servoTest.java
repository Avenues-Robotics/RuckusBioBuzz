package org.firstinspires.ftc.teamcode.diagnostics.tests;

import org.firstinspires.ftc.teamcode.diagnostics.TurretEndStop;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;

/**
 * Positional servo bench test. Edit {@code position} under servoTest in Panels
 * (http://192.168.43.1:8001, Configurables). Telemetry echoes the commanded value and
 * servo.getPosition(). If SERVO_NAME is not in the ACTIVE hardware config the OpMode stays
 * open and lists the servos that are, instead of dying at init.
 */
@Config
@Configurable
public class servoTest extends LinearOpMode {
    public static String SERVO_NAME = "pollenTurretServo";
    public static double position = 0.5; // was 0.0 = the pollen turret's travel end (TurretEndStop)

    /**
     * Fallback input: if this file exists its first line overrides {@code position}. Added
     * 2026-10-31 when Sloth stopped Panels edits reaching this static field (Panels held 1, the
     * OpMode logged 0.0). Write it from the PC:
     * adb shell "echo 0.5 > /sdcard/FIRST/servoTest_position.txt"
     */
    private static final File POSITION_FILE = new File("/sdcard/FIRST/servoTest_position.txt");
    private long fileCheckMs;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();
        Servo test = null;
        long lastLog = 0;
        try {
            test = hardwareMap.get(Servo.class, SERVO_NAME);
        } catch (IllegalArgumentException e) {
            while (!isStopRequested()) {
                telemetry.addData("ERROR", "no Servo named \"" + SERVO_NAME + "\" in the active config");
                telemetry.addData("Servos in active config", hardwareMap.servo.size());
                for (String name : names()) {
                    telemetry.addLine("  " + name);
                }
                telemetry.addLine("Select the right config on the Driver Station, or change SERVO_NAME.");
                telemetry.update();
                sleep(200);
            }
            return;
        }
        // Drive during INIT too, so the bench works without pressing PLAY.
        while (!isStarted() && !isStopRequested()) {
            readPositionFile();
            test.setPosition(TurretEndStop.clamp(SERVO_NAME, position));
            lastLog = report(panels, test, lastLog);
            sleep(20);
        }
        while (opModeIsActive()) {
            readPositionFile();
            test.setPosition(TurretEndStop.clamp(SERVO_NAME, position));
            lastLog = report(panels, test, lastLog);
        }
        if (!TurretEndStop.isTurret(SERVO_NAME)) {
            test.setPosition(0); // a turret stays where it is: 0 is its travel end
        }
    }

    /** Echo to the Driver Station, Panels and logcat (tag servoTest, twice a second). */
    private void readPositionFile() {
        long now = System.currentTimeMillis();
        if (now - fileCheckMs < 100 || !POSITION_FILE.exists()) return;
        fileCheckMs = now;
        try (BufferedReader r = new BufferedReader(new FileReader(POSITION_FILE))) {
            String line = r.readLine();
            if (line != null) position = Math.max(0, Math.min(1, Double.parseDouble(line.trim())));
        } catch (Exception ignored) {
        }
    }

    private long report(TelemetryManager panels, Servo test, long lastLog) {
        telemetry.addData("commanded position", position);
        telemetry.addData("servo getPosition()", test.getPosition());
        telemetry.update();
        panels.debug("commanded position", position);
        panels.debug("servo getPosition()", test.getPosition());
        panels.update(telemetry);
        long now = System.currentTimeMillis();
        if (now - lastLog > 500) {
            Log.i("servoTest", SERVO_NAME + " position=" + position + " getPosition=" + test.getPosition());
            return now;
        }
        return lastLog;
    }

    private java.util.List<String> names() {
        java.util.List<String> out = new java.util.ArrayList<>();
        for (Servo s : hardwareMap.servo) {
            out.addAll(hardwareMap.getNamesOf(s));
        }
        return out;
    }
}
