package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.auto.legacy.ExampleSwerveAuto;
import org.firstinspires.ftc.teamcode.auto.legacy.redOption2;
import org.firstinspires.ftc.teamcode.auto.legacy.shortAutoRed;
import org.firstinspires.ftc.teamcode.diagnostics.PreMatchSystemCheck;
import org.firstinspires.ftc.teamcode.diagnostics.motorbreakin;
import org.firstinspires.ftc.teamcode.diagnostics.autosim.AutoDryRun;
import org.firstinspires.ftc.teamcode.diagnostics.swerve.DashboardDriveTeleOp;
import org.firstinspires.ftc.teamcode.diagnostics.swerve.RawMotorTest;
import org.firstinspires.ftc.teamcode.diagnostics.swerve.SwerveBringUp;
import org.firstinspires.ftc.teamcode.diagnostics.swerve.SwerveDirectTeleOp;
import org.firstinspires.ftc.teamcode.diagnostics.tests.colorTuner;
import org.firstinspires.ftc.teamcode.diagnostics.tests.rangeTest;
import org.firstinspires.ftc.teamcode.diagnostics.tests.servoRangeTest;
import org.firstinspires.ftc.teamcode.diagnostics.tests.servoTest;
import org.firstinspires.ftc.teamcode.diagnostics.tests.spinUpTest;
import org.firstinspires.ftc.teamcode.diagnostics.tests.turretServoTest;
import org.firstinspires.ftc.teamcode.diagnostics.tests.turretTrackTest;
import org.firstinspires.ftc.teamcode.diagnostics.tuning.MechanismTuner;

public final class DevOpModes {
    private DevOpModes() {}

    @OpModeRegistrar
    public static void register(OpModeManager manager) {
        if (Config.competition()) {
            return;
        }
        auto(manager, "ExampleSwerveAuto", "Auto", ExampleSwerveAuto.class);
        auto(manager, "Pedro Pathing Autonomous", "Autonomous", redOption2.class);
        auto(manager, "Short RED auto", "Auto", shortAutoRed.class);
        auto(manager, "PreMatchSystemCheck", "Diagnostics", PreMatchSystemCheck.class);

        teleOp(manager, "motorbreakin", "TeleOp", motorbreakin.class);
        teleOp(manager, "Dashboard Drive (bring-up parity)", "Diagnostics", DashboardDriveTeleOp.class);
        teleOp(manager, "Raw Motor Test", "Diagnostics", RawMotorTest.class);
        teleOp(manager, "Swerve Bring-Up", "Diagnostics", SwerveBringUp.class);
        teleOp(manager, "Swerve TeleOp", "Diagnostics", SwerveDirectTeleOp.class);
        teleOp(manager, "Mechanism Tuner", "Diagnostics", MechanismTuner.class);
        teleOp(manager, "Auto Dry Run (on blocks, sim pose)", "Diagnostics", AutoDryRun.class);
        teleOp(manager, "colorTuner", "TeleOp", colorTuner.class);
        teleOp(manager, "Range Test", "TeleOp", rangeTest.class);
        teleOp(manager, "SpinUpTest", "TeleOp", spinUpTest.class);
        teleOp(manager, "servoRangeTest", "Test", servoRangeTest.class);
        teleOp(manager, "servoTest", "Test", servoTest.class);
        teleOp(manager, "Turret Servo Test (port 5)", "Test", turretServoTest.class);
        teleOp(manager, "turretTrackTest", "Test", turretTrackTest.class);
    }

    private static void auto(OpModeManager manager, String name, String group, Class<? extends OpMode> type) {
        add(manager, name, group, OpModeMeta.Flavor.AUTONOMOUS, type);
    }

    private static void teleOp(OpModeManager manager, String name, String group, Class<? extends OpMode> type) {
        add(manager, name, group, OpModeMeta.Flavor.TELEOP, type);
    }

    private static void add(OpModeManager manager, String name, String group, OpModeMeta.Flavor flavor,
                            Class<? extends OpMode> type) {
        manager.register(new OpModeMeta.Builder().setName(name).setGroup(group).setFlavor(flavor).build(), type);
    }
}
