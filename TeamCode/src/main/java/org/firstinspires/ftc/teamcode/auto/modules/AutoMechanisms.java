package org.firstinspires.ftc.teamcode.auto.modules;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helpers.Alliance;
import org.firstinspires.ftc.teamcode.modules.PredictiveAiming;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Scoopula;
import org.firstinspires.ftc.teamcode.systems.Shooter;

import java.util.List;

public final class AutoMechanisms {
    private final List<LynxModule> hubs;
    private final Shooter shooter;
    private final Intake intake;
    private final Scoopula scoopula;
    private boolean shooting;
    private boolean intaking;
    private boolean scoopDown;

    public AutoMechanisms(HardwareMap hardwareMap, Follower follower, Alliance alliance) {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        shooter = new Shooter(hardwareMap, new PredictiveAiming(follower), alliance);
        intake = new Intake(hardwareMap, alliance);
        scoopula = new Scoopula(hardwareMap);
    }

    public void clearCache() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
    }

    public void update() {
        shooter.update();
        if (shooting) {
            intake.update(false, shooter.shooting());
        } else {
            intake.update(false, intaking);
        }
        scoopula.update(scoopDown);
    }

    public void start(String event) {
        switch (event) {
            case "Shoot":
                shooting = true;
                shooter.setForceZone(true);
                break;
            case "Intake":
            case "Intake On":
                intaking = true;
                break;
            case "Intake Off":
                intaking = false;
                break;
            case "Scoopula Down":
                scoopDown = true;
                break;
            case "Scoopula Up":
                scoopDown = false;
                break;
            default:
                break;
        }
    }

    public void finish(String event) {
        switch (event) {
            case "Shoot":
                shooting = false;
                shooter.setForceZone(false);
                break;
            case "Intake":
                intaking = false;
                break;
            default:
                break;
        }
    }

    public void stop() {
        shooting = false;
        intaking = false;
        shooter.setForceZone(false);
        intake.update(false, false);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Shooting", shooting + (shooting ? " (gate " + (shooter.shooting() ? "open" : "waiting") + ")" : ""));
        telemetry.addData("Intake", intaking);
        telemetry.addData("Scoopula", scoopDown ? "down" : "up");
    }
}
