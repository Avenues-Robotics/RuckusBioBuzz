package org.firstinspires.ftc.teamcode.tele;

import org.firstinspires.ftc.teamcode.Config;
import org.firstinspires.ftc.teamcode.SystemConstants;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.drivetrains.Swerve;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.systems.Intake;

import java.util.List;

import org.firstinspires.ftc.teamcode.fieldview.FieldView;
import org.firstinspires.ftc.teamcode.helpers.Alliance;
import org.firstinspires.ftc.teamcode.helpers.PoseStorage;
import org.firstinspires.ftc.teamcode.modules.Cache;
import org.firstinspires.ftc.teamcode.modules.PredictiveAiming;
import org.firstinspires.ftc.teamcode.systems.Scoopula;
import org.firstinspires.ftc.teamcode.systems.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "TeleOp")
public class TeleOp extends OpMode {

    private Follower follower;
    private PredictiveAiming predictor;
    private Shooter shooterSystem;
    private List<LynxModule> hubs;
    private Alliance alliance = Alliance.RED;
    private Intake intake;
    private Scoopula scoopula;
    private final double[] dtMs = new double[256];
    private int dtCount;
    private long lastLoopNs;
    private final Cache telemetryCache = new Cache();
    private MultipleTelemetry out;

    @Override
    public void init() {
        out = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        follower = Constants.createFollower(hardwareMap);
        Pose autoEndPose = PoseStorage.take();
        if (autoEndPose != null) {
            follower.setPose(autoEndPose);
        }
        predictor = new PredictiveAiming(follower);
        scoopula = new Scoopula(hardwareMap);
        // Shooter and Intake are alliance-dependent: built in start(), after init_loop's X/B pick.
    }

    @Override
    public void init_loop() {
        if (gamepad1.x) alliance = Alliance.BLUE;
        if (gamepad1.b) alliance = Alliance.RED;
        out.addData("alliance (X blue, B red)", alliance);
        out.update();
    }

    @Override
    public void start() {
        shooterSystem = new Shooter(hardwareMap, predictor, alliance);
        intake = new Intake(hardwareMap, alliance);
    }

    @Override
    public void loop() {
        long now = System.nanoTime();
        if (lastLoopNs != 0) {
            dtMs[dtCount++ % dtMs.length] = (now - lastLoopNs) / 1e6;
        }
        lastLoopNs = now;
        Pose pose = follower.pose();
        if (Config.telemetryEnabled()) {
            FieldView.publish(pose, predictor.predictPose());
        }
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        Swerve.setTranslationPriority(SystemConstants.TeleOp.TRANSLATION_PRIORITY);
        follower.manual(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        follower.update();
        // Shooter first, so the intake follows this loop's SHOOT decision, not last loop's.
        shooterSystem.update();
        if(shooterSystem.shooting()){
            intake.update(false, true); //just sends intake command that's overridable
        }else {
            intake.update(gamepad1.left_bumper, gamepad1.right_bumper);
        }
        scoopula.update(gamepad2.left_bumper && gamepad2.right_bumper);

        if (!Config.telemetryEnabled() || !telemetryCache.due(SystemConstants.Cache.TELEMETRY_INTERVAL_MS)) {
            return;
        }
        out.addData("alliance", alliance);
        out.addData("x", pose.x());
        out.addData("y", pose.y());
        out.addData("heading (deg)", Math.toDegrees(pose.heading()));
        out.addData("in zone", shooterSystem.inZone());
        out.addData("ready", shooterSystem.ready());
        addLoopStats();
        out.update();
    }

    private void addLoopStats() {
        int n = Math.min(dtCount, dtMs.length);
        if (n == 0) {
            return;
        }
        double[] sorted = java.util.Arrays.copyOf(dtMs, n);
        double sum = 0;
        for (double d : sorted) {
            sum += d;
        }
        java.util.Arrays.sort(sorted);
        double mean = sum / n;
        double p90 = sorted[Math.min(n - 1, (int) Math.ceil(0.9 * n) - 1)];
        out.addData("loop Hz (1/mean dt)", String.valueOf(Math.round(1000.0 / mean)));
        out.addData("loop dt mean / p90 ms", Math.round(mean * 10) / 10.0 + " / " + Math.round(p90 * 10) / 10.0);
        out.addData("loop samples", n);
    }

    @Override
    public void stop() {
        follower.stop();
        follower.update();
        Swerve.setTranslationPriority(0);
    }
}
