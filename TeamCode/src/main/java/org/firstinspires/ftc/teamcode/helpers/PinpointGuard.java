package org.firstinspires.ftc.teamcode.helpers;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Pinpoint IMU guard for the competition OpModes (TeleOp, autos).
 *
 * <p>Why: the Pinpoint heading has frozen after an app restart (twice in TeleOp before 2026-11-15: heading
 * exactly 0.0 and rate 0.0 while the robot was being driven); a reset of position + IMU cleared it each time.
 * The cause is not known, and a freeze cannot be told apart from a robot sitting still - a live Pinpoint at rest
 * also reads the bit-identical heading for seconds (2026-11-15, firmware v4, READY at 1.17 kHz). So the guard
 * does not try to detect it at init: it resets position + IMU at init, when the robot is required to be still
 * anyway, waits for READY, and the OpMode re-applies its pose. The shared PinpointConfig keeps resetMode NONE on
 * purpose: Swerve Bring-Up rebuilds its follower mid-session and a reset there would wipe the fence frame.
 *
 * <p>At runtime it can only warn: {@link #frozenWhileTurning} flags a heading that does not change at all while
 * the driver turns hard.
 */
public final class PinpointGuard {
    /** The driver's reset takes ~0.25 s; wait at least this long before trusting READY. */
    private static final double RESET_MIN_S = 0.35;
    private static final double RESET_TIMEOUT_S = 2.0;
    private static final double FROZEN_TURN = 0.4;
    private static final double FROZEN_S = 0.6;

    private enum State { IDLE, RESETTING, OK, FAILED }

    private final GoBildaPinpointDriver pinpoint;
    private final ElapsedTime timer = new ElapsedTime();
    private State state = State.IDLE;
    private String detail = "not started";

    private double watchHeading = Double.NaN;
    private final ElapsedTime watchTimer = new ElapsedTime();
    private boolean turningHard;

    public PinpointGuard(HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Constants.pinpointConfig.name.get());
    }

    /** Init, robot stationary: zero position and recalibrate the IMU. Re-apply the pose once {@link #update} says so. */
    public void begin() {
        pinpoint.resetPosAndIMU();
        timer.reset();
        state = State.RESETTING;
        detail = "resetting IMU - keep the robot still";
    }

    /**
     * Call every init_loop. Returns true exactly once, when the reset has finished (or timed out) - that is when
     * the OpMode must set its pose again, because the reset zeroed it.
     */
    public boolean update() {
        if (state != State.RESETTING) {
            return false;
        }
        pinpoint.update();
        GoBildaPinpointDriver.DeviceStatus s = pinpoint.getDeviceStatus();
        if (timer.seconds() >= RESET_MIN_S && s == GoBildaPinpointDriver.DeviceStatus.READY) {
            state = State.OK;
            detail = "OK (IMU reset, READY after " + Math.round(timer.seconds() * 100) / 100.0 + " s)";
            RobotLog.ii("PinpointGuard", detail);
            return true;
        }
        if (timer.seconds() > RESET_TIMEOUT_S) {
            state = State.FAILED;
            detail = "NOT READY after reset (" + s + ") - odometry and aiming unreliable";
            RobotLog.ee("PinpointGuard", detail);
            return true;
        }
        return false;
    }

    /**
     * start() with the reset still running (START pressed within ~0.35 s of INIT): waits for it, at most
     * {@link #RESET_TIMEOUT_S}. Returns true if the OpMode must set its pose again.
     */
    public boolean finish() {
        while (state == State.RESETTING) {
            if (update()) {
                return true;
            }
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return false;
            }
        }
        return false;
    }

    public boolean ok() {
        return state == State.OK;
    }

    public String status() {
        return detail;
    }

    /**
     * TeleOp loop: true while the driver has been turning hard for {@value #FROZEN_S} s and the heading has not
     * changed by a single bit - a frozen Pinpoint. Turning moves a live heading every loop.
     */
    public boolean frozenWhileTurning(double headingRad, double turnCommand) {
        if (Math.abs(turnCommand) < FROZEN_TURN) {
            turningHard = false;
            watchHeading = headingRad;
            return false;
        }
        if (!turningHard || headingRad != watchHeading) {
            turningHard = true;
            watchHeading = headingRad;
            watchTimer.reset();
            return false;
        }
        return watchTimer.seconds() >= FROZEN_S;
    }

    /** Raw heading straight from the device, for telemetry. */
    public double headingDeg() {
        return pinpoint.getHeading(AngleUnit.DEGREES);
    }
}
