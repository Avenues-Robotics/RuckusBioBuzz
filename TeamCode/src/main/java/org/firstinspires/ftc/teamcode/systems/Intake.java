package org.firstinspires.ftc.teamcode.systems;

import org.firstinspires.ftc.teamcode.SystemConstants;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.helpers.Alliance;

public class Intake {
    private DcMotor intakeLeft, intakeRight;
    HardwareMap hardwareMap;
    private intakeState currentState = intakeState.STOP;
    private Alliance alliance;
    public enum intakeState {
        INTAKE, STOP, REJECT
    }
    private int lastSign = 0;
    private long lastDriveNanos = 0;

    public Intake(HardwareMap hardwareMap, Alliance alliance) {
        this.hardwareMap = hardwareMap;
        this.alliance = alliance;

        intakeLeft = hardwareMap.get(DcMotor.class, "intakeLeft");
        intakeRight = hardwareMap.get(DcMotor.class, "intakeRight");
        intakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // No color sensor (operator 2026-11-24: not on the robot, ever). Its read cost 7.3 ms each, 15 reads/s, in
        // every loop of every OpMode (blue auto loop-time log), on a device that was not plugged in. Reject is
        // the driver's left bumper only; ColorCheck's rule stays for the Mechanism Tuner.

        // Per-motor direction is SystemConstants.Intake.INTAKE_LEFT/RIGHT_REVERSED, applied in update()
        // (not setDirection) so a live edit from the Mechanism Tuner takes effect without a rebuild.
    }

    /** Runtime alliance change; only the Mechanism Tuner needs it (OpModes pass it to the constructor). */
    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }

    public intakeState getState(){
        return currentState;
    }

    public void update(boolean leftBumper, boolean rightBumper) {
        if(leftBumper){
            currentState = intakeState.REJECT;
        }else if(rightBumper){
            currentState = intakeState.INTAKE;
        }else{
            currentState = intakeState.STOP;
        }
        double power = 0;
        if (currentState == intakeState.INTAKE) {
            power = SystemConstants.Intake.INTAKE_FEED_POWER;
        } else if (currentState == intakeState.REJECT) {
            power = -SystemConstants.Intake.INTAKE_FEED_POWER;
        }
        if (SystemConstants.Intake.INTAKE_REVERSED) {
            power = -power;
        }
        double powerL = SystemConstants.Intake.INTAKE_LEFT_REVERSED ? -power : power;
        double powerR = SystemConstants.Intake.INTAKE_RIGHT_REVERSED ? -power : power;

        long now = System.nanoTime();
        int sign = (int) Math.signum(power);
        if (sign != 0 && lastSign != 0 && sign != lastSign && now - lastDriveNanos < SystemConstants.Intake.REVERSE_BRAKE_MS * 1e6) {
            powerL = 0;
            powerR = 0;
        } else if (sign != 0) {
            lastSign = sign;
            lastDriveNanos = now;
        }

        intakeLeft.setPower(powerL);
        intakeRight.setPower(powerR);

    }



}
