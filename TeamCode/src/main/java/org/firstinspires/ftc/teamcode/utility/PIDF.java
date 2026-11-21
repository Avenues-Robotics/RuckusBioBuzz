package org.firstinspires.ftc.teamcode.utility;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.util.ElapsedTime;
/**
 * This is a general plug and play PIDF class with a constant feedforward term accounting static friction.
 *
 * The derivative is taken on the measurement(not the error)
 * to avoid spikes when the target changes,
 * and the integralSum is clamped to avoid ramping in the case of a stuck component.
 * The class doesn't reset on a target change, so it can track a dynamic target.
 *
 * @author Ryder Liou
 */

public class PIDF {
    ElapsedTime timer = new ElapsedTime();
    private double lastTime = 0;
    private double lastPos;
    private double lastDer;
    private int runCount = 0;
    private final double Kp;
    private final double Ki;
    private final double Kd;
    private final double Kf;
    private double target = 0;
    private double integralSum = 0;
    private double derivative = 0;
    private double clamp;
    private double pos;

    public PIDF(double KP, double KI, double KD, double KF, double clampI) {
        this.Kp = KP;
        this.Ki = KI;
        this.Kd = KD;
        this.Kf = KF;
        this.clamp = clampI; //Changes per application.
    }
    /**
     * Calculates the proper output for the PIDF. Must be called once per loop.
     *
     * @param pos Describes the current position or value one desires to move with the PIDF.
     * @return The output power, limited to [0,1].
     */
    public double update(double pos) {
        this.pos = pos;
        double error = target - pos;
        if (runCount == 0) {
            derivative = 0;
            lastTime = timer.seconds();
            lastPos = pos;
        }
        double dt = timer.seconds() - lastTime;
        if (dt != 0) { //Ensures division by zero doesn't occur, if dt == 0, it just uses the values from last loop.
            integralSum = Math.max(Math.min(integralSum + (error * dt), clamp), -clamp); //Clamps value to +-clamp
            derivative = (lastPos - pos) / dt;
        }


        if (runCount == 0) {
            lastDer = derivative;
        }

        double P = error * Kp;
        double I = integralSum * Ki;
        double D = (derivative + lastDer) / 2 * Kd; //Averages the derivative over two runs to avoid a spike.
        double F = Kf; //For a flywheel, generally the only feedforward is a constant to account for static friction..

        lastTime = timer.seconds();
        lastPos = pos;
        lastDer = derivative;
        runCount++;
        double power = Math.min(Math.max(P + I + D + F,0),1);
        telemetry.addData("target", target);
        telemetry.addData("pose", pos);
        telemetry.addData("error", target -pos);
        telemetry.addData("output", power);
        telemetry.update();
        return power;
    }
    public void setTarget(int newTarget){
        this.target = newTarget;
    }
    public boolean atTarget(int tolerance){
        double error = target -pos;
        if(error <= error+tolerance && error >= error - tolerance){ //checks if it is within a given tolerance
            return true;
        }else{
            return false;
        }
    }

    public int getRunCount() {
        return runCount;
    }

    public void changeClamp(int newClamp) {
        this.clamp = newClamp;
    }
}
