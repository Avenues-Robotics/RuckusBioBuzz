package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.util.ElapsedTime;


public class PIDF { //assuming the feedforward term is simply a constant.
    ElapsedTime timer = new ElapsedTime();
    private double lastTime = 0;
    private double lastPos;
    private double lastDer;
    private int runCount = 0;
    private final double Kp;
    private final double Ki;
    private final double Kd;
    private final double Kf;
    private double integralSum = 0;
    private double derivative = 0;
    private double clamp;

    public PIDF(double KP, double KI, double KD, double KF, double clampI) {
        this.Kp = KP;
        this.Ki = KI;
        this.Kd = KD;
        this.Kf = KF;
        this.clamp = clampI;
    }
    public double update(double pos, double target){
        double error = target - pos;
        if(runCount == 0){
            derivative = 0;
            lastTime = timer.seconds();
            lastPos = pos;
        }
        double dt = timer.seconds()-lastTime;
        if(dt != 0){
            integralSum = Math.max(Math.min(integralSum+(error*dt), clamp),-clamp);
            derivative = (lastPos-pos)/dt;
        }


        if(runCount == 0) {
            lastDer = derivative;
        }

        double P = error*Kp;
        double I = integralSum*Ki;
        double D = (derivative+lastDer)/2*Kd; //avg over 2 instances
        double F = Kf; //Since I haven't specified a subsystem yet I will leave this as a constant.

        lastTime = timer.seconds();
        lastPos = pos;
        lastDer = derivative;
        runCount++;

        return P + I + D + F;
    }
    public int getRunCount(){
        return runCount;
    }
    public int changeClamp(int newClamp){
        this.clamp = newClamp;
    }
}
