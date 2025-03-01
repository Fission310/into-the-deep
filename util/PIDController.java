package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {

    private double kP, kI, kD;
    private double target = 0;
    private double integralSum = 0, lastError = 0;

    private ElapsedTime timer = new ElapsedTime();

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void setkP(double kP) {
        this.kP = kP;
    }
    public void setkD(double kD) {
        this.kD = kD;
    }
    public void setkI(double kI) {
        this.kI = kI;
    }


    public void setTarget(double target) {
        this.target = target;
        lastError = 0;
    };

    public double getTarget() { return target; }

    public double calculate(double reference) {
        double error = target - reference;
        double derivative = (error - lastError) / timer.seconds();
        integralSum = integralSum + (error * timer.seconds());

        lastError = error;
        timer.reset();

        return (kP * error) + (kI * integralSum) + (kD * derivative);
    }

}
