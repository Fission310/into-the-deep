package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.VoltageSensor;

public class Controller {
    private double targetPosition;
    private double currentFeedforward;
    private VoltageSensor voltage;
    private double pPower;
    private double pTargetPosition;

    private PIDFController controller;

    public Controller(double kP, double kI, double kD, VoltageSensor voltage) {
        controller = new PIDFController(kP, kI, kD, 0);
        this.voltage = voltage;
    }

    public double getPower(double position) {
        double power = controller.calculate(position, targetPosition);
        power += currentFeedforward;
        power = Math.max(-1, Math.min(1, power));
        if (Math.abs(targetPosition - pTargetPosition) > 0.005 ||
                Math.abs(power - pPower) > 0.005) {
            double correction = 12.0 / voltage.getVoltage();
            pPower = power;
            return power * correction;
        } else {
            return 0;
        }
    }

    public void setTargetPosition(double targetPosition) {
        this.targetPosition = targetPosition;
    }
}
