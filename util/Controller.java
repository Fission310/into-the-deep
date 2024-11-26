package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Config
public class Controller {
    private double targetPosition;
    private double currentFeedforward;
    private VoltageSensor voltage;
    private double highestPosition = 2000;
    public static double GRAVITY_MULTIPLIER = 0.00005;
    public static double OFFSET = 150;

    private PIDFController upController;
    private PIDFController downController;

    public Controller(double ukP, double ukI, double ukD, double dkP, double dkI, double dkD, VoltageSensor voltage, int highest) {
        upController = new PIDFController(ukP, ukI, ukD, 0);
        downController = new PIDFController(dkP, dkI, dkD, 0);
        this.voltage = voltage;
        highestPosition = highest;
    }

    public double getPower(double position) {
        double t = targetPosition;
        if (position > highestPosition) {
            if (position < targetPosition + OFFSET) {
                t = targetPosition + OFFSET;
            }
        } else {
            if (position > targetPosition - OFFSET) {
                t = targetPosition - OFFSET;
            }
        }
        Telemetry te = FtcDashboard.getInstance().getTelemetry();
        te.addData("real position", t);
        te.update();
        PIDFController controller = upController;
        if ((position > highestPosition) == (position > t)) {
            controller = upController;
            te.addData("controller", "up");
        } else {
            controller = downController;
            te.addData("controller", "down");
        }
        double power = controller.calculate(position, t);
        power += currentFeedforward;
        power = Math.max(-1, Math.min(1, power));
        double correction = 12.0 / voltage.getVoltage();
        return power * correction;
    }

    public void setTargetPosition(double targetPosition) {
        this.targetPosition = targetPosition;
    }
}
