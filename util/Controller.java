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

    private PIDFController upBottomController;
    private PIDFController downTopController;
    private PIDFController upTopController;
    private PIDFController downBottomController;

    public Controller(PIDFController uB, PIDFController dT, PIDFController uT, PIDFController dB, VoltageSensor voltage, int highest) {
        upBottomController = uB;
        downTopController = dT;
        upTopController = uT;
        downBottomController = dB;
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
        PIDFController controller = upBottomController;
        if ((position > highestPosition) == (position > t)) {
            if (Math.abs(position - highestPosition) > highestPosition / 2) {
                controller = upBottomController;
            } else {
                controller = upTopController;
            }
        } else {
            if (Math.abs(position - highestPosition) > highestPosition / 2) {
                controller = downBottomController;
            } else {
                controller = downTopController;
            }
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
