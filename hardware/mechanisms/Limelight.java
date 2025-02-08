package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import static org.firstinspires.ftc.teamcode.opmode.auton.util.LimelightConstants.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.stuyfission.fissionlib.util.Mechanism;

public class Limelight extends Mechanism {
    private Limelight3A limelight;
    private double tx, ty, tangle;

    public Limelight(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void init(HardwareMap hwMap) {
        limelight = hwMap.get(Limelight3A.class, "limelight");

        limelight.setPollRateHz(100);

        limelight.pipelineSwitch(PIPELINE);
        limelight.start();
    }

    public void update() {
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            tx = result.getTx();
            ty = result.getTy();
            tangle = result.getPythonOutput()[3];
        }
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        LLStatus status = limelight.getStatus();
        telemetry.addData("limelight temp", "%.1fC", status.getTemp());
        telemetry.addData("limelight tx", tx);
        telemetry.addData("limelight ty", ty);
        telemetry.addData("limelight tangle", tangle);
    }

    public void stop() {
        limelight.stop();
    }

    public void setPipeline(int pipeline) {
        limelight.pipelineSwitch(pipeline);
    }

    public double getTx() {
        return tx;
    }

    public double getTy() {
        return ty;
    }

    public double getTangle() {
        return tangle;
    }
}
