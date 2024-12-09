package org.firstinspires.ftc.teamcode.opmode.teleop;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "No Auto Main", group = "_amain")
public class NoAutoMain extends LinearOpMode {

    private Robot robot = new Robot(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.scoring.pivot.initPos();

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        while (opModeInInit()) {
            robot.scoring.pivot.update();
        }

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            robot.loop(gamepad1);

            robot.telemetry(telemetry);
        }
    }
}
