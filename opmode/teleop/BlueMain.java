package org.firstinspires.ftc.teamcode.opmode.teleop;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.opmode.auton.util.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "BlueMain", group = "_amain")
public class BlueMain extends LinearOpMode {

    private Robot robot = new Robot(this, Color.BLUE);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            robot.loop(gamepad1);

            robot.telemetry(telemetry);
        }
    }
}
