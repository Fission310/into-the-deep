package org.firstinspires.ftc.teamcode.opmode.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.mechanisms.Intake;

@TeleOp(name = "Intake Dev", group = "dev")
public class IntakeDev extends LinearOpMode {

    private Intake intake = new Intake(this);

    @Override
    public void runOpMode() throws InterruptedException {
        intake.init(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            intake.loop(gamepad1);

            intake.telemetry(telemetry);
            telemetry.update();
        }
    }
}
