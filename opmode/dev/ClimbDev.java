package org.firstinspires.ftc.teamcode.opmode.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.mechanisms.Climb;

@Config
@TeleOp(name = "Climb Dev", group = "dev")
public class ClimbDev extends LinearOpMode {

    private Climb climb = new Climb(this);

    @Override
    public void runOpMode() throws InterruptedException {
        climb.init(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            climb.loop(gamepad1);
            climb.telemetry(telemetry);
            telemetry.update();
        }
    }
}
