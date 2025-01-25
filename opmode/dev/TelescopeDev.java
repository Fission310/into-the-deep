package org.firstinspires.ftc.teamcode.opmode.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.mechanisms.Telescope;

@TeleOp(name = "Telescope Dev", group = "dev")
public class TelescopeDev extends LinearOpMode {

    private Telescope telescope = new Telescope(this);

    @Override
    public void runOpMode() throws InterruptedException {
        telescope.init(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            telescope.loop(gamepad1);

            telescope.telemetry(telemetry);
            telemetry.update();
        }
    }
}
