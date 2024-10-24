package org.firstinspires.ftc.teamcode.opmode.dev;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.mechanisms.Telescope;

@TeleOp(name = "Telescope Dev", group = "dev")
public class TelescopeDev extends LinearOpMode {

    private Telescope telescope = new Telescope(this);

    @Override
    public void runOpMode() throws InterruptedException {
        telescope.init(hardwareMap);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            telescope.loop(gamepad1);
        }
    }
}
