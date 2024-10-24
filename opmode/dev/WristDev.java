package org.firstinspires.ftc.teamcode.opmode.dev;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.mechanisms.Wrist;

@TeleOp(name = "Wrist Dev", group = "dev")
public class WristDev extends LinearOpMode {

    private Wrist wrist = new Wrist(this);

    @Override
    public void runOpMode() throws InterruptedException {
        wrist.init(hardwareMap);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            wrist.loop(gamepad1);
        }
    }
}
