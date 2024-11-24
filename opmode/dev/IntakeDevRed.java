package org.firstinspires.ftc.teamcode.opmode.dev;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.mechanisms.Intake;

@TeleOp(name = "Intake Dev Red", group = "dev")
public class IntakeDevRed extends LinearOpMode {

    private Intake intake = new Intake(this, "red");

    @Override
    public void runOpMode() throws InterruptedException {
        intake.init(hardwareMap);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            intake.loop(gamepad1);
        }
    }
}
