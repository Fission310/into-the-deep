package org.firstinspires.ftc.teamcode.opmode.dev;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.opmode.auton.util.Color;

@TeleOp(name = "Intake Dev Blue", group = "dev")
public class IntakeDevBlue extends LinearOpMode {

    private Intake intake = new Intake(this, Color.BLUE);

    @Override
    public void runOpMode() throws InterruptedException {
        intake.init(hardwareMap);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            intake.loop(gamepad1);
        }
    }
}
