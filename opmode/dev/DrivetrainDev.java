package org.firstinspires.ftc.teamcode.opmode.dev;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.mechanisms.Drivetrain;

@TeleOp(name = "DriveTrain Dev", group = "dev")
public class DrivetrainDev extends LinearOpMode {

    private Drivetrain drivetrain = new Drivetrain(this);

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain.init(hardwareMap);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            drivetrain.loop(gamepad1);
        }
    }
}
