package org.firstinspires.ftc.teamcode.opmode.dev;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.stuyfission.fissionlib.input.GamepadStatic;

import org.firstinspires.ftc.teamcode.hardware.mechanisms.Wrist;
import org.firstinspires.ftc.teamcode.opmode.teleop.Controls;

@TeleOp(name = "Wrist Dev", group = "dev")
public class WristDev extends LinearOpMode {

    private Wrist wrist = new Wrist(this);

    @Override
    public void runOpMode() throws InterruptedException {
        wrist.init(hardwareMap);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            wrist.loop(gamepad1);
            if (GamepadStatic.isButtonPressed(gamepad1, Controls.PIVOT_FRONT)) {
                wrist.intakePos();
            } else if (GamepadStatic.isButtonPressed(gamepad1, Controls.PIVOT_WALL)) {
                wrist.wallPos();
            } else if (GamepadStatic.isButtonPressed(gamepad1, Controls.PIVOT_CLIP)) {
                wrist.clipPos();
            } else if (GamepadStatic.isButtonPressed(gamepad1, Controls.PIVOT_BASKET)) {
                wrist.basketPos();
            }
        }
    }
}
