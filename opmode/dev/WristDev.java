package org.firstinspires.ftc.teamcode.opmode.dev;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.stuyfission.fissionlib.input.GamepadStatic;
import com.stuyfission.fissionlib.input.GamepadStatic.Input;

import org.firstinspires.ftc.teamcode.hardware.mechanisms.Wrist;
import org.firstinspires.ftc.teamcode.opmode.teleop.Controls;

@Config
@TeleOp(name = "Wrist Dev", group = "dev")
public class WristDev extends LinearOpMode {

    private Wrist wrist = new Wrist(this);
    public static double RIGHT_POS = 0.7;
    public static double LEFT_POS = 0.7;

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
            } else if (GamepadStatic.isButtonPressed(gamepad1, Input.DPAD_UP)) {
                wrist.updatePos(LEFT_POS, RIGHT_POS);
            }
        }
    }
}
