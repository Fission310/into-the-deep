package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.stuyfission.fissionlib.input.GamepadStatic;
import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.teamcode.opmode.teleop.Controls;

@Config
public class Wrist extends Mechanism {

    private Servo wristServo;

    public static double LEFT_POS = 0;
    public static double MIDDLE_POS = 0.25;
    public static double RIGHT_POS = -0.25;

    public Wrist(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void init(HardwareMap hwMap) {
        wristServo = hwMap.get(Servo.class, "wristServo");

        middlePos();
    }

    public void middlePos() {
        wristServo.setPosition(MIDDLE_POS);
    }

    public void leftPos() {
        wristServo.setPosition(LEFT_POS);
    }

    public void rightPos() {
        wristServo.setPosition(RIGHT_POS);
    }

    @Override
    public void loop(Gamepad gamepad) {
        if (GamepadStatic.isButtonPressed(gamepad, Controls.MIDDLE_WRIST)) {
            middlePos();
        }
        if (GamepadStatic.isButtonPressed(gamepad, Controls.LEFT_WRIST)) {
            leftPos();
        }
        if (GamepadStatic.isButtonPressed(gamepad, Controls.MIDDLE_WRIST)) {
            rightPos();
        }
    }

}
