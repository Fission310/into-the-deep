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
    private Servo wristServoRight;
    private Servo wristServoLeft;

    public static double FRONT_POS = 0;
    public static double FRONT_UP_POS = 0.5;
    public static double UP_POS = 0.4;
    public static double BACK_POS = 1;

    public Wrist(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void init(HardwareMap hwMap) {
        wristServoRight = hwMap.get(Servo.class, "wristServoRight");
        wristServoLeft = hwMap.get(Servo.class, "wristServoLeft");
        frontPos();
    }

    public void frontPos() {
        wristServoRight.setPosition(FRONT_POS);
        wristServoLeft.setPosition(FRONT_POS);
    }

    public void frontUpPos() {
        wristServoRight.setPosition(FRONT_UP_POS);
        wristServoLeft.setPosition(FRONT_UP_POS);
    }

    public void upPos() {
        wristServoRight.setPosition(UP_POS);
        wristServoLeft.setPosition(UP_POS);
    }

    public void backPos() {
        wristServoRight.setPosition(BACK_POS);
        wristServoLeft.setPosition(BACK_POS);
    }

    @Override
    public void loop(Gamepad gamepad) {
        if (GamepadStatic.isButtonPressed(gamepad, Controls.PIVOT_FRONT)) {
            frontPos();
        } else if (GamepadStatic.isButtonPressed(gamepad, Controls.PIVOT_UP_FRONT)) {
            frontUpPos();
        } else if (GamepadStatic.isButtonPressed(gamepad, Controls.PIVOT_UP)) {
            upPos();
        } else if (GamepadStatic.isButtonPressed(gamepad, Controls.PIVOT_BACK)) {
            backPos();
        }
    }
}
