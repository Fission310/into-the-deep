package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Servo.Direction;
import com.stuyfission.fissionlib.input.GamepadStatic;
import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.teamcode.opmode.teleop.Controls;

@Config
public class Wrist extends Mechanism {
    private Servo wristServoRight;
    private Servo wristServoLeft;

    public static double[][] INTAKE_POS = { { 0.7, 0.7 }, { 0.66, 0.74 }, { 0.61, 0.79 }, { 0.56, 0.84 } };
    public static double[][] FRONT_POS = { { 0.13, 0.13 }, { 0.13, 0.13 }, { 0.13, 0.13 }, { 0.13, 0.13 } };
    public static double[][] WALL_POS = { { 0.51, 0.49 }, { 0.51, 0.49 }, { 0.51, 0.49 }, { 0.51, 0.49 } };
    public static double[][] BASKET_POS = { { 0.3, 0.3 }, { 0.3, 0.3 }, { 0.3, 0.3 }, { 0.3, 0.3 } };
    public static double[][] CLIP_POS = { { 0.6, 0.6 }, { 0.6, 0.6 }, { 0.6, 0.6 }, { 0.6, 0.6 } };
    public static double[][] CLIP_SCORE_POS = { { 0.6, 0.6 }, { 0.6, 0.6 }, { 0.6, 0.6 }, { 0.6, 0.6 } };
    public static double[][] BACK_POS = { { 0.25, 0.25 }, { 0.25, 0.25 }, { 0.25, 0.25 }, { 0.25, 0.25 } };
    public static double[][] currPos;

    private int wristPos = 0;
    private boolean pressed = false;

    public Wrist(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void init(HardwareMap hwMap) {
        wristServoRight = hwMap.get(Servo.class, "wristServoRight");
        wristServoLeft = hwMap.get(Servo.class, "wristServoLeft");
        wristServoLeft.setDirection(Direction.REVERSE);
        frontPos();
    }

    private void setPosition() {
        wristServoRight.setPosition(currPos[wristPos][0]);
        wristServoLeft.setPosition(currPos[wristPos][1]);
    }

    public void rotateLeft() {
        wristPos = (wristPos + 1) % 4;
        setPosition();
    }

    public void rotateRight() {
        wristPos = (wristPos + 3) % 4;
        setPosition();
    }

    public void defaultPos() {
        wristPos = 0;
        setPosition();
    }

    public void frontPos() {
        currPos = FRONT_POS;
        setPosition();
    }

    public void intakePos() {
        currPos = INTAKE_POS;
        setPosition();
    }

    public void wallPos() {
        currPos = WALL_POS;
        setPosition();
    }

    public void basketPos() {
        currPos = BASKET_POS;
        setPosition();
    }

    public void clipPos() {
        currPos = CLIP_POS;
        setPosition();
    }

    public void clipScorePos() {
        currPos = CLIP_SCORE_POS;
        setPosition();
    }

    public void backPos() {
        currPos = BACK_POS;
        setPosition();
    }

    @Override
    public void loop(Gamepad gamepad) {
        if (GamepadStatic.isButtonPressed(gamepad, Controls.WRIST_LEFT)) {
            if (!pressed) {
                rotateLeft();
            }
            pressed = true;
        } else if (GamepadStatic.isButtonPressed(gamepad, Controls.WRIST_RIGHT)) {
            if (!pressed) {
                rotateRight();
            }
            pressed = true;
        } else {
            pressed = false;
        }
    }
}
