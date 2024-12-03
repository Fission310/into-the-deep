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

    public static double[][] INTAKE_POS = { { 0.61, 0.99 }, { 0.65, 0.95 }, { 0.71, 0.88 }, { 0.76, 0.83 } };
    public static double[][] FRONT_POS = { { 0.25, 0.25 }, { 0.25, 0.25 }, { 0.25, 0.25 }, { 0.25, 0.25 } };
    public static double[][] WALL_POS = { { 0.57, 0.57 }, { 0.57, 0.57 }, { 0.57, 0.57 }, { 0.57, 0.57 } };
    public static double[][] BASKET_POS = { { 0.25, 0.25 }, { 0.25, 0.25 }, { 0.25, 0.25 }, { 0.25, 0.25 } };
    public static double[][] CLIP_POS = { { 0.25, 0.25 }, { 0.25, 0.25 }, { 0.25, 0.25 }, { 0.25, 0.25 } };
    public static double[][] CLIP_SCORE_POS = { { 0.05, 0.05 }, { 0.05, 0.05 }, { 0.05, 0.05 }, { 0.05, 0.05 } };
    public static double[][] BACK_POS = { { 0.05, 0.05 }, { 0.05, 0.05 }, { 0.05, 0.05 }, { 0.05, 0.05 } };
    public static double[][] currPos;

    private int wristPos = 0;

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

    public void rotateLeft() {
        wristPos = (wristPos + 1) % 4;
        wristServoRight.setPosition(currPos[wristPos][0]);
        wristServoLeft.setPosition(currPos[wristPos][1]);
    }

    public void rotateRight() {
        wristPos = (wristPos + 3) % 4;
        wristServoRight.setPosition(currPos[wristPos][0]);
        wristServoLeft.setPosition(currPos[wristPos][1]);
    }

    public void frontPos() {
        currPos = FRONT_POS;
        wristServoRight.setPosition(currPos[wristPos][0]);
        wristServoLeft.setPosition(currPos[wristPos][1]);
    }

    public void intakePos() {
        currPos = INTAKE_POS; 
        wristServoRight.setPosition(currPos[wristPos][0]);
        wristServoLeft.setPosition(currPos[wristPos][1]);
    }

    public void wallPos() {
        currPos = WALL_POS;
        wristServoRight.setPosition(currPos[wristPos][0]);
        wristServoLeft.setPosition(currPos[wristPos][1]);
    }

    public void basketPos() {
        currPos = BASKET_POS;
        wristServoRight.setPosition(currPos[wristPos][0]);
        wristServoLeft.setPosition(currPos[wristPos][1]);
    }

    public void clipPos() {
        currPos = CLIP_POS;
        wristServoRight.setPosition(currPos[wristPos][0]);
        wristServoLeft.setPosition(currPos[wristPos][1]);
    }

    public void clipScorePos() {
        currPos = CLIP_SCORE_POS;
        wristServoRight.setPosition(currPos[wristPos][0]);
        wristServoLeft.setPosition(currPos[wristPos][1]);
    }

    public void backPos() {
        currPos = BACK_POS;
        wristServoRight.setPosition(currPos[wristPos][0]);
        wristServoLeft.setPosition(currPos[wristPos][1]);
    }

    @Override
    public void loop(Gamepad gamepad) {
        if (GamepadStatic.isButtonPressed(gamepad, Controls.WRIST_LEFT)) {
            rotateLeft();
        } else if (GamepadStatic.isButtonPressed(gamepad, Controls.WRIST_RIGHT)) {
            rotateRight();
        }
    }
}
