package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Servo.Direction;
import com.stuyfission.fissionlib.input.GamepadStatic;
import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmode.teleop.Controls;

@Config
public class Wrist extends Mechanism {
    private Servo wristServoRight;
    private Servo wristServoLeft;

    public static double INTAKE_DOWN_ABIT = 0.006;
    public static double BASKET_DOWN_ABIT = -0.15;
    public static double[][] AUTO_INTAKE_POS =  { { 0.54, 0.54 }, { 0.59, 0.58 }, { 0.59, 0.58 }, { 0.59, 0.58 } };
    public static double[][] AUTO_INTAKE_LL_POS =  { { .5, 0.48 }, { 0.59, 0.58 }, { 0.59, 0.58 }, { 0.59, 0.58 } };
    public static double[][] AUTO_BASKET_POS =  { { .75, 0.315 }, { 0.59, 0.58 }, { 0.59, 0.58 }, { 0.59, 0.58 } };
    public static double[][] AUTO_MID_POS =  { { .5, 0.460 }, { 0.59, 0.58 }, { 0.59, 0.58 }, { 0.59, 0.58 } };
    public static double[][] INTAKE_POS = { { 0.64, 0.64 }, { 0.59, 0.58 }, { 0.59, 0.58 }, { 0.59, 0.58 } };
    public static double[][] INTAKE_MID_POS = { { .6875, 0.330 }, { 0.59, 0.58 }, { 0.59, 0.58 }, { 0.59, 0.58 } };
    public static double[][] INTAKE_SHORT_POS = { { .65, 0.65 }, { 0.59, 0.58 }, { 0.59, 0.58 }, { 0.59, 0.58 } };
    public static double[][] INTAKE_DOWN_POS = { { 1.0, 1.0 }, { 0.76, 0.53 }, { 0.76, 0.53 }, { 0.76, 0.53 } };
    public static double[][] FRONT_POS = { { .8, 0.330 }, { 0.25, 0.25 }, { 0.25, 0.25 }, { 0.25, 0.25 } };
    public static double[][] WALL_POS = { { 0.325, 0.315 }, { 0.43, 0.42 }, { 0.43, 0.42 }, { 0.43, 0.42 } };
    public static double[][] BASKET_POS = { { .7, 0.310 }, { 0.35, 0.35 }, { 0.35, 0.35 }, { 0.35, 0.35 } };
    public static double[][] CLIP_POS = { { 0.545, 0.545 }, { 0.65, 0.65 }, { 0.65, 0.65 }, { 0.65, 0.65 } };
    public static double[][] CLIP_SCORE_POS = { { 0.5, 0.53 }, { 0.65, 0.65 }, { 0.65, 0.65 }, { 0.65, 0.65 } };
    public static double[][] BACK_POS = { { 0.15, 0.15 }, { 0.25, 0.25 }, { 0.25, 0.25 }, { 0.25, 0.25 } };
    public static double[][] CLIMB_POS = { { 0.5, 0.5 }, { 0.7, 0.7 }, { 0.7, 0.7 }, { 0.7, 0.7 } };
    public static double[][] RETRACT_POS = { { 0.695, 0.695 }, { 0.7, 0.7 }, { 0.7, 0.7 }, { 0.7, 0.7 } };
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
        currPos = FRONT_POS;
    }
    public void intakeABit(){
        wristServoRight.setPosition(currPos[wristPos][0] + INTAKE_DOWN_ABIT);
        wristServoLeft.setPosition(currPos[wristPos][0] + INTAKE_DOWN_ABIT);
    }

    public void basketABit(){
        wristServoRight.setPosition(currPos[wristPos][0] - BASKET_DOWN_ABIT);
        wristServoLeft.setPosition(currPos[wristPos][0] - BASKET_DOWN_ABIT);
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

    public void down() {
        currPos = INTAKE_DOWN_POS;
        setPosition();
    }

    public void retractPos() {
        currPos = RETRACT_POS;
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

    public void intakeMidPos() {
        currPos = INTAKE_MID_POS;
        setPosition();
    }

    public void intakeShortPos() {
        currPos = INTAKE_SHORT_POS;
        setPosition();
    }

    public void autoIntakePos() {
        currPos = AUTO_INTAKE_POS;
        setPosition();
    }

    public void autoIntakeLLPos() {
        currPos = AUTO_INTAKE_LL_POS;
        setPosition();
    }

    public void autoBasketPos() {
        currPos = AUTO_BASKET_POS;
        setPosition();
    }

    public void autoMidPos() {
        currPos = AUTO_MID_POS;
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

    public void climbPos() {
        currPos = CLIMB_POS;
        setPosition();
    }

    public void updatePos(double left, double right) {
        wristServoRight.setPosition(right);
        wristServoLeft.setPosition(left);
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("wrist pos", currPos[0][0]);
        telemetry.addData("wrist servo left", wristServoLeft.getPosition());
        telemetry.addData("wrist servo right", wristServoRight.getPosition());
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
