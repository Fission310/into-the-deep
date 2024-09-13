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
public class Arm extends Mechanism {

    private Servo leftServo;
    private Servo rightServo;

    public static double INTAKE_POS = 0;
    public static double SCORE_POS = 0;

    public Arm(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void init(HardwareMap hwMap) {
        leftServo = hwMap.get(Servo.class, "armLeftServo");
        rightServo = hwMap.get(Servo.class, "armRightServo");

        leftServo.setDirection(Servo.Direction.REVERSE);

        intakePos();
    }

    public void scorePos() {
        leftServo.setPosition(SCORE_POS);
        rightServo.setPosition(SCORE_POS);
    }

    public void intakePos() {
        leftServo.setPosition(INTAKE_POS);
        rightServo.setPosition(INTAKE_POS);
    }

    @Override
    public void loop(Gamepad gamepad) {
        if (GamepadStatic.isButtonPressed(gamepad, Controls.SCORE)) {
            scorePos();
        } else if (GamepadStatic.isButtonPressed(gamepad, Controls.GRAB)) {
            intakePos();
        }
    }
}
