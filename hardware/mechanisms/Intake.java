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
public class Intake extends Mechanism {

    private Servo intakeServo;

    //Will this cause an error?
    private double intakePosition = intakeServo.getPosition();

    public Intake(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void init(HardwareMap hwMap) {
        intakeServo = hwMap.get(Servo.class, "intakeServo");
    }

    @Override
    public void loop(Gamepad gamepad) {
        //Need help to run a servo continuously
        while (GamepadStatic.isButtonPressed(gamepad, Controls.INTAKE) && !GamepadStatic.isButtonPressed(gamepad, Controls.OUTTAKE) ) {
            intakeServo.setDirection(Servo.Direction.REVERSE);
            intakeServo.setPosition(1);
            if (intakePosition == 1) {
                intakeServo.setPosition(0.1);
            }
        }
        //Same down here
        while (GamepadStatic.isButtonPressed(gamepad, Controls.OUTTAKE) && !GamepadStatic.isButtonPressed(gamepad, Controls.INTAKE) ) {
            intakeServo.setDirection(Servo.Direction.FORWARD);
            intakeServo.setPosition(1);
            if (intakePosition == 1) {
                intakeServo.setPosition(0.1);
            }
        }

    }

}
