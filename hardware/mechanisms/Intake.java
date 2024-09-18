package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.stuyfission.fissionlib.input.GamepadStatic;
import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.teamcode.opmode.teleop.Controls;

@Config
public class Intake extends Mechanism {

    private CRServo intakeServo;

    public Intake(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void init(HardwareMap hwMap) {
        intakeServo = hwMap.get(CRServo.class, "intakeServo");
    }

    @Override
    public void loop(Gamepad gamepad) {

        while (GamepadStatic.isButtonPressed(gamepad, Controls.INTAKE) && !GamepadStatic.isButtonPressed(gamepad, Controls.OUTTAKE) ) {
            intakeServo.setPower(1);
        }

        while (GamepadStatic.isButtonPressed(gamepad, Controls.OUTTAKE) && !GamepadStatic.isButtonPressed(gamepad, Controls.INTAKE) ) {
            intakeServo.setPower(-1);
        }

        while (!GamepadStatic.isButtonPressed(gamepad, Controls.OUTTAKE) && !GamepadStatic.isButtonPressed(gamepad, Controls.INTAKE) ) {
            intakeServo.setPower(0);
        }

    }

}
