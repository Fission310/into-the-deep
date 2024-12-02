package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.stuyfission.fissionlib.input.GamepadStatic;
import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.teamcode.opmode.teleop.Controls;

@Config
public class Claw extends Mechanism {
    public static double GRAB_POS = 0.6;
    public static double RELEASE_POS = 0.29;

    private Servo clawServo;

    public Claw(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void grab() {
        clawServo.setPosition(GRAB_POS);
    }

    public void release() {
        clawServo.setPosition(RELEASE_POS);
    }

    @Override
    public void init(HardwareMap hwMap) {
        clawServo = hwMap.get(Servo.class, "clawServo");
        grab();
    }

    @Override
    public void loop(Gamepad gamepad) {
        if (GamepadStatic.isButtonPressed(gamepad, Controls.INTAKE)) {
            grab();
        } else if (GamepadStatic.isButtonPressed(gamepad, Controls.OUTTAKE)) {
            release();
        }
    }
}
