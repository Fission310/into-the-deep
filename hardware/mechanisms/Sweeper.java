package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.stuyfission.fissionlib.input.GamepadStatic;
import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.teamcode.opmode.teleop.Controls;
@Config
public class Sweeper extends Mechanism {
    private Servo sweeperServo;

    public static double currPos = 0.0;
    public static double LOW_POS = 0.7;
    public static double UP_POS = 0.05;

    public Sweeper(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void upPos() {
        currPos = UP_POS;
        updatePosition();
    }

    public void lowPos() {
        currPos = LOW_POS;
        updatePosition();
    }

    private void updatePosition() {
        sweeperServo.setPosition(currPos);
    }


    @Override
    public void init(HardwareMap hwMap) {
        sweeperServo = hwMap.get(Servo.class, "sweeperServo");
        currPos = LOW_POS;
        updatePosition();
    }

    @Override
    public void loop(Gamepad gamepad) {}
}
