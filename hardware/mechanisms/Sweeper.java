package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.stuyfission.fissionlib.util.Mechanism;

@Config
public class Sweeper extends Mechanism {
    private Servo sweeperServo;

    public static boolean extended = false;
    public static double RETRACT_POS = 0.92;
    public static double EXTEND_POS = 0.13;

    public Sweeper(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void init(HardwareMap hwMap) {
        sweeperServo = hwMap.get(Servo.class, "sweeperServo");
        retractPos();
    }

    public void extendPos() {
        sweeperServo.setPosition(EXTEND_POS);
        extended = true;
    }

    public void retractPos() {
        sweeperServo.setPosition(RETRACT_POS);
        extended = false;
    }

    public void toggle() {
        if (extended) {
            retractPos();
        } else {
            extendPos();
        }
    }

    @Override
    public void loop(Gamepad gamepad) {}
}
