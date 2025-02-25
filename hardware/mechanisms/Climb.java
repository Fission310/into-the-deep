package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.stuyfission.fissionlib.util.Mechanism;

@Config
public class Climb extends Mechanism {
    private Servo leftServo;
    private Servo rightServo;
    private DcMotorEx[] leftMotors;
    private DcMotorEx[] rightMotors;

    public static double ENGAGE_POS_LEFT = 1.0;
    public static double DISENGAGE_POS_LEFT = 0.16;
    public static double ENGAGE_POS_RIGHT = 1.0;
    public static double DISENGAGE_POS_RIGHT = 0.16;

    public static double CLIMB_POWER = 0.4;

    public Climb(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void init(HardwareMap hwMap) {
        leftServo = hwMap.get(Servo.class, "climbLeftServo");
        rightServo = hwMap.get(Servo.class, "climbRightServo");
        leftMotors = new DcMotorEx[] {
                hwMap.get(DcMotorEx.class, "leftFront"),
                hwMap.get(DcMotorEx.class, "leftRear")
        };
        rightMotors = new DcMotorEx[] {
                hwMap.get(DcMotorEx.class, "rightFront"),
                hwMap.get(DcMotorEx.class, "rightRear")
        };
        disengage();
    }

    public void disengage() {
        leftServo.setPosition(DISENGAGE_POS_LEFT);
        rightServo.setPosition(DISENGAGE_POS_RIGHT);
    }

    public void engage() {
        leftServo.setPosition(ENGAGE_POS_LEFT);
        rightServo.setPosition(ENGAGE_POS_RIGHT);
    }

    public void climb() {
        for (DcMotorEx motor : leftMotors) {
            motor.setPower(CLIMB_POWER);
        }
        for (DcMotorEx motor : rightMotors) {
            motor.setPower(CLIMB_POWER);
        }
    }

    @Override
    public void loop(Gamepad gamepad) {}
}
