// THANK YOU GRANT SO MUCH FOR MAKING LEFT RIGHT AND RIGHT LEFT AND IM WAY TOO LAZY TO CHANGE IT

package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import org.firstinspires.ftc.teamcode.opmode.teleop.Controls;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.stuyfission.fissionlib.command.Command;
import com.stuyfission.fissionlib.command.CommandSequence;
import com.stuyfission.fissionlib.input.GamepadStatic;
import com.stuyfission.fissionlib.util.Mechanism;

@Config
public class Climb extends Mechanism {
    private Servo leftServo;
    private Servo rightServo;
    private DcMotorEx[] leftMotors;
    private DcMotorEx[] rightMotors;

    public static double ENGAGE_POS_LEFT = 0;
    public static double DISENGAGE_POS_LEFT = 1;
    public static double ENGAGE_POS_RIGHT = 0;
    public static double DISENGAGE_POS_RIGHT = 1;

    public static double CLIMB_POWER = 0.4;

    private Command reverseMotors = () -> reverseMotors();
    private Command stopMotors = () -> stop();
    private CommandSequence unlock = new CommandSequence()
            .addCommand(reverseMotors).addWaitCommand(0.4)
            .addCommand(stopMotors)
            .build();

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

    public void unlock() {
        unlock.trigger();
    }

    public void stop() {
        for (DcMotorEx motor : leftMotors) {
            motor.setPower(0);
        }
        for (DcMotorEx motor : rightMotors) {
            motor.setPower(0);
        }
    }

    public void climb() {
        for (DcMotorEx motor : leftMotors) {
            motor.setPower(CLIMB_POWER);
        }
        for (DcMotorEx motor : rightMotors) {
            motor.setPower(CLIMB_POWER);
        }
    }

    private void reverseMotors() {
        for (DcMotorEx motor : leftMotors) {
            motor.setPower(-CLIMB_POWER);
        }
        for (DcMotorEx motor : rightMotors) {
            motor.setPower(-CLIMB_POWER);
        }
    }

    @Override
    public void loop(Gamepad gamepad) {
        if (GamepadStatic.isButtonPressed(gamepad, Controls.DISENGAGE)) {
            disengage();
        }
        if (GamepadStatic.isButtonPressed(gamepad, Controls.CLIMB_1)) {
            engage();
        }
        if (GamepadStatic.isButtonPressed(gamepad, Controls.CLIMB_2)) {
            climb();
        }
    }
}
