package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.stuyfission.fissionlib.input.GamepadStatic;
import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.teamcode.opmode.teleop.Controls;

@Config
public class Intake extends Mechanism {
    public static double POWER = 1;

    private CRServo intakeServo;

    public Intake(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void intake() {
        intakeServo.setPower(POWER);
    }

    public void outtake() {
        intakeServo.setPower(-POWER);
    }

    public void stop() {
        intakeServo.setPower(0);
    }

    @Override
    public void init(HardwareMap hwMap) {
        intakeServo = hwMap.get(CRServo.class, "intakeServo");
    }

    @Override
    public void loop(Gamepad gamepad) {
        if (GamepadStatic.isButtonPressed(gamepad, Controls.INTAKE)) {
            intake();
        } else if (GamepadStatic.isButtonPressed(gamepad, Controls.OUTTAKE)) {
            outtake();
        } else {
            stop();
        }
    }
}
