package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.stuyfission.fissionlib.input.GamepadStatic;
import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmode.teleop.Controls;

@Config
public class Claw extends Mechanism {
    public static double GRAB_POWER = 1;
    public static double RELEASE_POWER = -0.8;

    private CRServo clawServo;

    public Claw(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void grab() {
        stop();
        clawServo.setPower(GRAB_POWER);
        Telemetry t = FtcDashboard.getInstance().getTelemetry();
        t.addData("claw power", clawServo.getPower());
        t.update();
    }

    public void release() {
        clawServo.setPower(RELEASE_POWER);
    }

    public void stop() {
        clawServo.setPower(0);
    }

    @Override
    public void init(HardwareMap hwMap) {
        clawServo = hwMap.get(CRServo.class, "clawServo");
        clawServo.setDirection(Direction.REVERSE);
    }

    @Override
    public void loop(Gamepad gamepad) {
        if (GamepadStatic.isButtonPressed(gamepad, Controls.GRAB)) {
            grab();
        } else if (GamepadStatic.isButtonPressed(gamepad, Controls.RELEASE)) {
            release();
        }
    }
}
