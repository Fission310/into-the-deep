package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.stuyfission.fissionlib.input.GamepadStatic;
import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.opmode.auton.util.Color;
import org.firstinspires.ftc.teamcode.opmode.teleop.Controls;

@Config
public class Intake extends Mechanism {
    public static double POWER = 1;
    public static int DISTANCE_THRESHOLD = 40;
    public static int BLUE_THRESHOLD = 80;
    public static int RED_THRESHOLD = 300;

    private ColorRangeSensor sensor;
    private Color color;
    private CRServo intakeServoRight;
    private CRServo intakeServoLeft;

    public Intake(LinearOpMode opMode, Color color) {
        this.opMode = opMode;
        this.color = color;
    }

    public void intakeFront() {
        intakeServoRight.setPower(POWER);
        intakeServoLeft.setPower(-POWER);
    }

    public void outtakeFront() {
        intakeServoRight.setPower(-POWER);
        intakeServoLeft.setPower(POWER);
    }

    public void intakeWall() {
        intakeFront();
    }

    public void outtakeWall() {
        outtakeFront();
    }

    public void outtakeBasket() {
        intakeFront();
    }

    public void outtakeClip() {
        intakeFront();
    }

    public void intakeBack() {
        outtakeFront();
    }

    public void outtakeBack() {
        intakeFront();
    }

    public void stop() {
        intakeServoRight.setPower(0);
        intakeServoLeft.setPower(0);
    }

    @Override
    public void init(HardwareMap hwMap) {
        intakeServoRight = hwMap.get(CRServo.class, "intakeServoRight");
        intakeServoLeft = hwMap.get(CRServo.class, "intakeServoLeft");
        sensor = hwMap.get(ColorRangeSensor.class, "intakeSensor");
    }

    public boolean isSample() {
        Telemetry t = FtcDashboard.getInstance().getTelemetry();
        t.addData("distance", sensor.getDistance(DistanceUnit.MM));
        t.addData("blue", sensor.blue());
        t.addData("red", sensor.red());
        t.addData("distance check", sensor.getDistance(DistanceUnit.MM) < DISTANCE_THRESHOLD);
        t.addData("color check blue", (color == Color.BLUE && sensor.red() < RED_THRESHOLD));
        t.addData("color check reb", (color == Color.RED && sensor.blue() < BLUE_THRESHOLD));
        t.update();
        return (sensor.getDistance(DistanceUnit.MM) < DISTANCE_THRESHOLD)
                && ((color == Color.BLUE && sensor.red() < RED_THRESHOLD)
                        || (color == Color.RED && sensor.blue() < BLUE_THRESHOLD));
    }

    @Override
    public void loop(Gamepad gamepad) {
        if (!isSample() && GamepadStatic.isButtonPressed(gamepad, Controls.INTAKE)) {
            intakeBack();
        } else if (GamepadStatic.isButtonPressed(gamepad, Controls.OUTTAKE)) {
            outtakeBack();
        } else {
            stop();
        }
    }
}
