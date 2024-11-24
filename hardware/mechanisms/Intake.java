package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import com.acmerobotics.dashboard.FtcDashboard;import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.robotcore.external.Telemetry;import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.opmode.auton.util.Color;

@Config
public class Intake extends Mechanism {
    public static double POWER = 1;
    public static int DISTANCE_THRESHOLD = 11;
    public static int BLUE_THRESHOLD = 10;
    public static int RED_THRESHOLD = 10;

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
        outtakeFront();
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

    @Override
    public void loop(Gamepad gamepad) {
        if (sensor.getDistance(DistanceUnit.MM) < DISTANCE_THRESHOLD) {
            if (color == Color.BLUE && sensor.red() < RED_THRESHOLD) {
                stop();
            }
            if (color == Color.RED && sensor.blue() < BLUE_THRESHOLD) {
                stop();
            }
        }
        Telemetry t = FtcDashboard.getInstance().getTelemetry();
        t.addData("distance", sensor.getDistance(DistanceUnit.MM));
        t.addData("blue", sensor.blue());
        t.addData("red", sensor.red());
        t.update();
    }
}
