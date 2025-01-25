package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.stuyfission.fissionlib.input.GamepadStatic;
import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.opmode.teleop.Controls;

@Config
public class Intake extends Mechanism {
    public static double INTAKE_POWER = 1;
    public static double OUTTAKE_POWER = -0.8;
    public static int SAMPLE = 20;
    public static int RED = 10;
    public static int BLUE = 10;
    public static int YELLOW = 10;

    private CRServo intakeServo;
    private SampleSensor sampleSensor;

    public Intake(LinearOpMode opMode) {
        this.opMode = opMode;
        sampleSensor = new SampleSensor(opMode, "SampleSensor", SAMPLE);
    }

    public void intake() {
        stop();
        intakeServo.setPower(INTAKE_POWER);
    }

    public void outtake() {
        intakeServo.setPower(OUTTAKE_POWER);
    }

    public void stop() {
        intakeServo.setPower(0);
    }

    public boolean isSample(){
        return sampleSensor.isSample() && sampleSensor.isSampleColor();
    }

    @Override
    public void init(HardwareMap hwMap) {
        intakeServo = hwMap.get(CRServo.class, "intakeServo");
        intakeServo.setDirection(Direction.REVERSE);

        sampleSensor.init(hwMap);
    }

    @Override
    public void loop(Gamepad gamepad) {
        if (GamepadStatic.isButtonPressed(gamepad, Controls.GRAB)) {
            intake();
        } else if (GamepadStatic.isButtonPressed(gamepad, Controls.OUTTAKE)) {
            outtake();
        }
    }
    private class SampleSensor extends Mechanism {

        private ColorRangeSensor sensor;
        private String name;

        private double far;

        public SampleSensor(LinearOpMode opMode, String name, double far) {
            this.opMode = opMode;
            this.name = name;
            this.far = far;
        }

        public void init(HardwareMap hwMap) {
            sensor = hwMap.get(ColorRangeSensor.class, name);

            sensor.enableLed(false);
        }

        public boolean isSample() {
            return sensor.getDistance(DistanceUnit.MM) < far;
        }

        public boolean isSampleColor() {
            int blue = sensor.blue();
            int red = sensor.red();
            int green = sensor.green();
            int yellow = (red + green) / 2;
            boolean isSample = red > RED || blue > BLUE || yellow > YELLOW;
            Telemetry t = FtcDashboard.getInstance().getTelemetry();
            t.addData(name + " red", red);
            t.addData(name + " blue", blue);
            t.addData(name + " yellow", yellow);
            t.addData(name + " isSample", isSample);
            t.update();
            return isSample;
        }

        @Override
        public void telemetry(Telemetry telemetry) {
            telemetry.addData(name + " dist", sensor.getDistance(DistanceUnit.MM));
            telemetry.update();
        }
    }
}
