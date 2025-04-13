package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.stuyfission.fissionlib.input.GamepadStatic;
import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.opmode.auton.util.Color;
import org.firstinspires.ftc.teamcode.opmode.teleop.Controls;

@Config
public class Intake extends Mechanism {
    public static double INTAKE_POWER = -1;
    public static double OUTTAKE_POWER = 0.8;
    public static double SAMPLE_ROTATION = 200;
    public static int SAMPLE1 = 22;
    public static int RED1 = 500;
    public static int BLUE1 = 600;
    public static int YELLOW1 = 1000;

    public static int SAMPLE2 = 22;
    public static int RED2 = 1500;
    public static int BLUE2 = 1200;
    public static int YELLOW2 = 2500;

    private CRServo rightServo;
    private CRServo leftServo;
    private SampleSensor sampleSensor1;
    private SampleSensor sampleSensor2;
    private AnalogInput encoder;
    private double prevPos;
    private double currPos;
    private double rotation;

    public Intake(LinearOpMode opMode) {
        this.opMode = opMode;
        sampleSensor1 = new SampleSensor(opMode, "intakeSensor1", SAMPLE1, RED1, BLUE1, YELLOW1);
        sampleSensor2 = new SampleSensor(opMode, "intakeSensor2", SAMPLE2, RED2, BLUE2, YELLOW2);
    }

    public void intake() {
        stop();
        rightServo.setPower(INTAKE_POWER);
        leftServo.setPower(-INTAKE_POWER);
    }

    public void outtake() {

        rightServo.setPower(INTAKE_POWER);
        leftServo.setPower(-INTAKE_POWER);
    }

    public void stop() {

        rightServo.setPower(INTAKE_POWER);
        leftServo.setPower(-INTAKE_POWER);
    }

    public double getPosition() {
        return encoder.getVoltage() / 3.2 * 360;
    }

    public void setPosition() {
        prevPos = currPos;
        currPos = getPosition();
        rotation = Math.abs(currPos - prevPos);
    }

    public boolean hasSample() {
        return rotation <= SAMPLE_ROTATION;
    }

    public boolean hasColor(Color color) {
        return sampleSensor1.isSampleColor(color) || sampleSensor2.isSampleColor(color);
    }

    public boolean hasWrongColor(Color color) {
        return sampleSensor1.isSampleWrongColor(color) || sampleSensor2.isSampleWrongColor(color);
    }

    @Override
    public void init(HardwareMap hwMap) {
        rightServo = hwMap.get(CRServo.class, "rightServo");
        rightServo.setDirection(Direction.FORWARD);
        leftServo = hwMap.get(CRServo.class, "leftServo");
        leftServo.setDirection(Direction.FORWARD);

        sampleSensor1.init(hwMap);
        sampleSensor2.init(hwMap);
        encoder = hwMap.get(AnalogInput.class, "intakeEncoder");
        currPos = getPosition();
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("positionPrev", prevPos);
        telemetry.addData("positionCurr", currPos);
        telemetry.addData("rotation", rotation);
        sampleSensor1.telemetry(telemetry);
        sampleSensor2.telemetry(telemetry);
    }

    public void update() {
        setPosition();
        sampleSensor1.update();
        sampleSensor2.update();
    }

    @Override
    public void loop(Gamepad gamepad) {
        update();
        if (GamepadStatic.isButtonPressed(gamepad, Controls.GRAB)) {
            intake();
        } else if (GamepadStatic.isButtonPressed(gamepad, Controls.OUTTAKE)) {
            outtake();
        }
    }

    private class SampleSensor extends Mechanism {

        private ColorRangeSensor sensor;
        private String name;

        private double FAR;
        private double dist;
        private int red, blue, green;
        private double RED, BLUE, YELLOW;

        public SampleSensor(LinearOpMode opMode, String name, double far, double red, double blue, double yellow) {
            this.opMode = opMode;
            this.name = name;
            this.FAR = far;

            this.RED = red;
            this.BLUE = blue;
            this.YELLOW = yellow;
        }

        public void init(HardwareMap hwMap) {
            sensor = hwMap.get(ColorRangeSensor.class, name);

            sensor.enableLed(false);
        }

        public boolean isSample() {
            return dist < FAR;
        }

        public void update() {
            dist = sensor.getDistance(DistanceUnit.MM);
            blue = sensor.blue();
            red = sensor.red();
            green = sensor.green();
        }

        public boolean isSampleColor(Color color) {
            if (color == Color.BLUE) {
                return (isBlue() || isYellow()) && isSample();
            }
            return (isRed() || isYellow()) && isSample();
        }

        public boolean isSampleWrongColor(Color color) {
            if (color == Color.RED) {
                return isBlue();
            }
            return isRed();
        }

        public boolean isBlue() {
            return blue > BLUE;
        }

        public boolean isRed() {
            return red > RED && !isYellow();
        }

        public boolean isYellow() {
            return green > YELLOW;
        }

        @Override
        public void telemetry(Telemetry telemetry) {
            telemetry.addData(name + " red", red);
            telemetry.addData(name + " blue", blue);
            telemetry.addData(name + " green", green);
            telemetry.addData(name + " is red", isRed());
            telemetry.addData(name + " is blue", isBlue());
            telemetry.addData(name + " is yellow", isYellow());
            telemetry.addData(name + " is close", isSample());
            telemetry.addData(name + " dist", Math.round(dist));
        }
    }
}
