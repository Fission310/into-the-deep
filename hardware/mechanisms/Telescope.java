package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.stuyfission.fissionlib.input.GamepadStatic;
import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.teamcode.opmode.teleop.Controls;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Telescope extends Mechanism {
    public static int FRONT_POS = -50;
    public static int UP_FRONT_POS = 160;
    public static int UP_POS = 480;
    public static int BACK_POS = 400;
    public static int ABIT = 180;

    public static int FRONT_RETRACTION = -10;
    public static int FRONT_EXTENSION = -50;
    public static int FRONT_UP_RETRACTION = -60;
    public static int FRONT_UP_EXTENSION = 1000;
    public static int UP_RETRACTION = -10;
    public static int UP_EXTENSION = 350;
    public static int BACK_RETRACTION = -10;
    public static int BACK_EXTENSION = 300;
    public static int[] RETRACTIONS = new int[]{FRONT_RETRACTION, FRONT_UP_RETRACTION,
                                                UP_RETRACTION, BACK_RETRACTION};
    public static int retractionState = 0;

    public static double KP = 0.003;
    public static double KI = 0;
    public static double KD = 0;

    public static double target = 0;
    public static double power = 0;
    public static double POWER_MULTIPLIER = 1;

    private final PIDController controller = new PIDController(KP, KI, KD);

    private final DcMotorEx[] motors = new DcMotorEx[2];

    private VoltageSensor voltage;

    public Telescope(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void init(HardwareMap hwMap) {
        motors[0] = hwMap.get(DcMotorEx.class, "telescopeLeftMotor");
        motors[1] = hwMap.get(DcMotorEx.class, "telescopeRightMotor");

        motors[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motors[0].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); // might be wrong RunMode
        motors[1].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        motors[0].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motors[1].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        motors[0].setDirection(DcMotorEx.Direction.REVERSE);
        motors[1].setDirection(DcMotorEx.Direction.REVERSE);

        frontPos();
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Current Position", getPosition());
        telemetry.addData("Target", target);
        telemetry.addData("Power", power);
        telemetry.update();
    }

    public void frontPos() {
        setTarget(FRONT_EXTENSION);
        retractionState = 0;
    }
    public void frontUpPos() {
        setTarget(FRONT_UP_EXTENSION);
        retractionState = 1;
    }
    public void upPos() {
        setTarget(UP_EXTENSION);
        retractionState = 2;
    }
    public void backPos() {
        setTarget(BACK_EXTENSION);
        retractionState = 3;
    }

    public void retractPos(int state) {
        setTarget(RETRACTIONS[state]);
    }

    public void setTarget(double target) {
        Telescope.target = target;
    }

    public double getPosition() {
        return motors[0].getCurrentPosition();
    }

    public void upABit() {
        setTarget(target + ABIT);
    }

    public void downABit() {
        setTarget(target - ABIT);
    }

    public void update() {
        controller.setTarget(target);
        power = controller.calculate(getPosition()) * POWER_MULTIPLIER;
        motors[0].setPower(power);
        motors[1].setPower(power);
        Telemetry t = FtcDashboard.getInstance().getTelemetry();
        t.update();
        t.addData("Power", power);
        RETRACTIONS[0] = FRONT_RETRACTION;
        RETRACTIONS[1] = FRONT_UP_RETRACTION;
        RETRACTIONS[2] = UP_RETRACTION;
        RETRACTIONS[3] = BACK_RETRACTION;
    }

    @Override
    public void loop(Gamepad gamepad) {
        update();
        if (GamepadStatic.isButtonPressed(gamepad, Controls.PIVOT_FRONT)) {
            frontPos();
        } else if (GamepadStatic.isButtonPressed(gamepad, Controls.PIVOT_UP_FRONT)) {
            frontUpPos();
        } else if (GamepadStatic.isButtonPressed(gamepad, Controls.PIVOT_UP)) {
            upPos();
        } else if (GamepadStatic.isButtonPressed(gamepad, Controls.PIVOT_BACK)) {
            backPos();
        } else if (GamepadStatic.isButtonPressed(gamepad, Controls.RETRACT)) {
            retractPos(retractionState);
        }
    }
}