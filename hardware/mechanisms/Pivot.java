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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmode.teleop.Controls;
import org.firstinspires.ftc.teamcode.util.Controller;

@Config
public class Pivot extends Mechanism {
    public static int FRONT_POS = 500;
    public static int INTAKE_UP_POS = 650;
    public static int INTAKE_DOWN_POS = 275;
    public static int WALL_POS = 1950;
    public static int BASKET_POS = 2250; // FIGURE OUT POSITION
    public static int CLIP_POS = 2800;
    public static int BACK_POS = 3950;
    public static int UP_POS = 2100;
    public static int HIGHEST = 2100;

    public static double UKP = 0.000711;
    public static double UKI = 0;
    public static double UKD = 0;
    public static double DKP = 0.0002811;
    public static double DKI = 0;
    public static double DKD = 0;

    public static double target = 0;
    public static double actualTarget = 0;
    public static double power = 0;

    private Controller controller;

    private final DcMotorEx[] motors = new DcMotorEx[2];

    private VoltageSensor voltage;

    public Pivot(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void init(HardwareMap hwMap) {
        voltage = hwMap.voltageSensor.iterator().next();
        controller = new Controller(UKP, UKI, UKD, DKP, DKI, DKD, voltage, HIGHEST);

        motors[0] = hwMap.get(DcMotorEx.class, "pivotLeftMotor");
        motors[1] = hwMap.get(DcMotorEx.class, "pivotRightMotor");

        motors[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motors[0].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); // might be wrong RunMode
        motors[1].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        motors[0].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motors[1].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        motors[0].setDirection(DcMotorEx.Direction.REVERSE);
        motors[1].setDirection(DcMotorEx.Direction.FORWARD);

        frontPos();
    }

    public void telemetry(Telemetry telemetry) {
        //telemetry.addData("Current Position", getPosition());
        //telemetry.addData("Target", target);
        //telemetry.addData("Power", power);
        telemetry.update();
    }

    public void frontPos() {
        setTarget(FRONT_POS);
    }
    public void intakeUpPos() {
        setTarget(INTAKE_UP_POS);
    }
    public void intakeDownPos() {
        setTarget(INTAKE_DOWN_POS);
    }

    public void wallPos() {
        setTarget(WALL_POS);
    }

    public void basketPos() {
        setTarget(BASKET_POS);
    }

    public void clipPos() {
        setTarget(CLIP_POS);
    }

    public void backPos() {
        setTarget(BACK_POS);
    }

    public void upPos() {
        setTarget(UP_POS);
    }

    public void setTarget(double target) {
        Pivot.target = target;
        Pivot.actualTarget = target;
    }

    public double getPosition() {
        return motors[0].getCurrentPosition();
    }

    public void update() {
        controller.setTargetPosition(target);
        power = controller.getPower(getPosition());
        Telemetry t = FtcDashboard.getInstance().getTelemetry();
        t.addData("pivot power", power);
        t.addData("pivot position", getPosition());
        t.addData("target position", target);
        t.update();
        motors[0].setPower(power);
        motors[1].setPower(power);
    }

    @Override
    public void loop(Gamepad gamepad) {
        update();
        if (GamepadStatic.isButtonPressed(gamepad, Controls.PIVOT_FRONT)) {
            frontPos();
        } else if (GamepadStatic.isButtonPressed(gamepad, Controls.PIVOT_WALL)) {
            wallPos();
        } else if (GamepadStatic.isButtonPressed(gamepad, Controls.PIVOT_BASKET)) {
            basketPos();
        } else if (GamepadStatic.isButtonPressed(gamepad, Controls.PIVOT_CLIP)) {
            clipPos();
        } else if (GamepadStatic.isButtonPressed(gamepad, Controls.PIVOT_BACK)) {
            backPos();
        }
        Telemetry t = FtcDashboard.getInstance().getTelemetry();
        t.addData("pivot encoder reading", getPosition());
        t.update();
    }
}
