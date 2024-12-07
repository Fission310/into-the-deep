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
import org.firstinspires.ftc.teamcode.util.PIDFController;

@Config
public class Pivot extends Mechanism {
    public static int INIT_POS = 500;
    public static int FRONT_POS = 350;
    public static int INTAKE_UP_POS = 420;
    public static int INTAKE_DOWN_POS = 330;
    public static int INTAKE_GRAB_POS = 120;
    public static int WALL_POS = 800;
    public static int BASKET_POS = 2400;
    public static int CLIP_POS = 1650;
    public static int CLIP_DOWN_POS = 1500;
    public static int CLIP_BACK_POS = 2400;
    public static int CLIP_BACK_DOWN_POS = 2450;
    public static int BACK_POS = 3800;
    public static int UP_POS = 1850;
    public static int HIGHEST = 1950;

    public static double UP_BOTTOM_KP = 0.0009;
    public static double DOWN_TOP_KP = 0.0001;
    public static double UP_TOP_KP = 0.0006;
    public static double DOWN_BOTTOM_KP = 0.00004;

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
        PIDFController upBottom = new PIDFController(UP_BOTTOM_KP, 0, 0, 0);
        PIDFController downTop = new PIDFController(DOWN_TOP_KP, 0, 0, 0);
        PIDFController upTop = new PIDFController(UP_TOP_KP, 0, 0, 0);
        PIDFController downBottom = new PIDFController(DOWN_BOTTOM_KP, 0, 0, 0);
        controller = new Controller(upBottom, downTop, upTop, downBottom, voltage, HIGHEST);

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
        // telemetry.addData("Current Position", getPosition());
        // telemetry.addData("Target", target);
        // telemetry.addData("Power", power);
        telemetry.update();
    }

    public void initPos() {setTarget(INIT_POS);}

    public void frontPos() {
        setTarget(FRONT_POS);
    }

    public void intakeUpPos() {
        setTarget(INTAKE_UP_POS);
    }

    public void intakeDownPos() {
        setTarget(INTAKE_DOWN_POS);
    }

    public void intakeGrabPos() {
        setTarget(INTAKE_GRAB_POS);
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

    public void clipDownPos() {
        setTarget(CLIP_DOWN_POS);
    }

    public void clipBackPos() {
        setTarget(CLIP_BACK_POS);
    }

    public void clipBackDownPos() {
        setTarget(CLIP_BACK_DOWN_POS);
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
        }
        Telemetry t = FtcDashboard.getInstance().getTelemetry();
        t.addData("pivot encoder reading", getPosition());
        t.update();
    }
}
