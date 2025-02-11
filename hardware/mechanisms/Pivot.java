package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.stuyfission.fissionlib.command.CommandSequence;
import com.stuyfission.fissionlib.command.Command;
import com.stuyfission.fissionlib.input.GamepadStatic;
import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmode.auton.BasketConstants;
import org.firstinspires.ftc.teamcode.opmode.auton.BasketConstants;
import org.firstinspires.ftc.teamcode.opmode.teleop.Controls;
import org.firstinspires.ftc.teamcode.util.PIDFController;
import org.firstinspires.ftc.teamcode.util.PIDFController.FeedForward;

@Config
public class Pivot extends Mechanism {
    public static int ABIT = 1;
    public static int RESET_POS = 136;
    public static int RESET_WAIT = 1;
    public static int INIT_POS = 180;
    public static int FRONT_POS = 140;
    public static int INTAKE_UP_POS = 140;
    public static int INTAKE_DOWN_POS = 140;
    public static int INTAKE_GRAB_POS = 124;
    public static int WALL_POS = 162;
    public static int BASKET_POS = 250;
    public static int CLIP_POS = 200;
    public static int CLIP_DOWN_POS = 193;
    public static int CLIP_BACK_POS = 232;
    public static int CLIP_BACK_DOWN_POS = 235;
    public static int BACK_POS = 294;
    public static int UP_POS = 215;
    public static int HIGHEST = 220;
    public static int CLIMB_UP_POS = 240;
    public static int CLIMB_DOWN_POS = 80;
    public static int TICKS_PER_REV = 360;

    public static double KP = 0.012;
    public static double KI = 0.05;
    public static double KD = 0.0003;
    public static double KF = 0.0075;

    public static boolean climbPressed = false;
    public static double target = 0;
    public static double actualTarget = 0;
    public static double power = 0;

    private PIDFController controller;

    private Telescope telescope;

    private final DcMotorEx[] motors = new DcMotorEx[2];
    private AnalogInput encoder;

    private VoltageSensor voltage;

    public Pivot(LinearOpMode opMode, Telescope telescope) {
        this.opMode = opMode;
        this.telescope = telescope;
    }

    @Override
    public void init(HardwareMap hwMap) {
        voltage = hwMap.voltageSensor.iterator().next();
        controller = new PIDFController(KP, KI, KD, KF);
        controller.setFeedForward(FeedForward.ROTATIONAL);
        controller.setRotationConstants(HIGHEST, TICKS_PER_REV);

        encoder = hwMap.get(AnalogInput.class, "pivotEncoder");
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

    public void moveIntakeUp() {
        INTAKE_DOWN_POS += ABIT;
        INTAKE_UP_POS += ABIT;
    }

    public void moveIntakeDown() {
        INTAKE_DOWN_POS -= ABIT;
        INTAKE_UP_POS -= ABIT;
    }

    public void initPos() {
        setTarget(INIT_POS);
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

    public void reset() {
        setTarget(RESET_POS);
    }

    public void intakeGrabPos() {
        setTarget(INTAKE_GRAB_POS);
    }

    public void autoIntakeGrabPos() {
        setTarget(BasketConstants.PIVOT_INTAKE_GRAB_POS);
    }

    public void wallPos() {
        setTarget(WALL_POS);
    }

    public void basketPos() {
        setTarget(BASKET_POS);
    }

    public void autoBasketPos() {
        setTarget(BasketConstants.PIVOT_BASKET_POS);
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

    public void climbUpPos() {
        setTarget(CLIMB_UP_POS);
    }

    public void climbDownPos() {
        setTarget(CLIMB_DOWN_POS);
    }

    public void setTarget(double target) {
        Pivot.target = target;
        Pivot.actualTarget = target;
    }

    public double getPosition() {
        return encoder.getVoltage() / 3.2 * 360;
    }

    public void update() {
        controller.setPIDF(KP, KI, KD, KF);
        controller.setRotationConstants(HIGHEST, TICKS_PER_REV);
        controller.setLength(telescope.getLength());
        power = controller.calculate((getPosition() + TICKS_PER_REV) % TICKS_PER_REV, target) / voltage.getVoltage()
                * 12.0;
        motors[0].setPower(power);
        motors[1].setPower(power);
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("pivot position", getPosition());
        telemetry.addData("pivot target position", target);
        telemetry.addData("pivot power", power);
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
        } else if (GamepadStatic.isButtonPressed(gamepad, Controls.CLIMB_1)) {
            if (!climbPressed) {
                climbUpPos();
            } else {
                climbDownPos();
            }
            climbPressed = !climbPressed;
        }
    }
}
