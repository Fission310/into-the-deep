package org.firstinspires.ftc.teamcode.hardware.mechanisms;

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
    public static int ABIT = 60;
    public static int UP_RETRACTION = -60;
    public static int AUTO_INTAKE_FAR_POS = 292;
    public static int AUTO_INTAKE_CENTER_POS = 349;
    public static int AUTO_INTAKE_WALL_POS = 430;
    public static int AUTO_BASKET_POS = 630;
    public static int AUTO_SAMPLE_DROP = 300;
    public static int AUTO_INTAKE_SHORT_POS = 50;
    public static int FRONT_VERTICAL_POS = -50;
    public static int FRONT_HORIZONTAL_POS = -50;
    public static int INTAKE_POS = 450;
    public static int INTAKE_SHORT_POS = 50;
    public static int WALL_POS = 100;
    public static int BASKET_POS = 630;
    public static int LOW_BASKET_POS = 220;
    public static int CLIP_POS = 330;
    public static int CLIP_SCORE = 100;
    public static int CLIP_EXTENSION = 330;
    public static int BACK_POS = 300;
    public static int CLIMB_UP_1_POS = 500;
    public static int CLIMB_UP_2_POS = 200;
    public static int CLIMB_DOWN_POS = -500000;
    public static int MAX_EXTENSION = 750;
    public static int MIN_LENGTH = 15;
    public static double DOWN_MULTIPLIER = 0.15;
    public static double INCH_PER_TICK = 0.0369;
    public static double VERTICAL_KP = 0.007;
    public static double HORIZONTAL_KP = 0.002;

    public static boolean climbPressed = false;
    public static double target = 0;
    public static double power = 0;
    public static double POWER_MULTIPLIER = 1;

    private final PIDController verticalController = new PIDController(VERTICAL_KP, 0, 0);
    private final PIDController horizontalController = new PIDController(HORIZONTAL_KP, 0, 0);
    private PIDController controller;

    private final DcMotorEx[] motors = new DcMotorEx[2];

    private VoltageSensor voltage;

    public Telescope(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void init(HardwareMap hwMap) {
        voltage = hwMap.voltageSensor.iterator().next();
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

        frontHorizontalPos();
    }

    public void autoSampleDropPos() {
        controller = horizontalController;
        setTarget(AUTO_SAMPLE_DROP);
    }

    public void autoFarPos() {
        controller = horizontalController;
        setTarget(AUTO_INTAKE_FAR_POS);
    }

    public void autoCenterPos() {
        controller = horizontalController;
        setTarget(AUTO_INTAKE_CENTER_POS);
    }

    public void autoWallPos() {
        controller = horizontalController;
        setTarget(AUTO_INTAKE_WALL_POS);
    }

    public void autoBasketPos(){
        controller = verticalController;
        setTarget(AUTO_BASKET_POS);
    }

    public void upPos() {
        controller = verticalController;
        setTarget(UP_RETRACTION);
    }

    public void frontVerticalPos() {
        controller = horizontalController; // :)
        setTarget(FRONT_VERTICAL_POS);
    }
    public void frontHorizontalPos() {
        controller = horizontalController;
        setTarget(FRONT_HORIZONTAL_POS);
    }

    public void frontIntakePos() {
        controller = horizontalController;
        setTarget(INTAKE_POS);
    }

    public void frontIntakeShortPos() {
        controller = horizontalController;
        setTarget(INTAKE_SHORT_POS);
    }

    public void frontIntakeAutoShortPos() {
        controller = horizontalController;
        setTarget(AUTO_INTAKE_SHORT_POS);
    }

    public void wallPos() {
        controller = horizontalController;
        setTarget(WALL_POS);
    }

    public void basketPos() {
        controller = verticalController;
        setTarget(BASKET_POS);
    }

    public void lowBasketPos() {
        controller = verticalController;
        setTarget(LOW_BASKET_POS);
    }

    public void clipPos() {
        controller = verticalController;
        setTarget(CLIP_POS);
    }

    public void clipExtensionPos() {
        controller = verticalController;
        setTarget(CLIP_EXTENSION);
    }

    public void clipScorePos() {
        controller = verticalController;
        setTarget(CLIP_SCORE);
    }

    public void backPos() {
        controller = horizontalController;
        setTarget(BACK_POS);
    }

    public void climbUpPos1() {
        controller = verticalController;
        setTarget(CLIMB_UP_1_POS);
    }

    public void climbUpPos2() {
        controller = verticalController;
        setTarget(CLIMB_UP_2_POS);
    }

    public void climbDownPos() {
        controller = verticalController;
        setTarget(CLIMB_DOWN_POS);
    }

    public double getLength() {
        return MIN_LENGTH + getPosition() * INCH_PER_TICK;
    }

    public void setTarget(double target) {
        if (target > MAX_EXTENSION && controller == horizontalController) {
            target = MAX_EXTENSION;
        }
        Telescope.target = target;
    }

    public void setTargetInches(double target){
        controller = horizontalController;
        setTarget(target / INCH_PER_TICK);
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
        power = controller.calculate(getPosition()) * POWER_MULTIPLIER / voltage.getVoltage() * 12.0;
        if (target < getPosition() && controller == verticalController) {
            power *= DOWN_MULTIPLIER;
        }
        motors[0].setPower(power);
        motors[1].setPower(power);
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("telescope position", getPosition());
        telemetry.addData("telescope target", target);
        telemetry.addData("telescope power", power);
        telemetry.addData("telescope length", getLength());
    }

    @Override
    public void loop(Gamepad gamepad) {
        update();
        if (GamepadStatic.isButtonPressed(gamepad, Controls.PIVOT_FRONT)) {
            frontHorizontalPos();
        } else if (GamepadStatic.isButtonPressed(gamepad, Controls.PIVOT_WALL)) {
            wallPos();
        } else if (GamepadStatic.isButtonPressed(gamepad, Controls.PIVOT_BASKET)) {
            basketPos();
        } else if (GamepadStatic.isButtonPressed(gamepad, Controls.PIVOT_CLIP)) {
            clipPos();
        } else if (GamepadStatic.isButtonPressed(gamepad, Controls.CLIMB_1)) {
            if(!climbPressed){
                climbUpPos1();
            }
            else{
                climbDownPos();
            }
            climbPressed = !climbPressed;
        }
    }
}
