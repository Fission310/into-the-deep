package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import com.acmerobotics.dashboard.FtcDashboard;import com.acmerobotics.dashboard.config.Config;
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
import org.firstinspires.ftc.teamcode.util.PIDController;

@Config
public class Pivot extends Mechanism {
    public static int FRONT_POS = 180;
    public static int WALL_POS = 1950;
    public static int BASKET_POS = 2000; // FIGURE OUT POSITION
    public static int CLIP_POS = 2800;
    public static int BACK_POS = 3950;

    public static double KP = 0.003;
    public static double KI = 0;
    public static double KD = 0;

    public static double target = 0;
    public static double power = 0;
    public static double BASE_MULTIPLIER = 0.1;
    public static double GRAVITY_MULTIPLIER = 0.15;

    private final PIDController controller = new PIDController(KP, KI, KD);

    private final DcMotorEx[] motors = new DcMotorEx[2];

    private VoltageSensor voltage;

    public Pivot(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void init(HardwareMap hwMap) {
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
        telemetry.addData("Current Position", getPosition());
        telemetry.addData("Target", target);
        telemetry.addData("Power", power);
        telemetry.update();
    }

    public void frontPos() {
        setTarget(FRONT_POS);
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

    public void setTarget(double target) {
        Pivot.target = target;
    }

    public double getPosition() {
        return motors[0].getCurrentPosition();
    }

    public void update() {
        double multiplier = Math.abs(getPosition() - WALL_POS) / WALL_POS * GRAVITY_MULTIPLIER + BASE_MULTIPLIER; //FLINT WHAT TO DO HERE WALL USED TO BE UPFRONT
        controller.setTarget(target);
        power = controller.calculate(getPosition()) * multiplier;
        motors[0].setPower(power);
        motors[1].setPower(power);
    }

    @Override
    public void loop(Gamepad gamepad) {
        controller.setkP(KP);
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
