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
    public static int LOW_POS = 160;
    public static int MED_POS = 350;
    public static int HIGH_POS = 845;
    public static int ABIT = 180;

    public static double KP = 0.003;
    public static double KI = 0;
    public static double KD = 0;

    public static double target = 0;
    public static double power = 0;

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
        motors[1].setDirection(DcMotorEx.Direction.FORWARD);

        lowPos();
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Current Position", getPosition());
        telemetry.addData("Target", target);
        telemetry.addData("Power", power);
        telemetry.update();
    }

    public void lowPos() {
        setTarget(LOW_POS);
    }

    public void medPos() {
        setTarget(MED_POS);
    }

    public void highPos() {
        setTarget(HIGH_POS);
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
        power = controller.calculate(getPosition());
        motors[0].setPower(power);
        motors[1].setPower(power);
    }

    @Override
    public void loop(Gamepad gamepad) {
        update();
        if (GamepadStatic.isButtonPressed(gamepad, Controls.LOW)) {
            lowPos();
        } else if (GamepadStatic.isButtonPressed(gamepad, Controls.MEDIUM)) {
            medPos();
        }
        else if (GamepadStatic.isButtonPressed(gamepad, Controls.HIGH)) {
            highPos();
        }
    }
}