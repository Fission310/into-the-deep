package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.stuyfission.fissionlib.util.Mechanism;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Config
public class Drivetrain extends Mechanism {

    public static double SLOW_SPEED = 0.6;
    public static double SLOW_TURN = 0.4;
    public static double NORMAL_SPEED = 1;
    private double speed = NORMAL_SPEED;
    private double turnSpeed = NORMAL_SPEED;

    private Follower drivetrain;

    public Drivetrain(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void setIntake() {
        turnSpeed = SLOW_TURN;
        speed = NORMAL_SPEED;
    }

    public void setScore() {
        turnSpeed = SLOW_TURN;
        speed = SLOW_SPEED;
    }

    public void setNormal() {
        turnSpeed = NORMAL_SPEED;
        speed = NORMAL_SPEED;
    }

    @Override
    public void init(HardwareMap hwMap) {
        Constants.setConstants(FConstants.class, LConstants.class);
        drivetrain = new Follower(hwMap);
    }

    @Override
    public void loop(Gamepad gamepad) {
        drivetrain.setTeleOpMovementVectors(
                        -gamepad.left_stick_y * speed,
                        -gamepad.left_stick_x * speed,
                        -gamepad.right_stick_x * turnSpeed, true);

        drivetrain.update();

    }
}
