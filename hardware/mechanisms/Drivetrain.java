package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
public class Drivetrain extends Mechanism {

    public static double SLOW_SPEED = 0.4;
    public static double SLOW_TURN = 0.6;
    public static double NORMAL_SPEED = 1;
    private double speed = NORMAL_SPEED;
    private double turnSpeed = NORMAL_SPEED;

    private SampleMecanumDrive drivetrain;

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
        drivetrain = new SampleMecanumDrive(hwMap);
        drivetrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop(Gamepad gamepad) {
        drivetrain.setWeightedDrivePower(
                new Pose2d(
                        -gamepad.left_stick_y * speed,
                        -gamepad.left_stick_x * speed,
                        -gamepad.right_stick_x * turnSpeed));

        drivetrain.update();

    }

}
