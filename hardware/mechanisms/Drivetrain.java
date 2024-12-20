package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

@Config
public class Drivetrain extends Mechanism {

    private MecanumDrive drivetrain;

    public Drivetrain(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void init(HardwareMap hwMap) {
        drivetrain = new MecanumDrive(hwMap, new Pose2d(0.0, 0.0, 0.0));
    }

    @Override
    public void loop(Gamepad gamepad) {
        drivetrain.setDrivePowers(
                new PoseVelocity2d(
                        new Vector2d(
                        -gamepad.left_stick_y,
                        -gamepad.left_stick_x),
                        -gamepad.right_stick_x
                )
        );

        drivetrain.updatePoseEstimate();

    }

}