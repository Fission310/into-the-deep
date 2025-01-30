package org.firstinspires.ftc.teamcode.opmode.auton.util;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.PIDController;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class Drive {
    public static void p2p(SampleMecanumDrive drive, Pose2d target) {
        PIDController headingPid = new PIDController(0.5, 0, 0);
        PIDController translationPid = new PIDController(0.15, 0, 0);

        Pose2d currentPose = drive.getPoseEstimate();

        headingPid.setTarget(target.getHeading());
        double headingCorrection = headingPid.calculate(currentPose.getHeading());
        translationPid.setTarget(target.getX());
        double xCorrection = translationPid.calculate(currentPose.getX());
        translationPid.setTarget(target.getY());
        double yCorrection = translationPid.calculate(currentPose.getY());

        drive.setWeightedDrivePower(new Pose2d(xCorrection, yCorrection, headingCorrection));
    }
}
