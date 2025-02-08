package org.firstinspires.ftc.teamcode.opmode.auton.util;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.PIDController;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

@Config
public class Drive {
    public static double HEADING_P = 0.5;
    public static double HEADING_D = 0;
    public static double TRANS_P = 0.12;
    public static double TRANS_D = 0;

    public static void p2p(SampleMecanumDrive drive, Pose2d target) {
        PIDController headingPid = new PIDController(HEADING_P, 0, HEADING_D);
        PIDController translationPid = new PIDController(TRANS_P, 0, TRANS_D);

        Pose2d currentPose = drive.getPoseEstimate();

        double targetHeading = (target.getHeading() + 360) % 360;
        double currentHeading = (currentPose.getHeading() + 360) % 360;
        if (Math.abs(targetHeading - currentHeading) > 180) {
            targetHeading += 360;
        }
        if (Math.abs(targetHeading - currentHeading) > 180) {
            targetHeading -= 360;
            currentHeading += 360;
        }
        headingPid.setTarget(targetHeading);
        double headingCorrection = headingPid.calculate(currentHeading);
        translationPid.setTarget(target.getX());
        double xCorrection = translationPid.calculate(currentPose.getX());
        translationPid.setTarget(target.getY());
        double yCorrection = translationPid.calculate(currentPose.getY());

        drive.setWeightedDrivePower(new Pose2d(xCorrection, yCorrection, headingCorrection));
    }
}
