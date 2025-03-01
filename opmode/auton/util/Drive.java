package org.firstinspires.ftc.teamcode.opmode.auton.util;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.PIDController;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

@Config
public class Drive {
    public static double HEADING_P = 0.3;
    public static double HEADING_D = 0.045;
    public static double Y_P = 0.1;
    public static double Y_I = 0;
    public static double Y_D = 0.0001;
    public static double Y_S = 0.15;
    public static double X_P = 0.2;
    public static double X_D = 0.0000009;

    private static PIDController headingPid = new PIDController(HEADING_P, 0, HEADING_D);
    private static PIDController xPid = new PIDController(X_P, 0, X_D);
    private static PIDController yPid = new PIDController(Y_P, Y_I, Y_D);

    public static void p2p(SampleMecanumDrive drive, Pose2d target, double voltage) {
        Pose2d currentPose = drive.getPoseEstimate();

        xPid.setkD(X_D);
        xPid.setkP(X_P);

        yPid.setkD(Y_D);
        yPid.setkI(Y_I);
        yPid.setkP(Y_P);

        headingPid.setkD(HEADING_D);
        headingPid.setkP(HEADING_P);

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
        xPid.setTarget(target.getX());
        double xCorrection = xPid.calculate(currentPose.getX()) / voltage * 12.0;
        yPid.setTarget(target.getY());
        double yCorrection = yPid.calculate(currentPose.getY()) / voltage * 12.0;

        drive.setWeightedDrivePower(
                new Pose2d(xCorrection, yCorrection + (yCorrection / Math.abs(yCorrection)) * Y_S, headingCorrection));
    }
}
