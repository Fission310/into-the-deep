package org.firstinspires.ftc.teamcode.opmode.auton.util;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.PIDController;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

@Config
public class Drive {
    public static double HEADING_P = 0.3;
    public static double HEADING_D = 0.045;
    public static double TRANS_P = 0.000015;
    public static double TRANS_D = 0.00000045;
    public static double TRANS_P_SHORT = 0.2;
    public static double TRANS_D_SHORT = 0.0000004;
    public static double TRANS_P_MID = 0.2;
    public static double TRANS_D_MID = 0.0000004;

    private static PIDController headingPid = new PIDController(HEADING_P, 0, HEADING_D);
    private static PIDController translationPid = new PIDController(TRANS_P, 0, TRANS_D);

    public static void p2p(SampleMecanumDrive drive, Pose2d target, double voltage) {
        Pose2d currentPose = drive.getPoseEstimate();
        double diff = Math.abs(currentPose.getX() - target.getX()) + Math.abs(currentPose.getY() - target.getY());

        if (diff < 2) {
            translationPid.setkD(TRANS_P_SHORT);
            translationPid.setkP(TRANS_P_SHORT);
        } else if (diff < 6) {
            translationPid.setkD(TRANS_P_MID);
            translationPid.setkP(TRANS_P_MID);
        } else {
            translationPid.setkD(TRANS_D);
            translationPid.setkP(TRANS_P);
        }

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
        translationPid.setTarget(target.getX());
        double xCorrection = translationPid.calculate(currentPose.getX()) / voltage * 12.0;
        translationPid.setTarget(target.getY());
        double yCorrection = translationPid.calculate(currentPose.getY()) / voltage * 12.0;

        drive.setWeightedDrivePower(new Pose2d(xCorrection, yCorrection, headingCorrection));
    }
}
