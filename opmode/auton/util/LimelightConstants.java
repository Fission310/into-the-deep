package org.firstinspires.ftc.teamcode.opmode.auton.util;

// reference diagram: https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-estimating-distance
public class LimelightConstants {
    public static int PIPELINE = 7;
    public static double LIME_LIGHT_MOUNT_ANGLE = Math.toRadians(55); // a1
    public static double LIME_LIGHT_LENS_HEIGHT_INCHES = 7.00525945; // h1
    public static double SAMPLE_HEIGHT_INCHES = 1.5; // h2

    public static double calcYDistance(double ty){ // a2
        double angleToSampleRadians = LIME_LIGHT_MOUNT_ANGLE - Math.toRadians(ty);

        return Math.abs((SAMPLE_HEIGHT_INCHES - LIME_LIGHT_LENS_HEIGHT_INCHES) / Math.tan(angleToSampleRadians));
    }

    public static double calcXDistance(double tx, double ty){ // a2
        double angleToSampleRadians = Math.toRadians(tx);

        return calcYDistance(ty) * Math.tan(angleToSampleRadians);
    }
}
