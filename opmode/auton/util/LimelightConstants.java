package org.firstinspires.ftc.teamcode.opmode.auton.util;

import com.acmerobotics.dashboard.config.Config;

// reference diagram: https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-estimating-distance
@Config
public class LimelightConstants {
    public static double IDEAL_ASPECT_RATIO = 1.5 / 3.5; // Expected width:height ratio for a properly aligned sample
    public static int PIPELINE = 0;
    public static double LIME_LIGHT_MOUNT_ANGLE = 35; // a1
    public static double LIME_LIGHT_LENS_HEIGHT_INCHES = 7.00525945; // h1
    public static double LIME_LIGHT_OFFSET = 5.5; // h1
    public static double SAMPLE_HEIGHT_INCHES = 0; // h2
}
