package org.firstinspires.ftc.teamcode.opmode.auton.util;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class Drive {
    // PIDF constants (tune these for your robot)
    public static double Kp_distance = 0.5; // Proportional gain for distance
    public static double Ki_distance = 0.0; // Integral gain for distance
    public static double Kd_distance = 0.1; // Derivative gain for distance
    public static double Kf_distance = 0.2; // Feedforward for distance
    public static double Kp_heading = 0.5; // Proportional gain for heading
    public static double Ki_heading = 0.0; // Integral gain for heading
    public static double Kd_heading = 0.1; // Derivative gain for heading
    // Error terms
    static double distanceError = 0;
    static double previousDistanceError = 0;
    static double headingError = 0;
    static double previousHeadingError = 0;
    // Integral terms (if used)
    static double distanceIntegral = 0;
    static double headingIntegral = 0;
    // Robot-specific parameter
    static double targetX = 0, targetY = 0; // Target position (coordinates)
    static double targetHeading = 0; // Target heading in radians
    static double currentX = 0, currentY = 0, currentHeading = 0; // Current position and heading
    // Tuning parameters
    static double distanceTolerance = 0.5; // Allowable distance error (e.g., inches or cm)
    static double headingTolerance = Math.toRadians(4); // Allowable heading error (radians)

    // Main P2P method with target heading
    public static void p2p(SampleMecanumDrive drive, Pose2d targetPose) {
        while (true) {
            // Update robot's current position and heading (e.g., from odometry)
            currentX = drive.getPoseEstimate().getX(); // Replace with your odometry method

            currentY = drive.getPoseEstimate().getY(); // Replace with your odometry method
            currentHeading = drive.getPoseEstimate().getHeading(); // Replace with your heading method
            // Calculate distance and heading errors
            distanceError = Math.sqrt(Math.pow(targetX - currentX, 2) + Math.pow(targetY - currentY, 2));
            headingError = targetHeading - currentHeading;
            // Normalize heading error to range [-π, π]
            headingError = Math.atan2(Math.sin(headingError), Math.cos(headingError));
            // PIDF calculations for distance
            double distanceDerivative = (distanceError - previousDistanceError);
            double distanceOutput = Kp_distance * distanceError + Ki_distance * distanceIntegral +
                    Kd_distance * distanceDerivative + Kf_distance;
            // PIDF calculations for heading
            double headingDerivative = (headingError - previousHeadingError);
            double headingOutput = Kp_heading * headingError + Ki_heading * headingIntegral +
                    Kd_heading * headingDerivative;
            // Update integral terms
            distanceIntegral += distanceError;
            headingIntegral += headingError;
            // Break condition
            if (distanceError < distanceTolerance && Math.abs(headingError) < headingTolerance) {
                break;
            }
            // Calculate forward and strafe components based on the target direction
            double targetAngle = Math.atan2(targetY - currentY, targetX - currentX); // Target angle
            double forward = distanceOutput * Math.cos(targetAngle - currentHeading);
            double strafe = distanceOutput * Math.sin(targetAngle - currentHeading);
            // Update motor powers
            drive.setWeightedDrivePower(new Pose2d(forward, strafe, headingOutput));
            // Update previous errors
            previousDistanceError = distanceError;
            previousHeadingError = headingError;
        }
        // Stop the robot at the end
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
    }
}
