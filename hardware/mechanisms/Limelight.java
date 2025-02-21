package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import static org.firstinspires.ftc.teamcode.opmode.auton.util.LimelightConstants.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.stuyfission.fissionlib.util.Mechanism;

public class Limelight extends Mechanism {
    private Limelight3A limelight;
    private ArrayList<Location> locations;

    public class Location {
        public double translation;
        public double extension;
        public double rotation;

        public Location(double trans, double extension, double rotation) {
            this.translation = trans;
            this.extension = extension;
            this.rotation = rotation;
        }
    }

    public Limelight(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void init(HardwareMap hwMap) {
        limelight = hwMap.get(Limelight3A.class, "limelight");

        limelight.setPollRateHz(100);

        limelight.pipelineSwitch(PIPELINE);
        limelight.start();
    }

    public void update() {
        // thanks 20077 :)
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();

        // List to hold scored detections
        locations = new ArrayList<>();

        for (LLResultTypes.DetectorResult detection : detections) {
            int color = detection.getClassId(); // Detected class (color)

            // Calculate bounding box dimensions
            List<List<Double>> corners = detection.getTargetCorners();
            if (corners == null || corners.size() < 4) {
                continue; // Skip invalid detections
            }

            double width = calculateDistance(corners.get(0), corners.get(1)); // Top edge
            double height = calculateDistance(corners.get(1), corners.get(2)); // Side edge

            // Calculate aspect ratio and rotation score
            double aspectRatio = width / height;
            double rotationScore = -Math.abs(aspectRatio - IDEAL_ASPECT_RATIO); // Closer to ideal is better

            // Calculate distance (approximation based on angles)
            double actualYAngle = LIME_LIGHT_MOUNT_ANGLE - detection.getTargetYDegrees();
            double yDistance = LIME_LIGHT_LENS_HEIGHT_INCHES * Math.tan(Math.toRadians(actualYAngle));
            double xDistance = Math.tan(Math.toRadians(detection.getTargetXDegrees())) * yDistance;

            // If color matches unwanted sample, skip the rest of the current iteration
            //if (Arrays.stream(unwantedSamples).anyMatch(sample -> sample == color)) {
            //    continue;
            //}

            // Add the scored detection to the list
            locations.add(new Location(yDistance, xDistance, rotationScore));
        }
    }

    public Location getBest() {
        // TODO
        return locations.get(0);
    }

    private double calculateDistance(List<Double> point1, List<Double> point2) {
        double dx = point1.get(0) - point2.get(0);
        double dy = point1.get(1) - point2.get(1);
        return Math.sqrt(dx * dx + dy * dy);
    }

    @Override
    public void telemetry(Telemetry telemetry) {
    }

    public void stop() {
        limelight.stop();
    }

    public void setPipeline(int pipeline) {
        limelight.pipelineSwitch(pipeline);
    }
}
