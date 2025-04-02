package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import static org.firstinspires.ftc.teamcode.opmode.auton.util.LimelightConstants.*;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmode.auton.util.Color;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.stuyfission.fissionlib.util.Mechanism;

public class Limelight extends Mechanism {
    private Limelight3A limelight;
    private ArrayList<Location> locations = new ArrayList<>();
    private Color targetColor;

    public class Location {
        public double translation;
        public double extension;
        public double rotation;
        public double score;
        public Color color;

        public double rotScore;
        public double distScore;
        public double yScore;

        public Location(double trans, double extension, double rotation, Color color) {
            this.translation = trans;
            this.extension = extension;
            this.rotation = rotation;
            this.color = color;
        }
    }

    public Limelight(LinearOpMode opMode, Color targetColor) {
        this.opMode = opMode;
        this.targetColor = targetColor;
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
        if (result == null)
            return;
        List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();

        // List to hold scored detections
        locations = new ArrayList<>();

        for (LLResultTypes.DetectorResult detection : detections) {
            Color color; // Detected class (color)
            switch (detection.getClassId()) {
                case 0:
                    color = Color.BLUE;
                    break;
                case 1:
                    color = Color.RED;
                    break;
                case 2:
                    color = Color.YELLOW;
                    break;
                default:
                    color = Color.YELLOW;
                    break;
            }

            // Calculate bounding box dimensions
            List<List<Double>> corners = detection.getTargetCorners();
            if (corners == null || corners.size() < 4) {
                continue; // Skip invalid detections
            }

            double width = calculateDistance(corners.get(0), corners.get(1)); // Top edge
            double height = calculateDistance(corners.get(1), corners.get(2)); // Side edge

            // Calculate aspect ratio and rotation score
            double aspectRatio = width / height;
            double rotationScore = Math.abs(aspectRatio - IDEAL_ASPECT_RATIO); // Closer to ideal is better

            // Calculate distance (approximation based on angles)
            double actualYAngle = LIME_LIGHT_MOUNT_ANGLE - detection.getTargetYDegrees();
            double yDistance = (LIME_LIGHT_LENS_HEIGHT_INCHES - SAMPLE_HEIGHT_INCHES)
                    / Math.tan(Math.toRadians(actualYAngle)) + TELESCOPE_OFFSET;
            double xDistance = Math.tan(Math.toRadians(detection.getTargetXDegrees())) * yDistance - LIME_LIGHT_OFFSET;

            // Add the scored detection to the list
            locations.add(new Location(xDistance, yDistance, rotationScore, color));
        }
    }

    private boolean isColor(Color color) {
        return targetColor == color || color == Color.YELLOW;
    }

    public Location getBest() {
        if (locations.size() == 0) {
            return new Location(0, 0, 0, Color.YELLOW);
        }

        locations.sort((a, b) -> Double.compare(b.translation, a.translation));
        for (int i = 0; i < locations.size(); i++) {
            Location current = locations.get(i);
            if (!isColor(current.color)) {
                current.score = Integer.MIN_VALUE;
                continue;
            }
            Location left = new Location(current.translation, 0, 0, Color.YELLOW);
            Location right = new Location(current.translation, 0, 0, Color.YELLOW);
            if (i > 0) {
                left = locations.get(i - 1);
            }
            if (i < locations.size() - 1) {
                right = locations.get(i + 1);
            }

            double leftDist = Math.abs(current.translation - left.translation);
            double rightDist = Math.abs(current.translation - right.translation);
            current.distScore = 0;
            if (!isColor(left.color)) {
                if (leftDist == 0) {
                    leftDist = 0.00001;
                }
                current.distScore -= X_WEIGHT / leftDist;
            }
            if (!isColor(right.color)) {
                if (rightDist == 0) {
                    rightDist = 0.00001;
                }
                current.distScore -= X_WEIGHT / rightDist;
            }
            current.distScore += X_WEIGHT * (leftDist + rightDist);
            current.rotScore = -ROT_WEIGHT * current.rotation * current.rotation * current.rotation;
            current.yScore = -Y_WEIGHT * Math.abs(current.extension - IDEAL_Y);
            current.score = current.distScore + current.rotScore + current.yScore;
        }

        locations.sort((a, b) -> Double.compare(b.score, a.score));

        return locations.get(0);
    }

    private double calculateDistance(List<Double> point1, List<Double> point2) {
        double dx = point1.get(0) - point2.get(0);
        double dy = point1.get(1) - point2.get(1);
        return Math.sqrt(dx * dx + dy * dy);
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        Location best = getBest();
        telemetry.addData("rotScore", Math.round(best.rotScore * 100.0) / 100.0);
        telemetry.addData("yScore", Math.round(best.yScore * 100.0) / 100.0);
        telemetry.addData("distScore", Math.round(best.distScore * 100.0) / 100.0);
        telemetry.addData("totalScore", Math.round(best.score * 100.0) / 100.0);
        telemetry.addData("numSamples", locations.size());
        telemetry.update();
    }

    public void stop() {
        limelight.stop();
    }

    public void setPipeline(int pipeline) {
        limelight.pipelineSwitch(pipeline);
    }
}
