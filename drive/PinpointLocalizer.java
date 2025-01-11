package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class PinpointLocalizer implements Localizer {
    public GoBildaPinpointDriverRR odo;

    public PinpointLocalizer(GoBildaPinpointDriverRR odo) {
        this.odo = odo;
    }

    @NonNull
    @Override
    public Pose2d getPose() {
        Pose2D pose = odo.getPosition();

        Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();
        telemetry.addData("odo pos x", odo.getPosition().getX(DistanceUnit.INCH));
        telemetry.addData("odo pos y", odo.getPosition().getY(DistanceUnit.INCH));
        telemetry.update();
        return new Pose2d(pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH),
                pose.getHeading(AngleUnit.RADIANS));
    }

    @Override
    public void setPose(@NonNull Pose2d pose2d) {
        odo.setPosition(new Pose2D(DistanceUnit.INCH, pose2d.position.x, pose2d.position.y, AngleUnit.RADIANS,
                pose2d.heading.toDouble()));
        Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();
        odo.update();
        telemetry.addData("set odo pos x", pose2d.position.x);
        telemetry.addData("set odo pos y", pose2d.position.y);
        telemetry.addData("odo pos x", odo.getPosition().getX(DistanceUnit.INCH));
        telemetry.addData("odo pos y", odo.getPosition().getY(DistanceUnit.INCH));
        telemetry.update();
    }

    @Override
    public PoseVelocity2d update() {
        odo.update();
        Telemetry t = FtcDashboard.getInstance().getTelemetry();
        t.addData("odo v x", odo.getVelocity().getX(DistanceUnit.INCH));
        t.addData("odo v y", odo.getVelocity().getY(DistanceUnit.INCH));
        t.update();
        Pose2D pose = odo.getVelocity();
        return new PoseVelocity2d(new Vector2d(pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH)),
                pose.getHeading(AngleUnit.RADIANS));

    }
}
