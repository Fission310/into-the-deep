package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.util.GoBildaPinpointDriver;

public class PinpointLocalizer implements Localizer {
    private GoBildaPinpointDriver odo;

    public PinpointLocalizer(GoBildaPinpointDriver odo) {
        this.odo = odo;
    }

    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        Pose2D pose = odo.getPosition();
        return new Pose2d(-pose.getY(DistanceUnit.INCH), pose.getX(DistanceUnit.INCH),
                pose.getHeading(AngleUnit.RADIANS));
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose2d) {
        odo.setPosition(new Pose2D(DistanceUnit.INCH, pose2d.getY(), pose2d.getX(), AngleUnit.RADIANS, pose2d.getHeading()));
        Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();
        odo.update();
        telemetry.addData("set odo pos x", pose2d.getX());
        telemetry.addData("set odo pos y", pose2d.getY());
        telemetry.addData("odo pos x", odo.getPosition().getX(DistanceUnit.INCH));
        telemetry.addData("odo pos y", odo.getPosition().getY(DistanceUnit.INCH));
        telemetry.update();
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        Pose2D pose = odo.getVelocity();
        return new Pose2d(-pose.getY(DistanceUnit.INCH), pose.getX(DistanceUnit.INCH),
                pose.getHeading(AngleUnit.RADIANS));
    }

    @Override
    public void update() {
        odo.update();
        Telemetry t = FtcDashboard.getInstance().getTelemetry();
        t.addData("status", odo.getDeviceStatus());
        t.update();
    }
}
