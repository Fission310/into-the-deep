package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
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
    public Pose2d getPose() {
        Pose2D pose = odo.getPosition();
        return new Pose2d(-pose.getY(DistanceUnit.INCH), pose.getX(DistanceUnit.INCH),
                pose.getHeading(AngleUnit.RADIANS));
    }

    @Override
    public void setPose(@NonNull Pose2d pose2d) {
        odo.setPosition(new Pose2D(DistanceUnit.INCH, pose2d.position.y, pose2d.position.x, AngleUnit.RADIANS, pose2d.heading.toDouble()));
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
        t.addData("status", odo.getDeviceStatus());
        t.update();
        Pose2D pose = odo.getVelocity();
        return new PoseVelocity2d(new Vector2d(-pose.getY(DistanceUnit.INCH), pose.getX(DistanceUnit.INCH)),
                pose.getHeading(AngleUnit.RADIANS));

    }
}
