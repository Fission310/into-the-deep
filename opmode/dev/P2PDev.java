package org.firstinspires.ftc.teamcode.opmode.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.opmode.auton.util.Drive;

@Autonomous(name = "P2P Test", preselectTeleOp = "Main")
public class P2PDev extends LinearOpMode {
    private SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
            Drive.p2p(drive, new Pose2d(0, 0, 0));
            telemetry.addData("drive x", drive.getPoseEstimate().getX());
            telemetry.addData("drive y", drive.getPoseEstimate().getY());
            telemetry.update();
        }

        Thread.sleep(500);
    }
}
