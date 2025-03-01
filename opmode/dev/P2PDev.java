package org.firstinspires.ftc.teamcode.opmode.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Telescope;
import org.firstinspires.ftc.teamcode.opmode.auton.util.Drive;

@Config
@Autonomous(name = "P2P Test", preselectTeleOp = "Main")
public class P2PDev extends LinearOpMode {
    private SampleMecanumDrive drive;
    private Telescope telescope;

    public static double X = 0;
    public static double Y = 0;
    public static double HEADING = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        VoltageSensor voltage = hardwareMap.voltageSensor.iterator().next();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SampleMecanumDrive(hardwareMap);
        telescope = new Telescope(this);
        telescope.init(hardwareMap);

        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(90)));

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            Drive.p2p(drive, new Pose2d(X, Y, Math.toRadians(HEADING)), voltage.getVoltage());
            drive.update();
            telescope.update();
            telemetry.addData("drive x", drive.getPoseEstimate().getX());
            telemetry.addData("drive y", drive.getPoseEstimate().getY());
            telemetry.update();
        }

        Thread.sleep(500);
    }
}
