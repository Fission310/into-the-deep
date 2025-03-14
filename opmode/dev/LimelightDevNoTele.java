package org.firstinspires.ftc.teamcode.opmode.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.stuyfission.fissionlib.command.Command;
import com.stuyfission.fissionlib.command.CommandSequence;
import com.stuyfission.fissionlib.input.GamepadStatic;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Limelight;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Pivot;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Sweeper;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Telescope;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Wrist;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Limelight.Location;
import org.firstinspires.ftc.teamcode.opmode.auton.util.Color;
import org.firstinspires.ftc.teamcode.opmode.auton.util.Drive;

@Autonomous(name = "LimelightDev No Tele")
public class LimelightDevNoTele extends LinearOpMode {
    private ElapsedTime elapsedTime;
    private ElapsedTime elapsedTime2;
    private Pose2d targetPoint = null;
    private Pose2d botPos = null;
    private Location loc = null;

    private SampleMecanumDrive drive;

    private Intake intake;
    private Pivot pivot;
    private Telescope telescope;
    private Wrist wrist;
    private Limelight limelight;
    private Sweeper sweeper;

    private Command intakeCommand = () -> intake.intake();
    private Command pivotUpIntake = () -> pivot.intakeUpPos();
    private Command pivotGrabIntake = () -> pivot.autoIntakeGrabPos();
    private Command sweepExtend = () -> sweeper.extendPos();
    private Command sweepRetract = () -> sweeper.retractPos();
    private Command telescopeRetract = () -> telescope.frontHorizontalPos();
    private Command wristRetract = () -> wrist.frontPos();
    private Command wristIntakeScore = () -> wrist.autoIntakePos();
    private Command setResult = () -> {
        loc = limelight.getBest();
        botPos = drive.getPoseEstimate();
    };
    private Command lineUpP2P = () -> targetPoint = new Pose2d(botPos.getX(),
            botPos.getY() - loc.translation,
            botPos.getHeading());
    private Command forwardP2P = () -> targetPoint = new Pose2d(targetPoint.getX(),
            targetPoint.getY(), targetPoint.getHeading());
    private Command driveStop = () -> {
        targetPoint = null;
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
    };
    private Command telescopeExtendInches = () -> telescope.setTargetInches(loc.extension);
    private Command telescopeExtendABit = () -> telescope.frontIntakeAutoShortPos();

    private CommandSequence run = new CommandSequence()
            .addCommand(setResult)
            .addWaitCommand(2)
            .addCommand(lineUpP2P)
            .addWaitCommand(3)
            .addCommand(driveStop)
            .addCommand(sweepExtend)
            .addWaitCommand(0.2)
            // .addCommand(forwardP2P)
            // .addWaitCommand(0.4)
            // .addCommand(driveStop)
            .addCommand(telescopeExtendABit)
            .addCommand(intakeCommand)
            .addCommand(pivotGrabIntake)
            .addCommand(wristIntakeScore)
            .addWaitCommand(0.6)
            .addCommand(telescopeExtendInches)
            .addWaitCommand(2)
            .addCommand(sweepRetract)
            .addWaitCommand(0.9)
            .addCommand(pivotUpIntake)
            .addCommand(wristRetract)
            .addWaitCommand(0.2)
            .addCommand(telescopeRetract)
            .build();

    @Override
    public void runOpMode() throws InterruptedException {
        VoltageSensor voltage = hardwareMap.voltageSensor.iterator().next();
        elapsedTime = new ElapsedTime();
        elapsedTime2 = new ElapsedTime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        intake = new Intake(this);
        drive = new SampleMecanumDrive(hardwareMap);
        telescope = new Telescope(this);
        pivot = new Pivot(this, telescope);
        wrist = new Wrist(this);
        limelight = new Limelight(this, Color.BLUE);
        sweeper = new Sweeper(this);

        intake.init(hardwareMap);
        pivot.init(hardwareMap);
        wrist.init(hardwareMap);
        telescope.init(hardwareMap);
        limelight.init(hardwareMap);
        sweeper.init(hardwareMap);

        while (opModeInInit() && !isStopRequested()) {
            drive.updatePoseEstimate();
            limelight.update();
            Location l = limelight.getBest();
            telemetry.addData("limelight strafe distance", l.translation);
            telemetry.addData("telescope extend dist", l.extension);
            limelight.telemetry(telemetry);
        }

        drive.setPoseEstimate(new Pose2d(0, 0, 0));

        waitForStart();

        run.trigger();

        while (opModeIsActive() && !isStopRequested()) {
            elapsedTime2.reset();
            elapsedTime.reset();
            if (targetPoint != null) {
                Drive.p2p(drive, targetPoint, voltage.getVoltage());
            } else {
                drive.setMotorPowers(0, 0, 0, 0);
            }
            telemetry.addData("p2p time", (int) elapsedTime.milliseconds());
            telemetry.update();
            elapsedTime.reset();

            drive.update();
            telemetry.addData("drive update time", (int) elapsedTime.milliseconds());
            telemetry.update();
            elapsedTime.reset();

            telescope.update();
            telemetry.addData("telescope update time", (int) elapsedTime.milliseconds());
            telemetry.update();
            elapsedTime.reset();

            pivot.update();
            telemetry.addData("pivot update time", (int) elapsedTime.milliseconds());
            telemetry.update();
            elapsedTime.reset();

            limelight.update();
            telemetry.addData("limelight update time", (int) elapsedTime.milliseconds());
            telemetry.update();
            elapsedTime.reset();

            if (loc != null) {
                telemetry.addData("limelight strafe distance", loc.translation);
                telemetry.addData("telescope extend dist", loc.extension);
            }
            telemetry.addData("target pose", targetPoint);
            telemetry.addData("drive x", drive.getPoseEstimate().getX());
            telemetry.addData("drive y", drive.getPoseEstimate().getY());
            telemetry.addData("drive h", Math.toDegrees(drive.getPoseEstimate().getHeading()));
            telemetry.update();

            telemetry.addData("telemetry time", (int) elapsedTime.milliseconds());
            telemetry.update();
            elapsedTime.reset();

            telemetry.addData("loop time", (int) elapsedTime2.milliseconds());
            telemetry.update();
        }

        limelight.stop();
        Thread.sleep(500);
    }
}
