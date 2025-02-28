package org.firstinspires.ftc.teamcode.opmode.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
import org.firstinspires.ftc.teamcode.opmode.auton.util.Drive;

@TeleOp(name = "LimelightDev")
public class LimelightDev extends LinearOpMode {
    private Pose2d targetPoint = null;
    private Pose2d botPos = null;
    private Location loc = null;

    private enum State {
        DETECT,
        LINEUP,
        RETRACT
    }

    private State state = State.RETRACT;
    private boolean buttonClicked = false;

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
            drive.getPoseEstimate().getHeading());
    private Command forwardP2P = () -> targetPoint = new Pose2d(targetPoint.getX(),
            targetPoint.getY(), targetPoint.getHeading());
    private Command driveStop = () -> {
        targetPoint = null;
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
    };
    private Command telescopeExtendInches = () -> telescope.setTargetInches(loc.extension);
    private Command telescopeExtendABit = () -> telescope.frontIntakeAutoShortPos();
    private Command setDetect = () -> state = State.DETECT;
    private Command setLineup = () -> state = State.LINEUP;
    private Command setRetract = () -> state = State.RETRACT;

    private CommandSequence detect = new CommandSequence()
            .addCommand(setResult)
            .addCommand(setLineup)
            .build();
    private CommandSequence lineUp = new CommandSequence()
            .addCommand(lineUpP2P)
            .addWaitCommand(4)
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
            .addCommand(setRetract)
            .build();
    private CommandSequence retract = new CommandSequence()
            .addCommand(sweepRetract)
            .addWaitCommand(0.9)
            .addCommand(pivotUpIntake)
            .addCommand(wristRetract)
            .addWaitCommand(0.2)
            .addCommand(telescopeRetract)
            .addCommand(setDetect)
            .build();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        intake = new Intake(this);
        drive = new SampleMecanumDrive(hardwareMap);
        telescope = new Telescope(this);
        pivot = new Pivot(this, telescope);
        wrist = new Wrist(this);
        limelight = new Limelight(this);
        sweeper = new Sweeper(this);

        intake.init(hardwareMap);
        pivot.init(hardwareMap);
        wrist.init(hardwareMap);
        telescope.init(hardwareMap);
        limelight.init(hardwareMap);
        sweeper.init(hardwareMap);

        while (opModeInInit() && !isStopRequested()) {
            drive.updatePoseEstimate();
        }

        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(90)));

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            if (targetPoint != null) {
                Drive.p2p(drive, targetPoint);
            } else {
                drive.setMotorPowers(0, 0, 0, 0);
            }
            drive.update();
            telescope.update();
            pivot.update();
            limelight.update();

            if (!GamepadStatic.isButtonPressed(gamepad1, GamepadStatic.Input.LEFT_BUMPER)) {
                buttonClicked = false;
            }
            if (buttonClicked)
                continue;

            switch (state) {
                case DETECT:
                    if (GamepadStatic.isButtonPressed(gamepad1, GamepadStatic.Input.LEFT_BUMPER)) {
                        lineUp.trigger();
                        buttonClicked = true;
                    }
                    break;
                case LINEUP:
                    if (GamepadStatic.isButtonPressed(gamepad1, GamepadStatic.Input.LEFT_BUMPER)) {
                        retract.trigger();
                        buttonClicked = true;
                    }
                    break;
                case RETRACT:
                    if (GamepadStatic.isButtonPressed(gamepad1, GamepadStatic.Input.LEFT_BUMPER)) {
                        detect.trigger();
                        buttonClicked = true;
                    }
                    break;
            }
            if (loc != null) {
                telemetry.addData("limelight strafe distance", loc.translation);
                telemetry.addData("telescope extend dist", loc.extension);
            }
            telemetry.addData("target pose", targetPoint);
            telemetry.addData("drive x", drive.getPoseEstimate().getX());
            telemetry.addData("drive y", drive.getPoseEstimate().getY());
            telemetry.update();
        }

        limelight.stop();
        Thread.sleep(500);
    }
}
