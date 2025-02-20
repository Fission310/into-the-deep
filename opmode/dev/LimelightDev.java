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
import org.firstinspires.ftc.teamcode.opmode.auton.util.Drive;
import org.firstinspires.ftc.teamcode.opmode.auton.util.LimelightConstants;

@TeleOp(name = "LimelightDev")
public class LimelightDev extends LinearOpMode {
    private Pose2d targetPoint = null;
    public static double llTx = 0;
    public static double llTy = 0;
    public static double llTangle = 0;
    public static double lineUpY = 0;
    public static double targetInches = 0;

    private enum State {
        DETECT,
        LINEUP,
        RETRACT
    }

    private State state = State.DETECT;
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
        llTx = limelight.getTx();
        llTy = limelight.getTy();
        llTangle = limelight.getTangle();
        lineUpY =  -(LimelightConstants.calcXDistance(llTx, llTy) - 6.5);
        targetInches = LimelightConstants.calcYDistance(llTy) * 2 + 6;
    };
    private Command lineUpP2P = () -> targetPoint = new Pose2d(drive.getPoseEstimate().getX(),
            drive.getPoseEstimate().getY() + lineUpY,
            drive.getPoseEstimate().getHeading());
    private Command forwardP2P = () -> targetPoint = new Pose2d(targetPoint.getX() + 3,
            targetPoint.getY(), targetPoint.getHeading());
    private Command driveStop = () -> {
            targetPoint = null;
            drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
    };
    private Command telescopeExtendInches = () -> telescope
            .setTargetInches(targetInches);
    private Command telescopeExtendABit = () -> telescope.frontIntakeAutoShortPos();

    private CommandSequence detect = new CommandSequence()
            .addCommand(setResult)
            .build();
    private CommandSequence lineUp = new CommandSequence()
            .addCommand(lineUpP2P)
            .addWaitCommand(0.4)
            .addCommand(sweepExtend)
            .addWaitCommand(0.2)
            .addCommand(forwardP2P)
            .addWaitCommand(0.4)
            .addCommand(driveStop)
            .addCommand(telescopeExtendABit)
            .addCommand(intakeCommand)
            .addCommand(pivotGrabIntake)
            .addCommand(wristIntakeScore)
            .addWaitCommand(0.6)
            .addCommand(telescopeExtendInches)
            .build();
    private CommandSequence retract = new CommandSequence()
            .addCommand(sweepRetract)
            .addWaitCommand(0.9)
            .addCommand(pivotUpIntake)
            .addCommand(wristRetract)
            .addWaitCommand(0.2)
            .addCommand(telescopeRetract)
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
            }
            drive.update();
            telescope.update();
            pivot.update();
            limelight.update();

            if (buttonClicked) continue;
            if (!GamepadStatic.isButtonPressed(gamepad1, GamepadStatic.Input.LEFT_BUMPER)) {
                buttonClicked = false;
            }

            switch (state) {
                case DETECT:
                    if (GamepadStatic.isButtonPressed(gamepad1, GamepadStatic.Input.LEFT_BUMPER)) {
                        state = State.LINEUP;
                        lineUp.trigger();
                        buttonClicked = true;
                    }
                    break;
                case LINEUP:
                    if (GamepadStatic.isButtonPressed(gamepad1, GamepadStatic.Input.LEFT_BUMPER)) {
                        state = State.RETRACT;
                        retract.trigger();
                        buttonClicked = true;
                    }
                    break;
                case RETRACT:
                    if (GamepadStatic.isButtonPressed(gamepad1, GamepadStatic.Input.LEFT_BUMPER)) {
                        state = State.DETECT;
                        detect.trigger();
                        buttonClicked = true;
                    }
                    break;
            }
            telemetry.addData("limelight tx", llTx);
            telemetry.addData("limelight ty", llTy);
            telemetry.addData("limelight tangle", llTangle);
            telemetry.addData("limelight strafe distance", lineUpY);
            telemetry.addData("telescope extend dist", targetInches);
            telemetry.addData("telescope extend ticks", targetInches / Telescope.INCH_PER_TICK);
            telemetry.addData("target pose", targetPoint);
            telemetry.addData("drive x", drive.getPoseEstimate().getX());
            telemetry.addData("drive y", drive.getPoseEstimate().getY());
            telemetry.update();
        }

        limelight.stop();
        Thread.sleep(500);
    }
}
