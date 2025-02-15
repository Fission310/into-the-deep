package org.firstinspires.ftc.teamcode.opmode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.stuyfission.fissionlib.command.AutoCommandMachine;
import com.stuyfission.fissionlib.command.Command;
import com.stuyfission.fissionlib.command.CommandSequence;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Limelight;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Pivot;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Sweeper;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Telescope;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Wrist;
import org.firstinspires.ftc.teamcode.opmode.auton.util.Drive;
import org.firstinspires.ftc.teamcode.opmode.auton.util.LimelightConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "BasketAuto", preselectTeleOp = "Main")
public class BasketAuto extends LinearOpMode {
    private boolean commandBusy = false;
    private Pose2d targetPoint = null;
    public static double llTx = 0;
    public static double llTy = 0;
    public static double llTangle = 0;
    private TrajectorySequence basket1Traj;
    private TrajectorySequence farSampleTraj;
    private TrajectorySequence farSampleIntTraj;
    private TrajectorySequence basket2Traj;
    private TrajectorySequence centerSampleTraj;
    private TrajectorySequence centerSampleIntTraj;
    private TrajectorySequence basket3Traj;
    private TrajectorySequence wallSampleTraj;
    private TrajectorySequence wallSampleIntTraj;
    private TrajectorySequence basket4Traj;
    private TrajectorySequence sub1Traj;
    private TrajectorySequence basket5Traj;
    private TrajectorySequence sub2Traj;
    private TrajectorySequence basket6Traj;

    private SampleMecanumDrive drive;

    private Command basket1Command = () -> drive.followTrajectorySequenceAsync(basket1Traj);
    private Command farSampleCommand = () -> drive.followTrajectorySequenceAsync(farSampleTraj);
    private Command farSampleIntCommand = () -> drive.followTrajectorySequenceAsync(farSampleIntTraj);
    private Command basket2Command = () -> drive.followTrajectorySequenceAsync(basket2Traj);
    private Command centerSampleCommand = () -> drive.followTrajectorySequenceAsync(centerSampleTraj);
    private Command centerSampleIntCommand = () -> drive.followTrajectorySequenceAsync(centerSampleIntTraj);
    private Command basket3Command = () -> drive.followTrajectorySequenceAsync(basket3Traj);
    private Command wallSampleCommand = () -> drive.followTrajectorySequenceAsync(wallSampleTraj);
    private Command wallSampleIntCommand = () -> drive.followTrajectorySequenceAsync(wallSampleIntTraj);
    private Command basket4Command = () -> drive.followTrajectorySequenceAsync(basket4Traj);
    private Command sub1Command = () -> drive.followTrajectorySequenceAsync(sub1Traj);
    private Command basket5Command = () -> drive.followTrajectorySequenceAsync(basket5Traj);
    private Command sub2Command = () -> drive.followTrajectorySequenceAsync(sub2Traj);
    private Command basket6Command = () -> drive.followTrajectorySequenceAsync(basket6Traj);

    private Intake intake;
    private Pivot pivot;
    private Telescope telescope;
    private Wrist wrist;
    private Limelight limelight;
    private Sweeper sweeper;

    private Command commandBusyTrue = () -> commandBusy = true;
    private Command commandBusyFalse = () -> commandBusy = false;
    private Command outtake = () -> intake.outtake();
    private Command stopIntake = () -> intake.stop();
    private Command intakeCommand = () -> intake.intake();
    private Command pivotFront = () -> pivot.frontPos();
    private Command pivotInit = () -> pivot.initPos();
    private Command pivotClip = () -> pivot.clipPos();
    private Command pivotClipDown = () -> pivot.clipDownPos();
    private Command pivotBasket = () -> pivot.basketPos();
    private Command pivotUp = () -> pivot.upPos();
    private Command pivotUpIntake = () -> pivot.intakeUpPos();
    private Command pivotDownIntake = () -> pivot.intakeDownPos();
    private Command pivotGrabIntake = () -> pivot.autoIntakeGrabPos();
    private Command pivotRetract = () -> pivot.frontPos();
    private Command pivotReset = () -> pivot.reset();
    private Command sweepExtend = () -> sweeper.extendPos();
    private Command sweepRetract = () -> sweeper.retractPos();
    private Command telescopeFront = () -> telescope.frontPos();
    private Command telescopeBasket = () -> telescope.autoBasketPos();
    private Command telescopeIntake = () -> telescope.frontIntakePos();
    private Command telescopeFar = () -> telescope.autoFarPos();
    private Command telescopeCenter = () -> telescope.autoCenterPos();
    private Command telescopeWall = () -> telescope.autoWallPos();
    private Command telescopeScoreClip = () -> telescope.clipScorePos();
    private Command telescopeExtendClip = () -> telescope.clipExtensionPos();
    private Command telescopeRetract = () -> telescope.frontPos();
    private Command wristRetract = () -> wrist.frontPos();
    private Command wristRetractFirst = () -> wrist.retractPos();
    private Command wristIntakeScore = () -> wrist.autoIntakePos();
    private Command wristBasket = () -> wrist.basketPos();
    private Command wristClipScore = () -> wrist.clipScorePos();
    private Command wristDown = () -> wrist.intakeABit();
    private Command setResult = () -> {
        llTx = limelight.getTx();
        llTy = limelight.getTy();
        llTangle = limelight.getTangle();
        /*Uif (llTangle > 90) {
            llTangle = -8;
        } else if (llTangle < 90) {
            llTangle = 8;
        } else {
            llTangle = 0;
        }*/
    };
    private Command lineUpP2P = () -> targetPoint = new Pose2d(drive.getPoseEstimate().getX(),
            drive.getPoseEstimate().getY() - (LimelightConstants.calcXDistance(llTx, llTy) - 6.5),
            drive.getPoseEstimate().getHeading());
    private Command sweepP2P = () -> targetPoint = new Pose2d(drive.getPoseEstimate().getX(),
            drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading() + Math.toRadians(llTangle));
    private Command forwardP2P = () -> targetPoint = new Pose2d(targetPoint.getX() + 3,
            targetPoint.getY(), targetPoint.getHeading());
    private Command driveStop = () -> {
            targetPoint = null;
            drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
    };
    private Command telescopeExtendInches = () -> telescope
            .setTargetInches(LimelightConstants.calcYDistance(llTy) * 2 + 6);
    private Command telescopeExtendABit = () -> telescope.frontIntakeAutoShortPos();

    private CommandSequence basket1Sequence = new CommandSequence()
            .addCommand(commandBusyTrue)
            .addCommand(basket1Command)
            .addCommand(intakeCommand)
            .addCommand(wristIntakeScore)
            .addWaitCommand(0.5)
            .addCommand(pivotBasket)
            .addCommand(telescopeBasket)
            .addWaitCommand(0.6)
            .addCommand(wristBasket)
            .addCommand(intakeCommand)
            .addWaitCommand(0.4)
            .addCommand(outtake)
            .addWaitCommand(0.2)
            .addCommand(wristRetractFirst)
            .addWaitCommand(0.2)
            .addCommand(telescopeFront)
            .addWaitCommand(0.2)
            .addCommand(wristRetract)
            .addCommand(commandBusyFalse)
            .build();

    private CommandSequence farSampleSequence = new CommandSequence()
            .addCommand(commandBusyTrue)
            .addCommand(farSampleCommand)
            .addCommand(pivotGrabIntake)
            .addWaitCommand(0.4)
            .addCommand(telescopeFar)
            .addCommand(wristIntakeScore)
            .addWaitCommand(0.4)
            .addCommand(farSampleIntCommand)
            .addCommand(intakeCommand)
            .addWaitCommand(0.6)
            .addCommand(telescopeRetract)
            .addCommand(pivotUp)
            .addCommand(commandBusyFalse)
            .build();

    private CommandSequence basket2Sequence = new CommandSequence()
            .addCommand(commandBusyTrue)
            .addCommand(basket2Command)
            .addCommand(intakeCommand)
            .addWaitCommand(0.3)
            .addCommand(pivotBasket)
            .addCommand(telescopeBasket)
            .addWaitCommand(0.5)
            .addCommand(wristBasket)
            .addCommand(intakeCommand)
            .addWaitCommand(0.4)
            .addCommand(outtake)
            .addWaitCommand(0.2)
            .addCommand(wristRetractFirst)
            .addWaitCommand(0.2)
            .addCommand(telescopeFront)
            .addWaitCommand(0.2)
            .addCommand(wristRetract)
            .addCommand(commandBusyFalse)
            .build();
    private CommandSequence centerSampleSequence = new CommandSequence()
            .addCommand(commandBusyTrue)
            .addCommand(centerSampleCommand)
            .addCommand(pivotGrabIntake)
            .addWaitCommand(0.45)
            .addCommand(telescopeCenter)
            .addCommand(wristIntakeScore)
            .addWaitCommand(0.4)
            .addCommand(centerSampleIntCommand)
            .addCommand(intakeCommand)
            .addWaitCommand(0.6)
            .addCommand(telescopeRetract)
            .addCommand(pivotUp)
            .addCommand(commandBusyFalse)
            .build();
    private CommandSequence basket3Sequence = new CommandSequence()
            .addCommand(commandBusyTrue)
            .addCommand(basket3Command)
            .addCommand(intakeCommand)
            .addWaitCommand(0.3)
            .addCommand(pivotBasket)
            .addCommand(telescopeBasket)
            .addWaitCommand(0.5)
            .addCommand(wristBasket)
            .addCommand(intakeCommand)
            .addWaitCommand(0.4)
            .addCommand(outtake)
            .addWaitCommand(0.2)
            .addCommand(wristRetractFirst)
            .addWaitCommand(0.2)
            .addCommand(telescopeFront)
            .addWaitCommand(0.2)
            .addCommand(wristRetract)
            .addCommand(commandBusyFalse)
            .build();
    private CommandSequence wallSampleSequence = new CommandSequence()
            .addCommand(commandBusyTrue)
            .addCommand(wallSampleCommand)
            .addCommand(pivotGrabIntake)
            .addWaitCommand(0.65)
            .addCommand(telescopeWall)
            .addCommand(wristIntakeScore)
            .addWaitCommand(0.6)
            .addCommand(wallSampleIntCommand)
            .addCommand(intakeCommand)
            .addWaitCommand(0.6)
            .addCommand(telescopeRetract)
            .addCommand(pivotUp)
            .addCommand(commandBusyFalse)
            .build();
    private CommandSequence basket4Sequence = new CommandSequence()
            .addCommand(commandBusyTrue)
            .addCommand(basket4Command)
            .addCommand(intakeCommand)
            .addCommand(wristIntakeScore)
            .addWaitCommand(0.3)
            .addCommand(pivotBasket)
            .addCommand(telescopeBasket)
            .addWaitCommand(0.5)
            .addCommand(wristBasket)
            .addCommand(intakeCommand)
            .addWaitCommand(0.2)
            .addCommand(outtake)
            .addWaitCommand(0.2)
            .addCommand(wristRetractFirst)
            .addWaitCommand(0.2)
            .addCommand(telescopeFront)
            .addWaitCommand(0.2)
            .addCommand(wristRetract)
            .addCommand(commandBusyFalse)
            .build();

    private CommandSequence sub1Sequence = new CommandSequence()
            .addCommand(commandBusyTrue)
            .addCommand(sub1Command)
            .addCommand(pivotUpIntake)
            .addWaitCommand(1.9)
            .addCommand(setResult)
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
            .addCommand(sweepRetract)
            //.addCommand(sweepP2P)
            .addWaitCommand(0.9)
            //.addCommand(driveStop)
            .addCommand(pivotUpIntake)
            .addCommand(wristRetract)
            .addWaitCommand(0.2)
            .addCommand(telescopeRetract)
            .addWaitCommand(0.4)
            .addCommand(pivotUp)
            .addCommand(commandBusyFalse)
            .build();

    private CommandSequence basket5Sequence = new CommandSequence()
            .addCommand(commandBusyTrue)
            .addCommand(basket5Command)
            .addCommand(intakeCommand)
            .addCommand(wristIntakeScore)
            .addWaitCommand(1.3)
            .addCommand(pivotBasket)
            .addCommand(telescopeBasket)
            .addWaitCommand(0.5)
            .addCommand(wristBasket)
            .addCommand(intakeCommand)
            .addWaitCommand(0.2)
            .addCommand(outtake)
            .addWaitCommand(0.2)
            .addCommand(wristRetractFirst)
            .addWaitCommand(0.2)
            .addCommand(telescopeFront)
            .addWaitCommand(0.2)
            .addCommand(wristRetract)
            .addCommand(commandBusyFalse)
            .build();

    private CommandSequence sub2Sequence = new CommandSequence()
            .addCommand(commandBusyTrue)
            .addCommand(sub2Command)
            .addCommand(pivotUpIntake)
            .addWaitCommand(2.1)
            .addCommand(setResult)
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
            .addCommand(sweepRetract)
            //.addCommand(sweepP2P)
            .addWaitCommand(0.9)
            //.addCommand(driveStop)
            .addCommand(pivotUpIntake)
            .addCommand(wristRetract)
            .addWaitCommand(0.2)
            .addCommand(telescopeRetract)
            .addWaitCommand(0.4)
            .addCommand(pivotUp)
            .addCommand(commandBusyFalse)
            .build();

    private CommandSequence basket6Sequence = new CommandSequence()
            .addCommand(commandBusyTrue)
            .addCommand(basket6Command)
            .addCommand(intakeCommand)
            .addCommand(wristIntakeScore)
            .addWaitCommand(1.3)
            .addCommand(pivotBasket)
            .addCommand(telescopeBasket)
            .addWaitCommand(0.5)
            .addCommand(wristBasket)
            .addCommand(intakeCommand)
            .addWaitCommand(0.2)
            .addCommand(outtake)
            .addWaitCommand(0.2)
            .addCommand(wristRetractFirst)
            .addWaitCommand(0.2)
            .addCommand(telescopeFront)
            .addWaitCommand(0.2)
            .addCommand(wristRetract)
            .addCommand(commandBusyFalse)
            .build();
    private CommandSequence resetPivot = new CommandSequence()
            .addCommand(pivotReset)
            .addWaitCommand(2)
            .build();
    private CommandSequence doNothing = new CommandSequence().build();
    private AutoCommandMachine commandMachine = new AutoCommandMachine()
            .addCommandSequence(basket1Sequence)
            .addCommandSequence(farSampleSequence)
            .addCommandSequence(basket2Sequence)
            .addCommandSequence(centerSampleSequence)
            .addCommandSequence(basket3Sequence)
            .addCommandSequence(wallSampleSequence)
            .addCommandSequence(basket4Sequence)
            .addCommandSequence(sub1Sequence)
            .addCommandSequence(basket5Sequence)
            .addCommandSequence(sub2Sequence)
            .addCommandSequence(basket6Sequence)
            .addCommandSequence(resetPivot)
            .addCommandSequence(doNothing)
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
        pivot.initPos();
        wrist.init(hardwareMap);
        telescope.init(hardwareMap);
        limelight.init(hardwareMap);
        sweeper.init(hardwareMap);

        TrajectoryVelocityConstraint fastDT = SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL,
                DriveConstants.TRACK_WIDTH);
        TrajectoryAccelerationConstraint fastDTA = SampleMecanumDrive.getAccelerationConstraint(70);

        basket1Traj = drive
                .trajectorySequenceBuilder(BasketConstants.START.getPose())
                .splineToLinearHeading(BasketConstants.BASKET_1.getPose(), 5 * Math.PI / 4)
                .build();
        telemetry.addLine("Built basket1Traj");
        telemetry.update();
        farSampleTraj = drive
                .trajectorySequenceBuilder(basket1Traj.end())
                .setReversed(false)
                .splineToLinearHeading(BasketConstants.FAR_SAMPLE.getPose(), Math.PI / 2)
                .build();
        telemetry.addLine("Built farSampleTraj");
        telemetry.update();
        farSampleIntTraj = drive
                .trajectorySequenceBuilder(farSampleTraj.end())
                .setReversed(false)
                .splineToLinearHeading(BasketConstants.FAR_SAMPLE_INT.getPose(),
                        BasketConstants.FAR_SAMPLE_INT.getH())
                .build();
        telemetry.addLine("Built farSampleIntTraj");
        telemetry.update();
        basket2Traj = drive
                .trajectorySequenceBuilder(farSampleIntTraj.end())
                .setReversed(true)
                .splineToLinearHeading(BasketConstants.BASKET_2.getPose(), 5 * Math.PI / 4)
                .build();
        telemetry.addLine("Built basket2Traj");
        telemetry.update();
        centerSampleTraj = drive
                .trajectorySequenceBuilder(basket2Traj.end())
                .setReversed(false)
                .splineToLinearHeading(BasketConstants.CENTER_SAMPLE.getPose(), Math.PI / 2)
                .build();
        telemetry.addLine("Built centerSampleTraj");
        telemetry.update();
        centerSampleIntTraj = drive
                .trajectorySequenceBuilder(centerSampleTraj.end())
                .setReversed(false)
                .splineToLinearHeading(BasketConstants.CENTER_SAMPLE_INT.getPose(),
                        BasketConstants.CENTER_SAMPLE_INT.getH())
                .build();
        telemetry.addLine("Built centerSampleIntTraj");
        telemetry.update();
        basket3Traj = drive
                .trajectorySequenceBuilder(centerSampleIntTraj.end())
                .setReversed(true)
                .splineToLinearHeading(BasketConstants.BASKET_3.getPose(), 5 * Math.PI / 4)
                .build();
        telemetry.addLine("Built basket3Traj");
        telemetry.update();
        wallSampleTraj = drive
                .trajectorySequenceBuilder(basket3Traj.end())
                .setReversed(false)
                .splineToLinearHeading(BasketConstants.WALL_SAMPLE.getPose(), Math.PI / 2)
                .build();
        telemetry.addLine("Built wallSampleTraj");
        telemetry.update();
        wallSampleIntTraj = drive
                .trajectorySequenceBuilder(wallSampleTraj.end())
                .setReversed(false)
                .splineToLinearHeading(BasketConstants.WALL_SAMPLE_INT.getPose(),
                        BasketConstants.WALL_SAMPLE_INT.getH())
                .build();
        telemetry.addLine("Built wallSampleIntTraj");
        telemetry.update();
        basket4Traj = drive
                .trajectorySequenceBuilder(wallSampleIntTraj.end())
                .setReversed(true)
                .splineToLinearHeading(BasketConstants.BASKET_4.getPose(), 5 * Math.PI / 4)
                .build();
        telemetry.addLine("Built basket4Traj");
        telemetry.update();
        sub1Traj = drive
                .trajectorySequenceBuilder(basket4Traj.end())
                .setVelConstraint(fastDT)
                .setAccelConstraint(fastDTA)
                .setReversed(false)
                .splineToLinearHeading(BasketConstants.SUBMERSIBLE_1.getPose(),
                        BasketConstants.SUBMERSIBLE_1.getH())
                .build();
        telemetry.addLine("Built sub1Traj");
        telemetry.update();
        basket5Traj = drive
                .trajectorySequenceBuilder(sub1Traj.end())
                .setVelConstraint(fastDT)
                .setAccelConstraint(fastDTA)
                .setReversed(true)
                .splineToLinearHeading(BasketConstants.BASKET_5.getPose(), 5 * Math.PI / 4)
                .build();
        telemetry.addLine("Built basket5Traj");
        telemetry.update();
        sub2Traj = drive
                .trajectorySequenceBuilder(basket5Traj.end())
                .setVelConstraint(fastDT)
                .setAccelConstraint(fastDTA)
                .setReversed(false)
                .splineToLinearHeading(BasketConstants.SUBMERSIBLE_2.getPose(),
                        BasketConstants.SUBMERSIBLE_2.getH())
                .build();
        telemetry.addLine("Built sub2raj");
        telemetry.update();
        basket6Traj = drive
                .trajectorySequenceBuilder(sub2Traj.end())
                .setVelConstraint(fastDT)
                .setAccelConstraint(fastDTA)
                .setReversed(true)
                .splineToLinearHeading(BasketConstants.BASKET_6.getPose(), 5 * Math.PI / 4)
                .build();
        telemetry.addLine("Built basket6Traj");
        telemetry.update();

        while (opModeInInit() && !isStopRequested()) {
            drive.updatePoseEstimate();
            telemetry.addData("drive x", drive.getPoseEstimate().getX());
            telemetry.addData("drive y", drive.getPoseEstimate().getY());
            telemetry.update();
            pivot.update();
        }

        drive.setPoseEstimate(BasketConstants.START.getPose());

        waitForStart();

        while (opModeIsActive() && !isStopRequested() && !commandMachine.hasCompleted()) {
            if (targetPoint != null) {
                Drive.p2p(drive, targetPoint);
            }
            drive.update();
            telescope.update();
            pivot.update();
            limelight.update();
            commandMachine.run(drive.isBusy() || commandBusy);
            telemetry.addData("limelight tx", llTx);
            telemetry.addData("limelight ty", llTy);
            telemetry.addData("limelight tangle", llTangle);
            telemetry.addData("limelight strafe distance",
                    LimelightConstants.calcXDistance(llTx, llTy) - 6);
            telemetry.addData("telescope extend dist",
                    LimelightConstants.calcYDistance(llTy));
            telemetry.addData("telescope extend ticks",
                    (LimelightConstants.calcYDistance(llTy)) / Telescope.INCH_PER_TICK);
            telemetry.addData("target pose", targetPoint);
            telemetry.addData("drive x", drive.getPoseEstimate().getX());
            telemetry.addData("drive y", drive.getPoseEstimate().getY());
            telemetry.update();
        }

        limelight.stop();
        Thread.sleep(500);
    }
}
