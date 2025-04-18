package org.firstinspires.ftc.teamcode.opmode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
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
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Limelight.Location;
import org.firstinspires.ftc.teamcode.opmode.auton.util.Color;
import org.firstinspires.ftc.teamcode.opmode.auton.util.Drive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class BasketAuto extends LinearOpMode {

    public BasketAuto(Color color) {
        this.color = color;
    }

    private Color color;
    private boolean commandBusy = false;
    private Pose2d targetPoint = null;
    private Pose2d drivePos = null;
    private Location loc = null;
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
    private TrajectorySequence sub3Traj;
    private TrajectorySequence basket7Traj;
    private TrajectorySequence sub4Traj;
    private TrajectorySequence basket8Traj;
    private TrajectorySequence sub5Traj;
    private TrajectorySequence basket9Traj;

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
    private Command sub3Command = () -> drive.followTrajectorySequenceAsync(sub3Traj);
    private Command basket7Command = () -> drive.followTrajectorySequenceAsync(basket7Traj);
    private Command sub4Command = () -> drive.followTrajectorySequenceAsync(sub4Traj);
    private Command basket8Command = () -> drive.followTrajectorySequenceAsync(basket8Traj);
    private Command sub5Command = () -> drive.followTrajectorySequenceAsync(sub5Traj);
    private Command basket9Command = () -> drive.followTrajectorySequenceAsync(basket9Traj);

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
    private Command telescopeBasket = () -> telescope.autoBasketPos();
    private Command telescopeLimelightBasket = () -> telescope.autoLimelightBasketPos();
    private Command telescopeBasketFirst = () -> telescope.autoBasketFirstPos();
    private Command telescopeFar = () -> telescope.autoFarPos();
    private Command telescopeCenter = () -> telescope.autoCenterPos();
    private Command telescopeWall = () -> telescope.autoWallPos();
    private Command limelightIntake = () -> telescope.autoLimelightIntakePos();

    private Command telescopeHorizontalRetract = () -> telescope.frontHorizontalPos();
    private Command telescopeVerticalRetract = () -> telescope.frontVerticalPos();
    private Command wristRetract = () -> wrist.frontPos();
    private Command wristBasket = () -> wrist.autoBasketPos();
    private Command wristMid = () -> wrist.autoMidPos();
    private Command wristRetractFirst = () -> wrist.retractPos();
    private Command wristIntakeScore = () -> wrist.autoIntakePos();
    private Command wristIntakeLL = () -> wrist.autoIntakeLLPos();
    private Command setResult = () -> {
        loc = limelight.getBest();
        if (loc.extension == 0) {
            loc.extension = 10;
        }
        drivePos = drive.getPoseEstimate();
    };
    private Command outtakeWrong = () -> {
        intake.update();
        if (intake.hasWrongColor(color)) {
            intake.outtake();
        }
    };
    private Command lineUpP2P = () -> targetPoint = new Pose2d(drivePos.getX(),
            drivePos.getY() - loc.translation,
            drivePos.getHeading());
    private Command forwardP2P = () -> targetPoint = new Pose2d(targetPoint.getX() + 2,
            targetPoint.getY(), targetPoint.getHeading());
    private Command driveStop = () -> {
        targetPoint = null;
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
    };
    private Command telescopeExtendInches = () -> telescope
            .setTargetInches(loc.extension);
    private Command telescopeExtendABit = () -> telescope.frontIntakeAutoShortPos();

    private CommandSequence basket1Sequence = new CommandSequence()
            .addCommand(commandBusyTrue)
            .addCommand(basket1Command)
            .addCommand(intakeCommand)
            .addCommand(wristMid)
            .addWaitCommand(0.6)
            .addCommand(pivotBasket)
            .addWaitCommand(0.2)
            .addCommand(telescopeBasketFirst)
            .addWaitCommand(0.4)
            .addCommand(wristBasket)
            .addWaitCommand(0.2)
            .addCommand(outtake)
            .addWaitCommand(0.3)
            .addCommand(wristMid)
            .addCommand(telescopeVerticalRetract)
            .addWaitCommand(0.3)
            .addCommand(wristRetract)
            .addCommand(commandBusyFalse)
            .build();

    private CommandSequence farSampleSequence = new CommandSequence()
            .addCommand(commandBusyTrue)
            .addCommand(farSampleCommand)
            .addCommand(pivotFront)
            .addWaitCommand(0.45)
            .addCommand(telescopeFar)
            .addCommand(pivotGrabIntake)
            .addWaitCommand(0.5)
            .addCommand(wristIntakeScore)
            .addWaitCommand(0.2)
            .addCommand(farSampleIntCommand)
            .addCommand(intakeCommand)
            .addCommand(commandBusyFalse)
            .build();

    private CommandSequence basket2Sequence = new CommandSequence()
            .addCommand(commandBusyTrue)
            .addCommand(basket2Command)
            .addCommand(telescopeHorizontalRetract)
            .addCommand(intakeCommand)
            .addCommand(wristMid)
            .addCommand(pivotBasket)
            .addWaitCommand(0.4)
            .addCommand(telescopeBasket)
            .addCommand(intakeCommand)
            .addWaitCommand(0.4)
            .addCommand(wristBasket)
            .addWaitCommand(0.2)
            .addCommand(outtake)
            .addWaitCommand(0.30)
            .addCommand(wristMid)
            .addCommand(telescopeVerticalRetract)
            .addWaitCommand(0.3)
            .addCommand(wristRetract)
            .addCommand(commandBusyFalse)
            .build();
    private CommandSequence centerSampleSequence = new CommandSequence()
            .addCommand(commandBusyTrue)
            .addCommand(centerSampleCommand)
            .addCommand(pivotFront)
            .addWaitCommand(0.45)
            .addCommand(telescopeCenter)
            .addCommand(pivotGrabIntake)
            .addWaitCommand(0.5)
            .addCommand(wristIntakeScore)
            .addWaitCommand(0.2)
            .addCommand(centerSampleIntCommand)
            .addCommand(intakeCommand)
            .addCommand(commandBusyFalse)
            .build();
    private CommandSequence basket3Sequence = new CommandSequence()
            .addCommand(commandBusyTrue)
            .addCommand(basket3Command)
            .addCommand(telescopeHorizontalRetract)
            .addCommand(intakeCommand)
            .addCommand(wristMid)
            .addCommand(pivotBasket)
            .addWaitCommand(0.4)
            .addCommand(telescopeBasket)
            .addCommand(intakeCommand)
            .addWaitCommand(0.4)
            .addCommand(wristBasket)
            .addWaitCommand(0.2)
            .addCommand(outtake)
            .addWaitCommand(0.3)
            .addCommand(wristMid)
            .addCommand(telescopeVerticalRetract)
            .addWaitCommand(0.3)
            .addCommand(wristRetract)
            .addCommand(commandBusyFalse)
            .build();
    private CommandSequence wallSampleSequence = new CommandSequence()
            .addCommand(commandBusyTrue)
            .addCommand(wallSampleCommand)
            .addCommand(pivotFront)
            .addWaitCommand(BasketConstants.WALL_SAMPLE_DELAY)
            .addCommand(telescopeWall)
            .addCommand(pivotGrabIntake)
            .addWaitCommand(0.5)
            .addCommand(wristIntakeScore)
            .addWaitCommand(0.2)
            .addCommand(wallSampleIntCommand)
            .addCommand(intakeCommand)
            .addCommand(commandBusyFalse)
            .build();
    private CommandSequence basket4Sequence = new CommandSequence()
            .addCommand(commandBusyTrue)
            .addCommand(basket4Command)
            .addCommand(telescopeHorizontalRetract)
            .addCommand(intakeCommand)
            .addCommand(wristMid)
            .addCommand(pivotBasket)
            .addWaitCommand(BasketConstants.BASKET_WALL_DELAY)
            .addCommand(telescopeBasket)
            .addCommand(intakeCommand)
            .addWaitCommand(0.4)
            .addCommand(wristBasket)
            .addWaitCommand(0.2)
            .addCommand(outtake)
            .addWaitCommand(0.3)
            .addCommand(wristMid)
            .addCommand(telescopeVerticalRetract)
            .addWaitCommand(0.3)
            .addCommand(wristRetract)
            .addCommand(commandBusyFalse)
            .build();

    private CommandSequence sub1Sequence = new CommandSequence()
            .addCommand(commandBusyTrue)
            .addCommand(sub1Command)
            .addCommand(pivotUpIntake)
            .addWaitCommand(1.90)
            .addCommand(setResult)
            .addCommand(lineUpP2P)
            .addWaitCommand(0.4)
            .addCommand(sweepExtend)
            .addCommand(forwardP2P)
            .addWaitCommand(0.4)
            .addCommand(driveStop)
            .addCommand(telescopeExtendABit)
            .addCommand(intakeCommand)
            .addCommand(pivotGrabIntake)
            .addCommand(wristIntakeLL)
            .addWaitCommand(0.2)
            //.addCommand(telescopeExtendInches)
            .addCommand(limelightIntake)
            .addCommand(sweepRetract)
            // .addCommand(sweepP2P)
            .addWaitCommand(0.9)
            // .addCommand(driveStop)
            .addCommand(outtakeWrong)
            .addCommand(pivotUpIntake)
            .addCommand(wristRetract)
            .addWaitCommand(0.2)
            .addCommand(telescopeHorizontalRetract)
            .addWaitCommand(0.430)
            .addCommand(commandBusyFalse)
            .build();

    private CommandSequence basket5Sequence = new CommandSequence()
            .addCommand(commandBusyTrue)
            .addCommand(basket5Command)
            .addCommand(intakeCommand)
            .addWaitCommand(0.6)
            .addCommand(pivotBasket)
            .addWaitCommand(0.7)
            .addCommand(telescopeLimelightBasket)
            .addWaitCommand(0.3)
            .addCommand(wristBasket)
            .addCommand(intakeCommand)
            .addWaitCommand(0.3)
            .addCommand(outtake)
            .addWaitCommand(0.3)
            .addCommand(wristRetractFirst)
            .addWaitCommand(0.2)
            .addCommand(wristRetractFirst)
            .addCommand(telescopeVerticalRetract)
            .addWaitCommand(0.3)
            .addCommand(wristRetract)
            .addCommand(commandBusyFalse)
            .build();

    private CommandSequence sub2Sequence = new CommandSequence()
            .addCommand(commandBusyTrue)
            .addCommand(sub2Command)
            .addCommand(pivotUpIntake)
            .addWaitCommand(2.4)
            .addCommand(setResult)
            .addCommand(lineUpP2P)
            .addWaitCommand(0.4)
            .addCommand(sweepExtend)
            .addCommand(forwardP2P)
            .addWaitCommand(0.4)
            .addCommand(driveStop)
            .addCommand(telescopeExtendABit)
            .addCommand(intakeCommand)
            .addCommand(pivotGrabIntake)
            .addCommand(wristIntakeLL)
            .addWaitCommand(0.2)
            //.addCommand(telescopeExtendInches)
            .addCommand(limelightIntake)
            .addCommand(sweepRetract)
            // .addCommand(sweepP2P)
            .addWaitCommand(0.9)
            // .addCommand(driveStop)
            .addCommand(outtakeWrong)
            .addCommand(pivotUpIntake)
            .addCommand(wristRetract)
            .addWaitCommand(0.2)
            .addCommand(telescopeHorizontalRetract)
            .addWaitCommand(0.4)
            .addCommand(commandBusyFalse)
            .build();

    private CommandSequence basket6Sequence = new CommandSequence()
            .addCommand(commandBusyTrue)
            .addCommand(basket6Command)
            .addCommand(intakeCommand)
            .addWaitCommand(0.6)
            .addCommand(pivotBasket)
            .addWaitCommand(0.7)
            .addCommand(telescopeLimelightBasket)
            .addWaitCommand(0.3)
            .addCommand(wristBasket)
            .addCommand(intakeCommand)
            .addWaitCommand(0.3)
            .addCommand(outtake)
            .addWaitCommand(0.3)
            .addCommand(wristRetractFirst)
            .addWaitCommand(0.2)
            .addCommand(wristRetractFirst)
            .addCommand(telescopeVerticalRetract)
            .addWaitCommand(0.3)
            .addCommand(wristRetract)
            .addCommand(commandBusyFalse)
            .build();
    private CommandSequence sub3Sequence = new CommandSequence()
            .addCommand(commandBusyTrue)
            .addCommand(sub3Command)
            .addCommand(pivotUpIntake)
            .addWaitCommand(2.4)
            .addCommand(setResult)
            .addCommand(lineUpP2P)
            .addWaitCommand(0.4)
            .addCommand(sweepExtend)
            .addCommand(forwardP2P)
            .addWaitCommand(0.4)
            .addCommand(driveStop)
            .addCommand(telescopeExtendABit)
            .addCommand(intakeCommand)
            .addCommand(pivotGrabIntake)
            .addCommand(wristIntakeLL)
            .addWaitCommand(0.2)
            //.addCommand(telescopeExtendInches)
            .addCommand(limelightIntake)
            .addCommand(sweepRetract)
            // .addCommand(sweepP2P)
            .addWaitCommand(0.9)
            // .addCommand(driveStop)
            .addCommand(outtakeWrong)
            .addCommand(pivotUpIntake)
            .addCommand(wristRetract)
            .addWaitCommand(0.2)
            .addCommand(telescopeHorizontalRetract)
            .addWaitCommand(0.4)
            .addCommand(commandBusyFalse)
            .build();

    private CommandSequence basket7Sequence = new CommandSequence()
            .addCommand(commandBusyTrue)
            .addCommand(basket7Command)
            .addCommand(intakeCommand)
            .addWaitCommand(0.6)
            .addCommand(pivotBasket)
            .addWaitCommand(0.7)
            .addCommand(telescopeLimelightBasket)
            .addWaitCommand(0.3)
            .addCommand(wristBasket)
            .addCommand(intakeCommand)
            .addWaitCommand(0.3)
            .addCommand(outtake)
            .addWaitCommand(0.3)
            .addCommand(wristRetractFirst)
            .addWaitCommand(0.2)
            .addCommand(wristRetractFirst)
            .addCommand(telescopeVerticalRetract)
            .addWaitCommand(0.3)
            .addCommand(wristRetract)
            .addCommand(commandBusyFalse)
            .build();
    private CommandSequence sub4Sequence = new CommandSequence()
            .addCommand(commandBusyTrue)
            .addCommand(sub4Command)
            .addCommand(pivotUpIntake)
            .addWaitCommand(2.4)
            .addCommand(setResult)
            .addCommand(lineUpP2P)
            .addWaitCommand(0.4)
            .addCommand(sweepExtend)
            .addCommand(forwardP2P)
            .addWaitCommand(0.4)
            .addCommand(driveStop)
            .addCommand(telescopeExtendABit)
            .addCommand(intakeCommand)
            .addCommand(pivotGrabIntake)
            .addCommand(wristIntakeLL)
            .addWaitCommand(0.2)
            //.addCommand(telescopeExtendInches)
            .addCommand(limelightIntake)
            .addCommand(sweepRetract)
            // .addCommand(sweepP2P)
            .addWaitCommand(0.9)
            // .addCommand(driveStop)
            .addCommand(outtakeWrong)
            .addCommand(pivotUpIntake)
            .addCommand(wristRetract)
            .addWaitCommand(0.2)
            .addCommand(telescopeHorizontalRetract)
            .addWaitCommand(0.4)
            .addCommand(commandBusyFalse)
            .build();

    private CommandSequence basket8Sequence = new CommandSequence()
            .addCommand(commandBusyTrue)
            .addCommand(basket8Command)
            .addCommand(intakeCommand)
            .addWaitCommand(0.6)
            .addCommand(pivotBasket)
            .addWaitCommand(0.7)
            .addCommand(telescopeLimelightBasket)
            .addWaitCommand(0.3)
            .addCommand(wristBasket)
            .addCommand(intakeCommand)
            .addWaitCommand(0.3)
            .addCommand(outtake)
            .addWaitCommand(0.3)
            .addCommand(wristRetractFirst)
            .addWaitCommand(0.2)
            .addCommand(wristRetractFirst)
            .addCommand(telescopeVerticalRetract)
            .addWaitCommand(0.3)
            .addCommand(wristRetract)
            .addCommand(commandBusyFalse)
            .build();
    private CommandSequence sub5Sequence = new CommandSequence()
            .addCommand(commandBusyTrue)
            .addCommand(sub5Command)
            .addCommand(pivotUpIntake)
            .addWaitCommand(2.4)
            .addCommand(setResult)
            .addCommand(lineUpP2P)
            .addWaitCommand(0.4)
            .addCommand(sweepExtend)
            .addCommand(forwardP2P)
            .addWaitCommand(0.4)
            .addCommand(driveStop)
            .addCommand(telescopeExtendABit)
            .addCommand(intakeCommand)
            .addCommand(pivotGrabIntake)
            .addCommand(wristIntakeLL)
            .addWaitCommand(0.2)
            //.addCommand(telescopeExtendInches)
            .addCommand(limelightIntake)
            .addCommand(sweepRetract)
            // .addCommand(sweepP2P)
            .addWaitCommand(0.9)
            // .addCommand(driveStop)
            .addCommand(outtakeWrong)
            .addCommand(pivotUpIntake)
            .addCommand(wristRetract)
            .addWaitCommand(0.2)
            .addCommand(telescopeHorizontalRetract)
            .addWaitCommand(0.4)
            .addCommand(commandBusyFalse)
            .build();

    private CommandSequence basket9Sequence = new CommandSequence()
            .addCommand(commandBusyTrue)
            .addCommand(basket9Command)
            .addCommand(intakeCommand)
            .addWaitCommand(0.6)
            .addCommand(pivotBasket)
            .addWaitCommand(0.7)
            .addCommand(telescopeLimelightBasket)
            .addWaitCommand(0.3)
            .addCommand(wristBasket)
            .addCommand(intakeCommand)
            .addWaitCommand(0.3)
            .addCommand(outtake)
            .addWaitCommand(0.3)
            .addCommand(wristRetractFirst)
            .addWaitCommand(0.2)
            .addCommand(wristRetractFirst)
            .addCommand(telescopeVerticalRetract)
            .addWaitCommand(0.3)
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
            .addCommandSequence(sub3Sequence)
            .addCommandSequence(basket7Sequence)
            .addCommandSequence(sub4Sequence)
            .addCommandSequence(basket8Sequence)
            .addCommandSequence(sub5Sequence)
            .addCommandSequence(basket9Sequence)
            .addCommandSequence(resetPivot)
            .addCommandSequence(doNothing)
            .build();

    @Override
    public void runOpMode() throws InterruptedException {
        VoltageSensor voltage = hardwareMap.voltageSensor.iterator().next();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        intake = new Intake(this);
        drive = new SampleMecanumDrive(hardwareMap);
        telescope = new Telescope(this);
        pivot = new Pivot(this, telescope);
        wrist = new Wrist(this);
        limelight = new Limelight(this, color);
        sweeper = new Sweeper(this);

        intake.init(hardwareMap);
        pivot.init(hardwareMap);
        pivot.initPos();
        wrist.init(hardwareMap);
        telescope.init(hardwareMap);
        limelight.init(hardwareMap);
        sweeper.init(hardwareMap);
        wrist.frontPos();

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
                .lineToLinearHeading(BasketConstants.BASKET_2.getPose())
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
                .lineToLinearHeading(BasketConstants.BASKET_3.getPose())
                .build();
        telemetry.addLine("Built basket3Traj");
        telemetry.update();
        wallSampleTraj = drive
                .trajectorySequenceBuilder(basket3Traj.end())
                .setReversed(false)
                .lineToLinearHeading(BasketConstants.WALL_SAMPLE.getPose())
                .build();
        telemetry.addLine("Built wallSampleTraj");
        telemetry.update();
        wallSampleIntTraj = drive
                .trajectorySequenceBuilder(wallSampleTraj.end())
                .setReversed(false)
                .lineToLinearHeading(BasketConstants.WALL_SAMPLE_INT.getPose())
                .build();
        telemetry.addLine("Built wallSampleIntTraj");
        telemetry.update();
        basket4Traj = drive
                .trajectorySequenceBuilder(wallSampleIntTraj.end())
                .setVelConstraint(fastDT)
                .setAccelConstraint(fastDTA)
                .setReversed(true)
                .lineToLinearHeading(BasketConstants.BASKET_4.getPose())
                .build();
        telemetry.addLine("Built basket4Traj");
        telemetry.update();
        sub1Traj = drive
                .trajectorySequenceBuilder(basket4Traj.end())
                .setVelConstraint(fastDT)
                .setAccelConstraint(fastDTA)
                .setReversed(false)
                .splineToLinearHeading(BasketConstants.SUBMERSIBLE_1.getPose(), BasketConstants.SUBMERSIBLE_1.getH())
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
        telemetry.addLine("Built sub2Traj");
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
        sub3Traj = drive
                .trajectorySequenceBuilder(basket6Traj.end())
                .setVelConstraint(fastDT)
                .setAccelConstraint(fastDTA)
                .setReversed(false)
                .splineToLinearHeading(BasketConstants.SUBMERSIBLE_3.getPose(),
                        BasketConstants.SUBMERSIBLE_3.getH())
                .build();
        telemetry.addLine("Built sub3Traj");
        telemetry.update();
        basket7Traj = drive
                .trajectorySequenceBuilder(sub3Traj.end())
                .setVelConstraint(fastDT)
                .setAccelConstraint(fastDTA)
                .setReversed(true)
                .splineToLinearHeading(BasketConstants.BASKET_7.getPose(), 5 * Math.PI / 4)
                .build();
        telemetry.addLine("Built basket7Traj");
        telemetry.update();
        sub4Traj = drive
                .trajectorySequenceBuilder(basket7Traj.end())
                .setVelConstraint(fastDT)
                .setAccelConstraint(fastDTA)
                .setReversed(false)
                .splineToLinearHeading(BasketConstants.SUBMERSIBLE_4.getPose(),
                        BasketConstants.SUBMERSIBLE_4.getH())
                .build();
        telemetry.addLine("Built sub4Traj");
        telemetry.update();
        basket8Traj = drive
                .trajectorySequenceBuilder(sub4Traj.end())
                .setVelConstraint(fastDT)
                .setAccelConstraint(fastDTA)
                .setReversed(true)
                .splineToLinearHeading(BasketConstants.BASKET_8.getPose(), 5 * Math.PI / 4)
                .build();
        telemetry.addLine("Built basket8Traj");
        telemetry.update();
        sub5Traj = drive
                .trajectorySequenceBuilder(basket8Traj.end())
                .setVelConstraint(fastDT)
                .setAccelConstraint(fastDTA)
                .setReversed(false)
                .splineToLinearHeading(BasketConstants.SUBMERSIBLE_5.getPose(),
                        BasketConstants.SUBMERSIBLE_5.getH())
                .build();
        telemetry.addLine("Built sub5raj");
        telemetry.update();
        basket9Traj = drive
                .trajectorySequenceBuilder(sub5Traj.end())
                .setVelConstraint(fastDT)
                .setAccelConstraint(fastDTA)
                .setReversed(true)
                .splineToLinearHeading(BasketConstants.BASKET_9.getPose(), 5 * Math.PI / 4)
                .build();
        telemetry.addLine("Built basket9Traj");
        telemetry.update();

        while (opModeInInit() && !isStopRequested()) {
            drive.updatePoseEstimate();
            telemetry.addData("drive x", drive.getPoseEstimate().getX());
            telemetry.addData("drive y", drive.getPoseEstimate().getY());
            telemetry.addData("voltage", voltage.getVoltage());
            telemetry.update();
            pivot.update();
        }

        drive.setPoseEstimate(BasketConstants.START.getPose());

        waitForStart();

        while (opModeIsActive() && !isStopRequested() && !commandMachine.hasCompleted()) {
            if (targetPoint != null) {
                Drive.p2p(drive, targetPoint, voltage.getVoltage());
            }
            drive.update();
            telescope.update();
            pivot.update();
            limelight.update();
            commandMachine.run(drive.isBusy() || commandBusy);
            if (loc != null) {
                telemetry.addData("limelight strafe distance", loc.translation);
                telemetry.addData("telescope extend dist", loc.extension);
            } else {
                Location l = limelight.getBest();
                telemetry.addData("limelight strafe distance", l.translation);
                telemetry.addData("telescope extend dist", l.extension);
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
