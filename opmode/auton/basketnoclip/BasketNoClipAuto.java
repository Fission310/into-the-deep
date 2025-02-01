package org.firstinspires.ftc.teamcode.opmode.auton.basketnoclip;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.stuyfission.fissionlib.command.AutoCommandMachine;
import com.stuyfission.fissionlib.command.Command;
import com.stuyfission.fissionlib.command.CommandSequence;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Pivot;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Telescope;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Wrist;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "BasketNoClipAuto", preselectTeleOp = "Main")
public class BasketNoClipAuto extends LinearOpMode {
    private boolean commandBusy = false;
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

    private Intake intake;
    private Pivot pivot;
    private Telescope telescope;
    private Wrist wrist;

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
    private Command wristIntakeScore = () -> wrist.intakePos();
    private Command wristBasket = () -> wrist.basketPos();
    private Command wristClipScore = () -> wrist.clipScorePos();

    private CommandSequence basket1Sequence = new CommandSequence()
            .addCommand(commandBusyTrue)
            .addCommand(basket1Command)
            .addCommand(intakeCommand)
            .addCommand(wristIntakeScore)
            .addWaitCommand(0.6)
            .addCommand(pivotBasket)
            .addCommand(telescopeBasket)
            .addWaitCommand(0.5)
            .addCommand(wristBasket)
            .addCommand(intakeCommand)
            .addWaitCommand(0.3)
            .addCommand(outtake)
            .addWaitCommand(0.2)
            .addCommand(wristIntakeScore)
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
            .addWaitCommand(0.7)
            .addCommand(telescopeFar)
            .addCommand(wristIntakeScore)
            .addWaitCommand(0.5)
            .addCommand(farSampleIntCommand)
            .addCommand(intakeCommand)
            .addWaitCommand(0.4)
            .addCommand(telescopeRetract)
            .addCommand(pivotInit)
            .addCommand(commandBusyFalse)
            .build();

    private CommandSequence basket2Sequence = new CommandSequence()
            .addCommand(commandBusyTrue)
            .addCommand(basket2Command)
            .addCommand(intakeCommand)
            .addWaitCommand(0.6)
            .addCommand(pivotBasket)
            .addCommand(telescopeBasket)
            .addWaitCommand(0.5)
            .addCommand(wristBasket)
            .addCommand(intakeCommand)
            .addWaitCommand(0.3)
            .addCommand(outtake)
            .addWaitCommand(0.2)
            .addCommand(wristIntakeScore)
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
            .addWaitCommand(0.7)
            .addCommand(telescopeCenter)
            .addCommand(wristIntakeScore)
            .addWaitCommand(0.5)
            .addCommand(centerSampleIntCommand)
            .addCommand(intakeCommand)
            .addWaitCommand(0.4)
            .addCommand(telescopeRetract)
            .addCommand(pivotInit)
            .addCommand(commandBusyFalse)
            .build();
    private CommandSequence basket3Sequence = new CommandSequence()
            .addCommand(commandBusyTrue)
            .addCommand(basket3Command)
            .addCommand(intakeCommand)
            .addWaitCommand(0.6)
            .addCommand(pivotBasket)
            .addCommand(telescopeBasket)
            .addWaitCommand(0.5)
            .addCommand(wristBasket)
            .addCommand(intakeCommand)
            .addWaitCommand(0.3)
            .addCommand(outtake)
            .addWaitCommand(0.2)
            .addCommand(wristIntakeScore)
            .addWaitCommand(0.2)
            .addCommand(telescopeFront)
            .addWaitCommand(0.2)
            .addCommand(wristRetract)
            .addCommand(commandBusyFalse)
            .build();
    private CommandSequence wallSampleSequence = new CommandSequence()
            .addCommand(commandBusyTrue)
            .addCommand(pivotGrabIntake)
            .addCommand(wallSampleCommand)
            .addWaitCommand(0.7)
            .addCommand(telescopeWall)
            .addCommand(wristIntakeScore)
            .addWaitCommand(0.5)
            .addCommand(wallSampleIntCommand)
            .addCommand(intakeCommand)
            .addWaitCommand(0.4)
            .addCommand(telescopeRetract)
            .addCommand(pivotInit)
            .addCommand(commandBusyFalse)
            .build();
    private CommandSequence basket4Sequence = new CommandSequence()
            .addCommand(commandBusyTrue)
            .addCommand(basket1Command)
            .addCommand(intakeCommand)
            .addCommand(wristIntakeScore)
            .addWaitCommand(0.6)
            .addCommand(pivotBasket)
            .addCommand(telescopeBasket)
            .addWaitCommand(0.5)
            .addCommand(wristBasket)
            .addCommand(intakeCommand)
            .addWaitCommand(0.3)
            .addCommand(outtake)
            .addWaitCommand(0.2)
            .addCommand(wristIntakeScore)
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

        intake.init(hardwareMap);
        pivot.init(hardwareMap);
        pivot.initPos();
        wrist.init(hardwareMap);
        telescope.init(hardwareMap);

        basket1Traj = drive
                .trajectorySequenceBuilder(BasketNoClipConstants.START.getPose())
                .splineToLinearHeading(BasketNoClipConstants.BASKET_1.getPose(), 5 * Math.PI / 4)
                .build();
        farSampleTraj = drive
                .trajectorySequenceBuilder(basket1Traj.end())
                .setReversed(false)
                .splineToLinearHeading(BasketNoClipConstants.FAR_SAMPLE.getPose(), Math.PI / 2)
                .build();
        farSampleIntTraj = drive.trajectorySequenceBuilder(farSampleTraj.end())
                .setReversed(false)
                .splineToLinearHeading(BasketNoClipConstants.FAR_SAMPLE_INT.getPose(), Math.PI / 2)
                .build();
        basket2Traj = drive
                .trajectorySequenceBuilder(farSampleIntTraj.end())
                .setReversed(true)
                .splineToLinearHeading(BasketNoClipConstants.BASKET_2.getPose(), 5 * Math.PI / 4)
                .build();
        centerSampleTraj = drive
                .trajectorySequenceBuilder(basket2Traj.end())
                .setReversed(false)
                .splineToLinearHeading(BasketNoClipConstants.CENTER_SAMPLE.getPose(), Math.PI / 2)
                .build();
        centerSampleIntTraj = drive
                .trajectorySequenceBuilder(centerSampleTraj.end())
                .setReversed(false)
                .splineToLinearHeading(BasketNoClipConstants.CENTER_SAMPLE_INT.getPose(), Math.PI / 2)
                .build();
        basket3Traj = drive
                .trajectorySequenceBuilder(centerSampleIntTraj.end())
                .setReversed(true)
                .splineToLinearHeading(BasketNoClipConstants.BASKET_3.getPose(), 5 * Math.PI / 4)
                .build();
        wallSampleTraj = drive
                .trajectorySequenceBuilder(basket3Traj.end())
                .setReversed(false)
                .splineToLinearHeading(BasketNoClipConstants.WALL_SAMPLE.getPose(), Math.PI / 2)
                .build();
        wallSampleIntTraj = drive
                .trajectorySequenceBuilder(wallSampleTraj.end())
                .setReversed(false)
                .splineToLinearHeading(BasketNoClipConstants.WALL_SAMPLE_INT.getPose(), Math.PI / 2)
                .build();
        basket4Traj = drive
                .trajectorySequenceBuilder(wallSampleIntTraj.end())
                .setReversed(true)
                .splineToLinearHeading(BasketNoClipConstants.BASKET_4.getPose(), 5 * Math.PI / 4)
                .build();

        while (opModeInInit() && !isStopRequested()) {
            drive.updatePoseEstimate();
            telemetry.addData("drive x", drive.getPoseEstimate().getX());
            telemetry.addData("drive y", drive.getPoseEstimate().getY());
            telemetry.update();
            pivot.update();
        }

        drive.setPoseEstimate(BasketNoClipConstants.START.getPose());

        waitForStart();

        while (opModeIsActive() && !isStopRequested() && !commandMachine.hasCompleted()) {
            drive.update();
            telescope.update();
            pivot.update();
            commandMachine.run(drive.isBusy() || commandBusy);
            telemetry.addData("drive x", drive.getPoseEstimate().getX());
            telemetry.addData("drive y", drive.getPoseEstimate().getY());
            telemetry.update();
        }

        Thread.sleep(500);
    }
}
