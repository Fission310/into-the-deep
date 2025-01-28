package org.firstinspires.ftc.teamcode.opmode.auton.basketnoclip;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.stuyfission.fissionlib.command.AutoCommandMachine;
import com.stuyfission.fissionlib.command.Command;
import com.stuyfission.fissionlib.command.CommandSequence;

import org.firstinspires.ftc.teamcode.hardware.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Pivot;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Telescope;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Wrist;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "BasketNoClipAuto", preselectTeleOp = "Main")
public class BasketNoClipAuto extends LinearOpMode {
    private boolean commandBusy = false;
    private BasketNoClipConstants basketNoClipConstants = BasketNoClipConstantsDash.basketNoClipConstants;
    private PathChain basket1Traj;
    private PathChain farSampleTraj;
    private PathChain farSampleIntTraj;
    private PathChain basket2Traj;
    private PathChain centerSampleTraj;
    private PathChain centerSampleIntTraj;
    private PathChain basket3Traj;
    private PathChain wallSampleTraj;
    private PathChain wallSampleIntTraj;
    private PathChain basket4Traj;

    private Follower drive;

    private Command basket1Command = () -> drive.followPath(basket1Traj, true);
    private Command farSampleCommand = () -> drive.followPath(farSampleTraj, true);
    private Command farSampleIntCommand = () -> drive.followPath(farSampleIntTraj, true);
    private Command basket2Command = () -> drive.followPath(basket2Traj, true);
    private Command centerSampleCommand = () -> drive.followPath(centerSampleTraj, true);
    private Command centerSampleIntCommand = () -> drive.followPath(centerSampleIntTraj, true);
    private Command basket3Command = () -> drive.followPath(basket3Traj, true);
    private Command wallSampleCommand = () -> drive.followPath(wallSampleTraj, true);
    private Command wallSampleIntCommand = () -> drive.followPath(wallSampleIntTraj, true);
    private Command basket4Command = () -> drive.followPath(basket4Traj, true);

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
            /*.addCommand(intakeCommand)
            .addCommand(wristIntakeScore)
            .addWaitCommand(1.2)
            .addCommand(pivotBasket)
            .addCommand(telescopeBasket)
            .addWaitCommand(0.8)
            .addCommand(wristBasket)
            .addCommand(intakeCommand)
            .addWaitCommand(0.6)
            .addCommand(outtake)
            .addWaitCommand(0.2)
            .addCommand(wristIntakeScore)
            .addWaitCommand(0.2)
            .addCommand(telescopeFront)
            .addWaitCommand(0.6)
            .addCommand(wristRetract)
            .addCommand(pivotFront)
            .addWaitCommand(0.4)
            .addCommand(pivotGrabIntake)
            .addCommand(wristIntakeScore)*/
            .addCommand(commandBusyFalse)
            .build();

    private CommandSequence farSampleSequence = new CommandSequence()
            .addCommand(commandBusyTrue)
            .addCommand(farSampleCommand)
            /*.addWaitCommand(1)
            .addCommand(telescopeFar)
            .addCommand(pivotGrabIntake)
            .addCommand(wristIntakeScore)
            .addWaitCommand(1.1)
            .addCommand(farSampleIntCommand)
            .addCommand(intakeCommand)
            .addWaitCommand(0.6)
            .addCommand(telescopeRetract)
            .addCommand(pivotInit)*/
            .addCommand(commandBusyFalse)
            .build();

    private CommandSequence basket2Sequence = new CommandSequence()
            .addCommand(commandBusyTrue)
            .addCommand(basket2Command)
            /*.addCommand(intakeCommand)
            .addCommand(wristIntakeScore)
            .addWaitCommand(0.6)
            .addCommand(pivotBasket)
            .addCommand(telescopeBasket)
            .addWaitCommand(0.8)
            .addCommand(wristBasket)
            .addCommand(intakeCommand)
            .addWaitCommand(0.7)
            .addCommand(outtake)
            .addWaitCommand(0.2)
            .addCommand(wristIntakeScore)
            .addWaitCommand(0.2)
            .addCommand(telescopeFront)
            .addWaitCommand(0.6)
            .addCommand(wristRetract)
            .addCommand(pivotFront)
            .addWaitCommand(0.4)
            .addCommand(pivotGrabIntake)
            .addCommand(wristIntakeScore)*/
            .addCommand(commandBusyFalse)
            .build();
    private CommandSequence centerSampleSequence = new CommandSequence()
            .addCommand(commandBusyTrue)
            .addCommand(centerSampleCommand)
            /*.addWaitCommand(1)
            .addCommand(telescopeCenter)
            .addCommand(pivotGrabIntake)
            .addCommand(wristIntakeScore)
            .addWaitCommand(1)
            .addCommand(centerSampleIntCommand)
            .addCommand(intakeCommand)
            .addWaitCommand(0.6)
            .addCommand(telescopeRetract)
            .addCommand(pivotInit)*/
            .addCommand(commandBusyFalse)
            .build();
    private CommandSequence basket3Sequence = new CommandSequence()
            .addCommand(commandBusyTrue)
            .addCommand(basket3Command)
            /*.addCommand(intakeCommand)
            .addCommand(wristIntakeScore)
            .addWaitCommand(0.6)
            .addCommand(pivotBasket)
            .addCommand(telescopeBasket)
            .addWaitCommand(0.8)
            .addCommand(wristBasket)
            .addCommand(intakeCommand)
            .addWaitCommand(0.7)
            .addCommand(outtake)
            .addWaitCommand(0.2)
            .addCommand(wristIntakeScore)
            .addWaitCommand(0.2)
            .addCommand(telescopeFront)
            .addWaitCommand(0.6)
            .addCommand(wristRetract)
            .addCommand(pivotFront)
            .addWaitCommand(0.4)
            .addCommand(pivotGrabIntake)
            .addCommand(wristIntakeScore)*/
            .addCommand(commandBusyFalse)
            .build();
    private CommandSequence wallSampleSequence = new CommandSequence()
            .addCommand(commandBusyTrue)
            .addCommand(wallSampleCommand)
            /*.addWaitCommand(2.2)
            .addCommand(telescopeWall)
            .addCommand(pivotGrabIntake)
            .addCommand(wristIntakeScore)
            .addWaitCommand(0.6)
            .addCommand(intakeCommand)
            .addWaitCommand(1)
            .addCommand(telescopeRetract)
            .addCommand(pivotInit)*/
            .addCommand(commandBusyFalse)
            .build();
    private CommandSequence basket4Sequence = new CommandSequence()
            .addCommand(commandBusyTrue)
            .addCommand(basket4Command)
            /*.addCommand(intakeCommand)
            .addCommand(wristIntakeScore)
            .addWaitCommand(2.0)
            .addCommand(pivotBasket)
            .addCommand(telescopeBasket)
            .addWaitCommand(0.8)
            .addCommand(wristBasket)
            .addCommand(intakeCommand)
            .addWaitCommand(0.7)
            .addCommand(outtake)
            .addWaitCommand(0.2)
            .addCommand(wristIntakeScore)
            .addWaitCommand(0.2)
            .addCommand(telescopeFront)
            .addWaitCommand(0.6)
            .addCommand(wristRetract)
            .addCommand(pivotFront)
            .addWaitCommand(0.4)
            .addCommand(pivotGrabIntake)
            .addCommand(wristIntakeScore)*/
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
        drive = new Follower(hardwareMap);
        telescope = new Telescope(this);
        pivot = new Pivot(this, telescope);
        wrist = new Wrist(this);

        Constants.setConstants(FConstants.class, LConstants.class);
        intake.init(hardwareMap);
        pivot.init(hardwareMap);
        pivot.initPos();
        wrist.init(hardwareMap);
        telescope.init(hardwareMap);

        basket1Traj = drive
                .pathBuilder()
//                .addBezierCurve(BasketNoClipConstantsDash.START_POSE)
                //.splineToLinearHeading(basketNoClipConstants.BASKET_1.getPose(), 5 * Math.PI / 4)
                .build();
        farSampleTraj = drive
                .pathBuilder()
                //.trajectorySequenceBuilder(basketNoClipConstants.BASKET_1.getPose())
                //.setReversed(false)
                //.splineToLinearHeading(basketNoClipConstants.FAR_SAMPLE.getPose(), Math.PI / 2)
                .build();
        farSampleIntTraj = drive
                .pathBuilder()
                //.pathBuilder(basketNoClipConstants.FAR_SAMPLE.getPose())
                //.setReversed(false)
                //.splineToLinearHeading(basketNoClipConstants.FAR_SAMPLE_INT.getPose(), Math.PI / 2)
                .build();
        basket2Traj = drive
                .pathBuilder()
                //.trajectorySequenceBuilder(basketNoClipConstants.FAR_SAMPLE_INT.getPose())
                //.setReversed(true)
                //.splineToLinearHeading(basketNoClipConstants.BASKET_2.getPose(), 5 * Math.PI / 4)
                .build();
        centerSampleTraj = drive
                .pathBuilder()
                //.trajectorySequenceBuilder(basketNoClipConstants.BASKET_2.getPose())
                //.setReversed(false)
                //.splineToLinearHeading(basketNoClipConstants.CENTER_SAMPLE.getPose(), Math.PI / 2)
                .build();
        centerSampleIntTraj = drive
                .pathBuilder()
                //.trajectorySequenceBuilder(basketNoClipConstants.CENTER_SAMPLE.getPose())
                //.setReversed(false)
                //.splineToLinearHeading(basketNoClipConstants.CENTER_SAMPLE_INT.getPose(), Math.PI / 2)
                .build();
        basket3Traj = drive
                .pathBuilder()
                //.trajectorySequenceBuilder(basketNoClipConstants.CENTER_SAMPLE_INT.getPose())
                //.setReversed(true)
                //.splineToLinearHeading(basketNoClipConstants.BASKET_3.getPose(), 5 * Math.PI / 4)
                .build();
        wallSampleTraj = drive
                .pathBuilder()
                //.trajectorySequenceBuilder(basketNoClipConstants.BASKET_3.getPose())
                //.setReversed(false)
                //.splineToLinearHeading(basketNoClipConstants.WALL_SAMPLE.getPose(), Math.PI / 2)
                .build();
        wallSampleIntTraj = drive
                .pathBuilder()
                //.trajectorySequenceBuilder(basketNoClipConstants.WALL_SAMPLE.getPose())
                //.setReversed(false)
                //.splineToLinearHeading(basketNoClipConstants.WALL_SAMPLE_INT.getPose(), Math.PI / 2)
                .build();
        basket4Traj = drive
                .pathBuilder()
                //.trajectorySequenceBuilder(basketNoClipConstants.WALL_SAMPLE.getPose())
                //.setReversed(true)
                //.splineToLinearHeading(basketNoClipConstants.BASKET_4.getPose(), 5 * Math.PI / 4)
                .build();

        while (opModeInInit() && !isStopRequested()) {
            drive.update();
            telemetry.addData("drive x", drive.getPose().getX());
            telemetry.addData("drive y", drive.getPose().getY());
            telemetry.update();
            pivot.update();
        }

        drive.setStartingPose(BasketNoClipConstantsDash.START_POSE);

        waitForStart();

        while (opModeIsActive() && !isStopRequested() && !commandMachine.hasCompleted()) {
            drive.update();
            telescope.update();
            pivot.update();
            commandMachine.run(drive.isBusy() || commandBusy);
            telemetry.addData("drive x", drive.getPose().getX());
            telemetry.addData("drive y", drive.getPose().getY());
            telemetry.update();
        }

        Thread.sleep(500);
    }
}
