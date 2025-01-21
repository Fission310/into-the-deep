package org.firstinspires.ftc.teamcode.opmode.auton.basketnoclip;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.stuyfission.fissionlib.command.AutoCommandMachine;
import com.stuyfission.fissionlib.command.Command;
import com.stuyfission.fissionlib.command.CommandSequence;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Claw;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Pivot;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Telescope;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Wrist;

@Autonomous(name = "BasketNoClipAuto", preselectTeleOp = "Main")
public class BasketNoClipAuto extends LinearOpMode {
    private boolean driveBusy = false;
    private boolean commandBusy = false;
    private BasketNoClipConstants basketNoClipConstants = BasketNoClipConstantsDash.basketNoClipConstants;
    private Action basket1Action;
    private Action farSampleAction;
    private Action farSampleIntAction;
    private Action basket2Action;
    private Action centerSampleAction;
    private Action centerSampleIntAction;
    private Action basket3Action;
    private Action wallSampleAction;
    private Action wallSampleIntAction;
    private Action basket4Action;

    private MecanumDrive drive;

    private Command basket1Command = () -> followActionAsync(basket1Action);
    private Command farSampleCommand = () -> followActionAsync(farSampleAction);
    private Command farSampleIntCommand = () -> followActionAsync(farSampleIntAction);
    private Command basket2Command = () -> followActionAsync(basket2Action);
    private Command centerSampleCommand = () -> followActionAsync(centerSampleAction);
    private Command centerSampleIntCommand = () -> followActionAsync(centerSampleIntAction);
    private Command basket3Command = () -> followActionAsync(basket3Action);
    private Command wallSampleCommand = () -> followActionAsync(wallSampleAction);
    private Command wallSampleIntCommand = () -> followActionAsync(wallSampleIntAction);
    private Command basket4Command = () -> followActionAsync(basket4Action);

    private void followActionAsync(Action action) {
        driveBusy = true;
        Thread thread = new Thread(
                () -> {
                    Actions.runBlocking(action);
                    driveBusy = false;
                });
        thread.start();
    }

    private Claw claw;
    private Pivot pivot;
    private Telescope telescope;
    private Wrist wrist;

    private Command commandBusyTrue = () -> commandBusy = true;
    private Command commandBusyFalse = () -> commandBusy = false;
    private Command release = () -> claw.release();
    private Command stopIntake = () -> claw.stop();
    private Command grab = () -> claw.grab();
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
    private Command clawScore = () -> claw.release();

    private CommandSequence basket1Sequence = new CommandSequence()
            .addCommand(commandBusyTrue)
            .addCommand(grab)
            .addCommand(basket1Command)
            .addCommand(wristIntakeScore)
            .addWaitCommand(1.2)
            .addCommand(pivotBasket)
            .addCommand(telescopeBasket)
            .addWaitCommand(0.8)
            .addCommand(wristBasket)
            .addCommand(grab)
            .addWaitCommand(0.6)
            .addCommand(release)
            .addWaitCommand(0.2)
            .addCommand(wristIntakeScore)
            .addWaitCommand(0.2)
            .addCommand(telescopeFront)
            .addWaitCommand(0.6)
            .addCommand(wristRetract)
            .addCommand(pivotFront)
            .addWaitCommand(0.4)
            .addCommand(pivotGrabIntake)
            .addCommand(wristIntakeScore)
            .addCommand(commandBusyFalse)
            .build();

    private CommandSequence farSampleSequence = new CommandSequence()
            .addCommand(commandBusyTrue)
            .addCommand(farSampleCommand)
            .addCommand(commandBusyTrue)
            .addWaitCommand(1)
            .addCommand(telescopeFar)
            .addCommand(pivotGrabIntake)
            .addCommand(wristIntakeScore)
            .addWaitCommand(1.1)
            .addCommand(farSampleIntCommand)
            .addCommand(grab)
            .addWaitCommand(0.6)
            .addCommand(telescopeRetract)
            .addCommand(pivotInit)
            .addCommand(commandBusyFalse)
            .build();

    private CommandSequence basket2Sequence = new CommandSequence()
            .addCommand(commandBusyTrue)
            .addCommand(grab)
            .addCommand(basket2Command)
            .addCommand(wristIntakeScore)
            .addWaitCommand(0.6)
            .addCommand(pivotBasket)
            .addCommand(telescopeBasket)
            .addWaitCommand(0.8)
            .addCommand(wristBasket)
            .addCommand(grab)
            .addWaitCommand(0.7)
            .addCommand(release)
            .addWaitCommand(0.2)
            .addCommand(wristIntakeScore)
            .addWaitCommand(0.2)
            .addCommand(telescopeFront)
            .addWaitCommand(0.6)
            .addCommand(wristRetract)
            .addCommand(pivotFront)
            .addWaitCommand(0.4)
            .addCommand(pivotGrabIntake)
            .addCommand(wristIntakeScore)
            .addCommand(commandBusyFalse)
            .build();
    private CommandSequence centerSampleSequence = new CommandSequence()
            .addCommand(commandBusyTrue)
            .addCommand(centerSampleCommand)
            .addCommand(commandBusyTrue)
            .addWaitCommand(1)
            .addCommand(telescopeCenter)
            .addCommand(pivotGrabIntake)
            .addCommand(wristIntakeScore)
            .addWaitCommand(1)
            .addCommand(centerSampleIntCommand)
            .addCommand(grab)
            .addWaitCommand(0.6)
            .addCommand(telescopeRetract)
            .addCommand(pivotInit)
            .addCommand(commandBusyFalse)
            .build();
    private CommandSequence basket3Sequence = new CommandSequence()
            .addCommand(commandBusyTrue)
            .addCommand(grab)
            .addCommand(basket3Command)
            .addCommand(wristIntakeScore)
            .addWaitCommand(0.6)
            .addCommand(pivotBasket)
            .addCommand(telescopeBasket)
            .addWaitCommand(0.8)
            .addCommand(wristBasket)
            .addCommand(grab)
            .addWaitCommand(0.7)
            .addCommand(release)
            .addWaitCommand(0.2)
            .addCommand(wristIntakeScore)
            .addWaitCommand(0.2)
            .addCommand(telescopeFront)
            .addWaitCommand(0.6)
            .addCommand(wristRetract)
            .addCommand(pivotFront)
            .addWaitCommand(0.4)
            .addCommand(pivotGrabIntake)
            .addCommand(wristIntakeScore)
            .addCommand(commandBusyFalse)
            .build();
    private CommandSequence wallSampleSequence = new CommandSequence()
            .addCommand(commandBusyTrue)
            .addCommand(wallSampleCommand)
            .addWaitCommand(2.2)
            .addCommand(telescopeWall)
            .addCommand(pivotGrabIntake)
            .addCommand(wristIntakeScore)
            .addWaitCommand(0.6)
            .addCommand(grab)
            .addWaitCommand(1)
            .addCommand(telescopeRetract)
            .addCommand(pivotInit)
            .addCommand(commandBusyFalse)
            .build();
    private CommandSequence basket4Sequence = new CommandSequence()
            .addCommand(commandBusyTrue)
            .addCommand(grab)
            .addCommand(basket4Command)
            .addCommand(wristIntakeScore)
            .addWaitCommand(2.0)
            .addCommand(pivotBasket)
            .addCommand(telescopeBasket)
            .addWaitCommand(0.8)
            .addCommand(wristBasket)
            .addCommand(grab)
            .addWaitCommand(0.7)
            .addCommand(release)
            .addWaitCommand(0.2)
            .addCommand(wristIntakeScore)
            .addWaitCommand(0.2)
            .addCommand(telescopeFront)
            .addWaitCommand(0.6)
            .addCommand(wristRetract)
            .addCommand(pivotFront)
            .addWaitCommand(0.4)
            .addCommand(pivotGrabIntake)
            .addCommand(wristIntakeScore)
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
        claw = new Claw(this);
        drive = new MecanumDrive(hardwareMap, BasketNoClipConstantsDash.START_POSE);
        telescope = new Telescope(this);
        pivot = new Pivot(this, telescope);
        wrist = new Wrist(this);

        claw.init(hardwareMap);
        pivot.init(hardwareMap);
        pivot.initPos();
        wrist.init(hardwareMap);
        telescope.init(hardwareMap);

        // TrajectoryVelocityConstraint velConstraint =
        // SampleMecanumDrive.getVelocityConstraint(20, 1, DriveConstants.TRACK_WIDTH);
        // TrajectoryAccelerationConstraint accelConstraint =
        // SampleMecanumDrive.getAccelerationConstraint(30);

        basket1Action = drive
                .actionBuilder(BasketNoClipConstantsDash.START_POSE)
                .splineToLinearHeading(basketNoClipConstants.BASKET_1.getPose(), 5 * Math.PI / 4)
                .build();
        farSampleAction = drive
                .actionBuilder(basketNoClipConstants.BASKET_1.getPose())
                .setReversed(false)
                .splineToLinearHeading(basketNoClipConstants.FAR_SAMPLE.getPose(), Math.PI / 2)
                .build();
        farSampleIntAction = drive
                .actionBuilder(basketNoClipConstants.FAR_SAMPLE.getPose())
                .setReversed(false)
                .splineToLinearHeading(basketNoClipConstants.FAR_SAMPLE_INT.getPose(), Math.PI / 2)
                .build();
        basket2Action = drive
                .actionBuilder(basketNoClipConstants.FAR_SAMPLE_INT.getPose())
                .setReversed(true)
                .splineToLinearHeading(basketNoClipConstants.BASKET_2.getPose(), 5 * Math.PI / 4)
                .build();
        centerSampleAction = drive
                .actionBuilder(basketNoClipConstants.BASKET_2.getPose())
                .setReversed(false)
                .splineToLinearHeading(basketNoClipConstants.CENTER_SAMPLE.getPose(), Math.PI / 2)
                .build();
        centerSampleIntAction = drive
                .actionBuilder(basketNoClipConstants.CENTER_SAMPLE.getPose())
                .setReversed(false)
                .splineToLinearHeading(basketNoClipConstants.CENTER_SAMPLE_INT.getPose(), Math.PI / 2)
                .build();
        basket3Action = drive
                .actionBuilder(basketNoClipConstants.CENTER_SAMPLE_INT.getPose())
                .setReversed(true)
                .splineToLinearHeading(basketNoClipConstants.BASKET_3.getPose(), 5 * Math.PI / 4)
                .build();
        wallSampleAction = drive
                .actionBuilder(basketNoClipConstants.BASKET_3.getPose())
                .setReversed(false)
                .splineToLinearHeading(basketNoClipConstants.WALL_SAMPLE.getPose(), Math.PI / 2)
                .build();
        wallSampleIntAction = drive
                .actionBuilder(basketNoClipConstants.WALL_SAMPLE.getPose())
                .setReversed(false)
                .splineToLinearHeading(basketNoClipConstants.WALL_SAMPLE_INT.getPose(), Math.PI / 2)
                .build();
        basket4Action = drive
                .actionBuilder(basketNoClipConstants.WALL_SAMPLE.getPose())
                .setReversed(true)
                .splineToLinearHeading(basketNoClipConstants.BASKET_4.getPose(), 5 * Math.PI / 4)
                .build();

        while (opModeInInit() && !isStopRequested()) {
            drive.updatePoseEstimate();
            telemetry.addData("drive x", drive.localizer.getPose().position.x);
            telemetry.addData("drive y", drive.localizer.getPose().position.y);
            telemetry.update();
            pivot.update();
        }

        drive.localizer.setPose(BasketNoClipConstantsDash.START_POSE);

        waitForStart();

        while (opModeIsActive() && !isStopRequested() && !commandMachine.hasCompleted()) {
            telescope.update();
            pivot.update();
            commandMachine.run(driveBusy || commandBusy);
            telemetry.addData("drive x", drive.localizer.getPose().position.x);
            telemetry.addData("drive y", drive.localizer.getPose().position.y);
            telemetry.update();
        }

        Thread.sleep(500);
    }
}
