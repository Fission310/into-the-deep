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
import org.firstinspires.ftc.teamcode.opmode.auton.basket.BasketConstantsDash;

@Autonomous(name = "BasketNoClipAuto", preselectTeleOp = "Main")
public class BasketNoClipAuto extends LinearOpMode{
    private boolean busy = false;
    private BasketNoClipConstants basketNoClipConstants = BasketNoClipConstantsDash.basketNoClipConstants;
    private Action basket1Action;
    private Action farSampleAction;
    private Action basket2Action;
    private Action centerSampleAction;
    private Action basket3Action;
    private Action wallSampleAction;
    private Action basket4Action;

    private MecanumDrive drive;

    private Command basket1Command = () -> followActionAsync(basket1Action);
    private Command farSampleCommand = () -> followActionAsync(farSampleAction);
    private Command basket2Command = () -> followActionAsync(basket2Action);
    private Command centerSampleCommand = () -> followActionAsync(centerSampleAction);
    private Command basket3Command = () -> followActionAsync(basket3Action);
    private Command wallSampleCommand = () -> followActionAsync(wallSampleAction);
    private Command basket4Command = () -> followActionAsync(basket4Action);

    private void followActionAsync(Action action){
        busy = true;
        Thread thread = new Thread(
                () -> {
                    Actions.runBlocking(action);
                    busy = false;
                }
        );
        thread.start();
    }

    private Claw claw;
    private Pivot pivot;
    private Telescope telescope;
    private Wrist wrist;

    private Command busyTrue = () -> busy = true;
    private Command busyFalse = () -> busy = false;
    private Command release = () -> claw.release();
    private Command grab = () -> claw.grab();
    private Command pivotFront = () -> pivot.frontPos();
    private Command pivotClip = () -> pivot.clipPos();
    private Command pivotClipDown = () -> pivot.clipDownPos();
    private Command pivotBasket = () -> pivot.autoBasketPos();
    private Command pivotUp = () -> pivot.upPos();
    private Command pivotUpIntake = () -> pivot.intakeUpPos();
    private Command pivotDownIntake = () -> pivot.intakeDownPos();
    private Command pivotGrabIntake = () -> pivot.intakeGrabPos();
    private Command telescopeFront = () -> telescope.frontPos();
    private Command telescopeBasket = () -> telescope.autoBasketPos();
    private Command telescopeIntake = () -> telescope.frontIntakePos();
    private Command telescopeScoreClip = () -> telescope.clipScorePos();
    private Command telescopeExtendClip = () -> telescope.clipExtensionPos();
    private Command telescopeRetract = () -> telescope.frontPos();
    private Command wristRetract = () -> wrist.frontPos();
    private Command wristIntakeScore = () -> wrist.intakePos();
    private Command wristClipScore = () -> wrist.clipScorePos();

    private CommandSequence basket1Sequence = new CommandSequence()
            .addCommand(basket1Command)
//            .addCommand(pivotClip)
//            .addCommand(telescopeExtendClip)
//            .addWaitCommand(1.5)
//            .addCommand(pivotClipDown)
//            .addCommand(telescopeScoreClip)
//            .addWaitCommand(1.5)
//            .addCommand(wristClipScore)
//            .addWaitCommand(0.5)
//            .addCommand(release)
//            .addWaitCommand(0.6)
//            .addCommand(pivotFront)
//            .addCommand(wristRetract)
//            .addCommand(telescopeFront)
            .build();
    private CommandSequence farSampleSequence = new CommandSequence()
            .addCommand(farSampleCommand)
//            .addCommand(busyTrue)
//            .addWaitCommand(0.3)
//            .addCommand(pivotUpIntake)
//            .addCommand(telescopeIntake)
//            .addWaitCommand(1)
//            .addCommand(wristIntakeScore)
//            .addWaitCommand(1)
//            .addCommand(pivotGrabIntake)
//            .addWaitCommand(0.3)
//            .addCommand(grab)
//            .addWaitCommand(1)
//            .addCommand(pivotUpIntake)
//            .addCommand(wristRetract)
//            .addWaitCommand(0.1)
//            .addCommand(telescopeRetract)
//            .addWaitCommand(0.51)
//            .addCommand(pivotUp)
//            .addWaitCommand(1)
//            .addCommand(busyFalse)
            .build();
    private CommandSequence basket2Sequence = new CommandSequence()
            .addCommand(basket2Command)
//            .addCommand(busyTrue)
//            .addWaitCommand(0.2)
//            .addCommand(pivotBasket)
//            .addCommand(telescopeBasket)
//            .addWaitCommand(3)
//            .addCommand(release)
//            .addWaitCommand(0.2)
//            .addCommand(wristIntakeScore)
//            .addWaitCommand(0.4)
//            .addCommand(telescopeFront)
//            .addWaitCommand(0.7)
//            .addCommand(wristRetract)
//            .addWaitCommand(0.1)
//            .addCommand(pivotFront)
//            .addWaitCommand(0.1)
//            .addCommand(grab)
//            .addWaitCommand(0.2)
//            .addCommand(busyFalse)
            .build();
    private CommandSequence centerSampleSequence = new CommandSequence()
            .addCommand(centerSampleCommand)
//            .addCommand(busyTrue)
//            .addWaitCommand(2)
//            .addCommand(pivotDownIntake)
//            .addCommand(telescopeIntake)
//            .addWaitCommand(0.1)
//            .addCommand(wristIntakeScore)
//            .addWaitCommand(1)
//            .addCommand(pivotGrabIntake)
//            .addWaitCommand(0.3)
//            .addCommand(grab)
//            .addWaitCommand(1)
//            .addCommand(pivotUpIntake)
//            .addCommand(wristRetract)
//            .addWaitCommand(0.1)
//            .addCommand(telescopeRetract)
//            .addWaitCommand(0.5)
//            .addCommand(pivotUp)
//            .addWaitCommand(1)
//            .addCommand(busyFalse)
            .build();
    private CommandSequence basket3Sequence = new CommandSequence()
            .addCommand(basket3Command)
//            .addCommand(pivotBasket)
//            .addCommand(telescopeBasket)
//            .addWaitCommand(0.5)
//            .addCommand(release)
//            .addWaitCommand(0.5)
//            .addCommand(wristIntakeScore)
//            .addWaitCommand(0.5)
//            .addCommand(telescopeFront)
//            .addWaitCommand(0.5)
//            .addCommand(wristRetract)
//            .addWaitCommand(0.5)
//            .addCommand(pivotFront)
            .build();
    private CommandSequence wallSampleSequence = new CommandSequence()
            .addCommand(wallSampleCommand)
//            .addCommand(telescopeIntake)
//            .addCommand(wristIntakeScore)
//            .addWaitCommand(0.6)
//            .addCommand(pivotDownIntake)
//            .addWaitCommand(0.6)
//            .addCommand(grab)
//            .addCommand(pivotUpIntake)
//            .addCommand(wristRetract)
//            .addWaitCommand(0.6)
//            .addCommand(telescopeRetract)
//            .addWaitCommand(0.6)
//            .addCommand(pivotUp)
            .build();
    private CommandSequence basket4Sequence = new CommandSequence()
            .addCommand(basket4Command)
//            .addCommand(pivotBasket)
//            .addCommand(telescopeBasket)
//            .addWaitCommand(0.5)
//            .addCommand(release)
//            .addWaitCommand(0.5)
//            .addCommand(wristIntakeScore)
//            .addWaitCommand(0.5)
//            .addCommand(telescopeFront)
//            .addWaitCommand(0.5)
//            .addCommand(wristRetract)
//            .addWaitCommand(0.5)
//            .addCommand(pivotFront)
            .build();
    private AutoCommandMachine commandMachine = new AutoCommandMachine()
            .addCommandSequence(basket1Sequence)
            .addCommandSequence(farSampleSequence)
            .addCommandSequence(basket2Sequence)
            .addCommandSequence(centerSampleSequence)
            .addCommandSequence(basket3Sequence)
            .addCommandSequence(wallSampleSequence)
            .addCommandSequence(basket4Sequence)
            .addCommandSequence(farSampleSequence) // we need a final one cuz fissionlib is broken
            .build();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        claw = new Claw(this);
        drive = new MecanumDrive(hardwareMap, BasketConstantsDash.START_POSE);
        telescope = new Telescope(this);
        pivot = new Pivot(this, telescope);
        wrist = new Wrist(this);

        claw.init(hardwareMap);
        pivot.init(hardwareMap);
        pivot.initPos();
        wrist.init(hardwareMap);
        telescope.init(hardwareMap);


//        TrajectoryVelocityConstraint velConstraint = SampleMecanumDrive.getVelocityConstraint(20, 1, DriveConstants.TRACK_WIDTH);
//        TrajectoryAccelerationConstraint accelConstraint = SampleMecanumDrive.getAccelerationConstraint(30);

        basket1Action = drive
                .actionBuilder(BasketNoClipConstantsDash.START_POSE)
                .splineToLinearHeading(basketNoClipConstants.BASKET_1.getPose(), 5 * Math.PI / 4)
                .build();
        farSampleAction = drive
                .actionBuilder(basketNoClipConstants.BASKET_1.getPose())
                .setReversed(false)
                .splineToLinearHeading(basketNoClipConstants.FAR_SAMPLE.getPose(), Math.PI / 2)
                .splineToLinearHeading(basketNoClipConstants.FAR_SAMPLE_INT.getPose(), Math.PI / 2)
                .build();
        basket2Action = drive
                .actionBuilder(basketNoClipConstants.FAR_SAMPLE.getPose())
                .setReversed(true)
                .splineToLinearHeading(basketNoClipConstants.BASKET_2.getPose(), 5 * Math.PI / 4)
                .build();
        centerSampleAction = drive
                .actionBuilder(basketNoClipConstants.BASKET_2.getPose())
                .setReversed(false)
                .splineToLinearHeading(basketNoClipConstants.CENTER_SAMPLE.getPose(), Math.PI / 2)
                .splineToLinearHeading(basketNoClipConstants.CENTER_SAMPLE_INT.getPose(), Math.PI / 2)
                .build();
        basket3Action = drive
                .actionBuilder(basketNoClipConstants.CENTER_SAMPLE.getPose())
                .setReversed(true)
                .splineToLinearHeading(basketNoClipConstants.BASKET_3.getPose(), 5 * Math.PI / 4)
                .build();
        wallSampleAction = drive
                .actionBuilder(basketNoClipConstants.BASKET_3.getPose())
                .setReversed(false)
                .splineToLinearHeading(basketNoClipConstants.WALL_SAMPLE.getPose(), Math.PI / 2)
                .splineToLinearHeading(basketNoClipConstants.WALL_SAMPLE_INT.getPose(), Math.PI / 2)
                .build();
        basket4Action = drive
                .actionBuilder(basketNoClipConstants.WALL_SAMPLE.getPose())
                .setReversed(true)
                .splineToLinearHeading(basketNoClipConstants.BASKET_3.getPose(), 5 * Math.PI / 4)
                .build();

        while(opModeInInit() && !isStopRequested()){
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
            commandMachine.run(busy);
            telemetry.addData("drive x", drive.localizer.getPose().position.x);
            telemetry.addData("drive y", drive.localizer.getPose().position.y);
            telemetry.update();
        }

        Thread.sleep(500);
    }
}
