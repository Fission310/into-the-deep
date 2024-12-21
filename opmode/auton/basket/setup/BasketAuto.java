package org.firstinspires.ftc.teamcode.opmode.auton.basket.setup;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Claw;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Pivot;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Telescope;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Wrist;
import org.firstinspires.ftc.teamcode.opmode.auton.util.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.stuyfission.fissionlib.command.AutoCommandMachine;
import com.stuyfission.fissionlib.command.Command;
import com.stuyfission.fissionlib.command.CommandSequence;
import org.firstinspires.ftc.teamcode.opmode.auton.util.Constant;
import com.acmerobotics.roadrunner.Pose2d;

public class BasketAuto extends LinearOpMode{
    private boolean reflect;
    private boolean busy = false;
    private Color color;
    private BasketConstants basketConstants;
    public BasketAuto(Color color, BasketConstants basketConstants){
        this.color = color;
        this.basketConstants = basketConstants;
    }

    private Action basketTraj;
    private Action chamberTraj;
    private Action farTraj;
    private Action basket1Traj;
    private Action centerTraj;
    private Action basket2Traj;
    private Action wallTraj;
    private Action basket3Traj;

    private MecanumDrive drive;

    private Command basketCommand = () -> Actions.runBlocking(basketTraj);
    private Command chamberCommand = () -> Actions.runBlocking(chamberTraj);
    private Command farSampleCommand = () -> Actions.runBlocking(farTraj);
    private Command basket1Command = () -> Actions.runBlocking(basket1Traj);
    private Command centerSampleCommand = () -> Actions.runBlocking(centerTraj);
    private Command basket2Command = () -> Actions.runBlocking(basket2Traj);
    private Command wallSampleCommand = () -> Actions.runBlocking(wallTraj);
    private Command basket3Command = () -> Actions.runBlocking(basket3Traj);


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

    private CommandSequence basketSequence = new CommandSequence()
            .addCommand(busyTrue)
            .addWaitCommand(5)
            .addCommand(basketCommand)
            .addCommand(pivotUp)
            .addWaitCommand(7)
            .addCommand(pivotBasket)
            .addCommand(telescopeBasket)
            .addWaitCommand(2)
            .addCommand(release)
            .addWaitCommand(0.2)
            .addCommand(wristIntakeScore)
            .addWaitCommand(0.4)
            .addCommand(telescopeFront)
            .addWaitCommand(0.7)
            .addCommand(wristRetract)
            .addWaitCommand(0.1)
            .addCommand(pivotFront)
            .addWaitCommand(0.3)
            .addCommand(busyFalse)
            .build();
    private CommandSequence chamberSequence = new CommandSequence()
            .addCommand(chamberCommand)
            .addCommand(pivotClip)
            .addCommand(telescopeExtendClip)
            .addWaitCommand(1.5)
            .addCommand(pivotClipDown)
            .addCommand(telescopeScoreClip)
            .addWaitCommand(1.5)
            .addCommand(wristClipScore)
            .addWaitCommand(0.5)
            .addCommand(release)
            .addWaitCommand(0.6)
            .addCommand(pivotFront)
            .addCommand(wristRetract)
            .addCommand(telescopeFront)
            .build();
    private CommandSequence farSampleSequence = new CommandSequence()
            .addCommand(farSampleCommand)
            .addCommand(busyTrue)
            .addWaitCommand(0.3)
            .addCommand(pivotUpIntake)
            .addCommand(telescopeIntake)
            .addWaitCommand(1)
            .addCommand(wristIntakeScore)
            .addWaitCommand(1)
            .addCommand(pivotGrabIntake)
            .addWaitCommand(0.3)
            .addCommand(grab)
            .addWaitCommand(1)
            .addCommand(pivotUpIntake)
            .addCommand(wristRetract)
            .addWaitCommand(0.1)
            .addCommand(telescopeRetract)
            .addWaitCommand(0.51)
            .addCommand(pivotUp)
            .addWaitCommand(1)
            .addCommand(busyFalse)
            .build();
    private CommandSequence basket1Sequence = new CommandSequence()
            .addCommand(basket1Command)
            .addCommand(busyTrue)
            .addWaitCommand(0.2)
            .addCommand(pivotBasket)
            .addCommand(telescopeBasket)
            .addWaitCommand(3)
            .addCommand(release)
            .addWaitCommand(0.2)
            .addCommand(wristIntakeScore)
            .addWaitCommand(0.4)
            .addCommand(telescopeFront)
            .addWaitCommand(0.7)
            .addCommand(wristRetract)
            .addWaitCommand(0.1)
            .addCommand(pivotFront)
            .addWaitCommand(0.1)
            .addCommand(grab)
            .addWaitCommand(0.2)
            .addCommand(busyFalse)
            .build();
    private CommandSequence centerSampleSequence = new CommandSequence()
            .addCommand(centerSampleCommand)
            .addCommand(busyTrue)
            .addWaitCommand(2)
            .addCommand(pivotDownIntake)
            .addCommand(telescopeIntake)
            .addWaitCommand(0.1)
            .addCommand(wristIntakeScore)
            .addWaitCommand(1)
            .addCommand(pivotGrabIntake)
            .addWaitCommand(0.3)
            .addCommand(grab)
            .addWaitCommand(1)
            .addCommand(pivotUpIntake)
            .addCommand(wristRetract)
            .addWaitCommand(0.1)
            .addCommand(telescopeRetract)
            .addWaitCommand(0.5)
            .addCommand(pivotUp)
            .addWaitCommand(1)
            .addCommand(busyFalse)
            .build();
    private CommandSequence basket2Sequence = new CommandSequence()
            .addCommand(basket2Command)
            .addCommand(pivotBasket)
            .addCommand(telescopeBasket)
            .addWaitCommand(0.5)
            .addCommand(release)
            .addWaitCommand(0.5)
            .addCommand(wristIntakeScore)
            .addWaitCommand(0.5)
            .addCommand(telescopeFront)
            .addWaitCommand(0.5)
            .addCommand(wristRetract)
            .addWaitCommand(0.5)
            .addCommand(pivotFront)
            .build();
    private CommandSequence wallSampleSequence = new CommandSequence()
            .addCommand(wallSampleCommand)
            .addCommand(telescopeIntake)
            .addCommand(wristIntakeScore)
            .addWaitCommand(0.6)
            .addCommand(pivotDownIntake)
            .addWaitCommand(0.6)
            .addCommand(grab)
            .addCommand(pivotUpIntake)
            .addCommand(wristRetract)
            .addWaitCommand(0.6)
            .addCommand(telescopeRetract)
            .addWaitCommand(0.6)
            .addCommand(pivotUp)
            .build();
    private CommandSequence basket3Sequence = new CommandSequence()
            .addCommand(basket3Command)
            .addCommand(pivotBasket)
            .addCommand(telescopeBasket)
            .addWaitCommand(0.5)
            .addCommand(release)
            .addWaitCommand(0.5)
            .addCommand(wristIntakeScore)
            .addWaitCommand(0.5)
            .addCommand(telescopeFront)
            .addWaitCommand(0.5)
            .addCommand(wristRetract)
            .addWaitCommand(0.5)
            .addCommand(pivotFront)
            .build();
    private AutoCommandMachine commandMachine = new AutoCommandMachine()
//            .addCommandSequence(chamberSequence)
            .addCommandSequence(basketSequence)
            .addCommandSequence(farSampleSequence)
            .addCommandSequence(basket1Sequence)
//            .addCommandSequence(centerSampleSequence)
//            .addCommandSequence(basket2Sequence)
//            .addCommandSequence(wallSampleSequence)
//            .addCommandSequence(basket3Sequence)
            .addCommandSequence(farSampleSequence)
            .build();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        reflect = color == Color.BLUE;
        claw = new Claw(this);
        drive = new MecanumDrive(hardwareMap, BasketConstantsDash.START_POSE);
        pivot = new Pivot(this, telescope);
        telescope = new Telescope(this);
        wrist = new Wrist(this);

        claw.init(hardwareMap);
        pivot.init(hardwareMap);
        pivot.initPos();
        wrist.init(hardwareMap);
        telescope.init(hardwareMap);

        basketTraj = drive
                 .actionBuilder(BasketConstantsDash.START_POSE)
                 .lineToXConstantHeading(basketConstants.FORWARD.getV().x)
                 .lineToYConstantHeading(basketConstants.FORWARD.getV().y)
                 .lineToXLinearHeading(basketConstants.BASKET.X_POS, basketConstants.BASKET.HEADING)
                 .lineToYLinearHeading(basketConstants.BASKET.Y_POS, basketConstants.BASKET.HEADING)
                 .build();
//        chamberTraj = drive
//                .trajectorySequenceBuilder(reflect(BasketConstantsDash.START_POSE))
//                .lineTo(reflect(basketConstants.CHAMBER.getV()))
//                .setReversed(true)
//                .build();
        farTraj = drive
                .actionBuilder(drive.pose)
                .splineToLinearHeading(vecToPose(basketConstants.FAR_SAMPLE), basketConstants.FAR_SAMPLE.getH())
                .build();
        basket1Traj = drive
                .actionBuilder(drive.pose)
                .lineToXSplineHeading(basketConstants.BASKET_1.X_POS, basketConstants.BASKET_1.HEADING)
                .lineToYLinearHeading(basketConstants.BASKET_1.Y_POS, basketConstants.BASKET_1.HEADING)
                .build();
        centerTraj = drive
                .actionBuilder(drive.pose)
                .lineToXSplineHeading(basketConstants.CENTER_SAMPLE.X_POS, basketConstants.CENTER_SAMPLE.HEADING)
                .lineToYLinearHeading(basketConstants.CENTER_SAMPLE.Y_POS, basketConstants.CENTER_SAMPLE.HEADING)
                .build();
        basket2Traj = drive
                .actionBuilder(drive.pose)
                .lineToXSplineHeading(basketConstants.BASKET_2.X_POS, basketConstants.BASKET_2.HEADING)
                .lineToYLinearHeading(basketConstants.BASKET_2.Y_POS, basketConstants.BASKET_2.HEADING)
                .build();
        wallTraj = drive
                .actionBuilder(drive.pose)
                .lineToXSplineHeading(basketConstants.WALL_SAMPLE.X_POS, basketConstants.WALL_SAMPLE.HEADING)
                .lineToYLinearHeading(basketConstants.WALL_SAMPLE.Y_POS, basketConstants.WALL_SAMPLE.HEADING)
                .build();
        basket3Traj = drive
                .actionBuilder(drive.pose)
                .lineToXSplineHeading(basketConstants.BASKET_3.X_POS, basketConstants.BASKET_3.HEADING)
                .lineToYLinearHeading(basketConstants.BASKET_3.Y_POS, basketConstants.BASKET_3.HEADING)
                .build();

        while(opModeInInit() && !isStopRequested()){
            telemetry.addData("drive x", drive.pose.position.x);
            telemetry.addData("drive y", drive.pose.position.y);
            telemetry.update();
            pivot.update();
        }

        waitForStart();

        while (opModeIsActive() && !isStopRequested() && !commandMachine.hasCompleted()) {
            telescope.update();
            pivot.update();
            commandMachine.run(busy);
            telemetry.addData("drive x", drive.pose.position.x);
            telemetry.addData("drive y", drive.pose.position.y);
            telemetry.update();
            drive.updatePoseEstimate();
        }

        Thread.sleep(500);
    }

    public Pose2d vecToPose(Constant constant){
        Vector2d vec = constant.getV();
        return new Pose2d(vec.x, vec.y, constant.getH());
    }
}
