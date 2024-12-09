package org.firstinspires.ftc.teamcode.opmode.auton.basket.setup;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Claw;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Pivot;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Scoring;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Telescope;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Wrist;
import org.firstinspires.ftc.teamcode.opmode.auton.util.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.fasterxml.jackson.databind.JsonSerializer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.stuyfission.fissionlib.command.AutoCommandMachine;
import com.stuyfission.fissionlib.command.Command;
import com.stuyfission.fissionlib.command.CommandSequence;

import org.firstinspires.ftc.teamcode.opmode.auton.util.Constant;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class BasketAuto extends LinearOpMode{
    private boolean reflect;
    private boolean busy = false;
    private Color color;
    private BasketConstants basketConstants;
    public BasketAuto(Color color, BasketConstants basketConstants){
        this.color = color;
        this.basketConstants = basketConstants;
    }

    private TrajectorySequence basketTraj;
    private TrajectorySequence chamberTraj;
    private TrajectorySequence farTraj;
    private TrajectorySequence basket1Traj;
    private TrajectorySequence centerTraj;
    private TrajectorySequence basket2Traj;
    private TrajectorySequence wallTraj;
    private TrajectorySequence basket3Traj;

    private SampleMecanumDrive drive;

    private Command basketCommand = () -> drive.followTrajectorySequenceAsync(basketTraj);
    private Command chamberCommand = () -> drive.followTrajectorySequenceAsync(chamberTraj);
    private Command farSampleCommand = () -> drive.followTrajectorySequenceAsync(farTraj);
    private Command basket1Command = () -> drive.followTrajectorySequenceAsync(basket1Traj);
    private Command centerSampleCommand = () -> drive.followTrajectorySequenceAsync(centerTraj);
    private Command basket2Command = () -> drive.followTrajectorySequenceAsync(basket2Traj);
    private Command wallSampleCommand = () -> drive.followTrajectorySequenceAsync(wallTraj);
    private Command basket3Command = () -> drive.followTrajectorySequenceAsync(basket3Traj);


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
        drive = new SampleMecanumDrive(hardwareMap);
        pivot = new Pivot(this);
        telescope = new Telescope(this);
        wrist = new Wrist(this);

        claw.init(hardwareMap);
        pivot.init(hardwareMap);
        pivot.initPos();
        wrist.init(hardwareMap);
        telescope.init(hardwareMap);

        TrajectoryVelocityConstraint velConstraint = SampleMecanumDrive.getVelocityConstraint(20, 1, DriveConstants.TRACK_WIDTH);
        TrajectoryAccelerationConstraint accelConstraint = SampleMecanumDrive.getAccelerationConstraint(30);

        basketTraj = drive
                 .trajectorySequenceBuilder(reflect(BasketConstantsDash.START_POSE))
                 .lineToConstantHeading(reflect(basketConstants.FORWARD.getV()))
                 .lineToLinearHeading(vecToPose(basketConstants.BASKET))
                 .build();
//        chamberTraj = drive
//                .trajectorySequenceBuilder(reflect(BasketConstantsDash.START_POSE))
//                .lineTo(reflect(basketConstants.CHAMBER.getV()))
//                .setReversed(true)
//                .build();
        farTraj = drive
                .trajectorySequenceBuilder(reflect(basketTraj.end()))
                .splineToLinearHeading(vecToPose(basketConstants.FAR_SAMPLE), reflect(basketConstants.FAR_SAMPLE.getH()), velConstraint, accelConstraint)
                .build();
        basket1Traj = drive
                .trajectorySequenceBuilder(reflect(farTraj.end()))
                .lineToSplineHeading(vecToPose(basketConstants.BASKET_1), velConstraint, accelConstraint)
                .build();
        centerTraj = drive
                .trajectorySequenceBuilder(reflect(basket1Traj.end()))
                .lineToSplineHeading(vecToPose(basketConstants.CENTER_SAMPLE))
                .build();
        basket2Traj = drive
                .trajectorySequenceBuilder(reflect(centerTraj.end()))
                .lineToSplineHeading(vecToPose(basketConstants.BASKET_2))
                .build();
        wallTraj = drive
                .trajectorySequenceBuilder(reflect(basket2Traj.end()))
                .lineToSplineHeading(vecToPose(basketConstants.WALL_SAMPLE))
                .build();
        basket3Traj = drive
                .trajectorySequenceBuilder(reflect(wallTraj.end()))
                .lineToSplineHeading(vecToPose(basketConstants.BASKET_3))
                .build();

        drive.setPoseEstimate(reflect(BasketConstantsDash.START_POSE));

        while(opModeInInit() && !isStopRequested()){
            drive.update();
            telemetry.addData("drive x", drive.getPoseEstimate().getX());
            telemetry.addData("drive y", drive.getPoseEstimate().getY());
            telemetry.update();
            pivot.update();
        }

        drive.setPoseEstimate(reflect(BasketConstantsDash.START_POSE));

        waitForStart();

        while (opModeIsActive() && !isStopRequested() && !commandMachine.hasCompleted()) {
            drive.update();
            telescope.update();
            pivot.update();
            commandMachine.run(drive.isBusy() || busy);
            telemetry.addData("drive x", drive.getPoseEstimate().getX());
            telemetry.addData("drive y", drive.getPoseEstimate().getY());
            telemetry.update();
        }

        Thread.sleep(500);
    }

    public Pose2d reflect(Pose2d pose) {
        if (reflect) {
            return new Pose2d(pose.getX() * -1, pose.getY() * -1, reflect(pose.getHeading()));
        }
        return pose;
    }

    public Vector2d reflect(Vector2d vector) {
        if (reflect) {
            return new Vector2d(vector.getX() * -1, vector.getY() * -1);
        }
        return vector;
    }

    public double reflect(double theta) {
        if (reflect) {
            return Math.toRadians(180) + theta;
        }
        return theta;
    }

    public Pose2d vecToPose(Constant constant){
        Vector2d vec = reflect(constant.getV());
        return new Pose2d(vec.getX(), vec.getY(), reflect(constant.getH()));
    }
}
