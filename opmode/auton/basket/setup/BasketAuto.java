package org.firstinspires.ftc.teamcode.opmode.auton.basket.setup;

import static org.firstinspires.ftc.teamcode.opmode.auton.util.GameConstants.*;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Pivot;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Scoring;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Telescope;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Wrist;
import org.firstinspires.ftc.teamcode.opmode.auton.util.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.stuyfission.fissionlib.command.AutoCommandMachine;
import com.stuyfission.fissionlib.command.Command;
import com.stuyfission.fissionlib.command.CommandSequence;

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
    private TrajectorySequence chamberTraj;
    private TrajectorySequence farTraj;
    private TrajectorySequence basket1Traj;
    private TrajectorySequence centerTraj;
    private TrajectorySequence basket2Traj;
    private TrajectorySequence wallTraj;
    private TrajectorySequence basket3Traj;

    private SampleMecanumDrive drive;
    private Intake intake;
    private Pivot pivot;
    private Scoring scoring;
    private Telescope telescope;
    private Wrist wrist;

    private Command intakeStartCommand = () -> intake.intake();
    private Command intakeStopCommand = () -> intake.stop();

    private Command chamberCommand = () -> drive.followTrajectorySequenceAsync(chamberTraj);
    private Command farSampleCommand = () -> drive.followTrajectorySequenceAsync(farTraj);
    private Command basket1Command = () -> drive.followTrajectorySequenceAsync(basket1Traj);
    private Command centerSampleCommand = () -> drive.followTrajectorySequenceAsync(centerTraj);
    private Command basket2Command = () -> drive.followTrajectorySequenceAsync(basket2Traj);
    private Command wallSampleCommand = () -> drive.followTrajectorySequenceAsync(wallTraj);
    private Command basket3Command = () -> drive.followTrajectorySequenceAsync(basket3Traj);

    private CommandSequence chamberSequence = new CommandSequence()
            .addCommand(chamberCommand)
            .build();
    private CommandSequence farSampleSequence = new CommandSequence()
            .addCommand(farSampleCommand)
            .build();
    private CommandSequence basket1Sequence = new CommandSequence()
            .addCommand(basket1Command)
            .build();
    private CommandSequence centerSampleSequence = new CommandSequence()
            .addCommand(centerSampleCommand)
            .build();
    private CommandSequence basket2Sequence = new CommandSequence()
            .addCommand(basket2Command)
            .build();
    private CommandSequence wallSampleSequence = new CommandSequence()
            .addCommand(wallSampleCommand)
            .build();
    private CommandSequence basket3Sequence = new CommandSequence()
            .addCommand(basket3Command)
            .build();
    private AutoCommandMachine commandMachine = new AutoCommandMachine()
            .addCommandSequence(chamberSequence)
            .addCommandSequence(farSampleSequence)
            .addCommandSequence(basket1Sequence)
            .addCommandSequence(centerSampleSequence)
            .addCommandSequence(basket2Sequence)
            .addCommandSequence(wallSampleSequence)
            .addCommandSequence(basket3Sequence)
            .build();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        reflect = color == Color.RED;
        intake = new Intake(this);
        drive = new SampleMecanumDrive(hardwareMap);
        pivot = new Pivot(this);
        scoring = new Scoring(this);
        wrist = new Wrist(this);

        intake.init(hardwareMap);
        pivot.init(hardwareMap);
        scoring.init(hardwareMap);
        wrist.init(hardwareMap);

        drive.setPoseEstimate(reflectX(BasketConstantsDash.START_POSE));
        chamberTraj = drive
                .trajectorySequenceBuilder(reflectX(BasketConstantsDash.START_POSE))
                .lineTo(basketConstants.CHAMBER.getV())
                .waitSeconds(1)
                .back(5)
                .build();

        farTraj = drive
                .trajectorySequenceBuilder(reflectX(chamberTraj.end()))
                .splineTo(basketConstants.FAR_SAMPLE.getV(), basketConstants.FAR_SAMPLE.getH())
                .waitSeconds(1)
                .build();

        basket1Traj = drive
                .trajectorySequenceBuilder(reflectX(farTraj.end()))
                .setTangent(DOWN)
                .lineToSplineHeading(new Pose2d(basketConstants.BASKET_1.getV().getX(), basketConstants.BASKET_1.getV().getY(), basketConstants.BASKET_1.getH()))
                .waitSeconds(1)
                .build();
        centerTraj = drive
                .trajectorySequenceBuilder(reflectX(basket1Traj.end()))
                .setTangent(UP)
                .splineTo(basketConstants.CENTER_SAMPLE.getV(), basketConstants.CENTER_SAMPLE.getH())
                .waitSeconds(1)
                .build();
        basket2Traj = drive
                .trajectorySequenceBuilder(reflectX(centerTraj.end()))
                .lineToSplineHeading(new Pose2d(basketConstants.BASKET_2.getV().getX(), basketConstants.BASKET_2.getV().getY(), basketConstants.BASKET_2.getH()))
                .waitSeconds(1)
                .build();
        wallTraj = drive
                .trajectorySequenceBuilder(reflectX(basket2Traj.end()))
                .setTangent(UP)
                .lineToSplineHeading(new Pose2d(basketConstants.WALL_SAMPLE.getV().getX(), basketConstants.WALL_SAMPLE.getV().getY(), basketConstants.WALL_SAMPLE.getH()))
                .waitSeconds(1)
                .build();
        basket3Traj = drive
                .trajectorySequenceBuilder(reflectX(wallTraj.end()))
                .setTangent(DOWN)
                .lineToSplineHeading(new Pose2d(basketConstants.BASKET_3.getV().getX(), basketConstants.BASKET_3.getV().getY(), basketConstants.BASKET_3.getH()))
                .build();
        waitForStart();

        while (opModeIsActive() && !isStopRequested() && !commandMachine.hasCompleted()) {
            drive.update();
//            slides.update();
            commandMachine.run(drive.isBusy() || busy);
            telemetry.update();
        }
    }
    public Pose2d reflectX(Pose2d pose) {
        if (reflect) {
            return new Pose2d(pose.getX(), pose.getY() * -1, -pose.getHeading());
        }
        return pose;
    }

    public Vector2d reflectX(Vector2d vector) {
        if (reflect) {
            return new Vector2d(vector.getX(), vector.getY() * -1);
        }
        return vector;
    }

    public double reflectX(double theta) {
        if (reflect) {
            return -theta;
        }
        return theta;
    }

}