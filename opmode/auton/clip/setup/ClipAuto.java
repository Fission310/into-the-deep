package org.firstinspires.ftc.teamcode.opmode.auton.clip.setup;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Claw;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Pivot;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Scoring;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Telescope;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Wrist;
import org.firstinspires.ftc.teamcode.opmode.auton.clip.setup.ClipConstants;
import org.firstinspires.ftc.teamcode.opmode.auton.util.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.stuyfission.fissionlib.command.AutoCommandMachine;
import com.stuyfission.fissionlib.command.Command;
import com.stuyfission.fissionlib.command.CommandSequence;

import org.firstinspires.ftc.teamcode.opmode.auton.util.Constant;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class ClipAuto extends LinearOpMode{
    private boolean reflect;
    private boolean busy = false;
    private Color color;
    private ClipConstants clipConstants;
    public ClipAuto(Color color, ClipConstants clipConstants){
        this.color = color;
        this.clipConstants = clipConstants;
    }

    private TrajectorySequence chamberTraj;
    private TrajectorySequence farTraj;
    private TrajectorySequence observation1Traj;
    private TrajectorySequence centerTraj;
    private TrajectorySequence observation2Traj;
    private TrajectorySequence wallTraj;
    private TrajectorySequence observation3Traj;
    private TrajectorySequence wallIntakeTraj;
    private SampleMecanumDrive drive;

    private Command chamberCommand = () -> drive.followTrajectorySequenceAsync(chamberTraj);
    private Command farSampleCommand = () -> drive.followTrajectorySequenceAsync(farTraj);
    private Command observation1Command = () -> drive.followTrajectorySequenceAsync(observation1Traj);
    private Command centerSampleCommand = () -> drive.followTrajectorySequenceAsync(centerTraj);
    private Command observation2Command = () -> drive.followTrajectorySequenceAsync(observation2Traj);
    private Command wallSampleCommand = () -> drive.followTrajectorySequenceAsync(wallTraj);
    private Command observation3Command = () -> drive.followTrajectorySequenceAsync(observation3Traj);
    private Command wallIntakeCommand = () -> drive.followTrajectorySequenceAsync(wallIntakeTraj);

    private Claw claw;
    private Pivot pivot;
    private Scoring scoring;
    private Telescope telescope;
    private Wrist wrist;

    private Command pivotClip = () -> pivot.clipPos();
    private Command telescopeClip = () -> telescope.clipPos();
    private Command release = () -> claw.release();
    private Command pivotBack = () -> pivot.backPos();
    private Command telescopeBack = () -> telescope.backPos();
    private Command grab = () -> claw.grab();
    private Command pivotBasket = () -> pivot.basketPos();
    private Command telescopeBasket = () -> telescope.basketPos();

    private CommandSequence chamberSequence = new CommandSequence()
            .addCommand(chamberCommand)
            .addCommand(pivotClip)
            .addCommand(telescopeClip)
            .addWaitCommand(0.5)
            .addCommand(release)
            .build();
    private CommandSequence farSampleSequence = new CommandSequence()
            .addCommand(farSampleCommand)
            .addCommand(pivotBack)
            .addCommand(telescopeBack)
            .addCommand(grab)
            .build();
    private CommandSequence observation1Sequence = new CommandSequence()
            .addCommand(observation1Command)
            .addCommand(pivotBasket)
            .addCommand(telescopeBasket)
            .addWaitCommand(0.5)
            .addCommand(release)
            .build();
    private CommandSequence centerSampleSequence = new CommandSequence()
            .addCommand(centerSampleCommand)
            .addCommand(pivotBack)
            .addCommand(telescopeBack)
            .addCommand(grab)
            .build();
    private CommandSequence observation2Sequence = new CommandSequence()
            .addCommand(observation2Command)
            .addCommand(pivotBasket)
            .addCommand(telescopeBasket)
            .addWaitCommand(0.5)
            .addCommand(release)
            .build();
    private CommandSequence wallSampleSequence = new CommandSequence()
            .addCommand(wallSampleCommand)
            .addCommand(pivotBack)
            .addCommand(telescopeBack)
            .addCommand(grab)
            .build();
    private CommandSequence observation3Sequence = new CommandSequence()
            .addCommand(observation3Command)
            .addCommand(pivotBasket)
            .addCommand(telescopeBasket)
            .addWaitCommand(0.5)
            .addCommand(release)
            .addCommand(pivotBack)
            .addCommand(telescopeBack)
            .build();

    private CommandSequence wallIntakeSequence = new CommandSequence()
            .addCommand(wallIntakeCommand)
            //command needed to pick up specimens from edge of field
            .build();
    private AutoCommandMachine commandMachine = new AutoCommandMachine()
            .addCommandSequence(chamberSequence)
            .addCommandSequence(farSampleSequence)
            .addCommandSequence(observation1Sequence)
            .addCommandSequence(centerSampleSequence)
            .addCommandSequence(observation2Sequence)
            .addCommandSequence(wallSampleSequence)
            .addCommandSequence(observation3Sequence)
            .addCommandSequence(observation3Sequence)
            .addCommandSequence(wallIntakeSequence)
            .build();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        reflect = color == Color.RED;
        claw = new Claw(this);
        drive = new SampleMecanumDrive(hardwareMap);
        pivot = new Pivot(this);
        scoring = new Scoring(this);
        wrist = new Wrist(this);

        claw.init(hardwareMap);
        pivot.init(hardwareMap);
        scoring.init(hardwareMap);
        wrist.init(hardwareMap);

        drive.setPoseEstimate(reflect(ClipConstantsDash.START_POSE));
        chamberTraj = drive
                .trajectorySequenceBuilder(reflect(ClipConstantsDash.START_POSE))
                .lineTo(reflect(clipConstants.CHAMBER.getV()))
                .setReversed(true)
                .build();
        farTraj = drive
                .trajectorySequenceBuilder(reflect(chamberTraj.end()))
                .back(5)
                .splineTo(reflect(clipConstants.FAR_SAMPLE.getV()), reflect(clipConstants.FAR_SAMPLE.getH()))
                .setReversed(false)
                .build();
        observation1Traj = drive
                .trajectorySequenceBuilder(reflect(farTraj.end()))
                .lineToSplineHeading(vecToPose(clipConstants.OBSERVATION_1))
                .setReversed(true)
                .build();
        centerTraj = drive
                .trajectorySequenceBuilder(reflect(observation1Traj.end()))
                .splineTo(clipConstants.CENTER_SAMPLE.getV(), clipConstants.CENTER_SAMPLE.getH())
                .setReversed(false)
                .build();
        observation2Traj = drive
                .trajectorySequenceBuilder(reflect(centerTraj.end()))
                .lineToSplineHeading(vecToPose(clipConstants.OBSERVATION_2))
                .setReversed(true)
                .build();
        wallTraj = drive
                .trajectorySequenceBuilder(reflect(observation2Traj.end()))
                .splineTo(clipConstants.WALL_SAMPLE.getV(), clipConstants.WALL_SAMPLE.getH())
                .setReversed(false)
                .build();
        observation3Traj = drive
                .trajectorySequenceBuilder(reflect(wallTraj.end()))
                .lineToSplineHeading(vecToPose(clipConstants.OBSERVATION_3))
                .setReversed(true)
                .build();
        wallIntakeTraj = drive
                .trajectorySequenceBuilder(reflect(observation3Traj.end()))
                .splineTo(clipConstants.WALL_INTAKE.getV(), clipConstants.WALL_INTAKE.getH())
                .setReversed(false)
                .lineTo(clipConstants.CHAMBER.getV())
                .waitSeconds(1)
                .setReversed(true)
                .splineTo(clipConstants.WALL_INTAKE.getV(), clipConstants.WALL_INTAKE.getH())
                .waitSeconds(1)
                .setReversed(false)
                .lineTo(clipConstants.CHAMBER.getV())
                .waitSeconds(1)
                .build();

        waitForStart();

        while (opModeIsActive() && !isStopRequested() && !commandMachine.hasCompleted()) {
            drive.update();
            telescope.update();
            pivot.update();
            commandMachine.run(drive.isBusy() || busy);
            telemetry.update();
        }
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
