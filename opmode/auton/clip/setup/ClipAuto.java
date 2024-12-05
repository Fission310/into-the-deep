package org.firstinspires.ftc.teamcode.opmode.auton.clip.setup;

import static org.firstinspires.ftc.teamcode.hardware.mechanisms.Scoring.CLIP_CLIP_WAIT;
import static org.firstinspires.ftc.teamcode.hardware.mechanisms.Scoring.CLIP_EXTEND_WAIT;
import static org.firstinspires.ftc.teamcode.hardware.mechanisms.Scoring.CLIP_PIVOT_WAIT;
import static org.firstinspires.ftc.teamcode.hardware.mechanisms.Scoring.CLIP_RELEASE_WAIT;
import static org.firstinspires.ftc.teamcode.hardware.mechanisms.Scoring.PIVOT_DOWN_WAIT;
import static org.firstinspires.ftc.teamcode.hardware.mechanisms.Scoring.TELESCOPE_RETRACT_WAIT;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Claw;
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

    private TrajectorySequence startChamberTraj;
    private TrajectorySequence farTraj;
    private TrajectorySequence sampleDrop1Traj;
    private TrajectorySequence centerTraj;
    private TrajectorySequence sampleDrop2Traj;
    private TrajectorySequence wallTraj;
    private TrajectorySequence sampleDrop3Traj;
    private TrajectorySequence wallIntake1Traj;
    private TrajectorySequence chamber1Traj;
    private TrajectorySequence wallIntake2Traj;
    private TrajectorySequence chamber2Traj;
    private TrajectorySequence wallIntake3Traj;
    private TrajectorySequence chamber3Traj;
    private SampleMecanumDrive drive;

    private Command startChamberCommand = () -> drive.followTrajectorySequenceAsync(startChamberTraj);
    private Command farTrajCommand = () -> drive.followTrajectorySequenceAsync(farTraj);
    private Command sampleDrop1Command = () -> drive.followTrajectorySequenceAsync(sampleDrop1Traj);
    private Command centerTrajCommand = () -> drive.followTrajectorySequenceAsync(centerTraj);
    private Command sampleDrop2Command = () -> drive.followTrajectorySequenceAsync(sampleDrop2Traj);
    private Command wallTrajCommand = () -> drive.followTrajectorySequenceAsync(wallTraj);
    private Command sampleDrop3Command = () -> drive.followTrajectorySequenceAsync(sampleDrop3Traj);
    private Command wallIntake1Command = () -> drive.followTrajectorySequenceAsync(wallIntake1Traj);
    private Command chamber1Command = () -> drive.followTrajectorySequenceAsync(chamber1Traj);
    private Command wallIntake2Command = () -> drive.followTrajectorySequenceAsync(wallIntake2Traj);
    private Command chamber2Command = () -> drive.followTrajectorySequenceAsync(chamber2Traj);
    private Command wallIntake3Command = () -> drive.followTrajectorySequenceAsync(wallIntake3Traj);
    private Command chamber3Command = () -> drive.followTrajectorySequenceAsync(chamber3Traj);
    private Claw claw;
    private Pivot pivot;
    private Scoring scoring;
    private Telescope telescope;
    private Wrist wrist;

    private Command pivotClip = () -> pivot.clipPos();
    private Command pivotClipDown = () -> pivot.clipDownPos();
    private Command telescopeClip = () -> telescope.clipPos();
    private Command telescopeFront = () -> telescope.frontPos();
    private Command telescopeScoreClip = () -> telescope.clipScorePos();
    private Command telescopeExtendClip = () -> telescope.clipExtensionPos();
    private Command release = () -> claw.release();
    private Command pivotBack = () -> pivot.backPos();
    private Command pivotDownIntake = () -> pivot.intakeDownPos();
    private Command pivotFront = () -> pivot.frontPos();
    private Command pivotUpIntake = () -> pivot.intakeUpPos();
    private Command telescopeBack = () -> telescope.backPos();
    private Command telescopeIntake = () -> telescope.frontIntakePos();
    private Command telescopeRetract = () -> telescope.frontPos();
    private Command wristRetract = () -> wrist.frontPos();
    private Command wristIntakeScore = () -> wrist.intakePos();

    private CommandSequence startChamberSequence = new CommandSequence()
            .addCommand(startChamberCommand)
            .build();
    private CommandSequence pivotToClip = new CommandSequence()
            .addCommand(pivotClip)
            .build();
    private CommandSequence scoreClip = new CommandSequence()
            .addCommand(telescopeExtendClip)
            .addWaitCommand(CLIP_EXTEND_WAIT)
            .addCommand(pivotClipDown)
            .addWaitCommand(CLIP_PIVOT_WAIT)
            .addCommand(telescopeScoreClip)
            .addWaitCommand(CLIP_CLIP_WAIT)
            .addCommand(release)
            .addWaitCommand(CLIP_RELEASE_WAIT)
            .addCommand(pivotFront)
            .addCommand(wristRetract)
            .addCommand(telescopeFront)
            .build();
    private CommandSequence retractTele = new CommandSequence()
            .addCommand(pivotUpIntake)
            .addCommand(wristRetract)
            .addWaitCommand(TELESCOPE_RETRACT_WAIT)
            .addCommand(telescopeRetract)
            .addWaitCommand(PIVOT_DOWN_WAIT)
            .addCommand(pivotDownIntake)
            .build();
    private CommandSequence farTrajSequence = new CommandSequence()
            .addCommand(farTrajCommand)
            .build();
    public CommandSequence frontIntake = new CommandSequence()
            .addCommand(pivotUpIntake)
            .addCommand(telescopeIntake)
            .addWaitCommand(PIVOT_DOWN_WAIT)
            .addCommand(pivotDownIntake)
            .addCommand(wristIntakeScore)
            .addCommand(release)
            .build();
    private CommandSequence sampleDrop1Sequence = new CommandSequence()
            .addCommand(sampleDrop1Command)
            .build();
    private CommandSequence centerTrajSequence = new CommandSequence()
            .addCommand(centerTrajCommand)
            .build();
    private CommandSequence sampleDrop2Sequence = new CommandSequence()
            .addCommand(sampleDrop2Command)
            .build();
    private CommandSequence wallTrajSequence = new CommandSequence()
            .addCommand(wallTrajCommand)
            .build();
    private CommandSequence sampleDrop3Sequence = new CommandSequence()
            .addCommand(sampleDrop3Command)
            .build();
    private CommandSequence wallIntake1Sequence = new CommandSequence()
            .addCommand(wallIntake1Command)
            .build();
    private CommandSequence chamber1Sequence = new CommandSequence()
            .addCommand(chamber1Command)
            .build();
    private CommandSequence wallIntake2Sequence = new CommandSequence()
            .addCommand(wallIntake2Command)
            .build();
    private CommandSequence chamber2Sequence = new CommandSequence()
            .addCommand(chamber2Command)
            .build();
    private CommandSequence wallIntake3Sequence = new CommandSequence()
            .addCommand(wallIntake3Command)
            .build();
    private CommandSequence chamber3Sequence = new CommandSequence()
            .addCommand(chamber3Command)
            .build();
    private AutoCommandMachine commandMachine = new AutoCommandMachine() //need to do release and pick ups
            .addCommandSequence(startChamberSequence)
            .addCommandSequence(pivotToClip) //
            .addCommandSequence(scoreClip) //
            .addCommandSequence(retractTele) //
            .addCommandSequence(farTrajSequence)
            .addCommandSequence(frontIntake) //
            .addCommandSequence(retractTele) //
            .addCommandSequence(sampleDrop1Sequence)
            .addCommandSequence(centerTrajSequence)
            .addCommandSequence(frontIntake) //
            .addCommandSequence(retractTele) //
            .addCommandSequence(sampleDrop2Sequence)
            .addCommandSequence(wallTrajSequence)
            .addCommandSequence(frontIntake) //
            .addCommandSequence(retractTele) //
            .addCommandSequence(sampleDrop3Sequence)
            .addCommandSequence(wallIntake1Sequence)
            .addCommandSequence(chamber1Sequence)
            .addCommandSequence(pivotToClip) //
            .addCommandSequence(scoreClip) //
            .addCommandSequence(retractTele) //
            .addCommandSequence(wallIntake2Sequence)
            .addCommandSequence(chamber2Sequence)
            .addCommandSequence(pivotToClip) //
            .addCommandSequence(scoreClip) //
            .addCommandSequence(retractTele) //
            .addCommandSequence(wallIntake3Sequence)
            .addCommandSequence(chamber3Sequence)
            .addCommandSequence(pivotToClip) //
            .addCommandSequence(scoreClip) //
            .addCommandSequence(retractTele) //
            .build();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        reflect = color == Color.BLUE;
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
        startChamberTraj = drive
                .trajectorySequenceBuilder(reflect(ClipConstantsDash.START_POSE))
                .lineTo(reflect(clipConstants.START_CHAMBER.getV()))
                .setReversed(true)
                .build();
        farTraj = drive
                .trajectorySequenceBuilder(reflect(startChamberTraj.end()))
                .splineToLinearHeading(vecToPose(clipConstants.FAR_SAMPLE), reflect(clipConstants.FAR_SAMPLE.getH()))
                .build();
        sampleDrop1Traj = drive
                .trajectorySequenceBuilder(reflect(farTraj.end()))
                .lineToSplineHeading(vecToPose(clipConstants.SAMPLE_DROP_1))
                .build();
        centerTraj = drive
                .trajectorySequenceBuilder(reflect(sampleDrop1Traj.end()))
                .lineToSplineHeading(vecToPose(clipConstants.CENTER_SAMPLE))
                .build();
        sampleDrop2Traj = drive
                .trajectorySequenceBuilder(reflect(centerTraj.end()))
                .lineToSplineHeading(vecToPose(clipConstants.SAMPLE_DROP_2))
                .build();
        wallTraj = drive
                .trajectorySequenceBuilder(reflect(sampleDrop2Traj.end()))
                .lineToSplineHeading(vecToPose(clipConstants.WALL_SAMPLE))
                .build();
        sampleDrop3Traj = drive
                .trajectorySequenceBuilder(reflect(wallTraj.end()))
                .lineToSplineHeading(vecToPose(clipConstants.SAMPLE_DROP_3))
                .build();
        wallIntake1Traj = drive
                .trajectorySequenceBuilder(reflect(sampleDrop3Traj.end()))
                .lineToSplineHeading(vecToPose(clipConstants.WALL_INTAKE_1))
                .setReversed(true)
                .setTangent(reflect(Math.toRadians(135)))
                .build();
        chamber1Traj = drive
                .trajectorySequenceBuilder(reflect(wallIntake1Traj.end()))
                .splineTo(reflect(clipConstants.CHAMBER_1.getV()), reflect(clipConstants.CHAMBER_1.getH()))
                .setReversed(false)
                .build();
        wallIntake2Traj = drive
                .trajectorySequenceBuilder(reflect(chamber1Traj.end()))
                .lineToSplineHeading(vecToPose(clipConstants.WALL_INTAKE_2))
                .setReversed(true)
                .build();
        chamber2Traj = drive
                .trajectorySequenceBuilder(reflect(wallIntake2Traj.end()))
                .splineTo(reflect(clipConstants.CHAMBER_2.getV()), reflect(clipConstants.CHAMBER_2.getH()))
                .setReversed(false)
                .build();
        wallIntake3Traj = drive
                .trajectorySequenceBuilder(reflect(chamber2Traj.end()))
                .lineToSplineHeading(vecToPose(clipConstants.WALL_INTAKE_3))
                .setReversed(true)
                .build();
        chamber3Traj = drive
                .trajectorySequenceBuilder(reflect(wallIntake3Traj.end()))
                .splineTo(reflect(clipConstants.CHAMBER_3.getV()), reflect(clipConstants.CHAMBER_3.getH()))
                .setReversed(false)
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
