package org.firstinspires.ftc.teamcode.opmode.auton.clip.setup;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Claw;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Pivot;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Scoring;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Telescope;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Wrist;
import org.firstinspires.ftc.teamcode.opmode.auton.basket.setup.BasketConstantsDash;
import org.firstinspires.ftc.teamcode.opmode.auton.util.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.stuyfission.fissionlib.command.AutoCommandMachine;
import com.stuyfission.fissionlib.command.Command;
import com.stuyfission.fissionlib.command.CommandSequence;
import org.firstinspires.ftc.teamcode.opmode.auton.util.Constant;
import com.acmerobotics.roadrunner.Pose2d;

public class ClipAuto extends LinearOpMode{
    private boolean reflect;
    private boolean busy = false;
    private Color color;
    private ClipConstants clipConstants;
    public ClipAuto(Color color, ClipConstants clipConstants){
        this.color = color;
        this.clipConstants = clipConstants;
    }

    private Action startChamberTraj;
    private Action farTraj;
    private Action sampleDrop1Traj;
    private Action centerTraj;
    private Action sampleDrop2Traj;
    private Action wallTraj;
    private Action sampleDrop3Traj;
    private Action wallIntake1Traj;
    private Action chamber1Traj;
    private Action wallIntake2Traj;
    private Action chamber2Traj;
    private Action wallIntake3Traj;
    private Action chamber3Traj;
    private MecanumDrive drive;

    private Command startChamberCommand = () -> Actions.runBlocking(startChamberTraj);
    private Command farTrajCommand = () -> Actions.runBlocking(farTraj);
    private Command sampleDrop1Command = () -> Actions.runBlocking(sampleDrop1Traj);
    private Command centerTrajCommand = () -> Actions.runBlocking(centerTraj);
    private Command sampleDrop2Command = () -> Actions.runBlocking(sampleDrop2Traj);
    private Command wallTrajCommand = () -> Actions.runBlocking(wallTraj);
    private Command sampleDrop3Command = () -> Actions.runBlocking(sampleDrop3Traj);
    private Command wallIntake1Command = () -> Actions.runBlocking(wallIntake1Traj);
    private Command chamber1Command = () -> Actions.runBlocking(chamber1Traj);
    private Command wallIntake2Command = () -> Actions.runBlocking(wallIntake2Traj);
    private Command chamber2Command = () -> Actions.runBlocking(chamber2Traj);
    private Command wallIntake3Command = () -> Actions.runBlocking(wallIntake3Traj);
    private Command chamber3Command = () -> Actions.runBlocking(chamber3Traj);
    private Claw claw;
    private Pivot pivot;
    private Scoring scoring;
    private Telescope telescope;
    private Wrist wrist;

    private Command pivotClip = () -> pivot.clipPos();
    private Command pivotClipDown = () -> pivot.clipDownPos();
    private Command telescopeClip = () -> telescope.clipPos();
    private Command telescopeScoreClip = () -> telescope.clipScorePos();
    private Command telescopeExtendClip = () -> telescope.clipExtensionPos();
    private Command telescopeFar = () -> telescope.autoFarPos();
    private Command telescopeCenter = () -> telescope.autoCenterPos();
    private Command telescopeWall = () -> telescope.autoWallPos();
    private Command telescopeSampleDrop = () -> telescope.autoSampleDropPos();
    private Command release = () -> claw.release();
    private Command grab = () -> claw.grab();
    private Command pivotBack = () -> pivot.backPos();
    private Command pivotDownIntake = () -> pivot.intakeDownPos();
    private Command pivotFront = () -> pivot.frontPos();
    private Command pivotBackClip = () -> pivot.clipBackPos();
    private Command pivotBackClipDown = () -> pivot.clipBackDownPos();
    private Command pivotUpIntake = () -> pivot.intakeUpPos();
    private Command pivotWallIntake = () -> pivot.wallPos();
    private Command telescopeBack = () -> telescope.backPos();
    private Command telescopeIntake = () -> telescope.frontIntakePos();
    private Command telescopeRetract = () -> telescope.frontPos();
    private Command wristRetract = () -> wrist.frontPos();
    private Command wristIntakeScore = () -> wrist.intakePos();
    private Command wristWallIntake = () -> wrist.wallPos();

    private CommandSequence startChamberSequence = new CommandSequence()
            .addCommand(startChamberCommand)
            .addCommand(pivotClip)
            .addCommand(telescopeExtendClip)
            .addWaitCommand(0.6)
            .addCommand(pivotClipDown)
            .addWaitCommand(0.6)
            .addCommand(telescopeScoreClip)
            .addWaitCommand(0.6)
            .addCommand(release)
            .addWaitCommand(0.6)
            .addCommand(pivotFront)
            .addCommand(wristRetract)
            .addCommand(telescopeRetract)
            .build();
    private CommandSequence farTrajSequence = new CommandSequence()
            .addCommand(farTrajCommand)
            .addCommand(pivotFront)
            .addCommand(telescopeFar)
            .addCommand(wristIntakeScore)
            .addWaitCommand(0.6)
            .addCommand(pivotDownIntake)
            .addWaitCommand(0.6)
            .addCommand(grab)
            .addCommand(pivotFront)
            .build();
    private CommandSequence sampleDrop1Sequence = new CommandSequence()
            .addCommand(sampleDrop1Command)
            .addCommand(telescopeSampleDrop)
            .addWaitCommand(0.7)
            .addCommand(release)
            .build();
    private CommandSequence centerTrajSequence = new CommandSequence()
            .addCommand(centerTrajCommand)
            .addCommand(telescopeCenter)
            .addWaitCommand(0.6)
            .addCommand(pivotDownIntake)
            .addWaitCommand(0.6)
            .addCommand(grab)
            .addCommand(pivotFront)
            .build();
    private CommandSequence sampleDrop2Sequence = new CommandSequence()
            .addCommand(sampleDrop2Command)
            .addCommand(telescopeSampleDrop)
            .addWaitCommand(0.7)
            .addCommand(release)
            .build();
    private CommandSequence wallTrajSequence = new CommandSequence()
            .addCommand(wallTrajCommand)
            .addCommand(telescopeWall)
            .addWaitCommand(0.6)
            .addCommand(pivotDownIntake)
            .addWaitCommand(0.6)
            .addCommand(grab)
            .addCommand(pivotFront)
            .build();
    private CommandSequence sampleDrop3Sequence = new CommandSequence()
            .addCommand(sampleDrop3Command)
            .addCommand(telescopeSampleDrop)
            .addWaitCommand(0.7)
            .addCommand(release)
            .addWaitCommand(0.5)
            .addCommand(telescopeRetract)
            .build();
    private CommandSequence wallIntake1Sequence = new CommandSequence()
            .addCommand(wallIntake1Command)
            .addCommand(pivotWallIntake)
            .addCommand(telescopeWall)
            .addCommand(wristWallIntake)
            .addWaitCommand(0.7)
            .addCommand(grab)
            .addWaitCommand(0.3)
            .addCommand(pivotBackClip)
            .build();
    private CommandSequence chamber1Sequence = new CommandSequence()
            .addCommand(chamber1Command)
            .addCommand(telescopeExtendClip)
            .addWaitCommand(0.6)
            .addCommand(pivotBackClipDown)
            .addWaitCommand(0.6)
            .addCommand(telescopeScoreClip)
            .addWaitCommand(0.6)
            .addCommand(release)
            .addWaitCommand(0.6)
            .addCommand(pivotWallIntake)
            .addCommand(wristWallIntake)
            .addCommand(telescopeWall)
            .build();
    private CommandSequence wallIntake2Sequence = new CommandSequence()
            .addCommand(wallIntake2Command)
            .addWaitCommand(0.7)
            .addCommand(grab)
            .addWaitCommand(0.3)
            .addCommand(pivotBackClip)
            .build();
    private CommandSequence chamber2Sequence = new CommandSequence()
            .addCommand(chamber2Command)
            .addCommand(telescopeExtendClip)
            .addWaitCommand(0.6)
            .addCommand(pivotBackClipDown)
            .addWaitCommand(0.6)
            .addCommand(telescopeScoreClip)
            .addWaitCommand(0.6)
            .addCommand(release)
            .addWaitCommand(0.6)
            .addCommand(pivotWallIntake)
            .addCommand(wristWallIntake)
            .addCommand(telescopeWall)
            .build();
    private CommandSequence wallIntake3Sequence = new CommandSequence()
            .addCommand(wallIntake3Command)
            .addWaitCommand(0.7)
            .addCommand(grab)
            .addWaitCommand(0.3)
            .addCommand(pivotBackClip)
            .build();
    private CommandSequence chamber3Sequence = new CommandSequence()
            .addCommand(chamber3Command)
            .addCommand(telescopeExtendClip)
            .addWaitCommand(0.6)
            .addCommand(pivotBackClipDown)
            .addWaitCommand(0.6)
            .addCommand(telescopeScoreClip)
            .addWaitCommand(0.6)
            .addCommand(release)
            .addWaitCommand(0.6)
            .addCommand(pivotFront)
            .addCommand(wristRetract)
            .addCommand(telescopeRetract)
            .build();
    private AutoCommandMachine commandMachine = new AutoCommandMachine() //need to do release and pick ups
            .addCommandSequence(startChamberSequence)
            .addCommandSequence(farTrajSequence)
            .addCommandSequence(sampleDrop1Sequence)
            .addCommandSequence(centerTrajSequence)
            .addCommandSequence(sampleDrop2Sequence)
            .addCommandSequence(wallTrajSequence)
            .addCommandSequence(sampleDrop3Sequence)
            .addCommandSequence(wallIntake1Sequence)
            .addCommandSequence(chamber1Sequence)
            .addCommandSequence(wallIntake2Sequence)
            .addCommandSequence(chamber2Sequence)
            .addCommandSequence(wallIntake3Sequence)
            .addCommandSequence(chamber3Sequence)
            .build();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        reflect = color == Color.BLUE;
        claw = new Claw(this);
        drive = new MecanumDrive(hardwareMap, ClipConstantsDash.START_POSE);
        pivot = new Pivot(this, telescope);
        scoring = new Scoring(this);
        wrist = new Wrist(this);

        claw.init(hardwareMap);
        pivot.init(hardwareMap);
        scoring.init(hardwareMap);
        wrist.init(hardwareMap);

        startChamberTraj = drive
                .actionBuilder(drive.pose)
                .lineToX(ClipConstants.START_CHAMBER.X_POS)
                .lineToY(ClipConstants.START_CHAMBER.Y_POS)
                .setReversed(true)
                .build();
        farTraj = drive
                .actionBuilder(drive.pose)
                .splineToLinearHeading(vecToPose(clipConstants.FAR_SAMPLE), clipConstants.FAR_SAMPLE.getH())
                .build();
        sampleDrop1Traj = drive
                .actionBuilder(drive.pose)
                .lineToXSplineHeading(clipConstants.SAMPLE_DROP_1.X_POS, clipConstants.SAMPLE_DROP_1.HEADING)
                .lineToYSplineHeading(clipConstants.SAMPLE_DROP_1.Y_POS, clipConstants.SAMPLE_DROP_1.HEADING)
                .build();
        centerTraj = drive
                .actionBuilder(drive.pose)
                .lineToXSplineHeading(clipConstants.CENTER_SAMPLE.X_POS, clipConstants.CENTER_SAMPLE.HEADING)
                .lineToYSplineHeading(clipConstants.CENTER_SAMPLE.Y_POS, clipConstants.CENTER_SAMPLE.HEADING)
                .build();
        sampleDrop2Traj = drive
                .actionBuilder(drive.pose)
                .lineToXSplineHeading(clipConstants.SAMPLE_DROP_2.X_POS, clipConstants.SAMPLE_DROP_2.HEADING)
                .lineToYSplineHeading(clipConstants.SAMPLE_DROP_2.Y_POS, clipConstants.SAMPLE_DROP_2.HEADING)
                .build();
        wallTraj = drive
                .actionBuilder(drive.pose)
                .lineToXSplineHeading(clipConstants.WALL_SAMPLE.X_POS, clipConstants.WALL_SAMPLE.HEADING)
                .lineToYSplineHeading(clipConstants.WALL_SAMPLE.Y_POS, clipConstants.WALL_SAMPLE.HEADING)
                .build();
        sampleDrop3Traj = drive
                .actionBuilder(drive.pose)
                .lineToXSplineHeading(clipConstants.SAMPLE_DROP_3.X_POS, clipConstants.SAMPLE_DROP_3.HEADING)
                .lineToYSplineHeading(clipConstants.SAMPLE_DROP_3.Y_POS, clipConstants.SAMPLE_DROP_3.HEADING)
                .build();
        wallIntake1Traj = drive
                .actionBuilder(drive.pose)
                .lineToXSplineHeading(clipConstants.WALL_INTAKE_1.X_POS, clipConstants.WALL_INTAKE_1.HEADING)
                .lineToYSplineHeading(clipConstants.WALL_INTAKE_1.Y_POS, clipConstants.WALL_INTAKE_1.HEADING)
                .setReversed(true)
                .setTangent(Math.toRadians(135))
                .build();
        chamber1Traj = drive
                .actionBuilder(drive.pose)
                .splineTo(clipConstants.CHAMBER_1.getV(), clipConstants.CHAMBER_1.getH())
                .setReversed(false)
                .build();
        wallIntake2Traj = drive
                .actionBuilder(drive.pose)
                .lineToXSplineHeading(clipConstants.WALL_INTAKE_2.X_POS, clipConstants.WALL_INTAKE_2.HEADING)
                .lineToYSplineHeading(clipConstants.WALL_INTAKE_2.Y_POS, clipConstants.WALL_INTAKE_2.HEADING)
                .setReversed(true)
                .build();
        chamber2Traj = drive
                .actionBuilder(drive.pose)
                .splineTo(clipConstants.CHAMBER_2.getV(), clipConstants.CHAMBER_2.getH())
                .setReversed(false)
                .build();
        wallIntake3Traj = drive
                .actionBuilder(drive.pose)
                .lineToXSplineHeading(clipConstants.WALL_INTAKE_3.X_POS, clipConstants.WALL_INTAKE_3.HEADING)
                .lineToYSplineHeading(clipConstants.WALL_INTAKE_3.Y_POS, clipConstants.WALL_INTAKE_3.HEADING)
                .setReversed(true)
                .build();
        chamber3Traj = drive
                .actionBuilder(drive.pose)
                .splineTo(clipConstants.CHAMBER_3.getV(), clipConstants.CHAMBER_3.getH())
                .setReversed(false)
                .build();

        waitForStart();

        while (opModeIsActive() && !isStopRequested() && !commandMachine.hasCompleted()) {
            telescope.update();
            pivot.update();
            commandMachine.run(busy);
            telemetry.update();
            drive.updatePoseEstimate();
        }
    }

    public Pose2d vecToPose(Constant constant){
        Vector2d vec = constant.getV();
        return new Pose2d(vec.x, vec.y, constant.getH());
    }
}
