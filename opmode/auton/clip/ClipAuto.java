package org.firstinspires.ftc.teamcode.opmode.auton.clip;

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
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Scoring;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Telescope;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Wrist;

@Autonomous(name = "ClipAuto", preselectTeleOp = "Main")
public class ClipAuto extends LinearOpMode{
    private boolean busy = false;
    private ClipConstants clipConstants = ClipConstantsDash.clipConstants;
    private Action startChamberAction;
    private Action farAction;
    private Action sampleDrop1Action;
    private Action centerAction;
    private Action sampleDrop2Action;
    private Action wallAction;
    private Action sampleDrop3Action;
    private Action wallIntake1Action;
    private Action chamber1Action;
    private Action wallIntake2Action;
    private Action chamber2Action;
    private Action wallIntake3Action;
    private Action chamber3Action;
    private MecanumDrive drive;

    private Command startChamberCommand = () -> followActionAsync(startChamberAction);
    private Command farTrajCommand = () -> followActionAsync(farAction);
    private Command sampleDrop1Command = () -> followActionAsync(sampleDrop1Action);
    private Command centerTrajCommand = () -> followActionAsync(centerAction);
    private Command sampleDrop2Command = () -> followActionAsync(sampleDrop2Action);
    private Command wallTrajCommand = () -> followActionAsync(wallAction);
    private Command sampleDrop3Command = () -> followActionAsync(sampleDrop3Action);
    private Command wallIntake1Command = () -> followActionAsync(wallIntake1Action);
    private Command chamber1Command = () -> followActionAsync(chamber1Action);
    private Command wallIntake2Command = () -> followActionAsync(wallIntake2Action);
    private Command chamber2Command = () -> followActionAsync(chamber2Action);
    private Command wallIntake3Command = () -> followActionAsync(wallIntake3Action);
    private Command chamber3Command = () -> followActionAsync(chamber3Action);

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
        claw = new Claw(this);
        drive = new MecanumDrive(hardwareMap, ClipConstantsDash.START_POSE);
        pivot = new Pivot(this, new Telescope(this));
        scoring = new Scoring(this);
        wrist = new Wrist(this);

        claw.init(hardwareMap);
        pivot.init(hardwareMap);
        scoring.init(hardwareMap);
        wrist.init(hardwareMap);

        startChamberAction = drive
                .actionBuilder(ClipConstantsDash.START_POSE)
                .lineToY(clipConstants.START_CHAMBER.getVec().y)
                .setReversed(true)
                .build();
        farAction = drive
                .actionBuilder(clipConstants.START_CHAMBER.getPose())
                .splineToLinearHeading(clipConstants.FAR_SAMPLE.getPose(), clipConstants.FAR_SAMPLE.getH())
                .build();
        sampleDrop1Action = drive
                .actionBuilder(clipConstants.FAR_SAMPLE.getPose())
                .splineToLinearHeading(clipConstants.SAMPLE_DROP_1.getPose(), clipConstants.SAMPLE_DROP_1.getH())
                .build();
        centerAction = drive
                .actionBuilder(clipConstants.SAMPLE_DROP_1.getPose())
                .splineToLinearHeading(clipConstants.CENTER_SAMPLE.getPose(), clipConstants.CENTER_SAMPLE.getH())
                .build();
        sampleDrop2Action = drive
                .actionBuilder(clipConstants.CENTER_SAMPLE.getPose())
                .splineToLinearHeading(clipConstants.SAMPLE_DROP_2.getPose(), clipConstants.SAMPLE_DROP_2.getH())
                .build();
        wallAction = drive
                .actionBuilder(clipConstants.SAMPLE_DROP_2.getPose())
                .splineToLinearHeading(clipConstants.WALL_SAMPLE.getPose(), clipConstants.WALL_SAMPLE.getH())
                .build();
        sampleDrop3Action = drive
                .actionBuilder(clipConstants.WALL_SAMPLE.getPose())
                .splineToLinearHeading(clipConstants.SAMPLE_DROP_3.getPose(), clipConstants.SAMPLE_DROP_3.getH())
                .build();
        wallIntake1Action = drive
                .actionBuilder(clipConstants.SAMPLE_DROP_3.getPose())
                .splineToLinearHeading(clipConstants.WALL_INTAKE_1.getPose(), clipConstants.WALL_INTAKE_1.getH())
                .setTangent(Math.toRadians(135))//
                .build();
        chamber1Action = drive
                .actionBuilder(clipConstants.WALL_INTAKE_1.getPose())
                .splineToLinearHeading(clipConstants.CHAMBER_1.getPose(), clipConstants.CHAMBER_1.getH())
                .build();
        wallIntake2Action = drive
                .actionBuilder(clipConstants.CHAMBER_1.getPose())
                .splineToLinearHeading(clipConstants.WALL_INTAKE_2.getPose(), clipConstants.WALL_INTAKE_2.getH())
                .build();
        chamber2Action = drive
                .actionBuilder(clipConstants.WALL_INTAKE_2.getPose())
                .splineToLinearHeading(clipConstants.CHAMBER_2.getPose(), clipConstants.CHAMBER_2.getH())
                .build();
        wallIntake3Action = drive
                .actionBuilder(clipConstants.CHAMBER_2.getPose())
                .splineToLinearHeading(clipConstants.WALL_INTAKE_3.getPose(), clipConstants.WALL_INTAKE_3.getH())
                .build();
        chamber3Action = drive
                .actionBuilder(clipConstants.WALL_INTAKE_3.getPose())
                .splineToLinearHeading(clipConstants.CHAMBER_3.getPose(), clipConstants.CHAMBER_3.getH())
                .build();

        waitForStart();

        while (opModeIsActive() && !isStopRequested() && !commandMachine.hasCompleted()) {
            drive.updatePoseEstimate();
            telescope.update();
            pivot.update();
            commandMachine.run(busy);
            telemetry.update();
        }
    }
}
