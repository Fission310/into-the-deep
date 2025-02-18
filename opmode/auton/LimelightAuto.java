package org.firstinspires.ftc.teamcode.opmode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.stuyfission.fissionlib.command.AutoCommandMachine;
import com.stuyfission.fissionlib.command.Command;
import com.stuyfission.fissionlib.command.CommandSequence;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Limelight;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Pivot;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Sweeper;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Telescope;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Wrist;
import org.firstinspires.ftc.teamcode.opmode.auton.util.Drive;
import org.firstinspires.ftc.teamcode.opmode.auton.util.LimelightConstants;

import java.io.File;
import java.io.FileWriter;

@Autonomous(name = "LimelightAuto", preselectTeleOp = "Main")
public class LimelightAuto extends LinearOpMode {
    private boolean commandBusy = false;
    private Pose2d targetPoint = null;

    public static double[][] limelightInfo = new double[4][10];
    public static int limelightRun = 0;
    public static double llTx = 0;
    public static double llTa = 0;
    public static double llTy = 0;
    public static double llTangle = 0;

    private SampleMecanumDrive drive;

    private Intake intake;
    private Pivot pivot;
    private Telescope telescope;
    private Wrist wrist;
    private Limelight limelight;
    private Sweeper sweeper;

    private Command commandBusyTrue = () -> commandBusy = true;
    private Command commandBusyFalse = () -> commandBusy = false;
    private Command intakeCommand = () -> intake.intake();
    private Command outtakeCommand = () -> intake.outtake();
    private Command pivotUpIntake = () -> pivot.intakeUpPos();
    private Command pivotGrabIntake = () -> pivot.autoIntakeGrabPos();
    private Command pivotReset = () -> pivot.reset();
    private Command sweepExtend = () -> sweeper.extendPos();
    private Command sweepRetract = () -> sweeper.retractPos();
    private Command telescopeRetract = () -> telescope.frontPos();
    private Command wristRetract = () -> wrist.frontPos();
    private Command wristIntakeScore = () -> wrist.autoIntakePos();
    private Command setResult = () -> {
        llTa = limelight.getTa();
        llTx = limelight.getTx();
        llTy = limelight.getTy();
        llTangle = limelight.getTangle();

        limelightInfo[0][limelightRun] = llTa;
        limelightInfo[1][limelightRun] = llTx;
        limelightInfo[2][limelightRun] = llTy;
        limelightInfo[3][limelightRun] = llTangle;

        limelightRun += 1;
    };
    private Command lineUpP2P = () -> targetPoint = new Pose2d(drive.getPoseEstimate().getX(),
            drive.getPoseEstimate().getY() - (LimelightConstants.calcXDistance(llTx, llTy) - 6.5),
            drive.getPoseEstimate().getHeading());
    private Command forwardP2P = () -> targetPoint = new Pose2d(targetPoint.getX() + 3,
            targetPoint.getY(), targetPoint.getHeading());
    private Command driveStop = () -> {
        targetPoint = null;
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
    };
    private Command telescopeExtendInches = () -> telescope
            .setTargetInches(LimelightConstants.calcYDistance(llTy) * 2 + 6);
    private Command telescopeExtendABit = () -> telescope.frontIntakeAutoShortPos();

    private CommandSequence subSequence = new CommandSequence()
            .addCommand(commandBusyTrue)
            .addCommand(setResult)
            .addCommand(lineUpP2P)
            .addWaitCommand(0.5)
            .addCommand(sweepExtend)
            .addWaitCommand(0.2)
            .addCommand(forwardP2P)
            .addWaitCommand(0.4)
            .addCommand(driveStop)
            .addCommand(telescopeExtendABit)
            .addCommand(intakeCommand)
            .addCommand(pivotGrabIntake)
            .addCommand(wristIntakeScore)
            .addWaitCommand(0.6)
            .addCommand(telescopeExtendInches)
            .addCommand(sweepRetract)
            .addWaitCommand(0.8)
            .addCommand(pivotUpIntake)
            .addWaitCommand(0.2)
            .addCommand(outtakeCommand)
            .addWaitCommand(0.1)
            .addCommand(wristRetract)
            .addWaitCommand(0.2)
            .addCommand(telescopeRetract)
            .addCommand(commandBusyFalse)
            .build();
    private CommandSequence doNothing = new CommandSequence().build();
    private AutoCommandMachine commandMachine = new AutoCommandMachine()
            .addCommandSequence(subSequence) // 1
            .addCommandSequence(subSequence) // 2
            .addCommandSequence(subSequence) // 3
            .addCommandSequence(subSequence) // 4
            .addCommandSequence(subSequence) // 5
            .addCommandSequence(subSequence) // 6
            .addCommandSequence(subSequence) // 7
            .addCommandSequence(subSequence) // 8
            .addCommandSequence(subSequence) // 9
            .addCommandSequence(subSequence) // 10
            .addCommandSequence(doNothing)
            .build();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        intake = new Intake(this);
        drive = new SampleMecanumDrive(hardwareMap);
        telescope = new Telescope(this);
        pivot = new Pivot(this, telescope);
        wrist = new Wrist(this);
        limelight = new Limelight(this);
        sweeper = new Sweeper(this);

        intake.init(hardwareMap);
        pivot.init(hardwareMap);
        pivot.intakeUpPos();
        wrist.init(hardwareMap);
        telescope.init(hardwareMap);
        limelight.init(hardwareMap);
        sweeper.init(hardwareMap);

        while (opModeInInit() && !isStopRequested()) {
            drive.updatePoseEstimate();
            pivot.update();
            initTelemetry();
        }

        drive.setPoseEstimate(BasketConstants.SUBMERSIBLE_1.getPose());

        waitForStart();

        while (opModeIsActive() && !isStopRequested() && !commandMachine.hasCompleted()) {
            if (targetPoint != null) {
                Drive.p2p(drive, targetPoint);
            }
            drive.update();
            telescope.update();
            pivot.update();
            limelight.update();
            commandMachine.run(drive.isBusy() || commandBusy);
            activeTelemetry();
        }

        limelight.stop();

        try {
            appendCSVData();
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        Thread.sleep(500);
    }

    public void initTelemetry(){
        telemetry.addData("drive x", drive.getPoseEstimate().getX());
        telemetry.addData("drive y", drive.getPoseEstimate().getY());
        telemetry.update();
    }

    public void activeTelemetry(){
        telemetry.addData("limelight tx", llTx);
        telemetry.addData("limelight ty", llTy);
        telemetry.addData("limelight tangle", llTangle);
        telemetry.addData("limelight strafe distance",
                LimelightConstants.calcXDistance(llTx, llTy) - 6);
        telemetry.addData("telescope extend dist",
                LimelightConstants.calcYDistance(llTy));
        telemetry.addData("telescope extend ticks",
                (LimelightConstants.calcYDistance(llTy)) / Telescope.INCH_PER_TICK);
        telemetry.addData("target pose", targetPoint);
        telemetry.addData("drive x", drive.getPoseEstimate().getX());
        telemetry.addData("drive y", drive.getPoseEstimate().getY());
        telemetry.update();
    }

    public void appendCSVData() throws Exception {
        File csvFile = new File("util/limelight.csv");
        FileWriter writer = new FileWriter(csvFile, true);

        if (limelightInfo != null) {
            for (double[] info : limelightInfo) {
                for (int col = 0; col < info.length; col++) {
                    writer.append(String.valueOf(info[col]));
                    if (col < info.length - 1) {
                        writer.append(",");
                    }
                }
                writer.append("\n");
            }
        }
        writer.flush();
    }
}
