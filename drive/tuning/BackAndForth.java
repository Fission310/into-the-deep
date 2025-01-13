package org.firstinspires.ftc.teamcode.drive.tuning;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

@Autonomous(name = "BackAndForth")
public final class BackAndForth extends LinearOpMode {
    public static double DISTANCE = 64;
    private boolean busy = false;

    private void followActionAsync(Action action) {
        busy = true;
        Thread thread = new Thread(
                () -> {
                    Actions.runBlocking(action);
                    busy = false;
                });
        thread.start();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        waitForStart();
        while (opModeIsActive()) {
            if (busy == false) {
                followActionAsync(drive.actionBuilder(new Pose2d(0, 0, 0)).lineToX(DISTANCE).lineToX(0).build());
            }
        }
    }
}
