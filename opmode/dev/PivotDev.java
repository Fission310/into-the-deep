package org.firstinspires.ftc.teamcode.opmode.dev;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.mechanisms.Pivot;

@TeleOp(name = "Pivot Dev", group = "dev")
public class PivotDev extends LinearOpMode {

    private Pivot pivot = new Pivot(this);

    @Override
    public void runOpMode() throws InterruptedException {
        pivot.init(hardwareMap);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            pivot.loop(gamepad1);
        }
    }
}
