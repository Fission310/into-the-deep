package org.firstinspires.ftc.teamcode.opmode.dev;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.mechanisms.Pivot;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Telescope;

@TeleOp(name = "PivotTelescope Dev", group = "dev")
public class PivotTelescopeDev extends LinearOpMode {

    private Telescope telescope = new Telescope(this);
    private Pivot pivot = new Pivot(this, telescope);

    @Override
    public void runOpMode() throws InterruptedException {
        pivot.init(hardwareMap);
        telescope.init(hardwareMap);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            pivot.loop(gamepad1);
            telescope.loop(gamepad1);
        }
    }
}
