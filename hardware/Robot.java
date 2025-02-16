package org.firstinspires.ftc.teamcode.hardware;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Scoring;
import org.firstinspires.ftc.teamcode.opmode.auton.util.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.stuyfission.fissionlib.util.Mechanism;

public class Robot extends Mechanism {
    private ElapsedTime elapsedTime;
    public Scoring scoring;

    public Robot(LinearOpMode opMode, Color color) {
        this.opMode = opMode;
        scoring = new Scoring(opMode, color);
    }

    @Override
    public void init(HardwareMap hwMap) {
        scoring.init(hwMap);
        elapsedTime = new ElapsedTime();
        elapsedTime.reset();
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        scoring.telemetry(telemetry);
        telemetry.addData("loop time", elapsedTime.milliseconds());
        elapsedTime.reset();
        telemetry.update();
    }

    @Override
    public void loop(Gamepad gamepad) {
        scoring.loop(gamepad);
    }
}
