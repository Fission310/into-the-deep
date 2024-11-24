package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import java.lang.Thread.State;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmode.teleop.Controls;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.stuyfission.fissionlib.command.Command;
import com.stuyfission.fissionlib.command.CommandSequence;
import com.stuyfission.fissionlib.input.GamepadStatic;
import com.stuyfission.fissionlib.util.Mechanism;

public class Scoring extends Mechanism{
    private Drivetrain drivetrain = new Drivetrain(opMode);
    private Intake intake = new Intake(opMode);
    private Pivot pivot = new Pivot(opMode);
    private Telescope telescope = new Telescope(opMode);
    private Wrist wrist = new Wrist(opMode);

    public Scoring(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void init(HardwareMap hwMap) {
        drivetrain.init(hwMap);
        intake.init(hwMap);
        pivot.init(hwMap);
        telescope.init(hwMap);
        wrist.init(hwMap);
    }

    @Override
    public void telemetry(Telemetry telemetry) {

    }

    @Override
    public void loop(Gamepad gamepad) {
        drivetrain.loop(gamepad);
        intake.loop(gamepad);
        wrist.loop(gamepad);
        telescope.loop(gamepad);
        pivot.loop(gamepad);
    }

}
