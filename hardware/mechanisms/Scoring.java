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
    private String color;
    private Drivetrain drivetrain = new Drivetrain(opMode);
    private Intake intake = new Intake(opMode, color);
    private Pivot pivot = new Pivot(opMode);
    private Telescope telescope = new Telescope(opMode);
    private Wrist wrist = new Wrist(opMode);

    private State state = State.FRONT;
    private enum State {
        FRONT,
        WALL,
        BASKET,
        CLIP,
        BACK,
    }

    public Scoring(LinearOpMode opMode, String color) {
        this.opMode = opMode;
        this.color = color;
    }

    private Command setStateFront = () -> state = State.FRONT;
    private Command setStateWall = () -> state = State.WALL;
    private Command setStateBasket = () -> state = State.BASKET;
    private Command setStateClip = () -> state = State.CLIP;
    private Command setStateBack = () -> state = State.BACK;
    private Command pivotFront = () -> pivot.frontPos();
    private Command pivotWall = () -> pivot.wallPos();
    private Command pivotBasket = () -> pivot.basketPos();
    private Command pivotClip = () -> pivot.clipPos();
    private Command pivotBack = () -> pivot.backPos();
    private Command telescopeFront = () -> telescope.frontPos();
    private Command telescopeWall = () -> telescope.wallPos();
    private Command telescopeBasket = () -> telescope.basketPos();
    private Command telescopeClip = () -> telescope.clipPos();
    private Command telescopeBack = () -> telescope.backPos();
    private Command wristFront = () -> wrist.frontPos();
    private Command wristWall = () -> wrist.wallPos();
    private Command wristBasket = () -> wrist.basketPos();
    private Command wristClip = () -> wrist.clipPos();
    private Command wristBack = () -> wrist.backPos();
    private Command intakeFront = () -> intake.intakeFront();
    private Command intakeBack = () -> intake.intakeBack();
    private Command outtakeFront = () -> intake.outtakeFront();
    private Command outtakeBack = () -> intake.outtakeBack();

    private CommandSequence goFront = new CommandSequence()
            .addCommand(setStateFront)
            .addCommand(pivotFront)
            .addCommand(telescopeFront)
            .addCommand(wristFront);
    private CommandSequence goWall = new CommandSequence()
            .addCommand(setStateWall)
            .addCommand(pivotWall)
            .addCommand(telescopeWall)
            .addCommand(wristWall);
    private CommandSequence goBasket = new CommandSequence()
            .addCommand(setStateBasket)
            .addCommand(pivotBasket)
            .addCommand(telescopeBasket)
            .addCommand(wristBasket);
    private CommandSequence goClip = new CommandSequence()
            .addCommand(setStateClip)
            .addCommand(pivotClip)
            .addCommand(telescopeClip)
            .addCommand(wristClip);
    private CommandSequence goBack = new CommandSequence()
            .addCommand(setStateBack)
            .addCommand(pivotBack)
            .addCommand(telescopeBack)
            .addCommand(wristBack);
    private CommandSequence frontIntake = new CommandSequence().addCommand(intakeFront);
    private CommandSequence frontOuttake = new CommandSequence().addCommand(outtakeFront);
    private CommandSequence backIntake = new CommandSequence().addCommand(intakeBack);
    private CommandSequence backOuttake = new CommandSequence().addCommand(outtakeFront);

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

        if (GamepadStatic.isButtonPressed(gamepad, Controls.PIVOT_FRONT) && state != State.FRONT) {
            goFront.trigger();
        } else if (GamepadStatic.isButtonPressed(gamepad, Controls.PIVOT_WALL) && state != State.WALL) {
            goWall.trigger();
        } else if (GamepadStatic.isButtonPressed(gamepad, Controls.PIVOT_BASKET) && state != State.BASKET) {
            goBasket.trigger();
        } else if (GamepadStatic.isButtonPressed(gamepad, Controls.PIVOT_CLIP) && state != State.CLIP) {
            goClip.trigger();
        } else if (GamepadStatic.isButtonPressed(gamepad, Controls.PIVOT_BACK) && state != State.BACK) {
            goBack.trigger();
        }

        switch (state) {
            case FRONT:
            case WALL:
                if (GamepadStatic.isButtonPressed(gamepad, Controls.INTAKE)) {
                    frontIntake.trigger();
                }
                if (GamepadStatic.isButtonPressed(gamepad, Controls.OUTTAKE)) {
                    frontOuttake.trigger();
                }
                break;

            case BASKET:
            case CLIP:
                if (GamepadStatic.isButtonPressed(gamepad, Controls.OUTTAKE)) {
                    frontOuttake.trigger();
                }

            case BACK:
                if (GamepadStatic.isButtonPressed(gamepad, Controls.INTAKE)) {
                    backIntake.trigger();
                }
                if (GamepadStatic.isButtonPressed(gamepad, Controls.OUTTAKE)) {
                    backOuttake.trigger();
                }
        }
    }

}
