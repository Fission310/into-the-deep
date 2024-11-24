package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmode.auton.util.Color;
import org.firstinspires.ftc.teamcode.opmode.teleop.Controls;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.stuyfission.fissionlib.command.Command;
import com.stuyfission.fissionlib.command.CommandSequence;
import com.stuyfission.fissionlib.input.GamepadStatic;
import com.stuyfission.fissionlib.util.Mechanism;

@Config
public class Scoring extends Mechanism {
    private Color color;
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
        UP,
        CLIP,
        BACK,
    }

    public static double BASKET_OUTTAKE_WAIT = 0.10;
    public static double BASKET_RETRACT_WAIT = 0.10;
    public static double CLIP_WRIST_WAIT = 0.10;
    public static double CLIP_CLIP_WAIT = 0.10;
    public static double CLIP_OUTTAKE_WAIT = 0.10;

    public Scoring(LinearOpMode opMode, Color color) {
        this.opMode = opMode;
        this.color = color;
    }

    private Command outtakeBasket = () -> intake.outtakeBasket();
    private Command intakeStop = () -> intake.stop();
    private Command pivotUp = () -> pivot.upPos();
    private Command telescopeUp = () -> telescope.upPos();
    private Command setStateUp = () -> state = State.UP;
    private Command telescopeExtendClip = () -> telescope.clipExtensionPos();
    private Command wristClipScore = () -> wrist.clipScorePos();
    private Command outtakeClip = () -> intake.outtakeClip();

    private CommandSequence scoreBasket = new CommandSequence()
            .addCommand(outtakeBasket)
            .addWaitCommand(BASKET_OUTTAKE_WAIT)
            .addCommand(intakeStop)
            .addCommand(pivotUp)
            .addWaitCommand(BASKET_RETRACT_WAIT)
            .addCommand(telescopeUp)
            .addCommand(setStateUp)
            .build();
    private CommandSequence scoreClip = new CommandSequence()
            .addCommand(telescopeExtendClip)
            .addWaitCommand(CLIP_WRIST_WAIT)
            .addCommand(wristClipScore)
            .addCommand(telescopeUp)
            .addWaitCommand(CLIP_CLIP_WAIT)
            .addCommand(outtakeClip)
            .addWaitCommand(CLIP_OUTTAKE_WAIT)
            .addCommand(intakeStop)
            .addCommand(pivotUp)
            .addCommand(setStateUp)
            .build();

    public void goFront() {
        state = State.FRONT;
        pivot.frontPos();
        telescope.frontPos();
        wrist.frontPos();
    }

    public void goWall() {
        state = State.WALL;
        pivot.wallPos();
        telescope.wallPos();
        wrist.wallPos();
    }

    public void goBasket() {
        state = State.BASKET;
        pivot.basketPos();
        telescope.basketPos();
        wrist.basketPos();
    }

    public void goClip() {
        state = State.CLIP;
        pivot.clipPos();
        telescope.clipPos();
        wrist.clipPos();
    }

    public void goBack() {
        state = State.BACK;
        pivot.backPos();
        telescope.backPos();
        wrist.backPos();
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
        telemetry.addData("state", state);
        telemetry.update();
    }

    @Override
    public void loop(Gamepad gamepad) {
        drivetrain.loop(gamepad);
        pivot.update();
        telescope.update();

        if (GamepadStatic.isButtonPressed(gamepad, Controls.PIVOT_FRONT) && state != State.FRONT) {
            goFront();
        } else if (GamepadStatic.isButtonPressed(gamepad, Controls.PIVOT_WALL) && state != State.WALL) {
            goWall();
        } else if (GamepadStatic.isButtonPressed(gamepad, Controls.PIVOT_BASKET) && state != State.BASKET) {
            goBasket();
        } else if (GamepadStatic.isButtonPressed(gamepad, Controls.PIVOT_CLIP) && state != State.CLIP) {
            goClip();
        } else if (GamepadStatic.isButtonPressed(gamepad, Controls.PIVOT_BACK) && state != State.BACK) {
            goBack();
        }

        switch (state) {
            case UP:
                break;
            case FRONT:
                if (GamepadStatic.isButtonPressed(gamepad, Controls.INTAKE) && !intake.isSample()) {
                    intake.intakeFront();
                } else if (GamepadStatic.isButtonPressed(gamepad, Controls.OUTTAKE)) {
                    intake.outtakeFront();
                } else {
                    intake.stop();
                }
                intake.loop(gamepad);
                break;
            case WALL:
                if (GamepadStatic.isButtonPressed(gamepad, Controls.INTAKE) && !intake.isSample()) {
                    intake.intakeWall();
                } else if (GamepadStatic.isButtonPressed(gamepad, Controls.OUTTAKE)) {
                    intake.outtakeWall();
                } else {
                    intake.stop();
                }
                break;
            case BASKET:
                if (GamepadStatic.isButtonPressed(gamepad, Controls.SCORE)) {
                    scoreBasket.trigger();
                }
            case CLIP:
                if (GamepadStatic.isButtonPressed(gamepad, Controls.SCORE)) {
                    scoreClip.trigger();
                }
                break;
            case BACK:
                if (GamepadStatic.isButtonPressed(gamepad, Controls.INTAKE) && !intake.isSample()) {
                    intake.intakeBack();
                } else if (GamepadStatic.isButtonPressed(gamepad, Controls.OUTTAKE)) {
                    intake.outtakeBack();
                } else {
                    intake.stop();
                }
                break;
        }
    }

}
