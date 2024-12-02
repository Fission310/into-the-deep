package org.firstinspires.ftc.teamcode.hardware.mechanisms;

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

@Config
public class Scoring extends Mechanism {
    private Drivetrain drivetrain = new Drivetrain(opMode); // OPMODE NULL HERE
    private Claw claw = new Claw(opMode);
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

    public static double BASKET_OUTTAKE_WAIT = 0.5;
    public static double BASKET_RETRACT_WAIT = 0.1;
    public static double TELESCOPE_RETRACT_WAIT = 0.10;
    public static double WRIST_RETRACT_WAIT = 0.10;
    public static double PIVOT_DOWN_WAIT = 0.4;
    public static double CLIP_WRIST_WAIT = 0.10;
    public static double CLIP_CLIP_WAIT = 0.10;
    public static double CLIP_OUTTAKE_WAIT = 0.10;

    public Scoring(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    private Command release = () -> claw.release();
    private Command pivotFront = () -> pivot.frontPos();
    private Command pivotUpIntake = () -> pivot.intakeUpPos();
    private Command pivotDownIntake = () -> pivot.intakeDownPos();
    private Command telescopeFront = () -> telescope.frontPos();
    private Command telescopeIntake = () -> telescope.frontIntakePos();
    private Command setStateFront = () -> state = State.FRONT;
    private Command telescopeExtendClip = () -> telescope.clipExtensionPos();
    private Command telescopeRetract = () -> telescope.frontPos();
    private Command wristIntakeScore = () -> wrist.frontIntakePos();
    private Command wristRetract = () -> wrist.frontPos();
    private Command wristClipScore = () -> wrist.clipScorePos();
    private Command pivotUp = () -> pivot.upPos();

    private CommandSequence scoreBasket = new CommandSequence()
            .addCommand(release)
            .addCommand(pivotUp)
            .addWaitCommand(BASKET_RETRACT_WAIT)
            .addCommand(telescopeFront)
            .addWaitCommand(BASKET_RETRACT_WAIT)
            .addCommand(pivotFront)
            .addCommand(setStateFront)
            .build();

    private CommandSequence scoreClip = new CommandSequence()
            .addCommand(telescopeExtendClip)
            .addWaitCommand(CLIP_WRIST_WAIT)
            .addCommand(wristClipScore)
            .addCommand(telescopeFront)
            .addWaitCommand(CLIP_CLIP_WAIT)
            .addCommand(release)
            .addCommand(pivotFront)
            .addCommand(setStateFront)
            .build();

    public CommandSequence frontIntake = new CommandSequence()
            .addCommand(pivotUpIntake)
            .addCommand(telescopeIntake)
            .addWaitCommand(PIVOT_DOWN_WAIT)
            .addCommand(pivotDownIntake)
            .addCommand(wristIntakeScore)
            .build();

    public CommandSequence retractTele = new CommandSequence()
            .addCommand(pivotUpIntake)
            .addCommand(wristRetract)
            .addWaitCommand(TELESCOPE_RETRACT_WAIT)
            .addCommand(telescopeRetract)
            .addWaitCommand(PIVOT_DOWN_WAIT)
            .addCommand(pivotDownIntake)
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
        claw.init(hwMap);
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

        if (GamepadStatic.isButtonPressed(gamepad, Controls.TELE_EXTEND)) {
            telescope.upABit();
        }

        if (GamepadStatic.isButtonPressed(gamepad, Controls.TELE_RETRACT)) {
            telescope.downABit();
        }

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
                claw.loop(gamepad);
                break;
            case WALL:
                claw.loop(gamepad);
                break;
            case BASKET:
                if (GamepadStatic.isButtonPressed(gamepad, Controls.SCORE)) {
                    scoreBasket.trigger();
                }
                break;
            case CLIP:
                if (GamepadStatic.isButtonPressed(gamepad, Controls.SCORE)) {
                    scoreClip.trigger();
                }
                break;
            case BACK:
                claw.loop(gamepad);
                break;
        }
    }

}
