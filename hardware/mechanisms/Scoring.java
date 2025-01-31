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
    private Telescope telescope = new Telescope(opMode);
    public Pivot pivot = new Pivot(opMode, telescope);
    private Wrist wrist = new Wrist(opMode);
    private Sweeper sweeper = new Sweeper(opMode);

    private State state = State.FRONT;

    private enum State {
        FRONT,
        INTAKE,
        WALL,
        BASKET,
        UP,
        CLIP,
    }

    public static double BASKET_OUTTAKE_WAIT = 0.4;
    public static double BASKET_RELEASE_WAIT = 0.2;
    public static double BASKET_RETRACT_WAIT = 0.1;
    public static double UP_POS_WAIT = 0.1;
    public static double TELESCOPE_RETRACT_WAIT = 0.10;
    public static double WRIST_RETRACT_WAIT = 0.10;
    public static double PIVOT_DOWN_WAIT = 0.1;
    public static double PIVOT_GRAB_WAIT = 0.3;
    public static double PIVOT_UP_WAIT = 0.5;
    public static double CLIP_EXTEND_WAIT = 1;
    public static double CLIP_CLIP_WAIT = 0.3;
    public static double CLIP_RELEASE_WAIT = 0.1;
    public static double CLIP_PIVOT_WAIT = 0.1;

    private boolean frontClicked = false;
    private boolean dpadClicked = false;
    private boolean rightStickClicked = false;

    public Scoring(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    private Command release = () -> claw.release();
    private Command grab = () -> claw.grab();
    private Command stopIntake = () -> claw.stop();
    private Command pivotFront = () -> pivot.frontPos();
    private Command pivotUpIntake = () -> pivot.intakeUpPos();
    private Command pivotDownIntake = () -> pivot.intakeDownPos();
    private Command pivotGrabIntake = () -> pivot.intakeGrabPos();
    private Command telescopeFront = () -> telescope.frontPos();
    private Command telescopeIntake = () -> telescope.frontIntakePos();
    private Command telescopeIntakeShort = () -> telescope.frontIntakeShortPos();
    private Command setStateFront = () -> state = State.FRONT;
    private Command setStateIntake = () -> state = State.INTAKE;
    private Command setStateUp = () -> state = State.UP;
    private Command sweeperRetract = () -> sweeper.retractPos();
    private Command telescopeRetract = () -> telescope.frontPos();
    private Command wristIntakeScore = () -> wrist.intakePos();
    private Command wristClipScore = () -> wrist.clipScorePos();
    private Command wristRetract = () -> wrist.frontPos();
    private Command pivotUp = () -> pivot.upPos();
    private Command pivotClipDown = () -> pivot.clipDownPos();
    private Command telescopeScoreClip = () -> telescope.clipScorePos();

    private CommandSequence intakeGrab = new CommandSequence()
            .addCommand(grab)
            .build();

    private CommandSequence scoreBasket = new CommandSequence()
            .addCommand(release)
            .addWaitCommand(BASKET_RELEASE_WAIT)
            .addCommand(wristIntakeScore)
            .addWaitCommand(BASKET_OUTTAKE_WAIT)
            .addCommand(stopIntake)
            .addCommand(telescopeFront)
            .addWaitCommand(BASKET_RETRACT_WAIT)
            .addCommand(wristRetract)
            .addWaitCommand(UP_POS_WAIT)
            .addCommand(pivotFront)
            .addCommand(setStateFront)
            .build();

    private CommandSequence scoreClip = new CommandSequence()
            .addCommand(pivotClipDown)
            .addCommand(telescopeScoreClip)
            .addWaitCommand(CLIP_PIVOT_WAIT)
            .addCommand(wristClipScore)
            .addWaitCommand(CLIP_CLIP_WAIT)
            .addCommand(release)
            .addWaitCommand(CLIP_RELEASE_WAIT)
            .addCommand(pivotFront)
            .addCommand(wristRetract)
            .addCommand(telescopeFront)
            .addCommand(setStateFront)
            .build();

    public CommandSequence frontIntake = new CommandSequence()
            .addCommand(pivotDownIntake)
            .addCommand(telescopeIntake)
            .addWaitCommand(PIVOT_DOWN_WAIT)
            .addCommand(wristIntakeScore)
            .addCommand(setStateIntake)
            .addCommand(release)
            .build();

    public CommandSequence frontIntakeShort = new CommandSequence()
            .addCommand(sweeperRetract)
            .addCommand(pivotDownIntake)
            .addCommand(telescopeIntakeShort)
            .addWaitCommand(PIVOT_DOWN_WAIT)
            .addCommand(wristIntakeScore)
            .addCommand(setStateIntake)
            .addCommand(release)
            .build();

    public CommandSequence grabIntake = new CommandSequence()
            .addCommand(pivotGrabIntake)
            .addWaitCommand(PIVOT_GRAB_WAIT)
            .addCommand(grab)
            .build();

    public CommandSequence retractTele = new CommandSequence()
            .addCommand(pivotUpIntake)
            .addCommand(wristRetract)
            .addWaitCommand(TELESCOPE_RETRACT_WAIT)
            .addCommand(telescopeRetract)
            .addWaitCommand(PIVOT_UP_WAIT)
            .addCommand(pivotUp)
            .addCommand(setStateUp)
            .build();

    public void goFront() {
        state = State.FRONT;
        pivot.frontPos();
        telescope.frontPos();
        wrist.frontPos();
        wrist.defaultPos();
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

    public void goLowBasket() {
        state = State.BASKET;
        pivot.basketPos();
        telescope.lowBasketPos();
        wrist.basketPos();
    }

    public void goClip() {
        state = State.CLIP;
        pivot.clipPos();
        telescope.clipPos();
        wrist.clipPos();
    }

    @Override
    public void init(HardwareMap hwMap) {
        drivetrain.init(hwMap);
        claw.init(hwMap);
        pivot.init(hwMap);
        telescope.init(hwMap);
        wrist.init(hwMap);
        sweeper.init(hwMap);
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

        if (GamepadStatic.isButtonPressed(gamepad, Controls.PIVOT_FRONT) && state != State.FRONT
                && state != State.INTAKE) {
            goFront();
            frontClicked = true;
        } else if (GamepadStatic.isButtonPressed(gamepad, Controls.PIVOT_WALL) && state != State.WALL) {
            goWall();
            claw.release();
        } else if (GamepadStatic.isButtonPressed(gamepad, Controls.PIVOT_BASKET) && state != State.BASKET) {
            goBasket();
        } else if (GamepadStatic.isButtonPressed(gamepad, Controls.PIVOT_LOW_BASKET) && state != State.BASKET) {
            goLowBasket();
        } else if (GamepadStatic.isButtonPressed(gamepad, Controls.PIVOT_CLIP) && state != State.CLIP) {
            goClip();
        }

        if (GamepadStatic.isButtonPressed(gamepad, Controls.SWEEP)) {
            if (!rightStickClicked) {
                sweeper.toggle();
            }
            rightStickClicked = true;
        } else {
            rightStickClicked = false;
        }

        switch (state) {
            case UP:
                sweeper.retractPos();
                drivetrain.setNormal();
                claw.stop();
                break;
            case FRONT:
                drivetrain.setNormal();
                if (GamepadStatic.isButtonPressed(gamepad, Controls.INTAKE_SHORT)) {
                    frontIntakeShort.trigger();
                }
                if (GamepadStatic.isButtonPressed(gamepad, Controls.INTAKE)) {
                    if (!frontClicked) {
                        frontIntake.trigger();
                    }
                    frontClicked = true;
                } else {
                    frontClicked = false;
                }
                break;
            case INTAKE:
                drivetrain.setIntake();
                wrist.loop(gamepad);
                if (GamepadStatic.isButtonPressed(gamepad, Controls.PIVOT_FRONT)) {
                    if (!frontClicked) {
                        retractTele.trigger();
                    }
                    frontClicked = true;
                } else {
                    frontClicked = false;
                }
                if (GamepadStatic.isButtonPressed(gamepad, Controls.GRAB)) {
                    grabIntake.trigger();
                    claw.grab();
                }
                if (GamepadStatic.isButtonPressed(gamepad, Controls.WRIST_DOWN)) {
                    wrist.down();
                    pivot.intakeUpPos();
                    claw.grab();
                }
                if (GamepadStatic.isButtonPressed(gamepad, Controls.WRIST_UP)) {
                    wrist.intakePos();
                    pivot.intakeGrabPos();
                    claw.grab();
                }
                if (GamepadStatic.isButtonPressed(gamepad, Controls.RELEASE)) {
                    claw.release();
                    pivot.intakeDownPos();
                }
                break;
            case WALL:
                drivetrain.setIntake();
                claw.loop(gamepad);
                break;
            case BASKET:
                drivetrain.setScore();
                if (GamepadStatic.isButtonPressed(gamepad, Controls.RELEASE)) {
                    scoreBasket.trigger();
                }
                break;
            case CLIP:
                drivetrain.setScore();
                if (GamepadStatic.isButtonPressed(gamepad, Controls.RELEASE)) {
                    scoreClip.trigger();
                }
                break;

        }
    }
}
