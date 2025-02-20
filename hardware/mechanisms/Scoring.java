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
    private Drivetrain drivetrain = new Drivetrain(opMode); // OPMODE NULL HERE
    private Intake intake = new Intake(opMode);
    private Telescope telescope = new Telescope(opMode);
    public Pivot pivot = new Pivot(opMode, telescope);
    private Wrist wrist = new Wrist(opMode);
    private Sweeper sweeper = new Sweeper(opMode);

    private State state = State.FRONT;
    private Color color;

    private enum State {
        FRONT,
        INTAKE,
        WALL,
        BASKET,
        UP,
        CLIP,
        CLIMB_UP,
        CLIMB_DOWN,
    }

    public static double BASKET_OUTTAKE_WAIT = 0.1;
    public static double BASKET_RELEASE_WAIT = 0.2;
    public static double BASKET_RETRACT_WAIT = 0.1;
    public static double UP_POS_WAIT = 0.1;
    public static double TELESCOPE_RETRACT_WAIT = 0.10;
    public static double WRIST_RETRACT_WAIT = 0.10;
    public static double PIVOT_DOWN_WAIT = 0.25;
    public static double PIVOT_GRAB_WAIT = 0.2;
    public static double PIVOT_UP_WAIT = 0.3;
    public static double CLIP_EXTEND_WAIT = 1;
    public static double CLIP_CLIP_WAIT = 0.3;
    public static double CLIP_RELEASE_WAIT = 0.1;
    public static double CLIP_PIVOT_WAIT = 0.1;
    public static double CLIMB_UP_WAIT = 0.5;
    public static double CLIMB_DOWN_WAIT = 1;

    private boolean climbPressed = false;
    private boolean frontClicked = false;
    private boolean dpadClicked = false;
    private boolean rightStickClicked = false;

    public Scoring(LinearOpMode opMode, Color color) {
        this.opMode = opMode;
        this.color = color;
    }

    private Command intakeCommand = () -> intake.intake();
    private Command outakeCommand = () -> intake.outtake();
    private Command stopIntake = () -> intake.stop();
    private Command pivotFront = () -> pivot.frontPos();
    private Command pivotUpIntake = () -> pivot.intakeUpPos();
    private Command pivotDownIntake = () -> pivot.intakeDownPos();
    private Command pivotGrabIntake = () -> pivot.intakeGrabPos();
    private Command telescopeVerticalFront = () -> telescope.frontVerticalPos();
    private Command telescopeHorizontalFront = () -> telescope.frontHorizontalPos();
    private Command telescopeIntake = () -> telescope.frontIntakePos();
    private Command telescopeIntakeShort = () -> telescope.frontIntakeShortPos();
    private Command telescopeClimbDownPos = () -> telescope.climbDownPos();
    private Command telescopeClimbUpPos = () -> telescope.climbUpPos1();
    private Command telescopeClimbUpPos2 = () -> telescope.climbUpPos2();
    private Command setStateFront = () -> state = State.FRONT;
    private Command setStateIntake = () -> state = State.INTAKE;
    private Command setStateUp = () -> state = State.UP;
    private Command wristIntakeScore = () -> wrist.intakePos();
    private Command wristIntakeMid = () -> wrist.intakeMitPos();
    private Command wristIntakeShortScore = () -> wrist.intakeShortPos();
    private Command wristClipScore = () -> wrist.clipScorePos();
    private Command wristClimbPos = () -> wrist.climbPos();
    private Command wristRetract = () -> wrist.frontPos();
    private Command pivotUp = () -> pivot.upPos();
    private Command pivotClipDown = () -> pivot.clipDownPos();
    private Command pivotClimbDownPos = () -> pivot.climbDownPos();
    private Command pivotClimbUpPos = () -> pivot.climbUpPos();
    private Command telescopeScoreClip = () -> telescope.clipScorePos();

    private CommandSequence intakeGrab = new CommandSequence()
            .addCommand(intakeCommand)
            .build();

    private CommandSequence scoreBasket = new CommandSequence()
            .addCommand(outakeCommand)
            .addWaitCommand(BASKET_RELEASE_WAIT)
            .addCommand(wristIntakeScore)
            .addWaitCommand(BASKET_OUTTAKE_WAIT)
            .addCommand(stopIntake)
            .addCommand(telescopeVerticalFront)
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
            .addCommand(outakeCommand)
            .addWaitCommand(CLIP_RELEASE_WAIT)
            .addCommand(pivotFront)
            .addCommand(wristRetract)
            .addCommand(telescopeHorizontalFront)
            .addCommand(setStateFront)
            .build();

    public CommandSequence frontIntake = new CommandSequence()
            .addCommand(outakeCommand)
            .addCommand(pivotDownIntake)
            .addCommand(telescopeIntake)
            .addCommand(wristIntakeMid)
            .addWaitCommand(PIVOT_DOWN_WAIT)
            .addCommand(setStateIntake)
            .build();

    public CommandSequence frontIntakeShort = new CommandSequence()
            .addCommand(outakeCommand)
            .addCommand(pivotDownIntake)
            .addCommand(telescopeIntakeShort)
            .addCommand(wristIntakeMid)
            .addWaitCommand(PIVOT_DOWN_WAIT)
            .addCommand(setStateIntake)
            .build();

    public CommandSequence grabIntake = new CommandSequence()
            .addCommand(pivotGrabIntake)
            .addCommand(wristIntakeScore)
            .addWaitCommand(PIVOT_GRAB_WAIT)
            .addCommand(intakeCommand)
            .build();

    public CommandSequence retractTele = new CommandSequence()
            .addCommand(pivotUpIntake)
            .addCommand(wristRetract)
            .addWaitCommand(TELESCOPE_RETRACT_WAIT)
            .addCommand(telescopeHorizontalFront)
            .addWaitCommand(PIVOT_UP_WAIT)
            .addCommand(pivotUp)
            .addCommand(setStateUp)
            .build();

    public CommandSequence climbUp = new CommandSequence()
            .addCommand(stopIntake)
            .addCommand(pivotClimbUpPos)
            .addWaitCommand(CLIMB_UP_WAIT)
            .addCommand(telescopeClimbUpPos)
            .addCommand(wristClimbPos)
            .build();
    public CommandSequence climbDown = new CommandSequence()
            .addCommand(telescopeClimbUpPos2)
            .addWaitCommand(CLIMB_DOWN_WAIT)
            .addCommand(telescopeClimbDownPos)
            .addCommand(pivotClimbDownPos)
            .build();

    public void goFront() {
        state = State.FRONT;
        pivot.frontPos();
        telescope.frontHorizontalPos();
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
        intake.init(hwMap);
        pivot.init(hwMap);
        telescope.init(hwMap);
        wrist.init(hwMap);
        sweeper.init(hwMap);
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("state", state);
        drivetrain.telemetry(telemetry);
        intake.telemetry(telemetry);
        pivot.telemetry(telemetry);
        telescope.telemetry(telemetry);
        wrist.telemetry(telemetry);
        sweeper.telemetry(telemetry);
    }

    @Override
    public void loop(Gamepad gamepad) {
        drivetrain.loop(gamepad);
        pivot.update();
        telescope.update();
        intake.update();

        if (GamepadStatic.isButtonPressed(gamepad, Controls.TELE_EXTEND)) {
            telescope.upABit();
            if (state == State.BASKET) {
                wrist.basketABit();
            } else if (state == State.INTAKE) {
                wrist.intakeABit();
            }
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
            intake.outtake();
        } else if (GamepadStatic.isButtonPressed(gamepad, Controls.PIVOT_BASKET) && state != State.INTAKE) {
            goBasket();
        } else if (GamepadStatic.isButtonPressed(gamepad, Controls.PIVOT_LOW_BASKET)) {
            goLowBasket();
        } else if (GamepadStatic.isButtonPressed(gamepad, Controls.PIVOT_CLIP) && state != State.CLIP) {
            goClip();
        } else if (GamepadStatic.isButtonPressed(gamepad, Controls.CLIMB_1) && state != State.INTAKE) {
            if (state != State.CLIMB_DOWN && state != State.CLIMB_UP && !climbPressed) {
                state = State.CLIMB_UP;
                climbUp.trigger();
            } else if (state == State.CLIMB_UP && !climbPressed) {
                state = State.CLIMB_DOWN;
                climbDown.trigger();
            }
            climbPressed = true;
        }
        if (!GamepadStatic.isButtonPressed(gamepad, Controls.CLIMB_1)) {
            climbPressed = false;
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
                intake.intake();
                break;
            case FRONT:
                if (GamepadStatic.isButtonPressed(gamepad, Controls.OUTTAKE)) {
                    intake.outtake();
                }
                drivetrain.setNormal();
                if (GamepadStatic.isButtonPressed(gamepad, Controls.INTAKE_SHORT)) {
                    frontIntakeShort.trigger();
                    sweeper.extendPos();
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
                // wrist.loop(gamepad);
                if (intake.hasSample()) {
                    if (intake.hasWrongColor(color)) {
                        //intake.outtake();
                    } else {
                        intake.intake();
                    }
                    if (intake.hasColor(color)) {
                        retractTele.trigger();
                    }
                }
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
                    sweeper.retractPos();
                    intake.intake();
                }
                if (GamepadStatic.isButtonPressed(gamepad, Controls.WRIST_DOWN)) {
                    wrist.down();
                    pivot.intakeUpPos();
                    intake.intake();
                }
                if (GamepadStatic.isButtonPressed(gamepad, Controls.WRIST_UP)) {
                    wrist.intakePos();
                    pivot.intakeGrabPos();
                    intake.intake();
                }
                if (GamepadStatic.isButtonPressed(gamepad, Controls.OUTTAKE)) {
                    intake.outtake();
                    pivot.intakeDownPos();
                }
                break;
            case WALL:
                drivetrain.setIntake();
                intake.loop(gamepad);
                break;
            case BASKET:
                drivetrain.setScore();
                if (GamepadStatic.isButtonPressed(gamepad, Controls.OUTTAKE)) {
                    scoreBasket.trigger();
                }
                break;
            case CLIP:
                drivetrain.setScore();
                if (GamepadStatic.isButtonPressed(gamepad, Controls.OUTTAKE)) {
                    scoreClip.trigger();
                }
                break;
            case CLIMB_UP:
                drivetrain.setScore();
                break;
            case CLIMB_DOWN:
                drivetrain.setScore();
                break;
        }
    }
}
