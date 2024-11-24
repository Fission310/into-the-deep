package org.firstinspires.ftc.teamcode.opmode.auton.clip;

import static org.firstinspires.ftc.teamcode.opmode.auton.util.GameConstants.DOWN;
import static org.firstinspires.ftc.teamcode.opmode.auton.util.GameConstants.RIGHT;
import static org.firstinspires.ftc.teamcode.opmode.auton.util.GameConstants.TILE_LENGTH;
import static org.firstinspires.ftc.teamcode.opmode.auton.util.GameConstants.UP;

import org.firstinspires.ftc.teamcode.opmode.auton.clip.setup.ClipConstants;
import org.firstinspires.ftc.teamcode.opmode.auton.util.Constant;

public class ClipConstantsBlue extends ClipConstants {
    public ClipConstantsBlue(){
        CHAMBER = new Constant(
                TILE_LENGTH * -0.75 , TILE_LENGTH, DOWN
        );

        WALL_SAMPLE = new Constant(
                TILE_LENGTH * -3, TILE_LENGTH, DOWN
        );

        CENTER_SAMPLE = new Constant(
                TILE_LENGTH * -2.5, TILE_LENGTH, DOWN
        );

        FAR_SAMPLE = new Constant(
                TILE_LENGTH * -2, TILE_LENGTH, DOWN
        );

        OBSERVATION_1 = new Constant(
                TILE_LENGTH * -2.5, TILE_LENGTH * 2.5, Math.toRadians(135)
        );

        OBSERVATION_2 = new Constant(
                TILE_LENGTH * -2.5, TILE_LENGTH * 2.5, Math.toRadians(135)
        );

        OBSERVATION_3 = new Constant(
                TILE_LENGTH * -2.5, TILE_LENGTH * 2.5, Math.toRadians(135)
        );
    }
}
