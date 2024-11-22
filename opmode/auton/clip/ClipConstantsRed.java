package org.firstinspires.ftc.teamcode.opmode.auton.clip;

import static org.firstinspires.ftc.teamcode.opmode.auton.util.GameConstants.DOWN;
import static org.firstinspires.ftc.teamcode.opmode.auton.util.GameConstants.TILE_LENGTH;
import static org.firstinspires.ftc.teamcode.opmode.auton.util.GameConstants.UP;

import org.firstinspires.ftc.teamcode.opmode.auton.clip.setup.ClipConstants;
import org.firstinspires.ftc.teamcode.opmode.auton.util.Constant;

public class ClipConstantsRed extends ClipConstants {
    public ClipConstantsRed(){
        CHAMBER = new Constant(
                TILE_LENGTH * 0.75 , -TILE_LENGTH, UP
        );

        WALL_SAMPLE = new Constant(
                TILE_LENGTH * 3, -TILE_LENGTH, UP
        );

        CENTER_SAMPLE = new Constant(
                TILE_LENGTH * 2.5, -TILE_LENGTH, UP
        );

        FAR_SAMPLE = new Constant(
                TILE_LENGTH * 2, -TILE_LENGTH, UP
        );

        OBSERVATION_1 = new Constant(
                TILE_LENGTH * 2.5, TILE_LENGTH * -2.5, Math.toRadians(305)
        );

        OBSERVATION_2 = new Constant(
                TILE_LENGTH * 2.5, TILE_LENGTH * -2.5, Math.toRadians(305)
        );

        OBSERVATION_3 = new Constant(
                TILE_LENGTH * 2.5, TILE_LENGTH * -2.5, Math.toRadians(305)
        );

        PARK = new Constant(
                TILE_LENGTH * .5, TILE_LENGTH * -3, DOWN
        );
    }
}
