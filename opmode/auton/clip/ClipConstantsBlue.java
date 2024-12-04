package org.firstinspires.ftc.teamcode.opmode.auton.clip;

import static org.firstinspires.ftc.teamcode.opmode.auton.util.GameConstants.*;

import org.firstinspires.ftc.teamcode.opmode.auton.clip.setup.ClipConstants;
import org.firstinspires.ftc.teamcode.opmode.auton.util.Constant;

public class ClipConstantsBlue extends ClipConstants {
    public ClipConstantsBlue(){
        START_CHAMBER = new Constant(
                0,
                -TILE_LENGTH,
                Math.toRadians(90)
        );
        CHAMBER_1 = new Constant(
                5,
                -TILE_LENGTH,
                Math.toRadians(90)
        );
        CHAMBER_2 = new Constant(
                8,
                -TILE_LENGTH,
                Math.toRadians(90)
        );
        CHAMBER_3 = new Constant(
                11,
                -TILE_LENGTH,
                Math.toRadians(90)
        );
        FAR_SAMPLE = new Constant(
                TILE_LENGTH * 1.5,
                -TILE_LENGTH * 1.25,
                Math.toRadians(10)
        );
        WALL_SAMPLE = new Constant(
                TILE_LENGTH * 2,
                -TILE_LENGTH * 1.25,
                Math.toRadians(10)
        );
        CENTER_SAMPLE = new Constant(
                TILE_LENGTH * 1.75,
                -TILE_LENGTH * 1.25,
                Math.toRadians(10)
        );
        SAMPLE_DROP_1 = new Constant(
                TILE_LENGTH * 1.5,
                - TILE_LENGTH * 1.75,
                Math.toRadians(-45)
        );
        SAMPLE_DROP_2 = new Constant(
                TILE_LENGTH * 1.5,
                - TILE_LENGTH * 1.75,
                Math.toRadians(-45)
        );
        SAMPLE_DROP_3 = new Constant(
                TILE_LENGTH * 1.5,
                - TILE_LENGTH * 1.75,
                Math.toRadians(-45)
        );
        WALL_INTAKE_1 = new Constant(
                TILE_LENGTH * 1.25,
                - TILE_LENGTH * 2,
                Math.toRadians(-45)
        );
        WALL_INTAKE_2 = new Constant(
                TILE_LENGTH * 1.25,
                - TILE_LENGTH * 2,
                Math.toRadians(-45)
        );
        WALL_INTAKE_3 = new Constant(
                TILE_LENGTH * 1.25,
                - TILE_LENGTH * 2,
                Math.toRadians(-45)
        );
    }
}
