package org.firstinspires.ftc.teamcode.opmode.auton.clip;

import static org.firstinspires.ftc.teamcode.opmode.auton.util.GameConstants.TILE_LENGTH;

import org.firstinspires.ftc.teamcode.opmode.auton.util.Constant;

public class ClipConstants{
    public Constant START_CHAMBER = new Constant(
                0,
                        -TILE_LENGTH,
                Math.toRadians(90)
        );
    public Constant CHAMBER_1 = new Constant(
                5,
                        -TILE_LENGTH,
                Math.toRadians(90)
        );
    public Constant CHAMBER_2 = new Constant(
                8,
                        -TILE_LENGTH,
                Math.toRadians(90)
        );
    public Constant CHAMBER_3 = new Constant(
                11,
                        -TILE_LENGTH,
                Math.toRadians(90)
        );
    public Constant FAR_SAMPLE = new Constant(
            TILE_LENGTH * 1.5,
                -TILE_LENGTH * 1.25,
            Math.toRadians(10)
        );
    public Constant WALL_SAMPLE = new Constant(
            TILE_LENGTH * 2,
                -TILE_LENGTH * 1.25,
            Math.toRadians(10)
        );
    public Constant CENTER_SAMPLE = new Constant(
            TILE_LENGTH * 1.75,
                -TILE_LENGTH * 1.25,
            Math.toRadians(10)
        );
    public Constant SAMPLE_DROP_1 = new Constant(
            TILE_LENGTH * 1.5,
                - TILE_LENGTH * 1.75,
            Math.toRadians(-45)
        );
    public Constant SAMPLE_DROP_2 = new Constant(
            TILE_LENGTH * 1.5,
                - TILE_LENGTH * 1.75,
            Math.toRadians(-45)
        );
    public Constant SAMPLE_DROP_3 = new Constant(
            TILE_LENGTH * 1.5,
                - TILE_LENGTH * 1.75,
            Math.toRadians(-45)
        );
    public Constant WALL_INTAKE_1 = new Constant(
            TILE_LENGTH * 1.25,
                - TILE_LENGTH * 2,
            Math.toRadians(-45)
        );
    public Constant WALL_INTAKE_2 = new Constant(
            TILE_LENGTH * 1.25,
                - TILE_LENGTH * 2,
            Math.toRadians(-45)
        );
    public Constant WALL_INTAKE_3 = new Constant(
            TILE_LENGTH * 1.25,
                - TILE_LENGTH * 2,
            Math.toRadians(-45)
        );

}