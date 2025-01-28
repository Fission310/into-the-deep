package org.firstinspires.ftc.teamcode.opmode.auton.basketnoclip;

import static org.firstinspires.ftc.teamcode.opmode.auton.util.GameConstants.*;

import org.firstinspires.ftc.teamcode.opmode.auton.util.Constant;

public class BasketNoClipConstants {
    public static Constant BASKET_1 = new Constant(
            -TILE_LENGTH * 2.5 - BOT_WIDTH + 22.5,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 - 8,
            Math.toRadians(37.5)
    );
    public static Constant FAR_SAMPLE = new Constant(
            -TILE_LENGTH * 2.5 - BOT_WIDTH + 26,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 - 3.5,
            UP - Math.toRadians(5)
    );
    public static Constant FAR_SAMPLE_INT = new Constant(
            -TILE_LENGTH * 2.5 - BOT_WIDTH + 24,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 - 0.5,
            UP + Math.toRadians(7)
    );
    public static Constant CENTER_SAMPLE = new Constant(
            -TILE_LENGTH * 2.5 - BOT_WIDTH + 17.0,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 - 3.5,
            UP - Math.toRadians(5)
    );
    public static Constant CENTER_SAMPLE_INT = new Constant(
            -TILE_LENGTH * 2.5 - BOT_WIDTH + 16.0,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 - 1.2,
            UP + Math.toRadians(9)
    );
    public static Constant WALL_SAMPLE = new Constant(
            -TILE_LENGTH - BOT_WIDTH / 2 - 1,
            -TILE_LENGTH  - BOT_LENGTH / 2,
            UP + Math.toRadians(90)
    );
    public static Constant WALL_SAMPLE_INT = new Constant(
            -TILE_LENGTH - BOT_WIDTH / 2 - 1.5,
            -TILE_LENGTH  - BOT_LENGTH / 2 + 0.5,
            UP + Math.toRadians(87)
    );
    public static Constant BASKET_2 = new Constant(
            -TILE_LENGTH * 2.5 - BOT_WIDTH + 21.0,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 - 5.5,
            Math.toRadians(32.5)
    );
    public static Constant BASKET_3 = new Constant(
            -TILE_LENGTH * 2.5 - BOT_WIDTH + 20.0,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 - 4.5,
            Math.toRadians(37.5)
    );
    public static Constant BASKET_4 = new Constant(
            -TILE_LENGTH * 2.5 - BOT_WIDTH + 21.0,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 - 5.5,
            Math.toRadians(32.5)
    );
}
