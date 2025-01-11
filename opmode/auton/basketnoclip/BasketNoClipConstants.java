package org.firstinspires.ftc.teamcode.opmode.auton.basketnoclip;

import static org.firstinspires.ftc.teamcode.opmode.auton.util.GameConstants.*;

import org.firstinspires.ftc.teamcode.opmode.auton.util.Constant;

public class BasketNoClipConstants {
    public static Constant FORWARD = new Constant(
            -BOT_LENGTH * 1.5,
            -TILE_LENGTH * 1.8,
            UP
    );
    public static Constant BASKET = new Constant(
            -TILE_LENGTH * 2.5 + BOT_WIDTH / 2 - 21,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 + 0.5,
            Math.toRadians(45)
    );

    public static Constant NO_CLIP_CHAMBER = new Constant(
            0,
            -TILE_LENGTH,
            UP
    );
    public static Constant FAR_SAMPLE = new Constant(
            -TILE_LENGTH * 2.5 + BOT_WIDTH / 2 + 4 + 5,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 + 1,
            UP
    );
    public static Constant FAR_SAMPLE_INT = new Constant(
            -TILE_LENGTH * 2.5 + BOT_WIDTH / 2 + 4 + 5,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 + 1 + 3,
            UP + Math.toRadians(18)
    );
    public static Constant CENTER_SAMPLE = new Constant(
            -TILE_LENGTH * 2.5 + BOT_WIDTH / 2 - 5 + 2,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 + 6,
            UP
    );
    public static Constant CENTER_SAMPLE_INT = new Constant(
            -TILE_LENGTH * 2.5 + BOT_WIDTH / 2 - 5 + 2,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 + 6 + 3,
            UP + Math.toRadians(18)
    );
    public static Constant WALL_SAMPLE = new Constant(
            -TILE_LENGTH * 2.0,
            -TILE_LENGTH * 3 / 2 - BOT_LENGTH,
            Math.toRadians(135)
    );
    public static Constant WALL_SAMPLE_INT = new Constant(
            -TILE_LENGTH * 2.0,
            -TILE_LENGTH * 3 / 2 - BOT_LENGTH + 2,
            Math.toRadians(140)
    );
    public static Constant BASKET_1 = new Constant(
            -TILE_LENGTH * 2.5 - BOT_WIDTH + 16,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 - 2,
            Math.toRadians(41)
    );
    public static Constant BASKET_2 = new Constant(
            -TILE_LENGTH * 2.5 - BOT_WIDTH + 16,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 - 2,
            Math.toRadians(45)
    );
    public static Constant BASKET_3 = new Constant(
            -TILE_LENGTH * 2.5 - BOT_WIDTH + 16,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 - 2,
            Math.toRadians(45)
    );
    public static Constant BASKET_4 = new Constant(
            -TILE_LENGTH * 2.5 - BOT_WIDTH + 16,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 - 2,
            Math.toRadians(45)
    );
}
