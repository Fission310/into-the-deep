package org.firstinspires.ftc.teamcode.opmode.auton.basketnoclip;

import static org.firstinspires.ftc.teamcode.opmode.auton.util.GameConstants.*;

import org.firstinspires.ftc.teamcode.opmode.auton.util.Constant;

import com.acmerobotics.dashboard.config.Config;

@Config
public class BasketConstants {
    public static Constant START = new Constant(
            -TILE_LENGTH * 2 + BOT_WIDTH / 2,
            -WALL_POS + BOT_LENGTH / 2,
            UP
    );
    public static Constant BASKET_1 = new Constant(
            -TILE_LENGTH * 2.5 - BOT_WIDTH + 22.5,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 - 1,
            Math.toRadians(43)
    );
    public static Constant BASKET_2 = new Constant(
            -TILE_LENGTH * 2.5 - BOT_WIDTH + 20.5,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 + 0.5,
            Math.toRadians(37)
    );
    public static Constant BASKET_3 = new Constant(
            -TILE_LENGTH * 2.5 - BOT_WIDTH + 21.5,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 - 2,
            Math.toRadians(40)
    );
    public static Constant BASKET_4 = new Constant(
            -TILE_LENGTH * 2.5 - BOT_WIDTH + 20.5,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2,
            Math.toRadians(41)
    );
    public static Constant BASKET_5 = new Constant(
            -TILE_LENGTH * 2.5 - BOT_WIDTH + 20.5,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2,
            Math.toRadians(41)
    );
    public static Constant BASKET_6 = new Constant(
            -TILE_LENGTH * 2.5 - BOT_WIDTH + 20.5,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2,
            Math.toRadians(41)
    );
    public static Constant FAR_SAMPLE = new Constant(
            -TILE_LENGTH * 2.5 - BOT_WIDTH + 25,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 - 1.8,
            UP + Math.toRadians(2)
    );
    public static Constant FAR_SAMPLE_INT = new Constant(
            -TILE_LENGTH * 2.5 - BOT_WIDTH + 24.5,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 + 1,
            UP + Math.toRadians(3)
    );
    public static Constant CENTER_SAMPLE = new Constant(
            -TILE_LENGTH * 2.5 - BOT_WIDTH + 17.5,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 - 2.9,
            UP + Math.toRadians(2)
    );
    public static Constant CENTER_SAMPLE_INT = new Constant(
            -TILE_LENGTH * 2.5 - BOT_WIDTH + 16.0,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 + 3.3,
            UP + Math.toRadians(5)
    );
    public static Constant WALL_SAMPLE = new Constant(
            -TILE_LENGTH * 2.5 - BOT_WIDTH + 17,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 - 4,
            UP + Math.toRadians(14)
    );
    public static Constant WALL_SAMPLE_INT = new Constant(
            -TILE_LENGTH * 2.5 - BOT_WIDTH + 13,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2,
            UP + Math.toRadians(25)
    );
    public static Constant SUBMERSIBLE_1 = new Constant(
            -TILE_LENGTH,
            - TILE_LENGTH / 2,
            Math.toRadians(0)
    );
    public static Constant SUBMERSIBLE_2 = new Constant(
            -TILE_LENGTH,
            - TILE_LENGTH / 2,
            Math.toRadians(0)
    );
}
