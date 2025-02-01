package org.firstinspires.ftc.teamcode.opmode.auton.basketnoclip;

import static org.firstinspires.ftc.teamcode.opmode.auton.util.GameConstants.*;

import org.firstinspires.ftc.teamcode.opmode.auton.util.Constant;

import com.acmerobotics.dashboard.config.Config;

@Config
public class BasketNoClipConstants {
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
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 - 2,
            Math.toRadians(43)
    );
    public static Constant BASKET_3 = new Constant(
            -TILE_LENGTH * 2.5 - BOT_WIDTH + 19.5,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 - 3,
            Math.toRadians(43)
    );
    public static Constant BASKET_4 = new Constant(
            -TILE_LENGTH * 2.5 - BOT_WIDTH + 22.5,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 - 1,
            Math.toRadians(30)
    );
    public static Constant FAR_SAMPLE = new Constant(
            -TILE_LENGTH * 2.5 - BOT_WIDTH + 26,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 - 2.5,
            UP - Math.toRadians(5)
    );
    public static Constant FAR_SAMPLE_INT = new Constant(
            -TILE_LENGTH * 2.5 - BOT_WIDTH + 24,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 - 0.5,
            UP + Math.toRadians(6)
    );
    public static Constant CENTER_SAMPLE = new Constant(
            -TILE_LENGTH * 2.5 - BOT_WIDTH + 17.0,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 - 3.5,
            UP - Math.toRadians(5)
    );
    public static Constant CENTER_SAMPLE_INT = new Constant(
            -TILE_LENGTH * 2.5 - BOT_WIDTH + 16.0,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 - 1,
            UP + Math.toRadians(7)
    );
    public static Constant WALL_SAMPLE = new Constant(
            -TILE_LENGTH * 2.5 - BOT_WIDTH + 10.0,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 - 3.5,
            UP - Math.toRadians(5)
    );
    public static Constant WALL_SAMPLE_INT = new Constant(
            -TILE_LENGTH * 2.5 - BOT_WIDTH + 12.0,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 - 1,
            UP + Math.toRadians(25)
    );
}
