package org.firstinspires.ftc.teamcode.opmode.auton;

import static org.firstinspires.ftc.teamcode.opmode.auton.util.GameConstants.*;

import org.firstinspires.ftc.teamcode.opmode.auton.util.Constant;

import com.acmerobotics.dashboard.config.Config;

@Config
public class BasketConstants {
    public static Constant START = new Constant(
            -TILE_LENGTH * 2 + BOT_WIDTH / 2,
            -WALL_POS + BOT_LENGTH / 2,
            UP);
    public static Constant BASKET_1 = new Constant(
            -TILE_LENGTH * 2.5 - BOT_WIDTH + 21,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2,
            39.5);
    public static Constant BASKET_2 = new Constant(
            -TILE_LENGTH * 2.5 - BOT_WIDTH + 19.5,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2,
            37);
    public static Constant BASKET_3 = new Constant(
            -TILE_LENGTH * 2.5 - BOT_WIDTH + 21.5,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 - 2.5,
            40);
    public static Constant BASKET_4 = new Constant(
            -TILE_LENGTH * 2.5 - BOT_WIDTH + 20,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 - 4,
            39);
    public static Constant BASKET_5 = new Constant(
            -TILE_LENGTH * 2.5 - BOT_WIDTH + 22.5,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 - 6,
            39);
    public static Constant BASKET_6 = new Constant(
            -TILE_LENGTH * 2.5 - BOT_WIDTH + 22.5,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 - 6,
            39);
    public static Constant FAR_SAMPLE = new Constant(
            -TILE_LENGTH * 2.5 - BOT_WIDTH + 40.5,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2,
            UP + 12.5);
    public static Constant FAR_SAMPLE_INT = new Constant(
            -TILE_LENGTH * 2.5 - BOT_WIDTH + 25.5,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 + 4.25,
            UP + 13);
    public static Constant CENTER_SAMPLE = new Constant(
            -TILE_LENGTH * 2.5 - BOT_WIDTH + 16,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 - 3,
            UP + 2.5);
    public static Constant CENTER_SAMPLE_INT = new Constant(
            -TILE_LENGTH * 2.5 - BOT_WIDTH + 16,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 + 4.75,
            UP + 13.5);
    public static Constant WALL_SAMPLE = new Constant(
            -TILE_LENGTH * 2.5 - BOT_WIDTH + 15.5,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 - 3.5,
            UP + 24.5);
    public static Constant WALL_SAMPLE_INT = new Constant(
            -TILE_LENGTH * 2.5 - BOT_WIDTH + 15.5,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 - 2.75,
            UP + 26.5);
    public static Constant SUBMERSIBLE_1 = new Constant(
            -TILE_LENGTH,
            -TILE_LENGTH / 2,
            0);
    public static Constant SUBMERSIBLE_2 = new Constant(
            -TILE_LENGTH,
            -TILE_LENGTH / 2 + 6,
            0);
}
