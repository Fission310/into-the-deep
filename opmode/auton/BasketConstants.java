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
            -TILE_LENGTH * 2.5 - BOT_WIDTH + 19,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 - 1,
            41);
    public static Constant BASKET_2 = new Constant(
            -TILE_LENGTH * 2.5 - BOT_WIDTH + 19.5,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 - 0.5,
            37);
    public static Constant BASKET_3 = new Constant(
            -TILE_LENGTH * 2.5 - BOT_WIDTH + 21.5,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 - 4.5,
            40);
    public static Constant BASKET_4 = new Constant(
            -TILE_LENGTH * 2.5 - BOT_WIDTH + 20,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 - 2,
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
            -TILE_LENGTH * 2.5 - BOT_WIDTH + 26.5,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2,
            UP + 2);
    public static Constant FAR_SAMPLE_INT = new Constant(
            -TILE_LENGTH * 2.5 - BOT_WIDTH + 26.5,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 + 7.25,
            UP + 13.5);
    public static Constant CENTER_SAMPLE = new Constant(
            -TILE_LENGTH * 2.5 - BOT_WIDTH + 16,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 + 0.5,
            UP + 1.75);
    public static Constant CENTER_SAMPLE_INT = new Constant(
            -TILE_LENGTH * 2.5 - BOT_WIDTH + 16,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 + 7.75,
            UP + 12.5);
    public static Constant WALL_SAMPLE = new Constant(
            -TILE_LENGTH * 2.5 - BOT_WIDTH + 15.5,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 - 3.5,
            UP + 20.5);
    public static Constant WALL_SAMPLE_INT = new Constant(
            -TILE_LENGTH * 2.5 - BOT_WIDTH + 15.5,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 - 3.75,
            UP + 25);
    public static Constant SUBMERSIBLE_1 = new Constant(
            -TILE_LENGTH,
            -TILE_LENGTH / 2,
            0);
    public static Constant SUBMERSIBLE_2 = new Constant(
            -TILE_LENGTH,
            -TILE_LENGTH / 2 + 6,
            0);

    public static double[][] WRIST_INTAKE_POS =  { { 0.515, 0.515 }, { 0.59, 0.58 }, { 0.59, 0.58 }, { 0.59, 0.58 } };
    public static int PIVOT_INTAKE_GRAB_POS = 123;
    public static int PIVOT_BASKET_POS = 250;
    public static int TELESCOPE_INTAKE_FAR_POS = 367;
    public static int TELESCOPE_INTAKE_CENTER_POS = 307;
    public static int TELESCOPE_INTAKE_WALL_POS = 477;
    public static int TELESCOPE_BASKET_POS = 675;
    public static int TELESCOPE_SAMPLE_DROP = 300;
    public static int TELESCOPE_INTAKE_SHORT_POS = 50;
}
