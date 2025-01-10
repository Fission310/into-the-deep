package org.firstinspires.ftc.teamcode.opmode.auton.basket;

import static org.firstinspires.ftc.teamcode.opmode.auton.util.GameConstants.*;

import org.firstinspires.ftc.teamcode.opmode.auton.util.Constant;

public class BasketConstants{
    public static Constant FORWARD = new Constant(
            -TILE_LENGTH / 2.0 - 4,
            -WALL_POS + BOT_LENGTH / 2.0 + 7,
            Math.toRadians(90)
    );
    public static Constant BASKET = new Constant(
            -TILE_LENGTH * 2.5 + BOT_WIDTH / 2 - 21,
             -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 + 0.5,
             Math.toRadians(45)
    );
    public static Constant CHAMBER = new Constant(
             0,
             -TILE_LENGTH,
             UP
     );
    public static Constant FAR_SAMPLE = new Constant(
             -TILE_LENGTH * 2.5 + BOT_WIDTH / 2 + 4,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 + 1,
             Math.toRadians(90)
     );
    public static Constant CENTER_SAMPLE = new Constant(
             -TILE_LENGTH * 2.5 + BOT_WIDTH / 2 - 13,
             -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 + 6,
             Math.toRadians(115)
     );
    public static Constant WALL_SAMPLE = new Constant(
             -TILE_LENGTH * 2.5,
             -TILE_LENGTH * 3 / 2 + BOT_LENGTH / 2,
             Math.toRadians(135)
     );
    public static Constant BASKET_1 = new Constant(
             -TILE_LENGTH * 2.5 + BOT_WIDTH / 2 - 13,
             - TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 - 4,
             Math.toRadians(41)
     );
    public static Constant BASKET_2 = new Constant(
             -TILE_LENGTH * 2.5 + BOT_WIDTH / 2,
             - TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2,
             Math.toRadians(45)
     );
    public static Constant BASKET_3 = new Constant(
             -TILE_LENGTH * 2.5 + BOT_WIDTH / 2,
             - TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2,
             Math.toRadians(45)
     );
}
