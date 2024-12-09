package org.firstinspires.ftc.teamcode.opmode.auton.basket;

import static org.firstinspires.ftc.teamcode.opmode.auton.util.GameConstants.*;

import org.firstinspires.ftc.teamcode.opmode.auton.util.Constant;

public class BasketConstants extends org.firstinspires.ftc.teamcode.opmode.auton.basket.setup.BasketConstants {
    public BasketConstants() {
    FORWARD = new Constant(
            -TILE_LENGTH / 2.0 - 4,
            -WALL_POS + BOT_LENGTH / 2.0 + 7,
            Math.toRadians(90)
    );
    BASKET = new Constant(
            -TILE_LENGTH * 2.5 + BOT_LENGTH / 2 - 21,
             -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 + 0.5,
             Math.toRadians(45)
    );
    CHAMBER = new Constant(
             0,
             -TILE_LENGTH,
             UP
     );
     FAR_SAMPLE = new Constant(
             -TILE_LENGTH * 2.5 + BOT_LENGTH / 2 + 4,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 + 1,
             Math.toRadians(90)
     );
     CENTER_SAMPLE = new Constant(
             -TILE_LENGTH * 2.5 + BOT_LENGTH / 2 - 13,
             -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 + 6,
             Math.toRadians(115)
     );
     WALL_SAMPLE = new Constant(
             -TILE_LENGTH * 2.5,
             -TILE_LENGTH * 3 / 2 + BOT_LENGTH / 2,
             Math.toRadians(135)
     );
     BASKET_1 = new Constant(
             -TILE_LENGTH * 2.5 + BOT_LENGTH / 2 - 13,
             - TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 - 4,
             Math.toRadians(41)
     );
     BASKET_2 = new Constant(
             -TILE_LENGTH * 2.5 + BOT_LENGTH / 2,
             - TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2,
             Math.toRadians(45)
     );
     BASKET_3 = new Constant(
             -TILE_LENGTH * 2.5 + BOT_LENGTH/2,
             - TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2,
             Math.toRadians(45)
     );
    }
}
