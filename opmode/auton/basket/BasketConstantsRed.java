package org.firstinspires.ftc.teamcode.opmode.auton.basket;

import static org.firstinspires.ftc.teamcode.opmode.auton.util.GameConstants.*;

import org.firstinspires.ftc.teamcode.opmode.auton.basket.setup.BasketConstants;
import org.firstinspires.ftc.teamcode.opmode.auton.util.Constant;

public class BasketConstantsRed extends BasketConstants {
    public BasketConstantsRed() {
       CHAMBER = new Constant(
                0,
                -TILE_LENGTH,
                UP
        );

        WALL_SAMPLE = new Constant(
                -TILE_LENGTH * 2,
                -TILE_LENGTH,
                Math.toRadians(180)
        );
        CENTER_SAMPLE = new Constant(
                -TILE_LENGTH * 5 / 2,
                -TILE_LENGTH * 3 / 2 + BOT_LENGTH / 2,
                Math.toRadians(135)
        );
        FAR_SAMPLE = new Constant(
                -TILE_LENGTH * 2.5,
                -TILE_LENGTH * 3 / 2 + BOT_LENGTH / 2,
                Math.toRadians(0)
        );

        BASKET_1 = new Constant(
                -TILE_LENGTH * 2.5 + BOT_LENGTH / 2,
                - TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2,
                Math.toRadians(225)
        );
        BASKET_2 = new Constant(
                -TILE_LENGTH * 2.5 + BOT_LENGTH / 2,
                - TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2,
                Math.toRadians(225)
        );
        BASKET_3 = new Constant(
                -TILE_LENGTH * 2.5 + BOT_LENGTH/2,
                - TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2,
                Math.toRadians(225)
        );
    }
}
