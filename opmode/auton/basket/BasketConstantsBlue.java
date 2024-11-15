package org.firstinspires.ftc.teamcode.opmode.auton.basket;

import static org.firstinspires.ftc.teamcode.opmode.auton.util.GameConstants.*;

import android.service.quicksettings.Tile;

import org.firstinspires.ftc.teamcode.opmode.auton.basket.setup.BasketConstants;
import org.firstinspires.ftc.teamcode.opmode.auton.util.Constant;

public class BasketConstantsBlue extends BasketConstants {
    public BasketConstantsBlue() {
        CHAMBER = new Constant(
            TILE_LENGTH * -0.75 , TILE_LENGTH, UP
        );

        WALL_SAMPLE = new Constant(
            TILE_LENGTH * 3, TILE_LENGTH, DOWN
        );

        CENTER_SAMPLE = new Constant(
            TILE_LENGTH * 2.5, TILE_LENGTH, DOWN
        );

        FAR_SAMPLE = new Constant(
            TILE_LENGTH * 2, TILE_LENGTH, DOWN
        );

        BASKET_1 = new Constant(
        TILE_LENGTH * 2.5, TILE_LENGTH * 2.5, Math.toRadians(45)
        );

        BASKET_2 = new Constant(
        TILE_LENGTH * 2.5, TILE_LENGTH * 2.5, Math.toRadians(45)
        );

        BASKET_3 = new Constant(
        TILE_LENGTH * 2.5, TILE_LENGTH * 2.5, Math.toRadians(45)
        );

        PARK = new Constant(
                TILE_LENGTH * .5, TILE_LENGTH * 3, UP
        );
    }
}
