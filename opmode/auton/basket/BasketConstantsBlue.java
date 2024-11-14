package org.firstinspires.ftc.teamcode.opmode.auton.basket;

import static org.firstinspires.ftc.teamcode.opmode.auton.util.GameConstants.TILE_LENGTH;

import android.service.quicksettings.Tile;

import org.firstinspires.ftc.teamcode.opmode.auton.basket.setup.BasketConstants;
import org.firstinspires.ftc.teamcode.opmode.auton.util.Constant;

public class BasketConstantsBlue extends BasketConstants {
    public BasketConstantsBlue() {

        CHAMBER = new Constant(
            TILE_LENGTH * 0.75 , 0, Math.toRadians(180)
        );

        WALL_SAMPLE = new Constant(
            TILE_LENGTH * 3, TILE_LENGTH, Math.toRadians(0)
        );

        CENTER_SAMPLE = new Constant(
            TILE_LENGTH * 2.5, TILE_LENGTH, Math.toRadians(0)
        );

        FAR_SAMPLE = new Constant(
            TILE_LENGTH * 2, TILE_LENGTH, Math.toRadians(0)
        );

        BASKET_1 = new Constant(
        TILE_LENGTH*2.5, TILE_LENGTH*2.5, Math.toRadians(225)
        );

        BASKET_2 = new Constant(
        TILE_LENGTH*2.5, TILE_LENGTH*2.5, Math.toRadians(225)
        );

        BASKET_3 = new Constant(
        TILE_LENGTH*2.5, TILE_LENGTH*2.5, Math.toRadians(225)
        );

        PARK = new Constant(
            0, TILE_LENGTH*3, Math.toRadians(0)
        );

    }
}
