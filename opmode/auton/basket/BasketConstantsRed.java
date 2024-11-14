package org.firstinspires.ftc.teamcode.opmode.auton.basket;

import static org.firstinspires.ftc.teamcode.opmode.auton.util.GameConstants.TILE_LENGTH;

import org.firstinspires.ftc.teamcode.opmode.auton.basket.setup.BasketConstants;
import org.firstinspires.ftc.teamcode.opmode.auton.util.Constant;

public class BasketConstantsRed extends BasketConstants {
    public BasketConstantsRed() {
        CHAMBER = new Constant(
            TILE_LENGTH, 0, Math.toRadians(0)
        );

        WALL_SAMPLE = new Constant(
            TILE_LENGTH, -TILE_LENGTH*3, Math.toRadians(180)
        );

        CENTER_SAMPLE = new Constant(
            TILE_LENGTH, -TILE_LENGTH*2.5, Math.toRadians(180)
        );

        FAR_SAMPLE = new Constant(
            TILE_LENGTH, -TILE_LENGTH*2, Math.toRadians(180)
        );

        BASKET_1 = new Constant(
        TILE_LENGTH*3, -TILE_LENGTH*3, Math.toRadians(315)
        );

        BASKET_2 = new Constant(
        TILE_LENGTH*3, -TILE_LENGTH*3, Math.toRadians(315)
        );

        BASKET_3 = new Constant(
        TILE_LENGTH*3, -TILE_LENGTH*3, Math.toRadians(315)
        );

        PARK = new Constant(
        TILE_LENGTH*3, 0, Math.toRadians(180)
        );
    }
}
