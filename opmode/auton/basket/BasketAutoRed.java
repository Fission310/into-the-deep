package org.firstinspires.ftc.teamcode.opmode.auton.basket;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.auton.basket.setup.BasketAuto;
import org.firstinspires.ftc.teamcode.opmode.auton.basket.setup.BasketConstantsDash;
import org.firstinspires.ftc.teamcode.opmode.auton.util.Color;

@Autonomous(name = "BasketAutoRed", preselectTeleOp = "Main")
public class BasketAutoRed extends BasketAuto {
    public BasketAutoRed() {
        super(Color.RED, BasketConstantsDash.basketRed);
    }
}
