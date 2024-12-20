package org.firstinspires.ftc.teamcode.opmode.auton.basket;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.auton.basket.setup.BasketConstantsDash;

@Autonomous(name = "BasketAuto", preselectTeleOp = "Main")
public class BasketAuto extends org.firstinspires.ftc.teamcode.opmode.auton.basket.setup.BasketAuto {
    public BasketAuto() {
        super(BasketConstantsDash.basket);
    }
}
