package org.firstinspires.ftc.teamcode.opmode.auton.basket;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.auton.basket.setup.BasketAuto;
import org.firstinspires.ftc.teamcode.opmode.auton.basket.setup.BasketConstantsDash;
import org.firstinspires.ftc.teamcode.opmode.auton.util.Color;

@Autonomous(name = "BasketAutoBlue", preselectTeleOp = "Main")
public class BasketAutoBlue extends BasketAuto {
    public BasketAutoBlue() {
        super(Color.BLUE, BasketConstantsDash.basketBlue);
    }
}
