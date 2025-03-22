package org.firstinspires.ftc.teamcode.opmode.auton;

import org.firstinspires.ftc.teamcode.opmode.auton.util.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RedAuto", preselectTeleOp = "RedMain")
public class RedAuto extends BasketAuto {
    public RedAuto() {
        super(Color.RED);
    }
}
