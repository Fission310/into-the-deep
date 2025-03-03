package org.firstinspires.ftc.teamcode.opmode.auton;

import org.firstinspires.ftc.teamcode.opmode.auton.util.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BlueAuto", preselectTeleOp = "RedMain")
public class BlueAuto extends BasketAuto {
    public BlueAuto() {
        super(Color.BLUE);
    }
}
