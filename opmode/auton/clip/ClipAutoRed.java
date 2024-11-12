package org.firstinspires.ftc.teamcode.opmode.auton.clip;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.auton.clip.setup.ClipAuto;
import org.firstinspires.ftc.teamcode.opmode.auton.clip.setup.ClipConstantsDash;
import org.firstinspires.ftc.teamcode.opmode.auton.util.Color;

@Autonomous(name = "ClipAutoRed", preselectTeleOp = "Main")
public class ClipAutoRed extends ClipAuto {
    public ClipAutoRed() {
        super(Color.RED, ClipConstantsDash.clipRed);
    }
}
