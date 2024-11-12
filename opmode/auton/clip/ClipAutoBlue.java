package org.firstinspires.ftc.teamcode.opmode.auton.clip;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.auton.clip.setup.ClipAuto;
import org.firstinspires.ftc.teamcode.opmode.auton.clip.setup.ClipConstantsDash;
import org.firstinspires.ftc.teamcode.opmode.auton.util.Color;

@Autonomous(name = "ClipAutoBlue", preselectTeleOp = "Main")
public class ClipAutoBlue extends ClipAuto {
    public ClipAutoBlue() {
        super(Color.BLUE, ClipConstantsDash.clipBlue);
    }
}

