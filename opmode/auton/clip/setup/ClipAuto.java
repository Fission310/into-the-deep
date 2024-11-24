package org.firstinspires.ftc.teamcode.opmode.auton.clip.setup;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opmode.auton.util.Color;

public class ClipAuto extends LinearOpMode {
    Color color;
    ClipConstants clipConstants;
    public ClipAuto(Color color, ClipConstants clipConstants){
        this.color = color;
        this.clipConstants = clipConstants;
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}