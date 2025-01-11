package org.firstinspires.ftc.teamcode.opmode.auton.basketnoclip;

import static org.firstinspires.ftc.teamcode.opmode.auton.util.GameConstants.*;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

@Config
public class BasketNoClipConstantsDash {
    // Start Pose
    public static final double START_HEADING = UP;
    public static final double START_X = -BOT_WIDTH * 2.5;
    public static double START_Y = -WALL_POS + BOT_LENGTH / 2;

    public static Pose2d START_POSE = new Pose2d(START_X, START_Y, START_HEADING);

    public static BasketNoClipConstants basketNoClipConstants = new BasketNoClipConstants();
}