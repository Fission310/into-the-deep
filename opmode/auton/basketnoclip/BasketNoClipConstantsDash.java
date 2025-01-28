package org.firstinspires.ftc.teamcode.opmode.auton.basketnoclip;

import static org.firstinspires.ftc.teamcode.opmode.auton.util.GameConstants.*;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;

@Config
public class BasketNoClipConstantsDash {
    // Start Pose
    public static final double START_HEADING = UP;
    public static final double START_X = -TILE_LENGTH - BOT_WIDTH / 2;
    public static double START_Y = -WALL_POS + BOT_LENGTH / 2;

    public static Pose START_POSE = new Pose(START_X, START_Y, START_HEADING);

    public static BasketNoClipConstants basketNoClipConstants = new BasketNoClipConstants();
}
