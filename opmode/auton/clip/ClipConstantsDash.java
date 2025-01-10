package org.firstinspires.ftc.teamcode.opmode.auton.clip;

import static org.firstinspires.ftc.teamcode.opmode.auton.util.GameConstants.*;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
public class ClipConstantsDash {
    // Start Pose
    public static final double START_HEADING = UP;

    public static final double START_X = BOT_WIDTH / 2;
    public static final double START_Y = -WALL_POS + BOT_LENGTH / 2.0;

    public static Pose2d START_POSE = new Pose2d(START_X, START_Y, START_HEADING);

    public static ClipConstants clipConstants = new ClipConstants();
}