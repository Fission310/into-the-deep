package org.firstinspires.ftc.teamcode.opmode.auton.util;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public class Constant {
    public double X_POS;
    public double Y_POS;
    public double HEADING;

    public Constant(double x, double y, double h) {
        X_POS = x;
        Y_POS = y;
        HEADING = h;
    }

    public Vector2d getVec() {
        return new Vector2d(X_POS, Y_POS);
    }

    public Pose2d getPose(){
        return new Pose2d(X_POS, Y_POS, HEADING);
    }

    public double getH() {
        return HEADING;
    }
}
