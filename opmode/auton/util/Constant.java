package org.firstinspires.ftc.teamcode.opmode.auton.util;

import com.acmerobotics.roadrunner.geometry.Vector2d;

public class Constant {
    public double X_POS;
    public double Y_POS;
    public double HEADING;
    public double COLOR; // 0 = blue, 1 = red

    public Constant(double x, double y, double h, double color) {
        X_POS = x;
        Y_POS = y;
        HEADING = h;
    }

    public Vector2d getV() {
        return new Vector2d(X_POS, Y_POS);
    }

    public double getH() {
        return HEADING;
    }
}
