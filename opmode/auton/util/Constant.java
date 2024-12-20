package org.firstinspires.ftc.teamcode.opmode.auton.util;

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

    public Vector2d getV() {
        return new Vector2d(X_POS, Y_POS);
    }

    public double getH() {
        return HEADING;
    }
}
