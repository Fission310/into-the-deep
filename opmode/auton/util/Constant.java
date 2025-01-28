package org.firstinspires.ftc.teamcode.opmode.auton.util;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Point;

public class Constant {
    public double X_POS;
    public double Y_POS;
    public double HEADING;

    public Constant(double x, double y, double h) {
        X_POS = x;
        Y_POS = y;
        HEADING = h;
    }

    public Point getVec() {
        return new Point(X_POS, Y_POS);
    }

    public Pose getPose(){
        return new Pose(X_POS, Y_POS, HEADING);
    }

    public double getH() {
        return HEADING;
    }
}
