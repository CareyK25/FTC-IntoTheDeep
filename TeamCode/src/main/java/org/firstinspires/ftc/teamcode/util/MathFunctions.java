package org.firstinspires.ftc.teamcode.util;

public class MathFunctions {
    public static double angleWrap(double angle) {
        while (angle > Math.PI) {
            angle-=2*Math.PI;
        }
        while (angle < -Math.PI) {
            angle+=2*Math.PI;
        }
        return angle;
    }
    public static double clip(double val, double up, double low) {
        if (val>up) {
            return up/10;
        }
        if (val<low) {
            return low/10;
        }
        return val/10;
    }
}
