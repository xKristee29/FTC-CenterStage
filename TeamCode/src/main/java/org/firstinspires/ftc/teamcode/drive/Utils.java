package org.firstinspires.ftc.teamcode.drive;

public class Utils {

    public static double minAbs(double x, double y){
        if(Math.min(Math.abs(x),Math.abs(y)) == Math.abs(x)) return x;
        return y;
    }

    public static double toRobotDegrees(double x){
        return (x % 360 + 360) % 360;
    }
}
