package org.firstinspires.ftc.teamcode.drive;

public class Point {

    public double x, y, theta = Double.NaN;

    public Point(double x, double y){
        this.x = x;
        this.y = y;
    }

    public Point(double x, double y, double theta){
        this.x = x;
        this.y = y;
        this.theta = theta;
    }
}
