package org.firstinspires.ftc.teamcode.drive;

import java.util.LinkedList;
import java.util.Queue;

public class Path {

    public Queue<Point> waypoints = new LinkedList<>();

    public Point lastPoint;

    public long k = 0;

    public Path(Point startPoint){
        lastPoint = startPoint;
    }

    public Point getLastPoint(){
        return lastPoint;
    }

    public Path goTo(Point waypoint){
        waypoints.add(waypoint);
        lastPoint = waypoint;
        k++;
        return this;
    }

    public Point calculateBezierPoint(double t, Point p0, Point p1, Point p2, Point p3){

        double t2 = t * t;
        double t3 = t2 * t;
        double u = 1 - t;
        double u2 = u * u;
        double u3 = u2 * u;

        double x = u3 * p0.x + 3 * u2 * t * p1.x + 3 * u * t2 * p2.x + t3 * p3.x;
        double y = u3 * p0.y + 3 * u2 * t * p1.y + 3 * u * t2 * p2.y + t3 * p3.y;

        return new Point((int)x, (int)y);
    }

    public Path generateBezierCurve(Point p0, Point p1, Point p2, Point p3, long kPoints){

        double errorX = p3.x - p0.x;
        double errorY = p3.y - p0.y;

        double dist = Math.hypot(errorX, errorY);

        double targetRadius2 = 3 * ChassisConstants.targetRadius;

        double t = 1.0 / (kPoints + 1);

        for(double i = 1.0; i <= kPoints; ++i){
            goTo(calculateBezierPoint(t*i, p0, p1, p2, p3));
        }

        goTo(p3);

        return this;
    }

    public Path splineTo(Point p){

        generateBezierCurve(
                lastPoint,
                new Point(p.x, lastPoint.y),
                new Point(lastPoint.x, p.y),
                p, 6
        );

        return this;
    }



    public Path customSplineTo(Point p, double crossAxisOffset, double mainAxisOffset, double curvature, double ssness, long kPoints){

        Point p0,p1,p2,p3,p4;
        p0 = lastPoint; p3 = p;

        double errorX = p3.x - p0.x;
        double errorY = p3.y - p0.y;

        double dist = Math.hypot(errorX, errorY);

        p4 = new Point(
                p0.x + crossAxisOffset * errorY + (mainAxisOffset + 0.5) * errorX,
                p0.y - crossAxisOffset * errorX + (mainAxisOffset + 0.5) * errorY
        );

        double phi = ssness * Math.PI + Math.atan2(errorY, errorX);

        p1 = new Point(
                p4.x - curvature * dist * Math.cos(phi),
                p4.x - curvature * dist * Math.sin(phi)
        );

        p2 = new Point(
                2 * p4.x - p1.x,
                2 * p4.y - p1.y
        );

        generateBezierCurve(p0, p1, p2, p3, kPoints);

        return this;
    }
    

}
