package org.firstinspires.ftc.teamcode.drive;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;

public class TankDriveController {

    TankDriveChassis robot;
    Telemetry telemetry;

    Trigger trig = null;

    Path path = null;

    Point targetPoint = null;

    boolean killController;

    PIDFController pidGyro = new PIDFController(0, 0, 0, 0);
    PIDFController pidDrive = new PIDFController(0, 0, 0, 0);

    public TankDriveController(TankDriveChassis robot, Telemetry telemetry){
        this.robot = robot;
        this.telemetry = telemetry;
        pidGyro.setTolerance(5,1);
    }

    public Thread runThread = null;

    public void run(Path path){
        runThread = new Thread(new MotionPlayer(path));
        runThread.start();
    }

    public void interrupt(){
        if(runThread != null){
            runThread.interrupt();
            killController = true;
        }
    }

    public boolean isRunning(){
        if(runThread == null) return false;
        return runThread.isAlive();
    }

    public double getDriveCorrection(double error){
        pidDrive.setPIDF(ChassisConstants.DRIVE_PID.p,
                ChassisConstants.DRIVE_PID.i,
                ChassisConstants.DRIVE_PID.d,
                ChassisConstants.DRIVE_PID.f);

        return pidDrive.calculate(error);
    }

    public double getRotationalCorrection(double error){
        pidGyro.setPIDF(ChassisConstants.HEADING_PID.p,
                ChassisConstants.HEADING_PID.i,
                ChassisConstants.HEADING_PID.d,
                ChassisConstants.HEADING_PID.f);

        return pidGyro.calculate(error);
    }

    public void setPath(Path path){
        this.path = path;
    }

    public boolean followPath(){
        if(path == null) return false;
        if(path.waypoints.peek() == null) return false;

        targetPoint = path.waypoints.peek();

        robot.updatePosition();

        pidGyro.setPIDF(ChassisConstants.HEADING_PID.p,
                ChassisConstants.HEADING_PID.i,
                ChassisConstants.HEADING_PID.d,
                ChassisConstants.HEADING_PID.f);

        pidDrive.setPIDF(ChassisConstants.DRIVE_PID.p,
                ChassisConstants.DRIVE_PID.i,
                ChassisConstants.DRIVE_PID.d,
                ChassisConstants.DRIVE_PID.f);

        double errorX = targetPoint.x - robot.x;
        double errorY = targetPoint.y - robot.y;

        double distance = (int)Math.hypot(errorX, errorY);

        double targetAngle = ((int)Math.toDegrees(Math.atan2(errorX, errorY)) + 360) % 360;
        double angleError = targetAngle - robot.theta;

        angleError = Utils.minAbs(angleError, angleError - Math.signum(angleError) * 360);

        if(Math.abs(angleError) > 130){
            distance = -distance;
        }

        telemetry.addData("x", robot.x);
        telemetry.addData("y", robot.y);
        telemetry.addData("theta", robot.theta);
        telemetry.addLine();
        telemetry.addData("TargetX", targetPoint.x);
        telemetry.addData("TargetY", targetPoint.y);
        telemetry.addData("TargetR", targetAngle);
        telemetry.addData("Queue", path.k);
        telemetry.addLine();
        telemetry.addData("Dist", distance);
        telemetry.addData("AngleErr", angleError);
        telemetry.addLine();

        if(Math.abs(distance) <= ChassisConstants.targetRadius) {
            angleError = 0;
        }

        if(Math.abs(distance) <= ChassisConstants.toleranceXY){
            distance = 0;
        }

        if(angleError == 0 && path.waypoints.size() > 1) path.waypoints.remove();

        if(angleError == 0 && distance == 0 && path.waypoints.size() == 1) path.waypoints.remove();

        double powerR = -pidGyro.calculate(angleError);
        double powerX = -pidDrive.calculate(distance);

        robot.setPowerRamp(powerX, powerR);

        telemetry.addData("pwrX", powerX);
        telemetry.addData("pwrR", powerR);
        telemetry.update();

        return true;
    }

    public class MotionPlayer implements Runnable{

        Path path;

        Point targetPoint,lastPoint;

        PIDFController pidGyro = new PIDFController(0, 0, 0, 0);
        PIDFController pidDrive = new PIDFController(0, 0, 0, 0);

        public MotionPlayer(Path path){
            this.path = path;
            pidDrive.setTolerance(3,0.1);
            killController = false;
        }

        public Point getBestPoint(double m, double b, Point robot, Point target){
            double xR = robot.x;
            double yR = robot.y;

            if(Math.hypot(target.x - xR, target.y - yR) < ChassisConstants.targetR) return target;

            double xP = (xR + m * yR - m * b) / (m * m + 1);
            double yP = m * xP + b;

            double d = Math.hypot(xP - xR, yP - yR);

            double x1 = xP + Math.cos(Math.atan(m))
                    * (ChassisConstants.targetT * d + ChassisConstants.targetR);
            double y1 = m * x1 + b;

            double x2 = xP - Math.cos(Math.atan(m))
                    * (ChassisConstants.targetT * d + ChassisConstants.targetR);
            double y2 = m * x2 + b;

            double d1 = Math.hypot(target.x - x1, target.y - y1);
            double d2 = Math.hypot(target.x - x2, target.y - y2);

            double dMin = Math.min(d1,d2);

            if(dMin == d1) return new Point(x1,y1);
            return new Point(x2,y2);
        }

        public void runBetter(){
            trig = null;
            if(robot.localizerThread != null){

                lastPoint = path.wps.get(0);

                for(int pointIndex = 1;!Thread.interrupted() && !killController && pointIndex < path.wps.size();++pointIndex){
                    targetPoint = path.wps.get(pointIndex);

                    boolean check = false;

                    double dX = targetPoint.x - lastPoint.x;
                    double dY = targetPoint.y - lastPoint.y;

                    if(dX == 0) dX = 1e-9;

                    double m = dY/dX;
                    double b = lastPoint.y - m * lastPoint.x;

                    while (!check && !Thread.interrupted() && !killController){

                        pidGyro.setPIDF(ChassisConstants.HEADING_PID.p,
                                ChassisConstants.HEADING_PID.i,
                                ChassisConstants.HEADING_PID.d,
                                ChassisConstants.HEADING_PID.f);

                        pidDrive.setPIDF(ChassisConstants.DRIVE_PID.p,
                                ChassisConstants.DRIVE_PID.i,
                                ChassisConstants.DRIVE_PID.d,
                                ChassisConstants.DRIVE_PID.f);

                        Point target = getBestPoint(m, b, new Point(robot.x, robot.y), targetPoint);

                        double errorX = target.x - robot.x;
                        double errorY = target.y - robot.y;

                        double distance = Math.hypot(errorX,errorY);

                        double targetAngle = ((int)Math.toDegrees(Math.atan2(errorX, errorY)) + 360) % 360;
                        double angleError = targetAngle - robot.theta;

                        angleError = Utils.minAbs(angleError, angleError - Math.signum(angleError) * 360);

                        if(Math.abs(angleError) > 100){
                            distance = -distance;
                            angleError = angleError - Math.signum(angleError) * 180;
                        }

                        telemetry.addData("x", robot.x);
                        telemetry.addData("y", robot.y);
                        telemetry.addData("theta", robot.theta);
                        telemetry.addLine();
                        telemetry.addData("TargetX", target.x);
                        telemetry.addData("TargetY", target.y);
                        telemetry.addData("Queue", path.k);
                        telemetry.addLine();
                        telemetry.addData("Dist", distance);
                        telemetry.addData("AngleErr", angleError);
                        telemetry.addLine();

                        if(Math.abs(distance) <= ChassisConstants.targetRadius) {
                            angleError = 0;
                        }

                        if(Math.abs(distance) <= ChassisConstants.toleranceXY){
                            distance = 0;
                        }

                        double powerR = Math.tanh(-pidGyro.calculate(angleError));
                        double powerX = Math.cos(Math.toRadians(angleError)) * Math.tanh(-pidDrive.calculate(distance));

                        if (pointIndex == path.wps.size() - 1 &&
                                Math.abs(distance) <= ChassisConstants.targetRadius &&
                                powerX < 0.2) check = true;
                        if (pointIndex != path.wps.size() - 1 && Math.abs(distance) <= ChassisConstants.targetR) check = true;

                        robot.setPowerRamp(powerX, powerR);

                        telemetry.addData("pwrX", powerX);
                        telemetry.addData("pwrR", powerR);
                        telemetry.update();

                    }

                    while(!Double.isNaN(targetPoint.theta) && !Thread.interrupted() && !killController){

                        pidGyro.setPIDF(
                                ChassisConstants.STATIC_HEADING_PID.p,
                                ChassisConstants.STATIC_HEADING_PID.i,
                                ChassisConstants.STATIC_HEADING_PID.d,
                                ChassisConstants.STATIC_HEADING_PID.f
                        );

                        double error = targetPoint.theta - robot.theta;

                        error = Utils.minAbs(error, error - Math.signum(error) * 360);

                        double powerR = Math.tanh(-pidGyro.calculate(error));

                        if(Math.abs(powerR) < 0.1 && trig == null) trig = new Trigger((long)ChassisConstants.LIMIT_MS);
                        if(trig != null){
                            if(trig.getState() && Math.abs(powerR) < 0.1) targetPoint.theta = Double.NaN;
                        }

                        robot.setPowerRamp(0, powerR);

                        telemetry.addData("Err",error);
                        telemetry.addData("Slope",pidGyro.getVelocityError());
                        telemetry.addData("Period",pidGyro.getPeriod());
                        telemetry.addData("Pwr",powerR);
                        telemetry.update();

                    }
                    lastPoint = targetPoint;
                }
            }
        }

        @Override
        public void run(){
            runBetter();
            /*
            trig = null;
            if(robot.localizerThread != null){
                while(!Thread.interrupted() && !killController){

                    targetPoint = path.waypoints.peek();
                    if(targetPoint == null) return;
                    path.waypoints.remove();

                    boolean check = false;

                    while(!check && !Thread.interrupted() && !killController){

                        pidGyro.setPIDF(ChassisConstants.HEADING_PID.p,
                                ChassisConstants.HEADING_PID.i,
                                ChassisConstants.HEADING_PID.d,
                                ChassisConstants.HEADING_PID.f);

                        pidDrive.setPIDF(ChassisConstants.DRIVE_PID.p,
                                ChassisConstants.DRIVE_PID.i,
                                ChassisConstants.DRIVE_PID.d,
                                ChassisConstants.DRIVE_PID.f);

                        double errorX = targetPoint.x - robot.x;
                        double errorY = targetPoint.y - robot.y;

                        double distance = (int)Math.hypot(errorX, errorY);

                        double targetAngle = ((int)Math.toDegrees(Math.atan2(errorX, errorY)) + 360) % 360;
                        double angleError = targetAngle - robot.theta;

                        angleError = Utils.minAbs(angleError, angleError - Math.signum(angleError) * 360);

                        if(Math.abs(angleError) > 100){
                            distance = -distance;
                            angleError = angleError - Math.signum(angleError) * 180;
                        }

                        telemetry.addData("x", robot.x);
                        telemetry.addData("y", robot.y);
                        telemetry.addData("theta", robot.theta);
                        telemetry.addLine();
                        telemetry.addData("TargetX", targetPoint.x);
                        telemetry.addData("TargetY", targetPoint.y);
                        telemetry.addData("Queue", path.k);
                        telemetry.addLine();
                        telemetry.addData("Dist", distance);
                        telemetry.addData("AngleErr", angleError);
                        telemetry.addLine();

                        if(Math.abs(distance) <= ChassisConstants.targetRadius) {
                            angleError = 0;
                        }

                        if(Math.abs(distance) <= ChassisConstants.toleranceXY){
                            distance = 0;
                        }

                        /* OLD
                        double powerR = Math.abs(angleError) > 10 ? -pidGyro.calculate(angleError) : 0;
                        double powerX = Math.abs(powerR) < 0.2 ? -pidDrive.calculate(distance) : 0;
                         ///

                        double powerR = Math.tanh(-pidGyro.calculate(angleError));
                        double powerX = (1 - Math.abs(powerR)) * Math.tanh(-pidDrive.calculate(distance));

                        if(Math.abs(powerR) < 0.2 && Math.abs(powerX) < 0.2) check = true;

                        robot.setPowerRamp(powerX, powerR);

                        telemetry.addData("pwrX", powerX);
                        telemetry.addData("pwrR", powerR);
                        telemetry.addLine();
                        telemetry.addData("dX",Math.abs(pidDrive.getVelocityError()));
                        telemetry.addData("dR",Math.abs(pidGyro.getVelocityError()));
                        telemetry.update();
                    }

                    while(!Double.isNaN(targetPoint.theta) && !Thread.interrupted() && !killController){
                        double error = targetPoint.theta - robot.theta;

                        error = Utils.minAbs(error, error - Math.signum(error) * 360);

                        double powerR = Math.tanh(-pidGyro.calculate(error));

                        if(Math.abs(powerR) < 0.1 && trig == null) trig = new Trigger((long)ChassisConstants.LIMIT_MS);
                        if(trig != null){
                            if(trig.getState() && Math.abs(powerR) < 0.1) targetPoint.theta = Double.NaN;
                        }

                        robot.setPowerRamp(0, powerR);

                        telemetry.addData("Err",error);
                        telemetry.addData("Slope",pidGyro.getVelocityError());
                        telemetry.addData("Period",pidGyro.getPeriod());
                        telemetry.addData("Pwr",powerR);
                        telemetry.update();

                    }
                }
            }*/
        }

    }
}
