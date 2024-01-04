package org.firstinspires.ftc.teamcode.drive;

import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;

public class TankDriveController {

    TankDriveChassis robot;
    Telemetry telemetry;

    Path path = null;

    Point targetPoint = null;

    boolean killController;

    PIDController pidGyro = new PIDController(0, 0, 0);
    PIDController pidDrive = new PIDController(0, 0, 0);

    public TankDriveController(TankDriveChassis robot, Telemetry telemetry){
        this.robot = robot;
        this.telemetry = telemetry;
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

    public void setPath(Path path){
        this.path = path;
    }

    public boolean followPath(){
        if(path == null) return false;
        if(path.waypoints.peek() == null) return false;

        targetPoint = path.waypoints.peek();

        robot.updatePosition();

        pidGyro.setPID(ChassisConstants.HEADING_PID.p,
                ChassisConstants.HEADING_PID.i,
                ChassisConstants.HEADING_PID.d);

        pidDrive.setPID(ChassisConstants.DRIVE_PID.p,
                ChassisConstants.DRIVE_PID.i,
                ChassisConstants.DRIVE_PID.d);

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

        Point targetPoint;

        PIDController pidGyro = new PIDController(0, 0, 0);
        PIDController pidDrive = new PIDController(0, 0, 0);

        public MotionPlayer(Path path){
            this.path = path;
            pidDrive.setTolerance(0.05,0.1);
            killController = false;
        }

        @Override
        public void run(){
            if(robot.localizerThread != null){
                while(!Thread.interrupted() && !killController){

                    targetPoint = path.waypoints.peek();
                    if(targetPoint == null) return;
                    path.waypoints.remove();

                    boolean check = false;

                    while(!check && !Thread.interrupted() && !killController){

                        pidGyro.setPID(ChassisConstants.HEADING_PID.p,
                                ChassisConstants.HEADING_PID.i,
                                ChassisConstants.HEADING_PID.d);

                        pidDrive.setPID(ChassisConstants.DRIVE_PID.p,
                                ChassisConstants.DRIVE_PID.i,
                                ChassisConstants.DRIVE_PID.d);

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

                        double powerR = -pidGyro.calculate(angleError);
                        double powerX = Math.abs(powerR) < 0.2 ? -pidDrive.calculate(distance) : 0;

                        if((Math.abs(powerX) < 0.2 || (Math.abs(distance) <= ChassisConstants.toleranceXY &&
                                path.waypoints.peek() != null))
                                && Math.abs(powerR) < 0.1) check = true;

                        robot.setPowerRamp(powerX, powerR);

                        telemetry.addData("pwrX", powerX);
                        telemetry.addData("pwrR", powerR);
                        telemetry.update();
                    }

                    while(!Double.isNaN(targetPoint.theta) && !Thread.interrupted() && !killController){
                        double error = targetPoint.theta - robot.theta;

                        error = Utils.minAbs(error, error - Math.signum(error) * 360);

                        double powerR = -pidGyro.calculate(error);

                        if(pidGyro.atSetPoint()) targetPoint.theta = Double.NaN;

                        robot.setPowerRamp(0, powerR);
                    }
                }
            }
        }

    }



}
