package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class ArmController {

    public MotorEx motorBrat;
    Servo servoIntake, servoLaunch, servoPixel;

    Thread runThread;
    Thread distThread;

    DistanceSensor distLeft, distRight;

    double dLeft, lastdLeft, dRight, lastdRight, dist, lastDist, targetDist = 32;

    public static double kP = 0.004;
    public static double threshold = 10;

    public static double speedFactor = 0.15;

    boolean killController;

    public static enum Position{
        HOME(0),
        LEVEL1(3850),
        LEVEL2(3300),
        HANG(2100),
        LEVEL3(2800),
        AUTOPIXEL(3700);
        public final int val;
        Position(int val){
            this.val = val;
        }
    }

    public static enum PositionDistance{
        LEVEL1(24),
        LEVEL2(8);
        public final double val;
        PositionDistance(double val){
            this.val = val;
        }
    }

    public static enum IntakePosition{
        GRAB(1),
        MID(0.4),
        THROW(0);
        public final double val;
        IntakePosition(double val){
            this.val = val;
        }
    }

    public static enum LauncherPosition{
        LAUNCH(0.4),
        IDLE(1);
        public final double val;
        LauncherPosition(double val){
            this.val = val;
        }

    }

    public ArmController(HardwareMap hardwareMap){
        motorBrat = new MotorEx(hardwareMap,"motor_lift",3863.7,41.928721174);

        motorBrat.resetEncoder();
        motorBrat.setRunMode(Motor.RunMode.PositionControl);
        motorBrat.setPositionCoefficient(kP);
        motorBrat.setPositionTolerance(threshold);
        motorBrat.setTargetPosition(Position.HOME.val);
        motorBrat.set(0);

        servoIntake = hardwareMap.servo.get("servo_intake");
        servoLaunch = hardwareMap.servo.get("servo_launch");
        servoPixel = hardwareMap.servo.get("servo_pixel");

        distLeft = hardwareMap.get(DistanceSensor.class, "distance_left");
        distRight = hardwareMap.get(DistanceSensor.class, "distance_right");
    }

    public void setIntakePosition(IntakePosition target){
        servoIntake.setPosition(target.val);
    }

    public void dropPixel(){
        servoPixel.setPosition(0.6);
    }

    public void setLauncherPosition(LauncherPosition target){
        servoLaunch.setPosition(target.val);
    }

    public void setTarget(Position target){
        motorBrat.set(speedFactor);
        motorBrat.setTargetPosition(target.val);
        if(target == Position.LEVEL1){
            targetDist = PositionDistance.LEVEL1.val;
        }
        if(target == Position.LEVEL2){
            targetDist = PositionDistance.LEVEL2.val;
        }
    }

    public void startPositioning(){
        runThread = new Thread(new PositionPlayer());
        runThread.start();
    }

    public void startMeasuring(){
        distThread = new Thread(new DistanceReader());
        distThread.start();
    }

    public void interrupt(){
        if(runThread != null){
            runThread.interrupt();
        }
        if(distThread != null){
            distThread.interrupt();
        }
        killController = true;
    }

    public boolean isRunning(){
        if(runThread == null) return false;
        return runThread.isAlive();
    }

    public boolean isDistRunning(){
        if(distThread == null) return false;
        return distThread.isAlive();
    }

    public double getDist(){
        return dist;
    }

    public double getDistError(){
        return targetDist - dist;
    }

    public int getPosition(){
        return motorBrat.getCurrentPosition();
    }

    public boolean isPositioned(){
        return motorBrat.atTargetPosition();
    }

    public class PositionPlayer implements Runnable{

        public PositionPlayer(){
            killController = false;
        }

        @Override
        public void run() {
            while(!killController){
                motorBrat.setPositionCoefficient(kP);
                motorBrat.setPositionTolerance(threshold);
                motorBrat.set(speedFactor);
            }
        }
    }

    public class DistanceReader implements Runnable{

        public DistanceReader(){
            killController = false;
            lastdLeft = dLeft = distLeft.getDistance(DistanceUnit.CM);
            lastdRight = dRight = distRight.getDistance(DistanceUnit.CM);
            lastDist = dist = (dLeft + dRight) * 0.5;
        }

        @Override
        public void run() {
            while(!killController){
                lastdLeft = dLeft;
                lastdRight = dRight;
                lastDist = dist;

                dLeft = (lastdLeft + distLeft.getDistance(DistanceUnit.CM)) * 0.5;
                dRight = (lastdRight + distRight.getDistance(DistanceUnit.CM)) * 0.5;

                dist = (lastDist + (dLeft + dRight) * 0.5) * 0.5;
            }
        }
    }
}
