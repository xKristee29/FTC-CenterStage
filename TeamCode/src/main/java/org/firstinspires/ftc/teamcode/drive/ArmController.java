package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class ArmController {

    MotorEx motorBrat;
    Servo servoIntake;

    Thread runThread;

    public static double kP = 0.008;
    public static double threshold = 50;

    public static double speedFactor = 0.6;

    boolean killController;

    public static enum Position{
        HOME(0),
        LEVEL1(2000),
        LEVEL2(1600),
        HANG(1230),
        LAUNCH(750);
        public final int val;
        Position(int val){
            this.val = -val;
        }
    }

    public static enum IntakePosition{
        HOME(0.5),
        GRAB(1),
        THROW(0);

        public final double val;

        IntakePosition(double val){
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

    }

    public void setIntakePosition(IntakePosition target){
        servoIntake.setPosition(target.val);
    }

    public void setTarget(Position target){
        motorBrat.setTargetPosition(target.val);
    }

    public void startPositioning(){
        runThread = new Thread(new PositionPlayer());
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

}
