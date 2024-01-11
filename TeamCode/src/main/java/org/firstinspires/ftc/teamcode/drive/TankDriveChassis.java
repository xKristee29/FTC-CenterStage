package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.opencv.core.Mat;

@Config
public class TankDriveChassis {

    MotorEx leftMotor;
    MotorEx rightMotor;

    public KodiIMU gyro;

    public long leftEncoder = 0;
    public long rightEncoder = 0;
    public long prevLeftEncoder = 0;
    public long prevRightEncoder = 0;

    public double heading = 0;

    public double encoderHeading = 0;

    public double x = 0, y = 0, theta = 0;

    public double motorPowerLeft = 0, motorPowerRight = 0;
    public static double accelerationFactor = 0.015;

    public double forwardSpeedLimit = 0.5;
    public double rotationalSpeedLimit = 0.5;

    public Thread localizerThread = null;

    public TankDriveChassis(HardwareMap hMap){
        leftMotor = new MotorEx(
                hMap, "left_motor",
                ChassisConstants.TICKS_PER_REV,
                ChassisConstants.MAX_RPM
        );
        rightMotor = new MotorEx(
                hMap, "right_motor",
                ChassisConstants.TICKS_PER_REV,
                ChassisConstants.MAX_RPM
        );

        gyro = new KodiIMU(hMap);
    }

    public void init(){
        gyro.init();
        gyro.invertGyro();

        leftMotor.setDistancePerPulse(0.08414980322115517);
        leftMotor.setRunMode(Motor.RunMode.RawPower);
        leftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        rightMotor.setDistancePerPulse(0.08414980322115517);
        rightMotor.setRunMode(Motor.RunMode.RawPower);
        rightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightMotor.setInverted(true);

        resetEncoders();
        resetGyro();
    }

    public void setLimits(double fs, double rs){
        forwardSpeedLimit = fs;
        rotationalSpeedLimit = rs;
    }

    public void setPower(double forwardSpeed, double rotationSpeed){
        double fs = Range.clip(forwardSpeed, -forwardSpeedLimit, forwardSpeedLimit);
        double rs = Range.clip(rotationSpeed, -rotationalSpeedLimit, rotationalSpeedLimit);

        motorPowerLeft = fs + rs;
        motorPowerRight = fs - rs;

        leftMotor.set(motorPowerLeft);
        rightMotor.set(motorPowerRight);
    }

    public void setPowerRamp(double forwardSpeed, double rotationSpeed){
        double fs = Range.clip(forwardSpeed, -forwardSpeedLimit, forwardSpeedLimit);
        double rs = Range.clip(rotationSpeed, -rotationalSpeedLimit, rotationalSpeedLimit);

        double targetPowerLeft = fs + rs;
        double targetPowerRight = fs - rs;

        double deltaLeft = targetPowerLeft - motorPowerLeft;
        double deltaRight = targetPowerRight - motorPowerRight;

        if(targetPowerLeft == 0) motorPowerLeft = 0;
        else if(Math.abs(deltaLeft) > 0.03){
            motorPowerLeft += accelerationFactor * Math.signum(deltaLeft);
        }
        else motorPowerLeft = targetPowerLeft;

        if(targetPowerRight == 0) motorPowerRight = 0;
        if(Math.abs(deltaRight) > 0.03){
            motorPowerRight += accelerationFactor * Math.signum(deltaRight);
        }
        else motorPowerRight = targetPowerRight;

        leftMotor.set(motorPowerLeft);
        rightMotor.set(motorPowerRight);
    }

    public void updatePosition(){

        heading = gyro.getHeading();
        leftEncoder = leftMotor.getCurrentPosition();
        rightEncoder = rightMotor.getCurrentPosition();

        double multiplier = 1 / ChassisConstants.TICKS_PER_REV * ChassisConstants.WHEEL_DIAM * Math.PI;

        double distanceLeft = (leftEncoder - prevLeftEncoder) * multiplier;
        double distanceRight = (rightEncoder - prevRightEncoder) * multiplier;

        double distance = (distanceLeft + distanceRight) * 0.5;

        double deltaAngleEncoder = Math.toDegrees((distanceRight - distanceLeft) / ChassisConstants.TRACK_WIDTH);

        encoderHeading += deltaAngleEncoder;

        prevLeftEncoder = leftEncoder;
        prevRightEncoder = rightEncoder;

        theta = (int)(heading + encoderHeading * ChassisConstants.alphaEncoder + 360) % 360;

        double radians = Math.toRadians(theta);

        x += distance * Math.sin(radians);
        y += distance * Math.cos(radians);
    }

    public void startAsyncLocalization(){
        localizerThread = new Thread(new AsyncLocalization());
        localizerThread.start();
    }

    public void interruptAsyncLocalization(){
        if(localizerThread != null){
            localizerThread.interrupt();
        }
    }

    public void killSwitch(){
        interruptAsyncLocalization();
        setPower(0,0);
    }

    public class AsyncLocalization implements Runnable{

        @Override
        public void run() {

            while(!Thread.interrupted()) {

                heading = gyro.getHeading();
                leftEncoder = leftMotor.getCurrentPosition();
                rightEncoder = rightMotor.getCurrentPosition();

                double multiplier = 1 / ChassisConstants.TICKS_PER_REV * ChassisConstants.WHEEL_DIAM * Math.PI;

                double distanceLeft = (leftEncoder - prevLeftEncoder) * multiplier;
                double distanceRight = (rightEncoder - prevRightEncoder) * multiplier;

                double distance = (distanceLeft + distanceRight) * 0.5;

                double deltaAngleEncoder = Math.toDegrees((distanceRight - distanceLeft) / ChassisConstants.TRACK_WIDTH);

                encoderHeading += deltaAngleEncoder;

                prevLeftEncoder = leftEncoder;
                prevRightEncoder = rightEncoder;

                theta = (int)(heading + encoderHeading * ChassisConstants.alphaEncoder + 360) % 360;

                double radians = Math.toRadians(theta);

                x += distance * Math.sin(radians);
                y += distance * Math.cos(radians);
            }
        }
    }

    public void resetEncoders(){
        leftMotor.resetEncoder();
        rightMotor.resetEncoder();
    }

    public void resetGyro(){
        gyro.reset();
    }
}
