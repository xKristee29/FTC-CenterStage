package org.firstinspires.ftc.teamcode.testing.kristee;

import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.DifferentialOdometry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp
public class LocalizerUnitTest extends LinearOpMode {

    final double TRACKWIDTH = 30;
    final double TICKS_TO_CM = 0.08414980322;
    final double CPR = 336;
    final double RPM = 500;

    MotorEx leftMotor = null;
    MotorEx rightMotor = null;

    DifferentialDrive drive = null;
    DifferentialOdometry odom = null;

    public void initialize(){
        leftMotor = new MotorEx(hardwareMap, "left_motor", CPR, RPM);
        rightMotor = new MotorEx(hardwareMap, "right_motor", CPR, RPM);

        leftMotor.setDistancePerPulse(0.08414980322115517);
        leftMotor.setRunMode(Motor.RunMode.RawPower);
        leftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        rightMotor.setDistancePerPulse(0.08414980322115517);
        rightMotor.setRunMode(Motor.RunMode.RawPower);
        rightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        
        drive = new DifferentialDrive(leftMotor, rightMotor);
        drive.setRightSideInverted(true);

        odom = new DifferentialOdometry(
            () -> leftMotor.getCurrentPosition() * TICKS_TO_CM,
            () -> rightMotor.getCurrentPosition() * TICKS_TO_CM,
            TRACKWIDTH
        );


    }

    public void killSwitch(){
        drive.stop();
    }

    @Override
    public void runOpMode() throws InterruptedException{

        try{
            initialize();

            GamepadEx gp1 = new GamepadEx(gamepad1);

            waitForStart();

            while(!isStopRequested()){
                gp1.readButtons();

                drive.arcadeDrive(gp1.getLeftY(),gp1.getLeftX(),true);

                odom.updatePose();

                Pose2d pose = odom.getPose();
                
                telemetry.addData("X", pose.getX());
                telemetry.addData("Y", pose.getY());
                telemetry.addData("<", pose.getHeading());
                telemetry.update();
            }

            throw new InterruptedException();
        }
        catch (InterruptedException e){
            killSwitch();
        }
    }

}
