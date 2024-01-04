package org.firstinspires.ftc.teamcode.testing.kristee;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.ChassisConstants;
import org.firstinspires.ftc.teamcode.drive.TankDriveChassis;
import org.firstinspires.ftc.teamcode.drive.Utils;

@TeleOp
public class DrivePIDUnitTest extends LinearOpMode {

    TankDriveChassis robot;

    GamepadEx gp1;

    PIDController pidGyro = new PIDController(0, 0, 0);
    PIDController pidDrive = new PIDController(0, 0, 0);

    public void initialize(){
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new TankDriveChassis(hardwareMap);

        robot.init();

        robot.setLimits(0.5,0.4);

        gp1 = new GamepadEx(gamepad1);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        try{
            initialize();

            waitForStart();

            robot.startAsyncLocalization();

            while(!isStopRequested()){

                gp1.readButtons();

                pidGyro.setPID(ChassisConstants.HEADING_PID.p,
                        ChassisConstants.HEADING_PID.i,
                        ChassisConstants.HEADING_PID.d);

                pidDrive.setPID(ChassisConstants.DRIVE_PID.p,
                        ChassisConstants.DRIVE_PID.i,
                        ChassisConstants.DRIVE_PID.d);

                double targetX = 30;
                double targetY = 90;

                double errorX = targetX - robot.x;
                double errorY = targetY - robot.y;

                double distance = (int)Math.hypot(errorX, errorY);

                double targetAngle = ((int)Math.toDegrees(Math.atan2(errorX, errorY)) + 360) % 360;
                double angleError = targetAngle - robot.theta;

                angleError = Utils.minAbs(angleError, angleError - Math.signum(angleError) * 360);

                if(Math.abs(angleError) > 130){
                    distance = -distance;
                }

                if(Math.abs(distance) <= ChassisConstants.targetRadius) {
                    angleError = 0;
                }

                if(Math.abs(distance) <= ChassisConstants.toleranceXY){
                    distance = 0;
                }

                double powerR = -pidGyro.calculate(angleError);
                double powerX = -pidDrive.calculate(distance);

                robot.setPowerRamp(powerX, powerR);

                telemetry.addData("x", robot.x);
                telemetry.addData("y", robot.y);
                telemetry.addData("theta", robot.theta);
                telemetry.addLine();
                telemetry.addData("PWR", powerR);
                telemetry.addData("Target", targetAngle);
                telemetry.addData("Ang Error", angleError);
                telemetry.addLine();
                telemetry.addData("Dist", distance);
                telemetry.update();
            }
            throw new InterruptedException();
        }
        catch (InterruptedException e){
            robot.killSwitch();
        }

    }
}
