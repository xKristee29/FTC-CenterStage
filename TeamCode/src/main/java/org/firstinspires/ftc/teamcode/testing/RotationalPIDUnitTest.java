package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.ChassisConstants;
import org.firstinspires.ftc.teamcode.drive.TankDriveChassis;
import org.firstinspires.ftc.teamcode.drive.Utils;
import org.opencv.core.Mat;

@TeleOp
public class RotationalPIDUnitTest extends LinearOpMode {

    TankDriveChassis robot;

    GamepadEx gp1;

    PIDController pid = new PIDController(0,0,0);

    public void initialize(){
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new TankDriveChassis(hardwareMap);

        robot.init();

        gp1 = new GamepadEx(gamepad1);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        try{
            initialize();

            waitForStart();

            robot.startAsyncLocalization();

            double targetAngle = 0;

            while(!isStopRequested()){

                gp1.readButtons();

                pid.setPID(ChassisConstants.HEADING_PID.p,
                        ChassisConstants.HEADING_PID.i,
                        ChassisConstants.HEADING_PID.d);

                if(gp1.wasJustPressed(GamepadKeys.Button.A)) targetAngle = (targetAngle + 90 + 360) % 360;

                double error = targetAngle - robot.theta;

                error = - Utils.minAbs(error, error - Math.signum(error) * 360);

                double power = pid.calculate(error);

                robot.setPowerRamp(0, power);

                telemetry.addData("x", robot.x);
                telemetry.addData("y", robot.y);
                telemetry.addData("theta", robot.theta);
                telemetry.addLine();
                telemetry.addData("PWR", power);
                telemetry.addData("Target", targetAngle);
                telemetry.addData("Error", error);
                telemetry.update();
            }
            throw new InterruptedException();
        }
        catch (InterruptedException e){
            robot.killSwitch();
        }

    }
}
