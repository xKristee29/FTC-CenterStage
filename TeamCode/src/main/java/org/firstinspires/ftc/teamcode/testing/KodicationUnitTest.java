package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.TankDriveChassis;

@TeleOp
public class KodicationUnitTest extends LinearOpMode {

    TankDriveChassis robot;

    GamepadEx gp1;

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

            while(!isStopRequested()){

                robot.updatePosition();

                robot.setPowerRamp(
                        (gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) -
                                gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER))*0.4,
                        gp1.getLeftX()*0.4
                );

                telemetry.addData("x", robot.x);
                telemetry.addData("y", robot.y);
                telemetry.addData("theta", robot.theta);
                telemetry.addData("IMU", (int) robot.heading);
                telemetry.addData("<", (int) robot.encoderHeading);
                telemetry.addLine();
                telemetry.addData("left", robot.leftEncoder);
                telemetry.addData("right", robot.rightEncoder);
                telemetry.update();
            }
            throw new InterruptedException();
        }
        catch (InterruptedException e){
            robot.killSwitch();
        }

    }
}
