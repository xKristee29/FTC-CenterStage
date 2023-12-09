package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.ChassisConstants;
import org.firstinspires.ftc.teamcode.drive.TankDriveChassis;
import org.firstinspires.ftc.teamcode.drive.TankDriveController;

@Autonomous
public class KodikationAutoTest extends LinearOpMode {

    TankDriveChassis robot;
    TankDriveController controller;

    GamepadEx gp1;

    public void initialize(){
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new TankDriveChassis(hardwareMap);

        robot.init();

        controller = new TankDriveController(robot);

        gp1 = new GamepadEx(gamepad1);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        try{
            initialize();

            waitForStart();

            robot.startAsyncLocalization();

            double x = 60;
            double y = 0;

            while((Math.abs(x - robot.x) > ChassisConstants.toleranceXY ||
                    Math.abs(y - robot.y) > ChassisConstants.toleranceXY) &&
                    !isStopRequested()){

                double errorX = x - robot.x;
                double errorY = y - robot.y;

                double distance = Math.sqrt(errorX * errorX + errorY * errorY);

                double alpha = Math.atan2(errorY, errorX);

                double errorR = alpha - robot.theta;

                robot.setPowerRamp(gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) * Math.tanh(distance),
                        gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) * Math.tanh(errorR));


                telemetry.addData("x", robot.x);
                telemetry.addData("y", robot.y);
                telemetry.addData("theta", (int)Math.toDegrees(robot.theta));
                telemetry.addLine();
                telemetry.addData("Dist", distance);
                telemetry.addData("Alpha", Math.toDegrees(alpha));
                telemetry.addData("DR", Math.toDegrees(errorR));
                telemetry.update();
            }

            throw new InterruptedException();
        }
        catch (InterruptedException e){
            robot.killSwitch();
        }

    }
}
