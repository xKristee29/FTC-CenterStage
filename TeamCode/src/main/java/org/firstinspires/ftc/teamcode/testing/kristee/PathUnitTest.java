package org.firstinspires.ftc.teamcode.testing.kristee;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.ChassisConstants;
import org.firstinspires.ftc.teamcode.drive.Path;
import org.firstinspires.ftc.teamcode.drive.Point;
import org.firstinspires.ftc.teamcode.drive.TankDriveChassis;
import org.firstinspires.ftc.teamcode.drive.TankDriveController;
import org.firstinspires.ftc.teamcode.drive.Utils;

@TeleOp
public class PathUnitTest extends LinearOpMode {

    TankDriveChassis robot;
    TankDriveController controller;

    GamepadEx gp1;

    public void initialize(){
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new TankDriveChassis(hardwareMap);

        robot.init();

        robot.setLimits(0.5,0.3);

        controller = new TankDriveController(robot,telemetry);

        gp1 = new GamepadEx(gamepad1);

        robot.setPower(0,0);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        try{
            initialize();

            waitForStart();

            robot.startAsyncLocalization();

            Path path = new Path(new Point(0,0))
                    //.goTo(new Point(-60, 100, 270));
                    .customSplineTo(new Point(-60, 100, 270),
                            0.5,
                            0.12,
                            0.03,
                            0.02,
                            2
                    );//*/

            controller.run(path);

            while(!isStopRequested() && controller.isRunning());



            throw new InterruptedException();
        }
        catch (InterruptedException e){
            controller.interrupt();
            robot.killSwitch();
        }

    }
}
