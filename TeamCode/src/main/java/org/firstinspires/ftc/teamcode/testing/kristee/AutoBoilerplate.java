package org.firstinspires.ftc.teamcode.testing.kristee;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.ArmController;
import org.firstinspires.ftc.teamcode.drive.Robot;

@Autonomous
public class AutoBoilerplate extends LinearOpMode {

    Robot robot;
    public void initialize(){
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot(hardwareMap, telemetry);

        robot.init();
        robot.drive.setLimits(0.5,0.3);
    }


    @Override
    public void runOpMode() throws InterruptedException {

        try{

            initialize();

            waitForStart();

            robot.armController.startPositioning();
            robot.drive.startAsyncLocalization();

            throw new InterruptedException();
        }
        catch (InterruptedException e){
            robot.killSwitch();
        }

    }

}
