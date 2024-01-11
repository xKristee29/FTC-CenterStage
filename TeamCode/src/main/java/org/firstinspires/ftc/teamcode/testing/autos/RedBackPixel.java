package org.firstinspires.ftc.teamcode.testing.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.ArmController;
import org.firstinspires.ftc.teamcode.drive.Path;
import org.firstinspires.ftc.teamcode.drive.Point;
import org.firstinspires.ftc.teamcode.drive.Robot;

@Autonomous
public class RedBackPixel extends LinearOpMode {

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

            Path path1 = new Path(new Point(0,0))
                    .goTo(new Point(-60,70,270));

            robot.driveController.run(path1);

            while(robot.driveController.isRunning()){
                if(isStopRequested()) throw new InterruptedException();
            }

            robot.armController.setTarget(ArmController.Position.LEVEL1);

            while(!robot.armController.isPositioned()){
                if(isStopRequested()) throw new InterruptedException();
            }

            Thread.sleep(1000);

            robot.armController.setTarget(ArmController.Position.HOME);

            while(!robot.armController.isPositioned()){
                if(isStopRequested()) throw new InterruptedException();
            }

            Path path2 = new Path(path1.lastPoint)
                    .goTo(new Point(-80,10,270))
                    .goTo(new Point(-120,10));

            robot.driveController.run(path2);

            while(robot.driveController.isRunning()){
                if(isStopRequested()) throw new InterruptedException();
            }

            throw new InterruptedException();
        }
        catch (InterruptedException e){
            robot.killSwitch();
        }

    }
}
