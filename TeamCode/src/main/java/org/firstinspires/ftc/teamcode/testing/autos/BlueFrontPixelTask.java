package org.firstinspires.ftc.teamcode.testing.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.Path;
import org.firstinspires.ftc.teamcode.drive.Point;
import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.testing.sanke.CenterStageCVDetection;
import org.firstinspires.ftc.teamcode.testing.sanke.CenterStageCVDetection1;

@Autonomous
public class BlueFrontPixelTask extends LinearOpMode {

    Robot robot;
    public void initialize(){
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot(hardwareMap, telemetry);

        robot.init();
        robot.drive.setLimits(0.5,0.5);
    }


    @Override
    public void runOpMode() throws InterruptedException {
        try{

            CenterStageCVDetection.Location location = CenterStageCVDetection.Location.LEFT;

            initialize();

            waitForStart();

            robot.armController.startPositioning();
            robot.armController.startMeasuring();
            robot.drive.startAsyncLocalization();

            Path path1 = new Path(new Point(0,0))
                    .goTo(new Point(0,70));

            robot.driveController.run(path1);

            while(robot.driveController.isRunning()){
                if(isStopRequested()) throw new InterruptedException();
            }

            switch(location){
                case LEFT:
                    Path path2 = new Path(path1.lastPoint)
                            .goTo(new Point(0,70,270))
                            .goTo(new Point(-63,70,270));

                    robot.driveController.run(path2);

                    while(robot.driveController.isRunning()){
                        if(isStopRequested()) throw new InterruptedException();
                    }
                    break;
            }

            robot.armController.dropPixel();

            Thread.sleep(2000);

            throw new InterruptedException();
        }
        catch (InterruptedException e){
            robot.killSwitch();
        }

    }

}
