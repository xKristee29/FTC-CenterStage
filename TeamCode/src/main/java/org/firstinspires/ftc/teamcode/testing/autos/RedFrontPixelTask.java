package org.firstinspires.ftc.teamcode.testing.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.KodiCV;
import org.firstinspires.ftc.teamcode.drive.Path;
import org.firstinspires.ftc.teamcode.drive.Point;
import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.testing.sanke.CenterStageCVDetection;


@Autonomous(group = "auto")
public class RedFrontPixelTask extends LinearOpMode {

    KodiCV cv;
    Robot robot;
    public void initialize(){
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot(hardwareMap, telemetry);
        cv = new KodiCV(telemetry,hardwareMap);

        robot.init();
        robot.drive.setLimits(0.5,0.3);
    }


    @Override
    public void runOpMode() throws InterruptedException {
        try{

            CenterStageCVDetection.Location location = CenterStageCVDetection.Location.LEFT;

            initialize();

            cv.start();

            waitForStart();

            cv.kill();

            robot.armController.startPositioning();
            robot.armController.startMeasuring();
            robot.drive.startAsyncLocalization();

            Path path1 = new Path(new Point(0,0))
                    .goTo(new Point(0,70));

            robot.driveController.run(path1);

            while(robot.driveController.isRunning()){
                if(isStopRequested()) throw new InterruptedException();
            }

            switch(cv.detector.location){
                case LEFT:
                    Path pathLeft = new Path(path1.lastPoint)
                            .goTo(new Point(0,70,180))
                            .goTo(new Point(33,70,180));

                    robot.driveController.run(pathLeft);

                    while(robot.driveController.isRunning()){
                        if(isStopRequested()) throw new InterruptedException();
                    }

                    robot.armController.dropPixel();

                    Thread.sleep(2000);

                    while(robot.driveController.isRunning()){
                        if(isStopRequested()) throw new InterruptedException();
                    }

                    Path pathLeft2 = new Path(pathLeft.lastPoint)
                            .goTo(new Point(33,70,45))
                            .goTo(new Point(-30,140,270))
                            .goTo(new Point(-65,140,270));

                    robot.driveController.run(pathLeft2);

                    while(robot.driveController.isRunning()){
                        if(isStopRequested()) throw new InterruptedException();
                    }
                    //finished
                    break;

                case MIDDLE:
                    Path pathMiddle = new Path(path1.lastPoint)
                            .goTo(new Point(0,120));

                    robot.driveController.run(pathMiddle);

                    while(robot.driveController.isRunning()){
                        if(isStopRequested()) throw new InterruptedException();
                    }

                    robot.armController.dropPixel();

                    Thread.sleep(2000);

                    while(robot.driveController.isRunning()){
                        if(isStopRequested()) throw new InterruptedException();
                    }

                    Path pathMiddle2 = new Path(pathMiddle.lastPoint)
                            .goTo(new Point(0,140,90))
                            .goTo(new Point(65,140,90));

                    robot.driveController.run(pathMiddle2);

                    while(robot.driveController.isRunning()){
                        if(isStopRequested()) throw new InterruptedException();
                    }

                    //ne punem in fata tablei in functie de randomizare -> punem pixel -> ne parcam
                    //finished
                    break;

                case RIGHT:
                    Path pathRight = new Path(path1.lastPoint)
                            .goTo(new Point(0,70,270))
                            .goTo(new Point(63,70,270));

                    robot.driveController.run(pathRight);

                    while(robot.driveController.isRunning()){
                        if(isStopRequested()) throw new InterruptedException();
                    }

                    robot.armController.dropPixel();

                    Thread.sleep(2000);

                    while(robot.driveController.isRunning()){
                        if(isStopRequested()) throw new InterruptedException();
                    }

                    Path pathRight2 = new Path(pathRight.lastPoint)
                            .goTo(new Point(63,70,90))
                            .goTo(new Point(63,140,270));

                    robot.driveController.run(pathRight2);

                    while(robot.driveController.isRunning()){
                        if(isStopRequested()) throw new InterruptedException();
                    }
                    //finished
                    break;
            }

            throw new InterruptedException();
        }
        catch (InterruptedException e){
            robot.killSwitch();
        }

    }

}
