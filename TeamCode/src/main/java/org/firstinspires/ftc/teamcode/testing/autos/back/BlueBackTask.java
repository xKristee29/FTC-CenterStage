package org.firstinspires.ftc.teamcode.testing.autos.back;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.KodiCV;
import org.firstinspires.ftc.teamcode.drive.Path;
import org.firstinspires.ftc.teamcode.drive.Point;
import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.Utils;

@Autonomous(group = "auto-task")
public class BlueBackTask extends LinearOpMode {

    Robot robot;
    KodiCV cv;

    public void initialize(){
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot(hardwareMap, telemetry);
        cv = new KodiCV(telemetry,hardwareMap);

        robot.init();
        robot.drive.setLimits(0.4,0.25);
    }


    @Override
    public void runOpMode() throws InterruptedException {
        try{

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

            Path pathRandom = null;

            switch (cv.detector.location){
                case LEFT:
                    pathRandom = new Path(path1.lastPoint)
                            .goTo(new Point(0,70,270))
                            .goTo(new Point(-60,70,270));
                    break;
                case RIGHT:
                    pathRandom = new Path(path1.lastPoint)
                            .goTo(new Point(0,70,270))
                            .goTo(new Point(13,70,270));
                    break;
                case MIDDLE:
                    pathRandom = new Path(path1.lastPoint)
                            .goTo(new Point(0,70,0))
                            .goTo(new Point(0,110,0));
                    break;
            }

            robot.driveController.run(pathRandom);
            while(robot.driveController.isRunning()){
                if(isStopRequested()) throw new InterruptedException();
            }

            robot.drive.setPower(0,0);

            robot.armController.dropPixel();

            Thread.sleep(1000);

            switch (cv.detector.location){
                case LEFT:
                    pathRandom = new Path(pathRandom.lastPoint)
                            .goTo(new Point(-60,52,270));
                    break;
                case MIDDLE:
                    pathRandom = new Path(pathRandom.lastPoint)
                            .goTo(new Point(-40,110))
                            .goTo(new Point(-60,67,270));
                    break;
                case RIGHT:
                    pathRandom = new Path(pathRandom.lastPoint)
                            .goTo(new Point(-60,82,270));
                    break;
            }

            robot.driveController.run(pathRandom);
            while(robot.driveController.isRunning()){
                if(isStopRequested()) throw new InterruptedException();
            }

            while(Math.abs(robot.armController.getDistError()) > 3){
                double error = 270 - robot.drive.theta;

                error = Utils.minAbs(error, error - Math.signum(error) * 360);

                telemetry.addData("Dist", robot.armController.getDist());
                telemetry.addData("Err", robot.armController.getDistError());
                telemetry.update();

                robot.drive.setPower(
                        0.3 * -Math.signum(robot.armController.getDistError()),
                        0.3 * -Math.signum(robot.driveController.getRotationalCorrection(error))
                );

                if(isStopRequested()) throw new InterruptedException();
            }
            robot.drive.setPower(0,0);

            throw new InterruptedException();
        }
        catch (InterruptedException e){
            robot.killSwitch();
        }

    }

}
