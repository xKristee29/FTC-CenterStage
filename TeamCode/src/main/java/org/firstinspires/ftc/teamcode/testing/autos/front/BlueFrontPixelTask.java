package org.firstinspires.ftc.teamcode.testing.autos.front;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.ArmController;
import org.firstinspires.ftc.teamcode.drive.KodiCV;
import org.firstinspires.ftc.teamcode.drive.Path;
import org.firstinspires.ftc.teamcode.drive.Point;
import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.Utils;
import org.firstinspires.ftc.teamcode.testing.sanke.CenterStageCVDetection;


@Autonomous(group = "auto")
public class BlueFrontPixelTask extends LinearOpMode {

    KodiCV cv;
    Robot robot;
    public void initialize(){
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot(hardwareMap, telemetry);
        cv = new KodiCV(telemetry,hardwareMap);

        robot.init();
        robot.drive.setLimits(0.4,0.25);
    }
    public void positionToBackdrop(double angle){
        while(Math.abs(robot.armController.getDistError()) > 3){
            double error = angle - robot.drive.theta;

            error = Utils.minAbs(error, error - Math.signum(error) * 360);

            telemetry.addData("Dist", robot.armController.getDist());
            telemetry.addData("Err", robot.armController.getDistError());
            telemetry.update();

            robot.drive.setPower(
                    0.3 * -Math.tanh(robot.armController.getDistError()),
                    0.08 * Math.tanh(error)
            );

            if(isStopRequested()) break;
        }
        robot.drive.setPower(0,0);
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
                            .goTo(new Point(0,70,270))
                            .goTo(new Point(-60,70,270));

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
                            .goTo(new Point(0,70,0))
                            .goTo(new Point(0,130,270))
                            .goTo(new Point(-80,130,235));

                    robot.driveController.run(pathLeft2);

                    while(robot.driveController.isRunning()){
                        if(isStopRequested()) throw new InterruptedException();
                    }

                    Thread.sleep(1000);

                    Path pathLeft3 = new Path(pathLeft2.lastPoint)
                            .goTo(new Point(-180,65,270));

                    robot.driveController.run(pathLeft3);

                    //finished
                    break;
                case MIDDLE:
                    Path pathMiddle = new Path(path1.lastPoint)
                            .goTo(new Point(0,110));

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
                            .goTo(new Point(0,140,270))
                            .goTo(new Point(-60,140,270));

                    robot.driveController.run(pathMiddle2);

                    while(robot.driveController.isRunning()){
                        if(isStopRequested()) throw new InterruptedException();
                    }


                    //finished

                    break;
                case RIGHT:
                    Path pathRight = new Path(path1.lastPoint)
                            .goTo(new Point(0,70,90))
                            .goTo(new Point(33,70,90));

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
                            .goTo(new Point(33,70,325))
                            .goTo(new Point(-30,140,270))
                            .goTo(new Point(-65,140,270));

                    robot.driveController.run(pathRight2);

                    while(robot.driveController.isRunning()){
                        if(isStopRequested()) throw new InterruptedException();
                    }
                    //finished
                    break;
            }

            while(robot.driveController.isRunning()){
                if(isStopRequested()) throw new InterruptedException();
            }

            positionToBackdrop(270);

            robot.armController.setIntakePosition(ArmController.IntakePosition.GRAB);

            Thread.sleep(200);

            robot.armController.setTarget(ArmController.Position.AUTOPIXEL);

            while(!robot.armController.isPositioned()){
                if(isStopRequested()) throw new InterruptedException();
            }

            Thread.sleep(700);

            /////////////////////////////


            robot.armController.setIntakePosition(ArmController.IntakePosition.MID);

            Thread.sleep(1000);

            robot.armController.setTarget(ArmController.Position.HOME);

            while(!robot.armController.isPositioned()){
                if(isStopRequested()) throw new InterruptedException();
            }


            // Ma parchez in stanga
            Path path2 = new Path(path1.lastPoint)
                    .goTo(new Point(-180,60,200))
                    .goTo(new Point(-200,0,270))
                    .goTo(new Point(-210,0));

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
