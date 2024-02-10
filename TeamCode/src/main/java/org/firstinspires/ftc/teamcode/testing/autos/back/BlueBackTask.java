package org.firstinspires.ftc.teamcode.testing.autos.back;

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

    public void positionToBackdrop(double angle){
        double error = 2e9;
        while(Math.abs(robot.armController.getDistError()) > 3 || Math.abs(error) > 2){
            error = angle - robot.drive.theta;

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
                            .goTo(new Point(-63,70,270));
                    break;
                case MIDDLE:
                    pathRandom = new Path(path1.lastPoint)
                            .goTo(new Point(0,70,0))
                            .goTo(new Point(0,125,0));
                    break;
                case RIGHT:
                    pathRandom = new Path(path1.lastPoint)
                            .goTo(new Point(0,70,270))
                            .goTo(new Point(30,70,270));
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
                            .goTo(new Point(-70,45,270));
                    break;
                case MIDDLE:
                    pathRandom = new Path(pathRandom.lastPoint)
                            .goTo(new Point(0,150,0))
                            .goTo(new Point(0,130,0))
                            .goTo(new Point(-40,110))
                            .goTo(new Point(-70,60,270));
                    break;
                case RIGHT:
                    pathRandom = new Path(pathRandom.lastPoint)
                            .goTo(new Point(-70,75,270));
                    break;
            }

            robot.driveController.run(pathRandom);
            while(robot.driveController.isRunning()){
                if(isStopRequested()) throw new InterruptedException();
            }

            double error = 2e9;

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
                    .goTo(new Point(-40,60,200))
                    .goTo(new Point(-80,0,270))
                    .goTo(new Point(-130,0));

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
