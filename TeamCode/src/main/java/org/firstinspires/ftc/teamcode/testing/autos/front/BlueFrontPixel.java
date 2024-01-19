package org.firstinspires.ftc.teamcode.testing.autos.front;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.ArmController;
import org.firstinspires.ftc.teamcode.drive.Path;
import org.firstinspires.ftc.teamcode.drive.Point;
import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.Utils;

@Autonomous(group = "auto")
public class BlueFrontPixel extends LinearOpMode {

    Robot robot;
    public void initialize(){
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot(hardwareMap, telemetry);

        robot.init();
        robot.drive.setLimits(0.5,0.2);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        try{

            initialize();

            waitForStart();

            robot.armController.startPositioning();
            robot.armController.startMeasuring();
            robot.drive.startAsyncLocalization();

            // L path
            Path path1 = new Path(new Point(0,0))
                    .goTo(new Point(0,140,270))
                    .goTo(new Point(-140,140,225));

            robot.driveController.run(path1);

            //(-210,80)
            while(robot.driveController.isRunning()){
                if(isStopRequested()) throw new InterruptedException();
            }

            // Thread.sleep(1000);

            Path path2 = new Path(path1.lastPoint)
                    .goTo(new Point(-210,65,270));

            robot.driveController.run(path2);

            while(robot.driveController.isRunning()){
                if(isStopRequested()) throw new InterruptedException();
            }

            // Se pune langa tabla
            while(Math.abs(robot.armController.getDistError()) > 3){
                double error = 270 - robot.drive.theta;

                error = Utils.minAbs(error, error - Math.signum(error) * 360);

                telemetry.addData("Dist", robot.armController.getDist());
                telemetry.addData("Err", robot.armController.getDistError());
                telemetry.update();

                robot.drive.setPower(
                        0.3 * -Math.signum(robot.armController.getDistError()),
                        -robot.driveController.getRotationalCorrection(error * 0.3)
                );

                if(isStopRequested()) throw new InterruptedException();
            }
            robot.drive.setPower(0,0); //oprim motoarele (stie Cristi why :D )

            robot.armController.setIntakePosition(ArmController.IntakePosition.GRAB);

            Thread.sleep(400);

            robot.armController.setTarget(ArmController.Position.LEVEL1);

            while(!robot.armController.isPositioned()){
                if(isStopRequested()) throw new InterruptedException();
            }

            Thread.sleep(700);

            /////////////////////////////


            robot.armController.setIntakePosition(ArmController.IntakePosition.THROW);

            Thread.sleep(700);

            robot.armController.setTarget(ArmController.Position.HOME);

            while(!robot.armController.isPositioned()){
                if(isStopRequested()) throw new InterruptedException();
            }

            Thread.sleep(700);

            /////////////////////////////

            /*/Ne parcam
            while(!robot.armController.isPositioned()){
                if(isStopRequested()) throw new InterruptedException();
            }

            Path path3 = new Path(path2.lastPoint)
                    .goTo(new Point(-220,60,180));

            robot.driveController.run(path3);

            while(robot.driveController.isRunning()){
                if(isStopRequested()) throw new InterruptedException();
            }*/

            throw new InterruptedException();
        }
        catch (InterruptedException e){
            robot.killSwitch();
        }
    }

}