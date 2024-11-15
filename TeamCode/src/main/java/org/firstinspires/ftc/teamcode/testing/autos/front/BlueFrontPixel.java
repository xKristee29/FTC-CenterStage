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

@Autonomous(group = "auto-finished")
public class BlueFrontPixel extends LinearOpMode {

    Robot robot;
    public void initialize(){
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot(hardwareMap, telemetry);

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

            waitForStart();

            robot.armController.startPositioning();
            robot.armController.startMeasuring();
            robot.drive.startAsyncLocalization();

            Thread.sleep(12000);

            //daca pleaca bratul din control in sus devine inutilizabil

            // L path
            Path path1 = new Path(new Point(0,0))
                    .goTo(new Point(0,140,270))
                    .goTo(new Point(-140,140,225));

            robot.driveController.run(path1);

            //(-210,80)
            while(robot.driveController.isRunning()){
                if(isStopRequested()) throw new InterruptedException();
            }


            Path path2 = new Path(path1.lastPoint)
                    .goTo(new Point(-210,67,270));

            robot.driveController.run(path2);

            while(robot.driveController.isRunning()){
                if(isStopRequested()) throw new InterruptedException();
            }

            // Se pune langa tabla
            positionToBackdrop(270);

            robot.armController.setIntakePosition(ArmController.IntakePosition.GRAB);

            Thread.sleep(400);

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