package org.firstinspires.ftc.teamcode.testing.autos.back;

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
public class BlueBackPixel extends LinearOpMode {

    Robot robot;
    public void initialize(){
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot(hardwareMap, telemetry);

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
                    0.3 * -Math.tanh(error)
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

            // Se duce in fata tablei
            Path path1 = new Path(new Point(0,0))
                    .goTo(new Point(0,67,270))
                    .goTo(new Point(-60,67,270));

            robot.driveController.run(path1);

            while(robot.driveController.isRunning()){
                if(isStopRequested()) throw new InterruptedException();
            }

            ///////////////////////////////

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

            // Ma parchez in stanga
            Path path2 = new Path(path1.lastPoint)
                    .goTo(new Point(-30,60,200))
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