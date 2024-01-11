package org.firstinspires.ftc.teamcode.testing.sanke;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.TankDriveChassis;

@TeleOp
public class SensorAlignmentUnitTest extends LinearOpMode {

    TankDriveChassis robot;

    DistanceSensor distLeft, distRight;

    GamepadEx gp1;

    public void initialize(){
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new TankDriveChassis(hardwareMap);

        robot.init();

        gp1 = new GamepadEx(gamepad1);

        distLeft = hardwareMap.get(DistanceSensor.class, "distance_left");
        distRight = hardwareMap.get(DistanceSensor.class, "distance_right");
    }

    @Override
    public void runOpMode() throws InterruptedException {

        try{

            initialize();

            waitForStart();

            while (!isStopRequested()){

                double d1 = distLeft.getDistance(DistanceUnit.CM);
                double d2 = distRight.getDistance(DistanceUnit.CM);

                double r = (d1 - d2) * 0.115;

                robot.setPowerRamp(0, r);

                telemetry.addData("Left", d1);
                telemetry.addData("Right", d2);
                telemetry.addData("R",r);
                telemetry.update();
            }

            throw new InterruptedException();
        }
        catch (InterruptedException e){
            robot.killSwitch();
        }

    }
}
