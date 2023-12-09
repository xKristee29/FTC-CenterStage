package org.firstinspires.ftc.teamcode.drive;

public class TankDriveController {

    TankDriveChassis robot;

    public double targetX = 0, targetY = 0, targetR = 0;

    public TankDriveController(TankDriveChassis robot){
        this.robot = robot;
    }

    public void setPoint(double x, double y, double pwr){

        while(Math.abs(x - robot.x) > ChassisConstants.toleranceXY || Math.abs(y - robot.y) > ChassisConstants.toleranceXY){

            double errorX = x - robot.x;
            double errorY = y - robot.y;

            double distance = Math.sqrt(errorX * errorX + errorY * errorY);

            double alpha = Math.atan2(errorX, errorY);

            double errorR = alpha - robot.theta;

            robot.setPowerRamp(pwr * Math.tanh(distance), pwr * Math.tanh(errorR));

        }

        robot.setPower(0,0);


    }

}
