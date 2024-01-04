package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.TankDriveChassis;

@TeleOp
public class ControlTeleghidat extends LinearOpMode {

    TankDriveChassis robot;

    GamepadEx gp1;

    public void initialize(){
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new TankDriveChassis(hardwareMap);

        robot.init();

        robot.setLimits(0.5,0.3);

        gp1 = new GamepadEx(gamepad1);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        try{

            initialize();

            waitForStart();

            while (!isStopRequested()){
                gp1.readButtons();

                double fs = gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) -
                        gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
                double rs = gp1.getLeftX();

                robot.setPowerRamp(fs * 0.5, rs * 0.3);

                telemetry.addData("X", fs);
                telemetry.addData("R", rs);
                telemetry.update();

                /*
                X : {fs}
                R : {rs}
                 */
            }

            throw new InterruptedException();
        }
        catch (InterruptedException e){
            robot.killSwitch();
        }

    }
}
