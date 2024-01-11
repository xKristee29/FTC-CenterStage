package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.ArmController;
import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.TankDriveChassis;

@TeleOp
public class ControlTeleghidat extends LinearOpMode {

    Robot robot;

    GamepadEx gp1,gp2;

    public void initialize(){
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot(hardwareMap, telemetry);

        robot.init();

        gp1 = new GamepadEx(gamepad1);
        gp2 = new GamepadEx(gamepad2);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        try{

            initialize();

            waitForStart();

            robot.armController.startPositioning();

            while (!isStopRequested()){
                gp1.readButtons();
                gp2.readButtons();

                double fs = gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) -
                        gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
                double rs = gp1.getLeftX();

                robot.drive.setPowerRamp(fs * 0.5, rs * 0.3);

                if(gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
                    robot.armController.setTarget(ArmController.Position.LAUNCH);
                }
                if(gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT) ||
                        gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)){
                    robot.armController.setTarget(ArmController.Position.LEVEL2);
                }
                if(gp2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
                    robot.armController.setTarget(ArmController.Position.LEVEL1);
                }

                if(gp2.wasJustPressed(GamepadKeys.Button.Y)){
                    robot.armController.setTarget(ArmController.Position.HANG);
                }

                if(gp2.wasJustPressed(GamepadKeys.Button.A)){
                    robot.armController.setTarget(ArmController.Position.HOME);
                }

                telemetry.addData("X", fs);
                telemetry.addData("R", rs);
                telemetry.addData("ArmPos", robot.armController.getPosition());
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
