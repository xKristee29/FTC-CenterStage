package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.ArmController;
import org.firstinspires.ftc.teamcode.drive.Robot;

@TeleOp
public class ControlTeleghidat extends LinearOpMode {

    Robot robot;

    GamepadEx gp1,gp2;

    double robotAngle;

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
            robot.armController.startMeasuring();
            robot.drive.startAsyncLocalization();

            while (!isStopRequested()){
                gp1.readButtons();
                gp2.readButtons();

                double fs = gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) -
                        gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
                double rs = gp1.getLeftX();

                if(gamepad1.a){
                    fs = 0.3 * -Math.tanh(robot.armController.getDistError());
                }

                double xMulti = 0.4;
                double rMulti = 0.3;

                if(gamepad1.right_bumper){
                    xMulti = 0.25; rMulti = 0.25;
                }
                if (gamepad1.left_bumper) {
                    xMulti = 0.5;
                }
                robot.drive.setPower(fs * xMulti, rs * rMulti);

                if(gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
                    robot.armController.setTarget(ArmController.Position.LEVEL3);
                    robot.armController.setIntakePosition(ArmController.IntakePosition.GRAB);
                }
                if(gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT) ||
                        gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)){
                    robot.armController.setTarget(ArmController.Position.LEVEL2);
                    robot.armController.setIntakePosition(ArmController.IntakePosition.GRAB);
                }
                if(gp2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
                    robot.armController.setTarget(ArmController.Position.LEVEL1);
                    robot.armController.setIntakePosition(ArmController.IntakePosition.GRAB);
                }

                if(gp2.wasJustPressed(GamepadKeys.Button.Y)){
                    robot.armController.setTarget(ArmController.Position.HANG);
                }

                if(gp2.wasJustPressed(GamepadKeys.Button.A)){
                    robot.armController.setTarget(ArmController.Position.HOME);
                    robot.armController.setIntakePosition(ArmController.IntakePosition.THROW);
                }

                if(gp2.wasJustPressed(GamepadKeys.Button.B)){
                    robot.armController.setTarget(ArmController.Position.HOME);
                    robot.armController.motorBrat.set(1);
                    robot.armController.setIntakePosition(ArmController.IntakePosition.THROW);
                }

                if(gp2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)){
                    robot.armController.setIntakePosition(ArmController.IntakePosition.GRAB);
                }

                if(gp2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)){
                    robot.armController.setIntakePosition(ArmController.IntakePosition.THROW);
                }

                if(gp2.wasJustPressed(GamepadKeys.Button.X)){
                    robot.armController.setIntakePosition(ArmController.IntakePosition.MID);
                }

                if(gp1.wasJustPressed(GamepadKeys.Button.Y)){
                    robot.armController.setLauncherPosition(ArmController.LauncherPosition.LAUNCH);
                }

                if(gp1.wasJustPressed(GamepadKeys.Button.X)){
                    robot.armController.setLauncherPosition(ArmController.LauncherPosition.IDLE);
                }

                telemetry.addData("X", fs);
                telemetry.addData("R", rs);
                telemetry.addData("Dist", robot.armController.getDist());
                telemetry.addData("ArmPos", robot.armController.getPosition());
                telemetry.update();
            }

            throw new InterruptedException();
        }
        catch (InterruptedException e){
            robot.killSwitch();
        }

    }
}
