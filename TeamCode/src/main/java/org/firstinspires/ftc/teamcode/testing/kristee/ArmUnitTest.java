package org.firstinspires.ftc.teamcode.testing.kristee;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.ArmController;

@TeleOp
public class ArmUnitTest extends LinearOpMode {

    ArmController armController;

    GamepadEx gp1;

    public void initialize(){
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        armController = new ArmController(hardwareMap);

        gp1 = new GamepadEx(gamepad1);
    }


    @Override
    public void runOpMode() throws InterruptedException {

        try{
            initialize();

            waitForStart();

            while (!isStopRequested()){
                telemetry.addData("Pos", armController.getPosition());
                telemetry.update();
            }


            throw new InterruptedException();
        }
        catch(InterruptedException e){
            armController.interrupt();
        }

    }
}
