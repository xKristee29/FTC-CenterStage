package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {
    public TankDriveChassis drive;
    public TankDriveController driveController;

    public ArmController armController;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry){

        drive = new TankDriveChassis(hardwareMap);

        driveController = new TankDriveController(drive, telemetry);

        armController = new ArmController(hardwareMap);

    }

    public void init(){
        drive.init();
    }

    public void killSwitch(){
        driveController.interrupt();
        drive.killSwitch();
        armController.interrupt();
    }
}
