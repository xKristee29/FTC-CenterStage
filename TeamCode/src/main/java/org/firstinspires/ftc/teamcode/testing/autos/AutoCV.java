package org.firstinspires.ftc.teamcode.testing.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.KodiCV;
import org.firstinspires.ftc.teamcode.testing.sanke.CenterStageCVDetection;

@Autonomous
public class AutoCV extends LinearOpMode {
    KodiCV cv;
    @Override
    public void runOpMode() throws InterruptedException {
        cv = new KodiCV(telemetry, hardwareMap);

        cv.start();

        waitForStart();

        cv.kill();

        //checking ROI
        switch (cv.detector.location) {
            case RIGHT:
                //right case handler
                break;
            case MIDDLE:
                //middle case handler
                break;
        }

    }
}