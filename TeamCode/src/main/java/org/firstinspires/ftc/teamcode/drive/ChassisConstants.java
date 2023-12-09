package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

@Config
public class ChassisConstants {

    public static double TICKS_PER_REV = 287;
    public static double MAX_RPM = 500;
    public static double WHEEL_DIAM = 9.0; // cm
    public static double TRACK_WIDTH = 31.0; // cm

    public static double alphaEncoder = 0.0025;

    public static double toleranceXY = 3.5;
    public static double toleranceR = 1.0;

    public static PIDCoefficients DRIVE_PID = new PIDCoefficients(0.01, 0, 0);

    public static PIDCoefficients HEADING_PID = new PIDCoefficients(0.0055, 0.05, 0.05);


}
