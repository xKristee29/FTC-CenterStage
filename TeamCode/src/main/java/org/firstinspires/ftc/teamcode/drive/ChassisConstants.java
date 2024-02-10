package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class ChassisConstants {

    public static double TICKS_PER_REV = 288;
    public static double MAX_RPM = 500;
    public static double WHEEL_DIAM = 9.0; // cm
    public static double TRACK_WIDTH = 31.0; // cm

    public static double LIMIT_MS = 500;

    public static double alphaEncoder = 0.0025;

    public static double toleranceXY = 10.0;
    public static double toleranceR = 10.0;

    public static double targetRadius = 5.0;

    public static double targetR = 40;

    public static double targetT = 0.5;


    public static PIDFCoefficients DRIVE_PID = new PIDFCoefficients(0.015, 0, 0, 0);

    public static PIDFCoefficients HEADING_PID = new PIDFCoefficients(0.0025, 0.001, 0, 0);

    public static PIDFCoefficients STATIC_HEADING_PID = new PIDFCoefficients(0.006, 0.01, 0, 0);


}
