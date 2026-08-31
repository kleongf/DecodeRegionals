package org.firstinspires.ftc.teamcode.biobuzz.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class DrivetrainConstants {
    public static double MOVEMENT_SPEED_MULTIPLIER = 1.0;
    public static double TURN_SPEED_MULTIPLIER = 0.4;

    public static double xP = 0.04, xI = 0, xD = 0.001;
    public static double yP = 0.04, yI = 0, yD = 0.001;
    public static double headingP = 0.5, headingI = 0, headingD = 0.035;

    public static double LOCK_TOLERANCE_POSITION = 1.0; // inches
    public static double LOCK_TOLERANCE_HEADING = Math.toRadians(2); // radians
}
