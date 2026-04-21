package org.firstinspires.ftc.teamcode.decode2026.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstants {
    public static boolean useShootOnTheMove = true;
    public static boolean useAutomaticRelocalization = false;
    public static boolean useAutomaticTurretRelocalization = true;
    public static boolean useAutomateRobotDrive = false;
    public static double turretResetTimeSeconds = 20.0;
    public static double robotRelocalizationTimeSeconds = 20.0;
    public static double farShootingDistanceThreshold = 120.0;
    public static int robotRadius = 8;
    public static double dt = 0.02;
}
