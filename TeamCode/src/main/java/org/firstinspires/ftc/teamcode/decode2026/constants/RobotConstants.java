package org.firstinspires.ftc.teamcode.decode2026.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstants {
    public static boolean useShootOnTheMove = true;
    public static boolean useAutomaticRelocalization = false;
    public static boolean useAutomaticTurretRelocalization = true;
    public static boolean useAutomateRobotDrive = true;
    public static double turretResetTimeSeconds = 20.0;
    public static double robotRelocalizationTimeSeconds = 20.0;
    public static double farShootingDistanceThreshold = 120.0;
    public static double dt = 0.02;
    public static double autoShootWheelSpeedEpsilonTicks = 30;
    public static double autoShootTurretAngleEpsilon = Math.toRadians(3);
}
