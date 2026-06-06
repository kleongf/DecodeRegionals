package org.firstinspires.ftc.teamcode.decode2026.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstants {
    public static boolean useShootOnTheMove = true;
    public static boolean useAutomaticRelocalization = false;
    public static boolean useAutomaticTurretRelocalization = false;
    public static boolean useAutomateRobotDrive = true;
    public static double turretResetTimeSeconds = 20.0;
    public static double robotRelocalizationTimeSeconds = 20.0;
    public static double farShootingDistanceThreshold = 120.0;
    public static double dt = 0.02;
    public static double autoShootWheelSpeedEpsilonTicks = 40;
    // 2% error threshold
    public static double autoShootTurretTicksEpsilon = TurretConstants.ticksPerRevolution * 0.02;
    public static double autoShootTurretRangeEpsilon = 100; // ticks, dont shoot when really close to wraparoud
    public static final double MIN_SHOOTING_DISTANCE = 45; // inches
    public static boolean useFarZoneAutoShoot = false;
}
