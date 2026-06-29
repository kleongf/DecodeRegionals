package org.firstinspires.ftc.teamcode.decode2026.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class TurretConstants {
    public static double ticksPerRevolution = 1381d;
    public static double ticksPerRadian = ticksPerRevolution / (2 * Math.PI);
    public static double maxPower = 0.75;
    public static double kP = 0.008;
    public static double kI = 0.0000;
    public static double kD = 0.0002;
    public static double kF = 0.0;
    public static double kV = 0.11;
    public static double kS = 0.02;
    public static double beta = 1.0;
    public static double gamma = 0;
    public static double encoderGearRatio = 50/42d;
    public static double encoderOffsetDegrees = -94.5;
    public static double encoderMaxVoltage = 3.272;
    public static double encoderMinVoltage = 0.02;
    public static boolean useExternalEncoder = false;
    public static double nominalVoltage = 12.0;
    public static boolean useVoltageCompensation = true;
}
