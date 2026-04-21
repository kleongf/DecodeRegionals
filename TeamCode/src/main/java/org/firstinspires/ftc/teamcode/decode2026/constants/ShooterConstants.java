package org.firstinspires.ftc.teamcode.decode2026.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterConstants {
    public static double nominalVoltage = 12.4;
    public static double cachingThreshold = 0.005;
    public static double kV = 0.00036;
    public static double kS = 0.0;
    public static double kP = 0.002;
    public static double kA = 0;
    public static boolean useVoltageCompensation = true;
    public static double PITCH_I = Math.toRadians(27);
    public static double PITCH_F = Math.toRadians(50);
    public static double PITCH_SERVO_MIN = 0.17;
    public static double PITCH_SERVO_I = 0.17;
    public static double PITCH_SERVO_F = 0.83;
    public static double LATCH_CLOSED = 0.5;//0.56;
    public static double LATCH_OPEN = 0.7;
}
