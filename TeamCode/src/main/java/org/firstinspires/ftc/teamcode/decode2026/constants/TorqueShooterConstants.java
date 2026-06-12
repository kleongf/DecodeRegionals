package org.firstinspires.ftc.teamcode.decode2026.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class TorqueShooterConstants {
    public static double nominalVoltage = 12.0;
    public static double cachingThreshold = 0.005;
    public static double kV = 0.000005; // viscous friction or smth
    public static double kS = 0.01; // static friction
    public static double kP = 0.002;
    public static double kA = -0.00005;
    public static boolean useVoltageCompensation = true;
    public static double PITCH_I = Math.toRadians(26);
    public static double PITCH_F = Math.toRadians(53);
    public static double PITCH_SERVO_MIN = 0.23;
    public static double PITCH_SERVO_I = 0.23;
    public static double PITCH_SERVO_F = 0.97;
    public static double LATCH_CLOSED = 0.48;//0.56;
    public static double LATCH_OPEN = 0.63;
}
