package org.firstinspires.ftc.teamcode.decode2026.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterConstants {
    public static double nominalVoltage = 12.0;
    public static double cachingThreshold = 0.005;
    public static double kV = 0.0003;
    public static double kS = 0.1174;
    public static double kP = 0.01;
    public static double kA = 0.0005;
    public static boolean useVoltageCompensation = true;
    public static double PITCH_I = Math.toRadians(31);
    public static double PITCH_F = Math.toRadians(61);
    public static double PITCH_SERVO_MIN = TorqueShooterConstants.PITCH_SERVO_MIN;
    public static double PITCH_SERVO_I = TorqueShooterConstants.PITCH_SERVO_I;
    public static double PITCH_SERVO_F = TorqueShooterConstants.PITCH_SERVO_F;
    public static double LATCH_CLOSED = 0.63;//0.56;
    public static double LATCH_OPEN = 0.46;
}
