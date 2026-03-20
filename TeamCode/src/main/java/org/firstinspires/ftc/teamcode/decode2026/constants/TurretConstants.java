package org.firstinspires.ftc.teamcode.decode2026.constants;
public class TurretConstants {
    public static final double cachingThreshold = 0.005;
    public static final double ticksPerRevolution = 1381d;
    public static final double ticksPerRadian = ticksPerRevolution / (2 * Math.PI);
    public static final double maxPower = 0.75;
    public static final double kP = 0.01;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kF = 0.0;
    public static final double kV = 0.03;
    public static final double kS = 0.03;
    public static final double epsilonTicks = 5;
    public static final double encoderGearRatio = 50/42d;
    public static final double encoderOffsetDegrees = -107.2;
    public static final double encoderMaxVoltage = 3.272;
    public static final double encoderMinVoltage = 0.02;
    public static final boolean useExternalEncoder = false;
    public static final double nominalVoltage = 12.4;
    public static final boolean useVoltageCompensation = true;
}
