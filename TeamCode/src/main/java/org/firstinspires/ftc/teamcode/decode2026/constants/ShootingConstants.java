package org.firstinspires.ftc.teamcode.decode2026.constants;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import org.firstinspires.ftc.teamcode.util.decodeutil.LUT;

import java.util.function.Function;

@Config
public class ShootingConstants {
    public static class ShooterOutputs {
        public double turretAngle;
        public double turretFeedforward;
        public double wheelVelocity;
        public double wheelFeedforward;
        public final double hoodAngle;
        public ShooterOutputs(
                double turretAngle,
                double turretFeedforward,
                double wheelVelocity,
                double wheelFeedforward,
                double hoodAngle
        ) {
            this.turretAngle = turretAngle;
            this.turretFeedforward = turretFeedforward;
            this.wheelVelocity = wheelVelocity;
            this.wheelFeedforward = wheelFeedforward;
            this.hoodAngle = hoodAngle;
        }
    }
    public static final double TOF_ITERATIONS = 10;
    public static final double SAMPLING_DT = 0.001;
    public static final double DEFAULT_TOF = 1.0;
    public static double wheelSpeedMultiplier = 1.03; // idk dont seem like we have enough power
    public static double tofMultiplier = 0.8;
    public static final LUT wheelSpeedLUT = new LUT();
    public static final LUT hoodAngleLUT = new LUT();
    public static final Function<Double, Double> tofFunction = x -> 0.0031 * x + 0.46; // function of distance

    private static void addData(double distance, double hoodAngle, double wheelSpeed) {
        wheelSpeedLUT.addData(distance, wheelSpeed);
        hoodAngleLUT.addData(distance, hoodAngle);
    }

    public static double calculateTOF(
            Function<Double, Double> tofFunction,
            Pose robotPose,
            Pose targetPose,
            Vector robotVelocity
    ) {
        double runningX = robotPose.getX();
        double runningY = robotPose.getY();

        double tof = 0;

        for (int i = 0; i < TOF_ITERATIONS; i++) {
            // get distance
            double dx = targetPose.getX() - runningX;
            double dy = targetPose.getY() - runningY;
            double distance = Math.hypot(dx, dy);
            // get tof
            tof = tofFunction.apply(distance);
            // update running tof
            runningX = robotPose.getX() + robotVelocity.getXComponent() * tof;
            runningY = robotPose.getY() + robotVelocity.getYComponent() * tof;
        }

        return tof;
    }

    static {
        // TODO: tune TOF, put into a quadratic function for least squares, then put that into function
        addData(158, Math.toRadians(53), 2060+20); // tof: 1.5
        addData(148, Math.toRadians(53), 1980+20);
        addData(138, Math.toRadians(53), 1900+20);
        addData(128, Math.toRadians(53), 1840+20);
        addData(118, Math.toRadians(53), 1780+20);
        addData(108, Math.toRadians(51), 1720+10);
        addData(98, Math.toRadians(49), 1660+10);
        addData(88, Math.toRadians(46), 1590+10);
        addData(78, Math.toRadians(45), 1520+10);
        addData(68, Math.toRadians(43), 1460+10);
        addData(58, Math.toRadians(41), 1400+10); // 19 frames
        addData(48, Math.toRadians(39), 1340+10);
        addData(38, Math.toRadians(33), 1260+10); // 18 frames
        addData(32, Math.toRadians(26), 1230+10);
    }
}
