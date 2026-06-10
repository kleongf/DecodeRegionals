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
    public static double wheelSpeedMultiplier = 0.98; // idk dont seem like we have enough power
    public static double tofMultiplier = 0.8;
    public static final LUT wheelSpeedLUT = new LUT();
    public static final LUT hoodAngleLUT = new LUT();
    public static final Function<Double, Double> tofFunction = x -> 0.0026 * x + 0.5134; // function of distance

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
        addData(151, Math.toRadians(53), 2040); // tof: 1.5
        addData(141, Math.toRadians(52), 1960);
        addData(131, Math.toRadians(50), 1900);
        addData(121, Math.toRadians(49), 1830);
        addData(111, Math.toRadians(47), 1770);
        addData(101, Math.toRadians(45), 1720);
        addData(91, Math.toRadians(44), 1650);
        addData(81, Math.toRadians(43), 1590);
        addData(71, Math.toRadians(41), 1530);
        addData(61, Math.toRadians(41), 1470);
        addData(51, Math.toRadians(39), 1400);
        addData(41, Math.toRadians(36), 1340);
        addData(31, Math.toRadians(32), 1270);
        addData(26, Math.toRadians(26), 1200);
    }
}
