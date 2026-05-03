package org.firstinspires.ftc.teamcode.decode2026.constants;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import org.firstinspires.ftc.teamcode.util.decodeutil.LUT;

import java.util.function.Function;

@Config
public class ShootingConstants {
    public static class ShooterOutputs {
        public final double turretAngle;
        public final double turretFeedforward;
        public final double wheelVelocity;
        public final double wheelFeedforward;
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
    public static double tofMultiplier = 1.0;
    public static final LUT wheelSpeedLUT = new LUT();
    public static final LUT hoodAngleLUT = new LUT();
    public static final Function<Double, Double> tofFunction = x -> 1.0; // function of distance

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
        addData(145, Math.toRadians(50), 2180); // tof: 1.5
        addData(138, Math.toRadians(50), 2120);
        // and so on and so forth
    }
}
