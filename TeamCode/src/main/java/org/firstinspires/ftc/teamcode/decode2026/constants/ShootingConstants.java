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
    public static double wheelSpeedMultiplier = 1.00; // idk dont seem like we have enough power
    public static double tofMultiplier = 0.8; //0.9 was good from far, prob need to retune tof function or quadratic
    public static double teleTofMultiplier = 0.8;
    public static final LUT wheelSpeedLUT = new LUT();
    public static final LUT hoodAngleLUT = new LUT();
    public static final LUT offsetLUT = new LUT();
    public static final LUT blueOffsetLUT = new LUT();
    public static final LUT redOffsetLUT = new LUT();
    public static final Function<Double, Double> tofFunction = x -> 0.0031 * x + 0.46; // function of distance
    public static final Function<Double, Double> tofFunctionTele = x -> 0.0000205515 * x * x - 0.00134458 * x + 0.683101;

    private static void addData(double distance, double hoodAngle, double wheelSpeed, double offsetBlue) {
        wheelSpeedLUT.addData(distance, wheelSpeed);
        hoodAngleLUT.addData(distance, hoodAngle);
        offsetLUT.addData(distance, offsetBlue);
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
        addData(158, Math.toRadians(56-1), 2070+30+40, Math.toRadians(-1)); // tof: 1.5
        addData(148, Math.toRadians(56-1), 1980+30+40, Math.toRadians(-1));
        addData(138, Math.toRadians(56-1), 1900+30+40, Math.toRadians(-1));
        addData(128, Math.toRadians(55-1), 1860+30+40, Math.toRadians(-1));
        addData(118, Math.toRadians(53-1), 1800+30+40, Math.toRadians(-1));
        addData(108, Math.toRadians(52-1), 1740+30+40, Math.toRadians(-2));
        addData(98, Math.toRadians(50-2), 1700, Math.toRadians(-3));
        addData(88, Math.toRadians(46), 1600, Math.toRadians(-3));
        addData(78, Math.toRadians(43), 1500, Math.toRadians(-3));
        addData(68, Math.toRadians(41), 1420, Math.toRadians(-3));
        addData(58, Math.toRadians(39), 1360, Math.toRadians(-3)); // 19 frames
        addData(48, Math.toRadians(37), 1300, Math.toRadians(-3));
        addData(38, Math.toRadians(34), 1220, Math.toRadians(-3)); // 18 frames
        addData(28, Math.toRadians(31), 1140, Math.toRadians(-3));

        /*
        50, -1
        80, -2
        134, -3
        154, -3
         */
        blueOffsetLUT.addData(50, Math.toRadians(-3));
        // blueOffsetLUT.addData(80, Math.toRadians());
        blueOffsetLUT.addData(144, Math.toRadians(1));
        /*
        134: -1
        154: -1
        90: 0
        otherwise: 0
         */
        // need to tune red offset but i lazy
        // redOffsetLUT.addData(154, Math.toRadians(0));
        redOffsetLUT.addData(134, Math.toRadians(-1.5));
        redOffsetLUT.addData(50, Math.toRadians(0));
    }
}
