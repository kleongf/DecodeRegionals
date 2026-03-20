package org.firstinspires.ftc.teamcode.decode2026.constants;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.teamcode.util.decodeutil.LUT;

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
    public static final double TOF_MAX_ITERATIONS = 10;
    public static final double SAMPLING_DT = 0.001;
    public static final double DEFAULT_TOF = 1.0;
    public static final LUT wheelSpeedLUT = new LUT();
    public static final LUT hoodAngleLUT = new LUT();
    public static final LUT tofLUT = new LUT();

    private static void addData(double distance, double hoodAngle, double wheelSpeed, double tof) {
        wheelSpeedLUT.addData(distance, wheelSpeed);
        hoodAngleLUT.addData(distance, hoodAngle);
        tofLUT.addData(distance, tof);
    }

    public static double calculateTOF(LUT tofLUT, Vector robotVelocity, Pose robotPose, Pose goalPose) {
        Pose currentPose = robotPose;
        double currentTOF = DEFAULT_TOF;
        for (int i = 0; i < TOF_MAX_ITERATIONS; i++) {
            double distance = currentPose.distanceFrom(goalPose);
            currentTOF = tofLUT.getValue(distance);
            currentPose = new Pose(robotPose.getX() + robotVelocity.getXComponent() * currentTOF, robotPose.getY() + robotVelocity.getYComponent() * currentTOF);
        }
        return currentTOF;
    }



    static {
        // TODO: tune TOF
        addData(145, Math.toRadians(50), 2180, 1.5);
        addData(138, Math.toRadians(50), 2120, 1.42);
        // and so on and so forth
    }
}
