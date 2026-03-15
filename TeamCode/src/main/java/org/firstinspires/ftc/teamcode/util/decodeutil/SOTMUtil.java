package org.firstinspires.ftc.teamcode.util.decodeutil;

import android.util.Log;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import java.util.function.Function;

public class SOTMUtil {
    private final Pose goal;
    private final LUT thetaLUT;
    private final LUT velocityLUT;
    private Function<Double, Double> distanceToTime;
    final double dx = 1e-3;
    public SOTMUtil(Pose goal) {
        this.goal = goal;

        thetaLUT = new LUT();
        thetaLUT.addData(168, Math.toRadians(50));
        thetaLUT.addData(158, Math.toRadians(50));
        thetaLUT.addData(145, Math.toRadians(50));
        thetaLUT.addData(138, Math.toRadians(50));
        thetaLUT.addData(130, Math.toRadians(50));
        thetaLUT.addData(118, Math.toRadians(48));
        thetaLUT.addData(108, Math.toRadians(48));
        thetaLUT.addData(98, Math.toRadians(48));
        thetaLUT.addData(88, Math.toRadians(48));
        thetaLUT.addData(78, Math.toRadians(47));
        thetaLUT.addData(68, Math.toRadians(45.5));
        thetaLUT.addData(63, Math.toRadians(43));
        thetaLUT.addData(58, Math.toRadians(41));
        thetaLUT.addData(53, Math.toRadians(39));
        thetaLUT.addData(48, Math.toRadians(35));
        thetaLUT.addData(43, Math.toRadians(27));

        velocityLUT = new LUT();
        velocityLUT.addData(168, 2200);
        velocityLUT.addData(158, 2100);
        velocityLUT.addData(152, 2080);
        velocityLUT.addData(145, 2000);
        velocityLUT.addData(138, 1960);
        velocityLUT.addData(128, 1900);
        velocityLUT.addData(118, 1850);
        velocityLUT.addData(108, 1760);
        velocityLUT.addData(98, 1700);
        velocityLUT.addData(88, 1640);
        velocityLUT.addData(78, 1580);
        velocityLUT.addData(68, 1520);
        velocityLUT.addData(63, 1480);
        velocityLUT.addData(58, 1440);
        velocityLUT.addData(53, 1400);
        velocityLUT.addData(48, 1340);
        velocityLUT.addData(43, 1300);

        // TODO: interpolate a function (most likely 2nd degree polynomial) for time of flight based on distance (in) to time (s)
        // this is just an estimate
        distanceToTime = aDouble -> aDouble / 100;
    }

    /**
     * Find the optimal time of flight given robot position and speeds and the goal position. This
     * also requires a mapping between distance to target and the time of flight in a static setting,
     * along with the necessary derivative.
     *
     * @param robotPose The location of the robot.
     * @param robotSpeeds The speed of the robot.
     * @param distanceToTime The static distance-to-time mapping.
     * @param distanceToTimePrime The static distance-to-time mapping's derivative.
     * @return The actual time of flight in a non-static setting.
     */

    private double findOptimalTOF(
            Pose robotPose,
            Vector robotSpeeds,
            Function<Double, Double> distanceToTime,
            Function<Double, Double> distanceToTimePrime) {
        final double rx = robotPose.getX(), ry = robotPose.getY();
        final double vx = robotSpeeds.getXComponent(), vy = robotSpeeds.getYComponent();
        final double gx = goal.getX(), gy = goal.getY();

        // d = √( (gx - vx*t(d) - rx)^2 + (gy - vy*t(d) - ry)^2 )

        // f(d) gives the effective distance based on the velocity.
        // f(d) = (gx - vx*t(d) - rx)^2 + (gy - vy*t(d) - ry)^2 - d^2 = 0
        // f'(d) = 2(gx - vx*t(d) - rx)(-vx*t'(d)) + 2(gy - vy*t(d) - ry)(-vy*t'(d)) - 2d

        final Function<Double, Double> t = distanceToTime;
        final Function<Double, Double> tPrime = distanceToTimePrime;
        final Function<Double, Double> f =
                (d) ->
                        Math.pow(gx - vx * t.apply(d) - rx, 2)
                                + Math.pow(gy - vy * t.apply(d) - ry, 2)
                                - Math.pow(d, 2);
        final Function<Double, Double> fPrime =
                (d) ->
                        2 * (gx - vx * t.apply(d) - rx) * (-vx * tPrime.apply(d))
                                + 2 * (gy - vy * t.apply(d) - ry) * (-vy * tPrime.apply(d))
                                - 2 * d;

        double distance =
                MathUtil.findRoot(f, fPrime, goal.distanceFrom(robotPose), 20, 1e-3);
        return distanceToTime.apply(distance);
    }

    public double findOptimalTOF(
            Pose robotPose,
            Vector robotSpeeds,
            Function<Double, Double> distanceToTime) {
        return findOptimalTOF(
                robotPose,
                robotSpeeds,
                distanceToTime,
                (d) -> (distanceToTime.apply(d + dx) - distanceToTime.apply(d)) / dx);
    }

    public double[] calculateAzimuthFeedforwardThetaVelocity(Pose robotPose, Vector robotVelocity, double angularVelocity, double dt) {
        double tof = findOptimalTOF(robotPose, robotVelocity, distanceToTime);

        Pose virtualGoal = new Pose(goal.getX()-robotVelocity.getXComponent()*tof, goal.getY()-robotVelocity.getYComponent()*tof);

        double dx = goal.getX() - robotPose.getX();
        double dy = goal.getY() - robotPose.getY();

        double dxVirtual = virtualGoal.getX() - robotPose.getX();
        double dyVirtual = virtualGoal.getY() - robotPose.getY();
        double distVirtual = Math.hypot(dxVirtual, dyVirtual);

        double turretAngle = Math.atan2(-dx, dy) - robotPose.getHeading() + Math.toRadians(90);
        double futureTurretAngle = Math.atan2(-dxVirtual, dyVirtual) - robotPose.getHeading() - angularVelocity * tof + Math.toRadians(90);

        double azimuth = Math.atan2(-dxVirtual, dyVirtual) - robotPose.getHeading() + Math.toRadians(90);
        double theta = thetaLUT.getValue(distVirtual);
        double velocity = velocityLUT.getValue(distVirtual);
        double feedforward = MathUtil.getSmallestAngleDifference(turretAngle, futureTurretAngle) / dt;


        return new double[] {azimuth, feedforward, theta, velocity};
    }
}
