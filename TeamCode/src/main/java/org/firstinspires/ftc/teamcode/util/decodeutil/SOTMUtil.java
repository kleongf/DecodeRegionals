package org.firstinspires.ftc.teamcode.util.decodeutil;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import org.firstinspires.ftc.teamcode.decode2026.constants.ShootingConstants;

public class SOTMUtil {
    private final Pose goal;
    private final LUT thetaLUT;
    private final LUT velocityLUT;
    private final LUT tofLUT;
    public SOTMUtil(Pose goal) {
        this.goal = goal;
        thetaLUT = ShootingConstants.hoodAngleLUT;
        velocityLUT = ShootingConstants.wheelSpeedLUT;
        tofLUT = ShootingConstants.tofLUT;
    }
    private double sampleRate(
            LUT map, double distance, double distanceVelocity) {
        return (map.getValue(distance + distanceVelocity * ShootingConstants.SAMPLING_DT)
                - map.getValue(distance))
                / ShootingConstants.SAMPLING_DT;
    }

    public ShootingConstants.ShooterOutputs calculateShooterOutputs(Pose robotPose, Vector robotVelocity, double angularVelocity, double dt) {
        double tof = ShootingConstants.calculateTOF(tofLUT, robotVelocity, robotPose, goal);

        Pose virtualGoal = new Pose(goal.getX()-robotVelocity.getXComponent()*tof, goal.getY()-robotVelocity.getYComponent()*tof);

        double dx = goal.getX() - robotPose.getX();
        double dy = goal.getY() - robotPose.getY();
        double dist = Math.hypot(dx, dy);

        double dxVirtual = virtualGoal.getX() - robotPose.getX();
        double dyVirtual = virtualGoal.getY() - robotPose.getY();
        double distVirtual = Math.hypot(dxVirtual, dyVirtual);

        double turretAngle = Math.atan2(-dx, dy) - robotPose.getHeading() + Math.toRadians(90);
        double futureTurretAngle = Math.atan2(-dxVirtual, dyVirtual) - robotPose.getHeading() + Math.toRadians(90);

        double theta = thetaLUT.getValue(distVirtual);
        double velocity = velocityLUT.getValue(distVirtual);
        // distanceVelocity: change in goal distance over time.

        double wantedWheelAcceleration = sampleRate(velocityLUT, distVirtual, (distVirtual-dist) / dt);
        // angular velocity is subtracted, but i'll take it away for testing to see if the direction is correct
        double wantedTurretVelocity = MathUtil.getSmallestAngleDifference(turretAngle, futureTurretAngle) / dt - angularVelocity;

        return new ShootingConstants.ShooterOutputs(futureTurretAngle, wantedTurretVelocity, velocity, wantedWheelAcceleration, theta);
    }
}
