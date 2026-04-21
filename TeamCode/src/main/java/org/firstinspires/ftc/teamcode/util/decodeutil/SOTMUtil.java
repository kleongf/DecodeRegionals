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

    public ShootingConstants.ShooterOutputs calculateShooterOutputs(Pose robotPose, Vector robotVelocity, Vector robotAcceleration, double angularVelocity, double dt) {
        double tof = ShootingConstants.calculateTOF(tofLUT, robotPose, goal, robotVelocity) * ShootingConstants.tofMultiplier;

        Vector currentSpeeds = robotVelocity;
        Vector futureSpeeds = new Vector(currentSpeeds.getXComponent() + robotAcceleration.getXComponent() * dt, currentSpeeds.getYComponent() + robotAcceleration.getYComponent() * dt);

        Pose virtualGoal = new Pose(goal.getX()-currentSpeeds.getXComponent()*tof, goal.getY()-currentSpeeds.getYComponent()*tof);
        Pose futureVirtualGoal = new Pose(goal.getX() - futureSpeeds.getXComponent() * tof, goal.getY() - futureSpeeds.getYComponent() * tof);

        Pose turretPose = robotPose;
        Pose futureTurretPose = new Pose(turretPose.getX() + currentSpeeds.getXComponent() * dt, turretPose.getY() + currentSpeeds.getYComponent() * dt, turretPose.getHeading() + angularVelocity * dt);

        double distance = virtualGoal.distanceFrom(turretPose);
        double futureDistance = futureVirtualGoal.distanceFrom(futureTurretPose);

        double turretAngle = Math.atan2(-(virtualGoal.getX() - turretPose.getX()), virtualGoal.getY() - turretPose.getY()) - turretPose.getHeading() + Math.toRadians(90);
        double futureTurretAngle = Math.atan2(-(futureVirtualGoal.getX() - futureTurretPose.getX()), futureVirtualGoal.getY() - futureTurretPose.getY()) - futureTurretPose.getHeading() + Math.toRadians(90);

        double wantedHoodAngle = thetaLUT.getValue(distance);
        double wantedWheelSpeed = velocityLUT.getValue(distance);

        double wantedWheelAcceleration = sampleRate(velocityLUT, distance, (futureDistance - distance) / dt);

        double wantedTurretVelocity = MathUtil.getSmallestAngleDifference(turretAngle, futureTurretAngle) / dt;

        return new ShootingConstants.ShooterOutputs(
                turretAngle,
                wantedTurretVelocity,
                wantedWheelSpeed,
                wantedWheelAcceleration,
                wantedHoodAngle
        );
    }
}
