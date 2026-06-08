package org.firstinspires.ftc.teamcode.util.decodeutil;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import org.firstinspires.ftc.teamcode.decode2026.constants.ShootingConstants;
import org.firstinspires.ftc.teamcode.decode2026.subsystems.Turret;

public class SOTMUtil {
    private final Pose goal;
    private final LUT thetaLUT;
    private final LUT velocityLUT;
    public SOTMUtil(Pose goal) {
        this.goal = goal;
        thetaLUT = ShootingConstants.hoodAngleLUT;
        velocityLUT = ShootingConstants.wheelSpeedLUT;
    }
    private double sampleRate(
            LUT map, double distance, double distanceVelocity) {
        return (map.getValue(distance + distanceVelocity * ShootingConstants.SAMPLING_DT)
                - map.getValue(distance))
                / ShootingConstants.SAMPLING_DT;
    }

    public ShootingConstants.ShooterOutputs calculateShooterOutputs(Pose turretPose, Vector currentSpeeds, Vector robotAcceleration, double angularVelocity, double dt) {
        double tof = ShootingConstants.calculateTOF(ShootingConstants.tofFunction, turretPose, goal, currentSpeeds) * ShootingConstants.tofMultiplier;
        // future stuff is used for feedforward
        Vector futureSpeeds = new Vector(currentSpeeds.getXComponent() + robotAcceleration.getXComponent() * dt, currentSpeeds.getYComponent() + robotAcceleration.getYComponent() * dt);

        Pose virtualGoal = new Pose(goal.getX()-currentSpeeds.getXComponent()*tof, goal.getY()-currentSpeeds.getYComponent()*tof);
        Pose futureVirtualGoal = new Pose(goal.getX() - futureSpeeds.getXComponent() * tof, goal.getY() - futureSpeeds.getYComponent() * tof);

        Pose futureTurretPose = new Pose(turretPose.getX() + currentSpeeds.getXComponent() * dt, turretPose.getY() + currentSpeeds.getYComponent() * dt, turretPose.getHeading() + angularVelocity * dt);

        double distance = virtualGoal.distanceFrom(turretPose);
        double futureDistance = futureVirtualGoal.distanceFrom(futureTurretPose);

        double turretAngle = Math.atan2(-(virtualGoal.getX() - turretPose.getX()), virtualGoal.getY() - turretPose.getY()) - turretPose.getHeading() + Math.toRadians(90);
        double futureTurretAngle = Math.atan2(-(futureVirtualGoal.getX() - futureTurretPose.getX()), futureVirtualGoal.getY() - futureTurretPose.getY()) - futureTurretPose.getHeading() + Math.toRadians(90);

        double wantedHoodAngle = thetaLUT.getValue(distance);
        double wantedWheelSpeed = velocityLUT.getValue(distance) * ShootingConstants.wheelSpeedMultiplier;

        double wantedWheelAcceleration = sampleRate(velocityLUT, distance, (futureDistance - distance) / dt);

        double wantedTurretVelocity = MathUtil.getSmallestAngleDifferenceSigned(futureTurretAngle, turretAngle) / dt;

        return new ShootingConstants.ShooterOutputs(
                turretAngle,
                wantedTurretVelocity,
                wantedWheelSpeed,
                wantedWheelAcceleration,
                wantedHoodAngle
        );
    }

    public ShootingConstants.ShooterOutputs calculateShooterOutputs2(Pose turretPose, Vector currentSpeeds, Vector robotAcceleration, double angularVelocity, double dt) {
        double tof = ShootingConstants.calculateTOF(ShootingConstants.tofFunction, turretPose, goal, currentSpeeds) * ShootingConstants.tofMultiplier;

        Pose virtualGoal = new Pose(goal.getX()-currentSpeeds.getXComponent()*tof, goal.getY()-currentSpeeds.getYComponent()*tof);

        double dx = virtualGoal.getX() - turretPose.getX();
        double dy = virtualGoal.getY() - turretPose.getY();
        double r2 = dx * dx + dy * dy;
        double distance = Math.sqrt(r2);

        // Derivatives of dx and dy over time
        double dxDot = currentSpeeds.getXComponent();
        double dyDot = currentSpeeds.getYComponent();

        // d/dt of atan2(-dx, dy)
        double turretAngleRate = (dy * (-dxDot) - (-dx) * dyDot) / r2;

        // subtract robot angular velocity since turret angle is robot-relative
        double wantedTurretVelocity = turretAngleRate - angularVelocity;
        double wantedTurretAngle = Math.atan2(-(virtualGoal.getX() - turretPose.getX()), virtualGoal.getY() - turretPose.getY()) - turretPose.getHeading() + Math.toRadians(90);

        // take derivative with respect to time of distance (sqrt(dx^2 + dy^2))
        double distanceRate = -(dx * dxDot + dy * dyDot) / distance;

        double wantedHoodAngle = thetaLUT.getValue(distance);
        double wantedWheelSpeed = velocityLUT.getValue(distance) * ShootingConstants.wheelSpeedMultiplier;
        double wantedWheelAcceleration = sampleRate(velocityLUT, distance, distanceRate);

        return new ShootingConstants.ShooterOutputs(
                wantedTurretAngle,
                wantedTurretVelocity,
                wantedWheelSpeed,
                wantedWheelAcceleration,
                wantedHoodAngle
        );
    }
}
