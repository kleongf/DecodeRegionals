package org.firstinspires.ftc.teamcode.util.decodeutil;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import org.firstinspires.ftc.teamcode.decode2026.constants.ShootingConstants;

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
    public ShootingConstants.ShooterOutputs calculateShooterOutputs(Pose turretPose, Vector currentSpeeds, Vector robotAcceleration, double angularVelocity, double dt, Alliance alliance) {
        double tof = ShootingConstants.calculateTOF(ShootingConstants.tofFunctionTele, turretPose, goal, currentSpeeds) * ShootingConstants.tofMultiplier;

        Pose virtualGoal = new Pose(goal.getX()-currentSpeeds.getXComponent()*tof, goal.getY()-currentSpeeds.getYComponent()*tof);

        double dx = virtualGoal.getX() - turretPose.getX();
        double dy = virtualGoal.getY() - turretPose.getY();
        double r2 = dx * dx + dy * dy;
        double distance = Math.sqrt(r2);

        // Derivatives of dx and dy over time
        double dxDot = currentSpeeds.getXComponent();
        double dyDot = currentSpeeds.getYComponent();

        // d/dt of atan2(-dx, dy)
        double turretAngleRate = -(dy * (-dxDot) - (-dx) * dyDot) / r2;

        // subtract robot angular velocity since turret angle is robot-relative
        double wantedTurretVelocity = turretAngleRate - angularVelocity;
        // todo: red is opposite
        double turretOffset = alliance == Alliance.BLUE ? ShootingConstants.blueOffsetLUT.getValue(distance) : ShootingConstants.redOffsetLUT.getValue(distance);
        // MathUtil.lerp(Math.toRadians(0), Math.toRadians(-3.5), (distance - 70) / 160);
        double wantedTurretAngle = Math.atan2(-(virtualGoal.getX() - turretPose.getX()), virtualGoal.getY() - turretPose.getY()) - turretPose.getHeading() + Math.toRadians(90) + turretOffset;

        // take derivative with respect to time of distance (sqrt(dx^2 + dy^2))
        // double distanceRate = -(dx * dxDot + dy * dyDot) / distance;
        double distanceRate = (dx * dxDot + dy * dyDot) / distance;

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

    public ShootingConstants.ShooterOutputs calculateShooterOutputsTuning(Pose turretPose, Vector currentSpeeds, Vector robotAcceleration, double angularVelocity, double dt, Alliance alliance) {
        double tof = ShootingConstants.calculateTOF(ShootingConstants.tofFunctionTele, turretPose, goal, currentSpeeds) * ShootingConstants.tofMultiplier;

        Pose virtualGoal = new Pose(goal.getX()-currentSpeeds.getXComponent()*tof, goal.getY()-currentSpeeds.getYComponent()*tof);

        double dx = virtualGoal.getX() - turretPose.getX();
        double dy = virtualGoal.getY() - turretPose.getY();
        double r2 = dx * dx + dy * dy;
        double distance = Math.sqrt(r2);

        // Derivatives of dx and dy over time
        double dxDot = currentSpeeds.getXComponent();
        double dyDot = currentSpeeds.getYComponent();

        // d/dt of atan2(-dx, dy)
        double turretAngleRate = -(dy * (-dxDot) - (-dx) * dyDot) / r2;

        // subtract robot angular velocity since turret angle is robot-relative
        double wantedTurretVelocity = turretAngleRate - angularVelocity;
        // todo: red is opposite
        double turretOffset = 0;
        double wantedTurretAngle = Math.atan2(-(virtualGoal.getX() - turretPose.getX()), virtualGoal.getY() - turretPose.getY()) - turretPose.getHeading() + Math.toRadians(90) + turretOffset;

        // take derivative with respect to time of distance (sqrt(dx^2 + dy^2))
        // double distanceRate = -(dx * dxDot + dy * dyDot) / distance;
        double distanceRate = (dx * dxDot + dy * dyDot) / distance;

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
