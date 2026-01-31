package org.firstinspires.ftc.teamcode.util.decodeutil;

import android.util.Log;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

public class SOTM {
    private Pose goal;
    private LUT thetaLUT;
    private LUT velocityLUT;
    private double radius = 0.036; // 36 mm radius, 72mm wheel
    public double timeScaleFactor = 1;
    public double offsetFactor = 0; // think i found it! after doing some algebra. should be 0.05?
    public double constantTimeFactor = 0;
    private double efficiency = 0.73;
    private double MAX_ITERATIONS = 200;
    private double dt = 0.01;
    private double g = 9.8;
    private double m = 0.07845;
    private double rho = 1.225;
    private double A = 0.01216604657;

    private double Cd = 1.6;
    private double c = 0.5 * Cd * rho * A;
    private double Cl = 0.1;
    private double radiusBall =0.0635;

    public SOTM(Pose goal) {
        this.goal = goal;

        thetaLUT = new LUT();
        thetaLUT.addData(163, Math.toRadians(49));
        thetaLUT.addData(158, Math.toRadians(49));
        thetaLUT.addData(153, Math.toRadians(49));
        thetaLUT.addData(148, Math.toRadians(49));
        thetaLUT.addData(143, Math.toRadians(49));
        thetaLUT.addData(138, Math.toRadians(49));
        thetaLUT.addData(133, Math.toRadians(49));
        thetaLUT.addData(128, Math.toRadians(49));
        thetaLUT.addData(123, Math.toRadians(49));
        thetaLUT.addData(118, Math.toRadians(48.5));
        thetaLUT.addData(113, Math.toRadians(47.5));
        thetaLUT.addData(108, Math.toRadians(47));
        thetaLUT.addData(103, Math.toRadians(46.5));
        thetaLUT.addData(98, Math.toRadians(46));
        thetaLUT.addData(93, Math.toRadians(45));
        thetaLUT.addData(88, Math.toRadians(44));
        thetaLUT.addData(83, Math.toRadians(43));
        thetaLUT.addData(78, Math.toRadians(41));
        thetaLUT.addData(73, Math.toRadians(38));
        thetaLUT.addData(68, Math.toRadians(36));
        thetaLUT.addData(63, Math.toRadians(35));
        thetaLUT.addData(58, Math.toRadians(34));
        thetaLUT.addData(53, Math.toRadians(34));

        velocityLUT = new LUT();
        velocityLUT.addData(163, 1490);
        velocityLUT.addData(158, 1460);
        velocityLUT.addData(153, 1440);
        velocityLUT.addData(148, 1420);
        velocityLUT.addData(143, 1390);
        velocityLUT.addData(138, 1360);
        velocityLUT.addData(133, 1330);
        velocityLUT.addData(128, 1300);
        velocityLUT.addData(123, 1270);
        velocityLUT.addData(118, 1240);
        velocityLUT.addData(108, 1220);
        velocityLUT.addData(103, 1180);
        velocityLUT.addData(98, 1170);
        velocityLUT.addData(93, 1150);
        velocityLUT.addData(88, 1130);
        velocityLUT.addData(83, 1110);
        velocityLUT.addData(78, 1060);
        velocityLUT.addData(73, 1010);
        velocityLUT.addData(68, 990);
        velocityLUT.addData(63, 980);
        velocityLUT.addData(58, 960);
        velocityLUT.addData(53, 960);

    }

    private double calculateLinearVelocityInches(double ticksPerSecond) {
        return (ticksPerSecond * 2 * Math.PI / 28.0) * radius * (39.3701);
    }

    private double calculateTicksPerSecond(double linearVelocityInches) {
        return linearVelocityInches / ((2 * Math.PI / 28.0) * radius * 39.3701);
    }

    private double calculateLinearVelocityMeters(double ticksPerSecond) {
        return (ticksPerSecond * 2 * Math.PI / 28.0) * radius;
    }

    // assumes units meters, meters/s
    private double calculateShooterVelocity(double dist, double vRadial) {
        double x = dist;
        double y = vRadial;
        return 1112.391 + -296.545*x + -18.056*y + 171.496*x*x + -54.964*x*y + 15.887*y*y + -17.141*x*x*x + 7.496*x*x*y + -4.187*x*y*y + 1.525*y*y*y;
    }

    // assumes units meters, meters/s
    private double calculateShooterAngle(double dist, double vRadial) {
        double x = dist;
        double y = vRadial;
        return 6.729 + 16.503*x + -10.538*y + -1.039*x*x + 0.842*x*y + 0.019*y*y + -0.125*x*x*x + 0.286*x*x*y + -0.337*x*y*y + 0.102*y*y*y;
    }

    public double[] calculateAzimuthThetaVelocity(Pose robotPose, Vector robotVelocity) {
        double dx = goal.getX() - robotPose.getX();
        double dy = goal.getY() - robotPose.getY();
        double dist = Math.hypot(dx, dy);

        boolean isBlue = goal.getX() == 0;

        Vector v = MathUtil.subtractVectors(MathUtil.getVector(goal), MathUtil.getVector(robotPose));
        Vector u = robotVelocity;

        // (u ⋅ v / |v|²) * v
        Vector projuv = MathUtil.scalarMultiplyVector(v, MathUtil.dotProduct(u, v) / MathUtil.dotProduct(v, v));

        // get the tangential component
        Vector vTangential = MathUtil.subtractVectors(u, projuv);

        // if the vectors are in the same direction, then we should subtract the radial velocity
        // vectors are in the same direction if their dot product is positive, so dot it with the goal vector.
        double vRadial = MathUtil.dotProduct(projuv, v) > 0 ? projuv.getMagnitude() : -projuv.getMagnitude();

        double velocity = calculateShooterVelocity(MathUtil.inToM(dist), MathUtil.inToM(vRadial));
        double theta = Math.toRadians(calculateShooterAngle(MathUtil.inToM(dist), MathUtil.inToM(vRadial)));

        double timestep = constantTimeFactor + timeScaleFactor * simulateProjectileTOF(MathUtil.inToM(vRadial), MathUtil.inToM(dist), theta, velocity);

        // could possibly be used for offset calculations.
        // double angleToGoal = Math.atan2(-(dx-vTangential.getXComponent()*timestep), (dy-vTangential.getYComponent()*timestep));

        double offset = isBlue ? offsetFactor : -offsetFactor;

        double azimuth = Math.atan2(-(dx - vTangential.getXComponent() * timestep), (dy - vTangential.getYComponent() * timestep)) - robotPose.getHeading() + Math.toRadians(90) + offset;

        return new double[]{azimuth, theta, velocity};
    }

    // returns the amount of time a projectile will take in air. inputs radial vel (m/s), dist (m), theta (rad), velocity (ticks/s)
    public double simulateProjectileTOF(double radialVelocity, double ticksPerSecond, double theta, double dist) {
        double x = 0;
        double y = 0.24384;
        double omega_wheel = 2 * Math.PI * ticksPerSecond / 28;
        double v = radius * omega_wheel;
        double v0 = v * efficiency;

        double vx = v0 * Math.sin(theta) + radialVelocity;
        double vy = v0 * Math.cos(theta);
        double omega = omega_wheel * (radius / radiusBall);
        double K = 0.5 * rho * A * radius * Cl * omega / m;

        for (int i = 0; i < MAX_ITERATIONS; i++) {
            double speed = Math.hypot(vx, vy);

            // ---- Drag acceleration ----
            double ax_drag = -(c * speed * vx) / m;
            double ay_drag = -(m * g + c * speed * vy) / m;

            // ---- Magnus acceleration ----
            double ax_mag = K * (-vy);
            double ay_mag = K * (vx);

            // ---- Total acceleration ----
            double ax = ax_drag + ax_mag;
            double ay = ay_drag + ay_mag;

            // Integrate
            vx = vx + ax * dt;
            vy = vy + ay * dt;

            x += vx * dt;
            y += vy * dt;

            if (x > dist) {
                return i / 100d;
            }
        }
        return 2.0;
    }

    private double[] calculateAzimuthThetaVelocity(Pose robotPose, Pose virtualGoal) {
        double dx = virtualGoal.getX() - robotPose.getX();
        double dy = virtualGoal.getY() - robotPose.getY();
        double dist = Math.hypot(dx, dy);

        boolean isBlue = goal.getX() == 0;
        double offset = isBlue ? offsetFactor : -offsetFactor;

        double azimuth = Math.atan2(-dx, dy) - robotPose.getHeading() + Math.toRadians(90) + offset;
        double theta = thetaLUT.getValue(dist);
        double velocity = velocityLUT.getValue(dist);
        return new double[] {azimuth, theta, velocity};
    }

    public double[] calculateAzimuthThetaVelocityFRC(Pose robotPose, Vector robotVelocity) {
        double dx = goal.getX() - robotPose.getX();
        double dy = goal.getY() - robotPose.getY();
        double dist = Math.hypot(dx, dy);

        double theta = thetaLUT.getValue(dist);
        double velocity = velocityLUT.getValue(dist);

        // not perfect. timestep is going to overshoot a bit but not tryna use newtons method or do it again. it should be good enough.
        double timestep = constantTimeFactor + timeScaleFactor * simulateProjectileTOF(0, velocity, theta, MathUtil.inToM(dist));
        Log.d("Timestep" + timestep, "timestep: " + timestep);
        Pose virtualGoal = new Pose(goal.getX()-robotVelocity.getXComponent()*timestep, goal.getY()-robotVelocity.getYComponent()*timestep);

        return calculateAzimuthThetaVelocity(robotPose, virtualGoal);
    }
}
