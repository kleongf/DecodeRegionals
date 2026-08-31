package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

public class MathUtil {
    // unit conversions
    public static double radiansToDegrees(double radians) {
        return Math.toDegrees(radians);
    }
    public static double degreesToRadians(double degrees) {
        return Math.toRadians(degrees);
    }
    public static double inToMM(double in) {
        return in * 25.4;
    }
    public static double inToM(double in) {
        return inToMM(in) / 1000d;
    }

    public static double mmToIn(double mm) {
        return mm / 25.4;
    }
    // general math utilities
    public static double getSign(double get) {
        if (get == (double)0.0F) {
            return (double)0.0F;
        } else {
            return get > (double)0.0F ? (double)1.0F : (double)-1.0F;
        }
    }

    public static double clamp(double num, double lower, double upper) {
        if (num < lower) {
            return lower;
        } else {
            return num > upper ? upper : num;
        }
    }

    public static double lerp(double start, double end, double t) {
        return start + (end - start) * MathUtil.clamp(t, 0, 1);
    }

    public static double inverseLerp(double start, double end, double val) {
        double range = end - start;
        if (range <= 0) {
            return 0.0;
        }

        return val - start <= 0 ? 0 : (val-start) / range;
    }

    public static double normalizeAngle(double angleRadians) {
        double angle = angleRadians % (Math.PI * 2D);
        return angle < (double)0.0F ? angle + (Math.PI * 2D) : angle;
    }

    public static boolean epsilonEquals(double val, double expected, double epsilon) {
        return Math.abs(val - expected) <= epsilon;
    }

    public static double inputModulus(double input, double minimumInput, double maximumInput) {
        double modulus = maximumInput - minimumInput;
        // Wrap input if it's above the maximum input
        int numMax = (int) ((input - minimumInput) / modulus);
        input -= numMax * modulus;

        // Wrap input if it's below the minimum input
        int numMin = (int) ((input - maximumInput) / modulus);
        input -= numMin * modulus;

        return input;
    }

    public static double angleModulus(double input) {
        return inputModulus(input, 0, 2 * Math.PI);
    }
    public static double angleModulusSigned(double input) {
        return inputModulus(input, -Math.PI, Math.PI);
    }
    public static double getSmallestAngleDifference(double one, double two) {
        return Math.min(angleModulus(one - two), angleModulus(two - one));
    }

    // Pose and vector utilities
    public static double distance(Pose one, Pose two) {
        return Math.sqrt(Math.pow(one.getX() - two.getX(), (double)2.0F) + Math.pow(one.getY() - two.getY(), (double)2.0F));
    }

    public static Pose addPoses(Pose one, Pose two) {
        return new Pose(one.getX() + two.getX(), one.getY() + two.getY(), one.getHeading() + two.getHeading());
    }

    public static Pose subtractPoses(Pose one, Pose two) {
        return new Pose(one.getX() - two.getX(), one.getY() - two.getY(), one.getHeading() - two.getHeading());
    }

    public static Pose rotatePose(Pose pose, double theta, boolean rotateHeading) {
        double x = pose.getX() * Math.cos(theta) - pose.getY() * Math.sin(theta);
        double y = pose.getX() * Math.sin(theta) + pose.getY() * Math.cos(theta);
        double heading = rotateHeading ? normalizeAngle(pose.getHeading() + theta) : pose.getHeading();
        return new Pose(x, y, heading);
    }

    public static Vector copyVector(Vector vector) {
        return new Vector(vector.getMagnitude(), vector.getTheta());
    }

    public static Vector scalarMultiplyVector(Vector vector, double scalar) {
        return new Vector(vector.getMagnitude() * scalar, vector.getTheta());
    }

    public static Vector normalizeVector(Vector vector) {
        return vector.getMagnitude() == (double)0.0F ? new Vector((double)0.0F, vector.getTheta()) : new Vector(vector.getMagnitude() / Math.abs(vector.getMagnitude()), vector.getTheta());
    }

    public static Vector addVectors(Vector one, Vector two) {
        Vector returnVector = new Vector();
        returnVector.setOrthogonalComponents(one.getXComponent() + two.getXComponent(), one.getYComponent() + two.getYComponent());
        return returnVector;
    }

    public static Vector subtractVectors(Vector one, Vector two) {
        Vector returnVector = new Vector();
        returnVector.setOrthogonalComponents(one.getXComponent() - two.getXComponent(), one.getYComponent() - two.getYComponent());
        return returnVector;
    }

    public static double dotProduct(Vector one, Vector two) {
        return one.getXComponent() * two.getXComponent() + one.getYComponent() * two.getYComponent();
    }

    public static double crossProduct(Vector one, Vector two) {
        return one.getXComponent() * two.getYComponent() - one.getYComponent() * two.getXComponent();
    }

    public static Vector getVector(Pose p) {
        return new Vector(p.getX(), p.getY());
    }
}
