package org.firstinspires.ftc.teamcode.util.decodeutil;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

public class MathUtil {
    public static double nCr(int n, int r) {
        double num = (double)1.0F;
        double denom = (double)1.0F;

        for(int i = n; i > n - r; --i) {
            num *= (double)i;
        }

        for(int i = 1; i <= r; ++i) {
            denom *= (double)i;
        }

        return num / denom;
    }

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

    public static double normalizeAngle(double angleRadians) {
        double angle = angleRadians % (Math.PI * 2D);
        return angle < (double)0.0F ? angle + (Math.PI * 2D) : angle;
    }

    public static double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }
        return radians;
    }

    public static double getSmallestAngleDifference(double one, double two) {
        return Math.min(normalizeAngle(one - two), normalizeAngle(two - one));
    }

    public static double getTurnDirection(double startHeading, double endHeading) {
        return normalizeAngle(endHeading - startHeading) >= (double)0.0F && normalizeAngle(endHeading - startHeading) <= Math.PI ? (double)1.0F : (double)-1.0F;
    }


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

    public static boolean roughlyEquals(double one, double two, double accuracy) {
        return one < two + accuracy && one > two - accuracy;
    }

    public static boolean roughlyEquals(double one, double two) {
        return roughlyEquals(one, two, 1.0E-4);
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
    public static Vector getVector(Pose p) {
        return new Vector(p.getX(), p.getY());
    }
}
