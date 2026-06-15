package org.firstinspires.ftc.teamcode.util.decodeutil;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import java.util.ArrayList;
import java.util.function.Function;

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

    public static double normalizeAngleSigned(double angleRadians) {
        double angle = angleRadians % (Math.PI * 2D);
        if (angle > Math.PI)  angle -= Math.PI * 2D;
        if (angle <= -Math.PI) angle += Math.PI * 2D;
        return angle;
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
    // this one bad
    public static double getSmallestAngleDifference(double one, double two) {
        return Math.min(normalizeAngle(one - two), normalizeAngle(two - one));
    }

    public static double getSmallestAngleDifferenceSigned(double one, double two) {
        double t1 = normalizeAngleSigned(one - two);
        double t2 = normalizeAngleSigned(two - one);
        return Math.abs(t1) < Math.abs(t2) ? t1 : t2;
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

    public static boolean roughlyEquals(double num1, double num2, double epsilon) {
        return Math.abs(num1-num2) < epsilon;
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

    // TODO: used for relocalization when multiple tags are seen
    public static Pose weightedAveragePoses(ArrayList<Pose> poses, ArrayList<Double> weights) {
        double sumX = 0;
        double sumY = 0;
        double sumWeights = 0;
        double sumXHeadingVectors = 0;
        double sumYHeadingVectors = 0;

        for (int i = 0; i < poses.size(); i++) {
            sumX += poses.get(i).getX();
            sumY += poses.get(i).getY();
            sumWeights += weights.get(i);
            sumXHeadingVectors += weights.get(i) * Math.cos(poses.get(i).getHeading());
            sumYHeadingVectors += weights.get(i) * Math.sin(poses.get(i).getHeading());
        }

        double avgX = sumX / sumWeights;
        double avgY = sumY / sumWeights;
        double avgHeading = Math.atan2(sumYHeadingVectors, sumXHeadingVectors);

        return new Pose(avgX, avgY, avgHeading);
    }

    private static double[] solveLinearSystem(double[][] A, double[] B) {
        int n = B.length;

        // Build augmented matrix [A | B]
        double[][] aug = new double[n][n + 1];
        for (int i = 0; i < n; i++) {
            System.arraycopy(A[i], 0, aug[i], 0, n);
            aug[i][n] = B[i];
        }

        // Forward elimination with partial pivoting
        for (int col = 0; col < n; col++) {
            // Find pivot row
            int pivotRow = col;
            for (int row = col + 1; row < n; row++) {
                if (Math.abs(aug[row][col]) > Math.abs(aug[pivotRow][col])) {
                    pivotRow = row;
                }
            }

            // Swap rows
            double[] temp = aug[col];
            aug[col] = aug[pivotRow];
            aug[pivotRow] = temp;

            if (Math.abs(aug[col][col]) < 1e-10) {
                throw new IllegalArgumentException("Matrix is singular or nearly singular");
            }

            // Eliminate below
            for (int row = col + 1; row < n; row++) {
                double factor = aug[row][col] / aug[col][col];
                for (int j = col; j <= n; j++) {
                    aug[row][j] -= factor * aug[col][j];
                }
            }
        }

        // Back substitution
        double[] X = new double[n];
        for (int i = n - 1; i >= 0; i--) {
            X[i] = aug[i][n];
            for (int j = i + 1; j < n; j++) {
                X[i] -= aug[i][j] * X[j];
            }
            X[i] /= aug[i][i];
        }

        return X;
    }

    // Build the normal equations: (X^T * X) * coeffs = X^T * y
    public static double[] multipleLinearRegression(double[][] X, double[] y) {
        int m = X.length;    // number of observations
        int p = X[0].length; // number of predictors (already includes intercept column)

        // Compute X^T * X
        double[][] XtX = new double[p][p];
        for (int i = 0; i < p; i++) {
            for (int j = 0; j < p; j++) {
                for (int k = 0; k < m; k++) {
                    XtX[i][j] += X[k][i] * X[k][j];
                }
            }
        }

        // Compute X^T * y
        double[] Xty = new double[p];
        for (int i = 0; i < p; i++) {
            for (int k = 0; k < m; k++) {
                Xty[i] += X[k][i] * y[k];
            }
        }

        return solveLinearSystem(XtX, Xty);
    }

    public static double lerp(double start, double end, double t) {
        return start + (end - start) * MathUtil.clamp(t, 0, 1);
    }
}
