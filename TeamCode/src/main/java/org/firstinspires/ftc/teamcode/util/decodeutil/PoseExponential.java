package org.firstinspires.ftc.teamcode.util.decodeutil;

import com.pedropathing.geometry.Pose;

public class PoseExponential {
    // public static
    public static Pose calculatePose(Pose prevPose, Pose currentPose) {
        double sinH = Math.sin(currentPose.getHeading());
        double cosH = Math.cos(currentPose.getHeading());

        double dTheta = MathUtil.angleWrap(currentPose.getHeading() - prevPose.getHeading());
        double dxField = currentPose.getX() - prevPose.getX();
        double dyField = currentPose.getY() - prevPose.getY();

        // this is a bit different because our rotation is a bit different
        // this matrix rotates from global to robot coordinate frames.
        // this is really just here for visualization
        // also its inverse is its transpose, which is what i am integrating
        Matrix rotationMatrix = new Matrix(new double[][]{
                {sinH, -cosH, 0},
                {cosH,  sinH, 0},
                {0,     0,    1},
        });

        // the integrals from zero to t of the rotation matrix dt
        // however this needs to be transposed, as we are going from field to robot

//        Matrix updateMatrix = new Matrix(new double[][]{
//                {(1-cosH)/dTheta, -sinH/dTheta, 0},
//                {sinH/dTheta,  (1-cosH)/dTheta, 0},
//                {0,     0,    1},
//        });

        // therefore this matrix should be correct

        Matrix updateMatrix = new Matrix(new double[][]{
                {taylorSeriesOneMinusCosThetaOverTheta(dTheta), taylorSeriesSinThetaOverTheta(dTheta), 0},
                {-taylorSeriesSinThetaOverTheta(dTheta),  taylorSeriesOneMinusCosThetaOverTheta(dTheta), 0},
                {0,     0,    1},
        });

        // there is a case where the change in angle is zero:
        // in this case we must use the taylor series approximation

        // in a way this is really stupid, since i am multiplying by the inverse of the rotation matrix
        // i rotate velocity from global to robot back to global (because i multiply back from robot to global)
        // so i made it like this, which is mathematically equivalent

        Matrix robotVelocityFieldRelative = new Matrix(new double[][]{
                {dxField},
                {dyField},
                {dTheta}
        });

        Matrix updatedPosition = updateMatrix.multiply(robotVelocityFieldRelative);

        Pose deltaPose = new Pose(updatedPosition.get(0, 0), updatedPosition.get(0, 1), updatedPosition.get(0, 2));
        Pose updatedPose = MathUtil.addPoses(currentPose, deltaPose);

        return updatedPose;
    }

    public static double taylorSeriesSinThetaOverTheta(double theta) {
        // 2nd order taylor series approximation
        return 1 - theta * theta / 6d;
    }

    public static double taylorSeriesOneMinusCosThetaOverTheta(double theta) {
        return theta / 2d;
    }
    // for theta let's say small is 0.01 radians
    public static boolean isSmall(double x, double compareTo) {
        return Math.abs(x) <= compareTo;
    }
}
