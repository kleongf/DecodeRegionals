package org.firstinspires.ftc.teamcode.util.decodeutil;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PoseFollower {
    // i am lazy and this is for proof of concept only so no d component. yes its gonna overshoot but idgaf
    private Pose goalPose;
    private double kPDrive = 0.05;
    private double kPStrafe = 0.1;
    private double kPHeading = 1.5;
    private double kDDrive;
    private double kDStrafe;
    private double kDHeading;
    private double period;
    private double prevDriveError;
    private double prevStrafeError;
    private double prevHeadingError;
    private ElapsedTime updateTimer;
    private double MAX_DISTANCE_DELTA = 5.0;
    private double TIME_DELTA = 0.1;

    public PoseFollower() {
        goalPose = new Pose();
        updateTimer = new ElapsedTime();
    }
    // maybe this bad

    public void setGoal(Pose p) {
        goalPose = p;
    }
    // returns x, y, and heading values from the pid (robot-centric)
    public double[] calculate(Pose currentPose, Pose newPose) {
        // if it has been enough time and the new pose is close to the old pose. not tryna drive to a completely different one.
        // right now i am not including time so there may be more noise, but it should be alright
        if (MathUtil.distance(newPose, goalPose) < MAX_DISTANCE_DELTA) {
            goalPose = newPose;
        }
        double dy = goalPose.getY()-currentPose.getY();
        double dx = goalPose.getX()-currentPose.getX();
        double dTheta = MathUtil.normalizeAngle(goalPose.getHeading()-currentPose.getHeading());

        // Convert GLOBAL outputs to ROBOT-LOCAL frame using current heading
        double cosH = Math.cos(currentPose.getHeading());
        double sinH = Math.sin(currentPose.getHeading());

        double xPower =  (sinH * dx  -  cosH * dy) * kPDrive;
        double yPower =  (cosH * dx  +  sinH * dy) * kPStrafe;
        double thetaPower = kPHeading * dTheta; // rotation is already body-centric sign

        // normalize powers
        double total = Math.abs(xPower) + Math.abs(yPower) + Math.abs(thetaPower);
        if (total > 1) {
            xPower /= total;
            yPower /= total;
            thetaPower /= total;
        }

        return new double[] {xPower, yPower, -thetaPower};
    }
}
