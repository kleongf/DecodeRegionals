package org.firstinspires.ftc.teamcode.util.decodeutil;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.decode2026.constants.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.decode2026.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.decode2026.opmode.teleop.MainTeleop;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.control.PIDFController;

import java.util.function.Supplier;
// TODO: make lockedX for gate cycles and automatic turn (not drive) to closest spot in zone tangentially in best direction
public class TeleopDrivetrain {
    public enum DrivetrainState {
        TELEOP_DRIVE,
        KICK,
        INTAKE_GATE,
        OPEN_GATE,
        PARK
    }
    private Supplier<PathChain> currentPathChain;

    private ElapsedTime kickTimer;
    public Follower follower;
    private double targetHeading = 0;
    private ElapsedTime elapsedTime;
    private DrivetrainState state;
    private boolean robotCentric = false;
    private double KICK_TIME = 1.0;
    public boolean gateHeadingLock = false;
    public boolean openGateHeadingLock = false;
    private final Alliance alliance;
    private Pose parkPose;
    private PIDFController headingPIDFController;
    private PIDFController yController;

    public TeleopDrivetrain(HardwareMap hardwareMap, Alliance alliance) {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, Math.toRadians(0)));
        follower.startTeleopDrive(true);
        follower.usePredictiveBraking = true;

        headingPIDFController = new PIDFController(new PIDFCoefficients(0.4, 0, 0.03, 0));
        yController = new PIDFController(new PIDFCoefficients(0.03, 0, 0, 0));

        elapsedTime = new ElapsedTime();
        kickTimer = new ElapsedTime();

        this.alliance = alliance;
        state = DrivetrainState.TELEOP_DRIVE;

        this.parkPose = alliance == Alliance.BLUE ? FieldConstants.BLUE_PARK_POSE :  FieldConstants.RED_PARK_POSE;

        currentPathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();
    }

    public void setStartingPose(Pose p) {
        follower.setStartingPose(p);
        targetHeading = p.getHeading();
    }

    public void setRobotCentric(boolean r) {
        robotCentric = r;
    }

    public boolean getRobotCentric() {
        return robotCentric;
    }

    public void breakFollowing() {
        follower.startTeleopDrive(true);
        state = DrivetrainState.TELEOP_DRIVE;
        targetHeading = follower.getHeading();
    }

    public Vector getVelocity() {
        return follower.getVelocity();
    }

    public Pose getPose() {
        return follower.getPose();
    }

    public double getAngularVelocity() {
        return follower.poseTracker.getAngularVelocity();
    }
    public Vector getAcceleration() {return follower.poseTracker.getAcceleration();}

    public void kick(Pose closestPose) {
        currentPathChain = () -> follower.pathBuilder()
                .addPath(
                        new Path(
                                new BezierLine(follower.getPose(), closestPose)
                        )
                )
                .setConstantHeadingInterpolation(follower.getPose().getHeading()).build();

        state = DrivetrainState.KICK;
        follower.followPath(currentPathChain.get(), true);
        kickTimer.reset();
    }

    public void park() {
        // now it doesn't really matter because we have full park
        currentPathChain = () -> follower.pathBuilder()
                .addPath(
                        new Path(
                                new BezierLine(
                                        follower.getPose(),
                                        parkPose
                                )
                        )
                )
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), parkPose.getHeading())
                .build();
        state = DrivetrainState.PARK;
        follower.followPath(currentPathChain.get(), true);
    }

    public boolean isBusy() {
        return state != DrivetrainState.TELEOP_DRIVE;
    }

    public String getState() {
        switch (state) {
            case TELEOP_DRIVE:
                return "TELEOP_DRIVE";
            case PARK:
                return "PARKING";
            case KICK:
                return "KICKING";
            case INTAKE_GATE:
                return "INTAKING_GATE";
        }
        return "TELEOP_DRIVE";
    }

    private double[] calculateDrivetrainPowers(double x, double y, double rx, double currentHeading) {
        if (gateHeadingLock) {
            // added 6 deg
            targetHeading = alliance == Alliance.BLUE ? FieldConstants.BLUE_GATE_AUTO_POSE.getHeading()+Math.toRadians(6) : FieldConstants.RED_GATE_AUTO_POSE.getHeading()-Math.toRadians(6);
            double headingError = MathFunctions.getTurnDirection(follower.getPose().getHeading(), targetHeading) * MathFunctions.getSmallestAngleDifference(follower.getPose().getHeading(), targetHeading);
            headingPIDFController.updateError(headingError);

            double lockedY = alliance == Alliance.BLUE ? FieldConstants.BLUE_GATE_AUTO_POSE.getY() : FieldConstants.RED_GATE_AUTO_POSE.getY();
            double yError = lockedY - follower.getPose().getY();
            yController.updateError(yError);

            double outX = x * DrivetrainConstants.xSpeed;
            double outY = yController.run();
            double outHeading = headingPIDFController.run();

            return new double[] {outX, outY, outHeading};
        } else if (openGateHeadingLock) {
            // find closest angle
            double[] alignAngles = {0, 0.5 * Math.PI, Math.PI, 1.5 * Math.PI};
            int bestAlignIndex = 0;
            double smallestDifference = Double.POSITIVE_INFINITY;
            for (int i = 0; i < alignAngles.length; i++) {
                double diff = Math.abs(MathFunctions.getSmallestAngleDifference(currentHeading, alignAngles[i]));
                if (diff < smallestDifference) {
                    bestAlignIndex = i;
                    smallestDifference = diff;
                }
            }

            targetHeading = alignAngles[bestAlignIndex];
            double headingError = MathFunctions.getTurnDirection(follower.getPose().getHeading(), targetHeading) * MathFunctions.getSmallestAngleDifference(follower.getPose().getHeading(), targetHeading);
            headingPIDFController.updateError(headingError);

            double lockedY = alliance == Alliance.BLUE ? FieldConstants.BLUE_SIDE_GATE_POSE.getY() : FieldConstants.RED_SIDE_GATE_POSE.getY();
            double yError = lockedY - follower.getPose().getY();
            yController.updateError(yError);

            double outHeading = headingPIDFController.run();
            double outY = yController.run();

            return new double[] {x * DrivetrainConstants.xSpeed, outY, outHeading};
        } else {
            targetHeading = currentHeading;
            return new double[] {x * DrivetrainConstants.xSpeed, y * DrivetrainConstants.ySpeed, rx * DrivetrainConstants.headingSpeed};
        }
    }

    public void update(double x, double y, double rx) {
        follower.update();

        switch (state) {
            case TELEOP_DRIVE:
                // todo: swap forward backward up down robot centric
                double[] powers = calculateDrivetrainPowers(x, y, rx, follower.getHeading());
                if (alliance == Alliance.BLUE) {

                    follower.setTeleOpDrive(powers[0], powers[1], powers[2], robotCentric, Math.toRadians(180));
                    // follower.setTeleOpDrive(powers[1], powers[0], powers[2], robotCentric);
                } else {
                    follower.setTeleOpDrive(powers[0], powers[1], powers[2], robotCentric);
                    // follower.setTeleOpDrive(powers[1], powers[0], powers[2], robotCentric, Math.toRadians(180));
                }
                break;
            case PARK:
                if (!follower.isBusy()) {
                    breakFollowing();
                }
                break;
            case KICK:
                if (!follower.isBusy() || kickTimer.seconds() > KICK_TIME) {
                    breakFollowing();
                }
                break;
            case INTAKE_GATE:
                if (!follower.isBusy()) { // follower is not busy even when holding point
                    breakFollowing();
                }
                break;
        }

        elapsedTime.reset();
    }
}
