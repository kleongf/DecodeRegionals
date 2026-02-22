package org.firstinspires.ftc.teamcode.util.decodeutil;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.constants.PoseConstants;

import java.util.function.Supplier;

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
    private boolean gateHeadingLock = false;
    private Alliance alliance;
    private Pose gatePose;
    private Pose gateIntakePose;
    private Pose parkPose;
    private double kPHeading = 0.2;

    public TeleopDrivetrain(HardwareMap hardwareMap, Alliance alliance) {
        // how to heading lock:
        // run a pid on heading, but just a p loop
        // if rx == 0 (or about equals 0)
        // save targetHeading: targetHeading = currentHeading, error = target-current
        // if rx > 0
        // targetheading = currentheading


        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, Math.toRadians(0)));
        follower.startTeleopDrive(true);
        follower.usePredictiveBraking = true;

        elapsedTime = new ElapsedTime();
        kickTimer = new ElapsedTime();

        this.alliance = alliance;
        state = DrivetrainState.TELEOP_DRIVE;

        this.parkPose = alliance == Alliance.BLUE ? PoseConstants.BLUE_PARK_POSE :  PoseConstants.RED_PARK_POSE;
        this.gatePose = alliance == Alliance.BLUE ? PoseConstants.BLUE_GATE_POSE : PoseConstants.RED_GATE_POSE;
        this.gateIntakePose = alliance == Alliance.BLUE ? PoseConstants.BLUE_GATE_AUTO_POSE : PoseConstants.RED_GATE_AUTO_POSE;

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

    public void kick(boolean isClose, boolean isReversed, Pose closestPose) {
        if (isClose) {
            currentPathChain = () -> follower.pathBuilder()
                    .addPath(
                        new Path(
                                new BezierLine(follower.getPose(), closestPose)
                        )
                    )
                    .setConstantHeadingInterpolation(follower.getPose().getHeading()).build();
        } else {
            if (isReversed) {
                currentPathChain = () -> follower.pathBuilder()
                    .addPath(
                            new Path(
                                    new BezierLine(follower.getPose(), closestPose)
                            )
                    ).setTangentHeadingInterpolation().setReversed().build();
            } else {
                currentPathChain = () -> follower.pathBuilder()
                        .addPath(
                                new Path(
                                        new BezierLine(follower.getPose(), closestPose)
                                )
                        ).setTangentHeadingInterpolation().build();
            }
        }
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
//        if (follower.getPose().getY() < 32) {
//            currentPathChain = () -> follower.pathBuilder()
//                    .addPath(
//                            new Path(
//                                    new BezierLine(
//                                            follower.getPose(),
//                                            parkPose
//                                    )
//                            )
//                    )
//                    .setLinearHeadingInterpolation(follower.getPose().getHeading(), parkPose.getHeading())
//                    .build();
//        } else { // y coord is high so go on top
//            currentPathChain = () -> follower.pathBuilder()
//                    .addPath(
//                            new Path(
//                                    new BezierLine(
//                                            follower.getPose(),
//                                            new Pose(parkPose.getX(), parkPose.getY() + 36)
//                                    )
//                            )
//                    )
//                    .setLinearHeadingInterpolation(follower.getPose().getHeading(), parkPose.getHeading()+Math.toRadians(180))
//                    .build();
//        }
        state = DrivetrainState.PARK;
        follower.followPath(currentPathChain.get(), true);
    }

    public void intakeGate() {
        currentPathChain = () -> follower.pathBuilder()
                .addPath(
                        new Path(
                                new BezierLine(
                                        follower.getPose(),
                                        new Pose((alliance == Alliance.BLUE ? gatePose.getX()+15: gatePose.getX()-15), gatePose.getY())
                                )
                        )
                )
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), gatePose.getHeading())
                .addPath(
                        new Path(
                                new BezierLine(
                                        new Pose((alliance == Alliance.BLUE ? gatePose.getX()+15: gatePose.getX()-15), gatePose.getY()),
                                        gatePose
                                )
                        )
                )
                .setConstantHeadingInterpolation(gatePose.getHeading())
                .build();
        state = DrivetrainState.INTAKE_GATE;
        follower.followPath(currentPathChain.get(), true);
        // follower.followPath(intakePath, true);
    }

    public void openGate() {
        currentPathChain = () -> follower.pathBuilder()
                .addPath(
                        new Path(
                                new BezierLine(
                                        follower.getPose(),
                                        new Pose((alliance == Alliance.BLUE ? gateIntakePose.getX()+15: gateIntakePose.getX()-15), gateIntakePose.getY())
                                )
                        )
                )
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), gateIntakePose.getHeading())
                .addPath(
                        new Path(
                                new BezierLine(
                                        new Pose((alliance == Alliance.BLUE ? gateIntakePose.getX()+15: gateIntakePose.getX()-15), gateIntakePose.getY()),
                                        gateIntakePose
                                )
                        )
                )
                .setConstantHeadingInterpolation(gateIntakePose.getHeading())
                .build();
        state = DrivetrainState.OPEN_GATE;
        follower.followPath(currentPathChain.get(), true);
    }

    public boolean isBusy() {
        return state != DrivetrainState.TELEOP_DRIVE;
    }

    public String getState() {
        switch (state) {
            case TELEOP_DRIVE:
                return "TELEOP_DRIVE";
            case OPEN_GATE:
                return "OPENING_GATE";
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
            targetHeading = alliance == Alliance.BLUE ? PoseConstants.BLUE_GATE_AUTO_POSE.getHeading() : PoseConstants.RED_GATE_AUTO_POSE.getHeading();
            // we aren't going to update
            double headingError = MathUtil.angleWrap(targetHeading-currentHeading);
            double outHeading = kPHeading * headingError;
            return new double[] {x, y, outHeading};
        } else {
            // update target position heading
            if (Math.abs(rx) < 0.001) { // we are not rotating: hold heading
                double headingError = MathUtil.angleWrap(targetHeading-currentHeading);
                double outHeading = kPHeading * headingError;
                return new double[] {x, y, outHeading};
            } else {
                targetHeading = currentHeading;
                return new double[] {x, y, rx};
            }
        }
    }

    public void setGateHeadingLock(boolean x) {
        gateHeadingLock = x;
    }

    public boolean getGateHeadingLock() {return gateHeadingLock;}

    public void update(double x, double y, double rx) {
        follower.update();

        switch (state) {
            case TELEOP_DRIVE:
                double[] powers = calculateDrivetrainPowers(x, y, rx, follower.getHeading());
                if (alliance == Alliance.BLUE) {
                    follower.setTeleOpDrive(powers[0], powers[1], powers[2], robotCentric, Math.toRadians(180));
                } else {
                    follower.setTeleOpDrive(powers[0], powers[1], powers[2], robotCentric);
                }
                break;
            case OPEN_GATE:
                if (!follower.isBusy()) {
                    breakFollowing();
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
