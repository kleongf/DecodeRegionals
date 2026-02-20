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
    private ElapsedTime holdPointTimer;
    public Follower follower;
    private double targetHeading = 0;
    private ElapsedTime elapsedTime;
    private double kp = 0.5;
    private double kd = 0.015;
    private double lastError = 0;
    private double lastTimeStamp = 0;
    private DrivetrainState state;
    private boolean robotCentric = false;
    private double KICK_TIME = 1.0;
    private double HOLD_TIME = 1.0;
    private boolean holdingPoint = false;
    private Alliance alliance;
    private Pose gatePose;
    private Pose gateIntakePose;
    private Pose parkPose;

    public TeleopDrivetrain(HardwareMap hardwareMap, Alliance alliance) {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, Math.toRadians(0)));
        follower.startTeleopDrive(true);
        follower.usePredictiveBraking = true;

        elapsedTime = new ElapsedTime();
        kickTimer = new ElapsedTime();
        holdPointTimer = new ElapsedTime();

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
        // follower.breakFollowing();
        // follower.
        follower.startTeleopDrive(true);
        state = DrivetrainState.TELEOP_DRIVE;
        holdingPoint = false;
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

    public void update(double x, double y, double rx) {
        follower.update();

        switch (state) {
            case TELEOP_DRIVE:
                if (alliance == Alliance.BLUE) {
                    follower.setTeleOpDrive(x, y, rx, false, Math.toRadians(180));
                } else {
                    follower.setTeleOpDrive(x, y, rx, false);
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
//                    if (!holdingPoint) { // if not holding point, then start holding point
//                        if (alliance == Alliance.BLUE) {
//                            follower.holdPoint(new BezierPoint(PoseConstants.BLUE_GATE_AUTO_POSE_IN), PoseConstants.BLUE_GATE_AUTO_POSE_IN.getHeading());
//                        } else {
//                            follower.holdPoint(new BezierPoint(PoseConstants.RED_GATE_AUTO_POSE_IN), PoseConstants.RED_GATE_AUTO_POSE_IN.getHeading());
//                        }
//                        holdingPoint = true;
//                        holdPointTimer.reset();
//                    } else { // else we are holding point
//                        if (holdPointTimer.seconds() > HOLD_TIME) {
//                            breakFollowing();
//                        }
//                    }
                }
                break;
        }

        elapsedTime.reset();
    }
}
