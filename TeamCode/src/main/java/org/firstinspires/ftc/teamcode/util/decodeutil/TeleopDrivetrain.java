package org.firstinspires.ftc.teamcode.util.decodeutil;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.constants.PoseConstants;

public class TeleopDrivetrain {
    public enum DrivetrainState {
        TELEOP_DRIVE,
        KICK,
        INTAKE_GATE,
        OPEN_GATE,
        PARK
    }

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

    public TeleopDrivetrain(HardwareMap hardwareMap, Alliance alliance) {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, Math.toRadians(0)));
        follower.startTeleopDrive(true);

        elapsedTime = new ElapsedTime();
        kickTimer = new ElapsedTime();
        holdPointTimer = new ElapsedTime();

        this.alliance = alliance;
        state = DrivetrainState.TELEOP_DRIVE;
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
        follower.breakFollowing();
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

    public void kick(PathChain kickPath) {
        state = DrivetrainState.KICK;
        follower.followPath(kickPath, true);
        kickTimer.reset();
    }

    public void park(PathChain parkPath) {
        state = DrivetrainState.PARK;
        follower.followPath(parkPath, true);
    }

    public void intakeGate(PathChain intakePath) {
        state = DrivetrainState.INTAKE_GATE;
        follower.followPath(intakePath, true);
    }

    public void openGate(PathChain openPath) {
        state = DrivetrainState.OPEN_GATE;
        follower.followPath(openPath, true);
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
                follower.setTeleOpDrive(x, y, rx, robotCentric);
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
                    if (!holdingPoint) { // if not holding point, then start holding point
                        if (alliance == Alliance.BLUE) {
                            follower.holdPoint(new BezierPoint(PoseConstants.BLUE_GATE_AUTO_POSE_IN), PoseConstants.BLUE_GATE_AUTO_POSE_IN.getHeading());
                        } else {
                            follower.holdPoint(new BezierPoint(PoseConstants.RED_GATE_AUTO_POSE_IN), PoseConstants.RED_GATE_AUTO_POSE_IN.getHeading());
                        }
                        holdingPoint = true;
                        holdPointTimer.reset();
                    } else { // else we are holding point
                        if (holdPointTimer.seconds() > HOLD_TIME) {
                            breakFollowing();
                        }
                    }
                }
                break;
        }

        elapsedTime.reset();
    }
}
