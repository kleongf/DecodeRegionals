package org.firstinspires.ftc.teamcode.opmode.autonomous;

import static java.lang.Thread.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.decodeutil.Alliance;
import org.firstinspires.ftc.teamcode.robot.constants.PoseConstants;
import org.firstinspires.ftc.teamcode.robot.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.robots.AutonomousRobot;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.decodeutil.PoseFollower;
import org.firstinspires.ftc.teamcode.util.fsm.State;
import org.firstinspires.ftc.teamcode.util.fsm.StateMachine;
import org.firstinspires.ftc.teamcode.util.fsm.Transition;
import org.firstinspires.ftc.teamcode.util.decodeutil.SOTM;
import org.firstinspires.ftc.teamcode.util.decodeutil.MathUtil;

// this one drives to a good point to "scout" for balls before driving to them. it seems that cam has not enough fov, so we are going to drive closer.
// TODO: maybe add a "safety" path: if not over two balls were collected, make a new path to drive to corner?
@Autonomous(name="Blue Pile Cycle Drive test", group="!")
public class DriveToBallTest extends OpMode {
    private Follower follower;
    private StateMachine followBall;
    private AutonomousRobot robot;
    private SOTM sotm2;
    private PoseFollower poseFollower;
    private boolean isTrackingBall = false;
    private final Pose startPose = new Pose(42.65, 9, Math.toRadians(180));
    private final Pose goalPose = PoseConstants.BLUE_GOAL_POSE;
    private double optimalX;

    public void buildPaths() {
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        robot = new AutonomousRobot(hardwareMap, Alliance.BLUE);
        sotm2 = new SOTM(goalPose);
        poseFollower = new PoseFollower();
        buildPaths();

        followBall = new StateMachine(
                // start following the ball
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.startTeleopDrive(false);
                            isTrackingBall = true;
                        })
                        // alternatively, once we pass x = 24 then just pid to the current one
                        .transition(new Transition(() -> follower.getPose().getX() < 24)),
                new State()
                        .onEnter(() -> {
                            PathChain finishPath = follower.pathBuilder()
                                    .addPath(
                                            new BezierLine(
                                                    follower.getPose(),
                                                    new Pose(9, MathUtil.clamp(follower.getPose().getY() + optimalX, 9, 40))
                                            )
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180))
                                    .build();
                            follower.followPath(finishPath);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            isTrackingBall = false;
                            PathChain goBack = follower.pathBuilder()
                                    .addPath(
                                            new BezierCurve(
                                                    follower.getPose(),
                                                    new Pose(50, 16)
                                            )
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180))
                                    .build();
                            follower.breakFollowing();
                            follower.followPath(goBack, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            robot.shootCommand.start();
                        })
                        .onExit(() -> {
                            follower.breakFollowing(); // so that we can move robot again
                        })
                        .transition(new Transition(() -> robot.shootCommand.isFinished()))
        );

        try {
            sleep(500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        robot.initPositions();
    }

    @Override
    public void loop() {
        if (follower.getPose() != null && follower.getVelocity() != null) {
            double[] values = sotm2.calculateAzimuthThetaVelocityFeedforward(follower.getPose(), follower.getVelocity(), follower.getAngularVelocity());
            robot.turret.setTarget(values[0]);
            robot.shooter.setShooterPitch(values[1]);
            robot.shooter.setTargetVelocity(values[2]);
            robot.turret.setFeedforward(values[3]);
        }
        if (isTrackingBall) {
            optimalX = robot.vision.getBestXMovingAverage();
            Pose targetPose = new Pose(9, MathUtil.clamp(follower.getPose().getY() + optimalX, 9, 40), Math.toRadians(180));
            double[] powers = poseFollower.calculate(follower.getPose(), targetPose);
            follower.setTeleOpDrive(powers[0], powers[1], powers[2], true);
        }

        if (gamepad1.leftBumperWasPressed()) {
            followBall.start();
        }

        telemetry.addData("Pose", follower.getPose());

        followBall.update();
        follower.update();
        robot.update();
        telemetry.update();
    }

    @Override
    public void start() {
        double[] values = sotm2.calculateAzimuthThetaVelocity(new Pose(42.65, 9, Math.toRadians(180)), new Vector());
        robot.setAzimuthThetaVelocity(values);
        robot.shooter.state = Shooter.ShooterState.SHOOTER_ON;
        robot.start();
        follower.usePredictiveBraking = true;
        // follower.breakFollowing();
    }

    @Override
    public void stop() {
        blackboard.put(RobotConstants.END_POSE_KEY, follower.getPose());
    }
}
