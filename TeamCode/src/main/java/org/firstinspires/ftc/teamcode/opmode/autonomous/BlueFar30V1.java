package org.firstinspires.ftc.teamcode.opmode.autonomous;

import static java.lang.Thread.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
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
@Autonomous(name="Blue Far 27 pid to ball", group="!")
public class BlueFar30V1 extends OpMode {
    // TODO: we might want to do 30 or park, idk how much time we have.
    private Follower follower;
    private StateMachine stateMachine;
    private AutonomousRobot robot;
    private SOTM sotm2;
    private PoseFollower poseFollower;
    private boolean isTrackingBall = false;
    private final Pose startPose = new Pose(43, 9, Math.toRadians(180)); // TODO: get accurate position from the test.
    private final Pose currentShootPose = new Pose(43, 9, Math.toRadians(180));
    private final Pose goalPose = PoseConstants.BLUE_GOAL_POSE;
    private PathChain intakeCorner, shootCorner, intakeThird, shootThird;
    private double optimalX = 0;

    public void buildPaths() {
        intakeCorner = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(43, 9.000), new Pose(12.000, 9.000)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        shootCorner = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(12.000, 9.000), new Pose(43, 9)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        intakeThird = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(43, 9),
                                new Pose(40.000, 36.000),
                                new Pose(38.000, 36.000),
                                new Pose(13.000, 36.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        shootThird = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(13.000, 36.000), new Pose(50, 16))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.usePredictiveBraking = true;
        robot = new AutonomousRobot(hardwareMap, Alliance.BLUE);
        sotm2 = new SOTM(goalPose);
        poseFollower = new PoseFollower();
        buildPaths();

        stateMachine = new StateMachine(
                // preload
                new State()
                        .maxTime(2000) // in case it takes too long
                        .transition(new Transition(() -> robot.shooter.atTarget(20) && !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            robot.shootCommand.start();
                            follower.setMaxPower(1);
                        })
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // corner
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.followPath(intakeCorner, false);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> follower.followPath(shootCorner, true))
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // third
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.followPath(intakeThird, false);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> follower.followPath(shootThird, true))
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // pile 1
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.startTeleopDrive(false);
                            isTrackingBall = true;
                        })
                        .maxTime(2000)
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
                            follower.followPath(finishPath, false);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            isTrackingBall = false;
                            PathChain goBack = follower.pathBuilder()
                                    .addPath(
                                            new BezierLine(
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
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // pile 2
                new State()
                    .onEnter(() -> {
                        robot.intakeCommand.start();
                        follower.startTeleopDrive(false);
                        isTrackingBall = true;
                    })
                    .maxTime(2000)
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
                            follower.followPath(finishPath, false);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            isTrackingBall = false;
                            PathChain goBack = follower.pathBuilder()
                                    .addPath(
                                            new BezierLine(
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
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // pile 3
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.startTeleopDrive(false);
                            isTrackingBall = true;
                        })
                        .maxTime(2000)
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
                            follower.followPath(finishPath, false);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            isTrackingBall = false;
                            PathChain goBack = follower.pathBuilder()
                                    .addPath(
                                            new BezierLine(
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
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // pile 4
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.startTeleopDrive(false);
                            isTrackingBall = true;
                        })
                        .maxTime(2000)
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
                            follower.followPath(finishPath, false);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            isTrackingBall = false;
                            PathChain goBack = follower.pathBuilder()
                                    .addPath(
                                            new BezierLine(
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
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // pile 5
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.startTeleopDrive(false);
                            isTrackingBall = true;
                        })
                        .maxTime(2000)
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
                            follower.followPath(finishPath, false);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            isTrackingBall = false;
                            PathChain goBack = follower.pathBuilder()
                                    .addPath(
                                            new BezierLine(
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
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // pile 6
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.startTeleopDrive(false);
                            isTrackingBall = true;
                        })
                        .maxTime(2000)
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
                            follower.followPath(finishPath, false);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            isTrackingBall = false;
                            PathChain goBack = follower.pathBuilder()
                                    .addPath(
                                            new BezierLine(
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
                        .onEnter(() -> robot.shootCommand.start())
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
        robot.turret.setFeedforward(0);
        if (follower.getPose() != null) {
            double[] values = sotm2.calculateAzimuthThetaVelocityFRCBetter(currentShootPose, new Vector(), 0);
            robot.setAzimuthThetaVelocity(values);
        }
        stateMachine.update();
        follower.update();
        robot.update();
        telemetry.update();
    }

    @Override
    public void start() {
        double[] values = sotm2.calculateAzimuthThetaVelocityFRCBetter(currentShootPose, new Vector(), 0);
        robot.setAzimuthThetaVelocity(values);
        robot.turret.setPDCoefficients(0.01, 0.0005);

        robot.shooter.state = Shooter.ShooterState.SHOOTER_ON;

        stateMachine.start();
        robot.start();
    }

    @Override
    public void stop() {
        blackboard.put(RobotConstants.END_POSE_KEY, follower.getPose());
    }
}

