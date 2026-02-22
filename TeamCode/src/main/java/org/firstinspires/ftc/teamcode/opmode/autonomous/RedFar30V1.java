package org.firstinspires.ftc.teamcode.opmode.autonomous;

import static java.lang.Thread.sleep;

import android.util.Log;

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
@Autonomous(name="Red Far 30 Comp", group="!")
public class RedFar30V1 extends OpMode {
    // TODO: we might want to do 30 or park, idk how much time we have.
    private Follower follower;
    private StateMachine stateMachine;
    private AutonomousRobot robot;
    private SOTM sotm2;
    private PoseFollower poseFollower;
    private boolean isTrackingBall = false;
    private final Pose startPose = PoseConstants.RED_FAR_AUTO_POSE; // TODO: get accurate position from the test.
    private Pose currentShootPose = PoseConstants.RED_FAR_AUTO_POSE;
    private final Pose goalPose = PoseConstants.RED_GOAL_POSE;
    private PathChain intakeCorner, shootCorner, intakeThird, shootThird, intakeCorner1, goBack1, intakeCorner2, goBack2, intakeCorner3, goBack3, intakeCorner4, goBack4, intakeCorner5, goBack5, intakeCorner6, goBack6, intakeCorner7, goBack7;
    private double optimalX = 0;

    public void buildPaths() {
        intakeCorner = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(144-42, 8.000), new Pose(144-12.000, 9.000)))
                .setConstantHeadingInterpolation(Math.toRadians(180-180))
                .build();
        shootCorner = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(144-12.000, 9.000), new Pose(144-46, 9)))
                .setConstantHeadingInterpolation(Math.toRadians(180-180))
                .build();

        intakeThird = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(144-46, 9),
                                new Pose(144-40.000, 36.000),
                                new Pose(144-38.000, 36.000),
                                new Pose(144-13.000, 36.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180-180))
                .build();

        shootThird = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(144-13.000, 36.000), new Pose(144-46, 9))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180-180))
                .build();
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.usePredictiveBraking = true;
        robot = new AutonomousRobot(hardwareMap, Alliance.RED);
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
                        .onEnter(() -> {
                            follower.followPath(shootCorner, true);
                            currentShootPose = new Pose(144-46, 9, Math.toRadians(180-180));
                        })
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
                            // optimalX = 0;
                            robot.intakeCommand.start();
                            intakeCorner1 = follower.pathBuilder()
                                    .addPath(
                                            new BezierLine(
                                                    follower.getPose(),
                                                    new Pose(144-9, MathUtil.clamp(follower.getPose().getY() - optimalX, 9, 40))
                                            )
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180-180))
                                    .setNoDeceleration()
                                    .build();
                            follower.followPath(intakeCorner1, false);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            goBack1 = follower.pathBuilder()
                                    .addPath(
                                            new BezierLine(
                                                    follower.getPose(),
                                                    new Pose(144-46, 9)
                                            )
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180-180))
                                    .build();
                            follower.followPath(goBack1, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // pile 2
                new State()
                        .onEnter(() -> {
                            // optimalX = 24;
                            robot.intakeCommand.start();
                            intakeCorner2 = follower.pathBuilder()
                                    .addPath(
                                            new BezierLine(
                                                    follower.getPose(),
                                                    new Pose(144-9, MathUtil.clamp(follower.getPose().getY() - optimalX, 9, 40))
                                            )
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180-180))
                                    .setNoDeceleration()
                                    .build();
                            follower.followPath(intakeCorner2, false);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            goBack2 = follower.pathBuilder()
                                    .addPath(
                                            new BezierLine(
                                                    follower.getPose(),
                                                    new Pose(144-46, 9)
                                            )
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180-180))
                                    .build();
                            follower.followPath(goBack2, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // pile 3
                new State()
                        .onEnter(() -> {
                            // optimalX = 0;
                            robot.intakeCommand.start();
                            intakeCorner3 = follower.pathBuilder()
                                    .addPath(
                                            new BezierLine(
                                                    follower.getPose(),
                                                    new Pose(144-9, MathUtil.clamp(follower.getPose().getY() - optimalX, 9, 40))
                                            )
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180-180))
                                    .setNoDeceleration()
                                    .build();
                            follower.followPath(intakeCorner3, false);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            goBack3 = follower.pathBuilder()
                                    .addPath(
                                            new BezierLine(
                                                    follower.getPose(),
                                                    new Pose(144-46, 9)
                                            )
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180-180))
                                    .build();
                            follower.followPath(goBack3, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // pile 4
                new State()
                        .onEnter(() -> {
                            // optimalX = 24;
                            robot.intakeCommand.start();
                            intakeCorner4 = follower.pathBuilder()
                                    .addPath(
                                            new BezierLine(
                                                    follower.getPose(),
                                                    new Pose(144-9, MathUtil.clamp(follower.getPose().getY() - optimalX, 9, 40))
                                            )
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180-180))
                                    .setNoDeceleration()
                                    .build();
                            follower.followPath(intakeCorner4, false);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            goBack4 = follower.pathBuilder()
                                    .addPath(
                                            new BezierLine(
                                                    follower.getPose(),
                                                    new Pose(144-46, 9)
                                            )
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180-180))
                                    .build();
                            follower.followPath(goBack4, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // pile 5
                new State()
                        .onEnter(() -> {
                            // optimalX = 0;
                            robot.intakeCommand.start();
                            intakeCorner5 = follower.pathBuilder()
                                    .addPath(
                                            new BezierLine(
                                                    follower.getPose(),
                                                    new Pose(144-9, MathUtil.clamp(follower.getPose().getY() - optimalX, 9, 40))
                                            )
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180-180))
                                    .setNoDeceleration()
                                    .build();
                            follower.followPath(intakeCorner5, false);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            goBack5 = follower.pathBuilder()
                                    .addPath(
                                            new BezierLine(
                                                    follower.getPose(),
                                                    new Pose(144-46, 9)
                                            )
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180-180))
                                    .build();
                            follower.followPath(goBack5, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // pile 6
                new State()
                        .onEnter(() -> {
                            // optimalX = 24;
                            robot.intakeCommand.start();
                            intakeCorner6 = follower.pathBuilder()
                                    .addPath(
                                            new BezierLine(
                                                    follower.getPose(),
                                                    new Pose(144-9, MathUtil.clamp(follower.getPose().getY() - optimalX, 9, 40))
                                            )
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180-180))
                                    .setNoDeceleration()
                                    .build();
                            follower.followPath(intakeCorner6, false);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            goBack6 = follower.pathBuilder()
                                    .addPath(
                                            new BezierLine(
                                                    follower.getPose(),
                                                    new Pose(144-46, 9)
                                            )
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180-180))
                                    .build();
                            follower.followPath(goBack6, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // pile 7
                new State()
                        .onEnter(() -> {
                            // optimalX = 0;
                            robot.intakeCommand.start();
                            intakeCorner7 = follower.pathBuilder()
                                    .addPath(
                                            new BezierLine(
                                                    follower.getPose(),
                                                    new Pose(144-9, MathUtil.clamp(follower.getPose().getY() - optimalX, 9, 40))
                                            )
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180-180))
                                    .setNoDeceleration()
                                    .build();
                            follower.followPath(intakeCorner7, false);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            goBack7 = follower.pathBuilder()
                                    .addPath(
                                            new BezierLine(
                                                    follower.getPose(),
                                                    new Pose(144-46, 9)
                                            )
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180-180))
                                    .build();
                            follower.followPath(goBack7, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                new State()
                        .onEnter(() -> follower.holdPoint(new Pose(144-38, 9, Math.toRadians(180-180))))

        );

        try {
            sleep(500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        robot.initPositions();
        robot.turret.resetEncoderWithAbsoluteReading();
        robot.turret.setUseExternal(false);
    }

    @Override
    public void loop() {
        robot.turret.setFeedforward(0);
        if (follower.getPose() != null) {
            double[] values = sotm2.calculateAzimuthThetaVelocityFRCBetter(currentShootPose, new Vector(), 0);
            robot.setAzimuthThetaVelocity(values);
        }
        optimalX = robot.vision.getLargestClusterX();
        stateMachine.update();
        follower.update();
        robot.update();

        blackboard.put(RobotConstants.END_POSE_KEY, follower.getPose());

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
