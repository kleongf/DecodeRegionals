package org.firstinspires.ftc.teamcode.opmode.autonomous;

import static java.lang.Thread.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.decodeutil.Alliance;
import org.firstinspires.ftc.teamcode.robot.constants.PoseConstants;
import org.firstinspires.ftc.teamcode.robot.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.robots.AutonomousRobot;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.fsm.State;
import org.firstinspires.ftc.teamcode.util.fsm.StateMachine;
import org.firstinspires.ftc.teamcode.util.fsm.Transition;
import org.firstinspires.ftc.teamcode.util.decodeutil.SOTM;
import org.firstinspires.ftc.teamcode.util.decodeutil.MathUtil;

// this one drives to a good point to "scout" for balls before driving to them. it seems that cam has not enough fov, so we are going to drive closer.
// TODO: maybe add a "safety" path: if not over two balls were collected, make a new path to drive to corner?
@Disabled
@Autonomous(name="Blue Far 30 new testing", group="?")
public class BlueFar30Better extends OpMode {
    private Follower follower;
    private StateMachine stateMachine;
    private AutonomousRobot robot;
    private SOTM sotm2;
    private final Pose startPose = new Pose(42.65, 9, Math.toRadians(180));
    private final Pose goalPose = PoseConstants.BLUE_GOAL_POSE;
    private PathChain intakeCorner, shootCorner, intakeThird, shootThird, lookPile1, intakePile1, shootPile1, lookPile2, intakePile2, shootPile2, lookPile3, intakePile3, shootPile3;

    public void buildPaths() {
        intakeCorner = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(42.650, 9.000), new Pose(12.000, 9.000)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        shootCorner = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(12.000, 9.000), new Pose(46, 9)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        intakeThird = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(46, 9),
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

        // path to "look" for balls
        lookPile1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(50.000, 16.000),
                                new Pose(30.000, 20.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        lookPile2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(50.000, 16.000),
                                new Pose(30.000, 20.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        lookPile3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(50.000, 16.000),
                                new Pose(30.000, 20.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

//        follower.startTeleopDrive();
//        follower.setTeleOpDrive();
        // idea: set to teleop drive field centric and make it work idk
        // pid to pose
        // but in this case we need exact coords of ball
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        robot = new AutonomousRobot(hardwareMap, Alliance.BLUE);
        sotm2 = new SOTM(goalPose);
        buildPaths();

        stateMachine = new StateMachine(
                // preload
                new State()
                        .maxTime(3000) // in case it takes too long
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
                            follower.followPath(lookPile1, true);
                        })
                        // iteration 1: updating ONCE! not multiple times, so we go to corner originally
                        .transition(new Transition(() -> follower.atParametricEnd())),
                new State()
                        .onEnter(() -> {
                            double optimalX = robot.vision.getLargestClusterX();
                            double currentY = follower.getPose().getY();
                            double currentX = follower.getPose().getX();
                            intakePile1 = follower.pathBuilder()
                                    .addPath(
                                            new BezierCurve(
                                                    new Pose(currentX, currentY),
                                                    new Pose(9, MathUtil.clamp(currentY + optimalX, 9, 40))
                                            )
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180))
                                    .setVelocityConstraint(20)
                                    .setTValueConstraint(0.9)
                                    .setHeadingConstraint(Math.toRadians(5))

                                    .build();
                            shootPile1 = follower.pathBuilder()
                                    .addPath(
                                            new BezierCurve(
                                                    new Pose(9, MathUtil.clamp(currentY + optimalX, 9, 40)),
                                                    new Pose(50, 16)
                                            )
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180))
                                    .build();
                            follower.breakFollowing();
                            follower.followPath(intakePile1, false);
                        })
                        // iteration 3: just go till the end, the intake is literally like 10 inches away so it shouldnt move much
                        .maxTime(2000)
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> follower.followPath(shootPile1, true))
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // pile 2
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.followPath(lookPile2, true);
                        })
                        // iteration 1: updating ONCE! not multiple times, so we go to corner originally
                        .transition(new Transition(() -> follower.atParametricEnd())),
                new State()
                        .onEnter(() -> {
                            double optimalX = robot.vision.getLargestClusterX();
                            double currentY = follower.getPose().getY();
                            double currentX = follower.getPose().getX();
                            intakePile2 = follower.pathBuilder()
                                    .addPath(
                                            new BezierCurve(
                                                    new Pose(currentX, currentY),
                                                    new Pose(9, MathUtil.clamp(currentY + optimalX, 9, 40))
                                            )
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180))
                                    .setVelocityConstraint(20)
                                    .setTValueConstraint(0.9)
                                    .setHeadingConstraint(Math.toRadians(5))

                                    .build();
                            shootPile2 = follower.pathBuilder()
                                    .addPath(
                                            new BezierCurve(
                                                    new Pose(9, MathUtil.clamp(currentY + optimalX, 9, 40)),
                                                    new Pose(50, 16)
                                            )
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180))
                                    .build();
                            follower.breakFollowing();
                            follower.followPath(intakePile1, false);
                        })
                        // iteration 3: just go till the end, the intake is literally like 10 inches away so it shouldnt move much
                        .maxTime(2000)
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> follower.followPath(shootPile2, true))
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // pile 3
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.followPath(lookPile3, true);
                        })
                        // iteration 1: updating ONCE! not multiple times, so we go to corner originally
                        .transition(new Transition(() -> follower.atParametricEnd())),
                new State()
                        .onEnter(() -> {
                            double optimalX = robot.vision.getLargestClusterX();
                            double currentY = follower.getPose().getY();
                            double currentX = follower.getPose().getX();
                            intakePile3 = follower.pathBuilder()
                                    .addPath(
                                            new BezierCurve(
                                                    new Pose(currentX, currentY),
                                                    new Pose(9, MathUtil.clamp(currentY + optimalX, 9, 40))
                                            )
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180))
                                    .setVelocityConstraint(20)
                                    .setTValueConstraint(0.9)
                                    .setHeadingConstraint(Math.toRadians(5))

                                    .build();
                            shootPile3 = follower.pathBuilder()
                                    .addPath(
                                            new BezierCurve(
                                                    new Pose(9, MathUtil.clamp(currentY + optimalX, 9, 40)),
                                                    new Pose(50, 16)
                                            )
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180))
                                    .build();
                            follower.breakFollowing();
                            follower.followPath(intakePile3, false);
                        })
                        // iteration 3: just go till the end, the intake is literally like 10 inches away so it shouldnt move much
                        .maxTime(2000)
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> follower.followPath(shootPile3, true))
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
        double[] values;
        robot.turret.setFeedforward(0);
        if (follower.getPose() != null && follower.getVelocity() != null) {
            values = sotm2.calculateAzimuthThetaVelocity(follower.getPose(), follower.getVelocity());
            robot.setAzimuthThetaVelocity(values);
        }
        stateMachine.update();
        follower.update();
        robot.update();
        telemetry.update();
    }

    @Override
    public void start() {
        double[] values = sotm2.calculateAzimuthThetaVelocity(new Pose(42.65, 9, Math.toRadians(180)), new Vector());
        robot.setAzimuthThetaVelocity(values);

        robot.shooter.state = Shooter.ShooterState.SHOOTER_ON;

        stateMachine.start();
        robot.start();
    }

    @Override
    public void stop() {
        blackboard.put(RobotConstants.END_POSE_KEY, follower.getPose());
    }
}

