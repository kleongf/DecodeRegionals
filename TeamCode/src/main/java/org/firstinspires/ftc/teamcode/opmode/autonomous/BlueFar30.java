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
import org.firstinspires.ftc.teamcode.util.fsm.State;
import org.firstinspires.ftc.teamcode.util.fsm.StateMachine;
import org.firstinspires.ftc.teamcode.util.fsm.Transition;
import org.firstinspires.ftc.teamcode.util.decodeutil.SOTM;
import org.firstinspires.ftc.teamcode.util.decodeutil.MathUtil;

// realistically, these are the only two autonomous programs needed: an extra gate, a far auto,
// and a safe far auto with Theseus pathing just in case.

// that means that we must do our job and do it well.
// the far auto must work well. cv must not just work in close ranges like this - it should work from all distances.
// idea: we can update the closest point and always point to it with the face point interpolation?
// but i think we should always be 180.
// for more accuracy, maybe go to the middle? about x = 24 ish? we are going from x = 48 to x = 9 to check.
// and check when we are like 24 inches away or something idk, so that where we go is good
// could always be driving to the best point or something

@Autonomous(name="Blue Far 30", group="?")
public class BlueFar30 extends OpMode {
    private Follower follower;
    private StateMachine stateMachine;
    private AutonomousRobot robot;
    private SOTM sotm2;
    private final Pose startPose = new Pose(42.65, 9, Math.toRadians(180));
    private final Pose goalPose = PoseConstants.BLUE_GOAL_POSE;
    private PathChain intakeCorner, shootCorner, intakeThird, shootThird, intakePile1, shootPile1, intakePile2, shootPile2, intakePile3, shootPile3, intakePile4, shootPile4, intakePile5, shootPile5, intakePile6, shootPile6, intakePile7, shootPile7, park;

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
                        new BezierLine(new Pose(13.000, 36.000), new Pose(46, 9))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        intakePile1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(46, 9),
                                new Pose(9.000, 9)
                        )
                )

                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setTValueConstraint(0.9)
                .build();

        intakePile2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(46, 9),
                                new Pose(9.000, 9)
                        )
                )

                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setTValueConstraint(0.9)
                .build();

        intakePile3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(46, 9),
                                new Pose(9.000, 9)
                        )
                )

                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setTValueConstraint(0.9)
                .build();

        intakePile4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(46, 9),
                                new Pose(9.000, 9)
                        )
                )

                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setTValueConstraint(0.9)
                .build();

        intakePile5 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(46, 9),
                                new Pose(9.000, 9)
                        )
                )

                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setTValueConstraint(0.9)
                .build();

        intakePile6 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(46, 9),
                                new Pose(9.000, 9)
                        )
                )

                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setTValueConstraint(0.9)
                .build();

        intakePile7 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(46, 9),
                                new Pose(9.000, 9)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setTValueConstraint(0.9)
                .build();

        park = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(46, 9),
                                new Pose(36, 12)
                        )
                )
                .setBrakingStart(3) // idk i want to brake really fast
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
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
                            follower.followPath(intakePile1, false);
                        })
                        // iteration 1: updating ONCE! not multiple times, so we go to corner originally
                        .transition(new Transition(() -> follower.getPose().getX() < 30)),
                new State()
                        .onEnter(() -> {
                            double optimalX = robot.vision.getLargestClusterX();
                            double currentY = follower.getPose().getY();
                            double currentX = follower.getPose().getX();
                            intakePile1 = follower.pathBuilder()
                                    .addPath(
                                            new BezierCurve(
                                                    new Pose(currentX, 9),
                                                    new Pose(currentX - 4, MathUtil.clamp(currentY + optimalX, 9, 40)),
                                                    new Pose(currentX - 6, MathUtil.clamp(currentY + optimalX, 9, 40)),
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
                                                    new Pose(46, 9)
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
                            follower.followPath(intakePile2, false);
                        })
                        // iteration 1: updating ONCE! not multiple times, so we go to corner originally
                        .transition(new Transition(() -> follower.getPose().getX() < 30)),
                new State()
                        .onEnter(() -> {
                            double optimalX = robot.vision.getLargestClusterX();
                            double currentY = follower.getPose().getY();
                            double currentX = follower.getPose().getX();
                            intakePile2 = follower.pathBuilder()
                                    .addPath(
                                            new BezierCurve(
                                                    new Pose(currentX, 9),
                                                    new Pose(currentX - 4, MathUtil.clamp(currentY + optimalX, 9, 40)),
                                                    new Pose(currentX - 6, MathUtil.clamp(currentY + optimalX, 9, 40)),
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
                                                    new Pose(46, 9)
                                            )
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180))

                                    .build();
                            follower.breakFollowing();
                            follower.followPath(intakePile2, false);
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
                            follower.followPath(intakePile3, false);
                        })
                        // iteration 1: updating ONCE! not multiple times, so we go to corner originally
                        .transition(new Transition(() -> follower.getPose().getX() < 30)),
                new State()
                        .onEnter(() -> {
                            double optimalX = robot.vision.getLargestClusterX();
                            double currentY = follower.getPose().getY();
                            double currentX = follower.getPose().getX();
                            intakePile3 = follower.pathBuilder()
                                    .addPath(
                                            new BezierCurve(
                                                    new Pose(currentX, 9),
                                                    new Pose(currentX - 4, MathUtil.clamp(currentY + optimalX, 9, 40)),
                                                    new Pose(currentX - 6, MathUtil.clamp(currentY + optimalX, 9, 40)),
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
                                                    new Pose(46, 9)
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
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // pile 4
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.followPath(intakePile4, false);
                        })
                        // iteration 1: updating ONCE! not multiple times, so we go to corner originally
                        .transition(new Transition(() -> follower.getPose().getX() < 30)),
                new State()
                        .onEnter(() -> {
                            double optimalX = robot.vision.getLargestClusterX();
                            double currentY = follower.getPose().getY();
                            double currentX = follower.getPose().getX();
                            intakePile4 = follower.pathBuilder()
                                    .addPath(
                                            new BezierCurve(
                                                    new Pose(currentX, 9),
                                                    new Pose(currentX - 4, MathUtil.clamp(currentY + optimalX, 9, 40)),
                                                    new Pose(currentX - 6, MathUtil.clamp(currentY + optimalX, 9, 40)),
                                                    new Pose(9, MathUtil.clamp(currentY + optimalX, 9, 40))
                                            )
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180))
                                    .setVelocityConstraint(20)
                                    .setTValueConstraint(0.9)
                                    .setHeadingConstraint(Math.toRadians(5))

                                    .build();
                            shootPile4 = follower.pathBuilder()
                                    .addPath(
                                            new BezierCurve(
                                                    new Pose(9, MathUtil.clamp(currentY + optimalX, 9, 40)),
                                                    new Pose(46, 9)
                                            )
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180))

                                    .build();

                            follower.breakFollowing();
                            follower.followPath(intakePile4, false);
                        })
                        // iteration 3: just go till the end, the intake is literally like 10 inches away so it shouldnt move much
                        .maxTime(2000)
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> follower.followPath(shootPile4, true))
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // pile 5
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.followPath(intakePile5, false);
                        })
                        // iteration 1: updating ONCE! not multiple times, so we go to corner originally
                        .transition(new Transition(() -> follower.getPose().getX() < 30)),
                new State()
                        .onEnter(() -> {
                            double optimalX = robot.vision.getLargestClusterX();
                            double currentY = follower.getPose().getY();
                            double currentX = follower.getPose().getX();
                            intakePile5 = follower.pathBuilder()
                                    .addPath(
                                            new BezierCurve(
                                                    new Pose(currentX, 9),
                                                    new Pose(currentX - 4, MathUtil.clamp(currentY + optimalX, 9, 40)),
                                                    new Pose(currentX - 6, MathUtil.clamp(currentY + optimalX, 9, 40)),
                                                    new Pose(9, MathUtil.clamp(currentY + optimalX, 9, 40))
                                            )
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180))
                                    .setVelocityConstraint(20)
                                    .setTValueConstraint(0.9)
                                    .setHeadingConstraint(Math.toRadians(5))

                                    .build();
                            shootPile5 = follower.pathBuilder()
                                    .addPath(
                                            new BezierCurve(
                                                    new Pose(9, MathUtil.clamp(currentY + optimalX, 9, 40)),
                                                    new Pose(46, 9)
                                            )
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180))

                                    .build();

                            follower.breakFollowing();
                            follower.followPath(intakePile5, false);
                        })
                        .maxTime(2000)
                        // iteration 3: just go till the end, the intake is literally like 10 inches away so it shouldnt move much
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> follower.followPath(shootPile5, true))
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // pile 6
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.followPath(intakePile6, false);
                        })
                        // iteration 1: updating ONCE! not multiple times, so we go to corner originally
                        .transition(new Transition(() -> follower.getPose().getX() < 30)),
                new State()
                        .onEnter(() -> {
                            double optimalX = robot.vision.getLargestClusterX();
                            double currentY = follower.getPose().getY();
                            double currentX = follower.getPose().getX();
                            intakePile6 = follower.pathBuilder()
                                    .addPath(
                                            new BezierCurve(
                                                    new Pose(currentX, 9),
                                                    new Pose(currentX - 4, MathUtil.clamp(currentY + optimalX, 9, 40)),
                                                    new Pose(currentX - 6, MathUtil.clamp(currentY + optimalX, 9, 40)),
                                                    new Pose(9, MathUtil.clamp(currentY + optimalX, 9, 40))
                                            )
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180))
                                    .setVelocityConstraint(20)
                                    .setTValueConstraint(0.9)
                                    .setHeadingConstraint(Math.toRadians(5))

                                    .build();
                            shootPile6 = follower.pathBuilder()
                                    .addPath(
                                            new BezierCurve(
                                                    new Pose(9, MathUtil.clamp(currentY + optimalX, 9, 40)),
                                                    new Pose(46, 9)
                                            )
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180))

                                    .build();

                            follower.breakFollowing();
                            follower.followPath(intakePile6, false);
                        })
                        .maxTime(2000)
                        // iteration 3: just go till the end, the intake is literally like 10 inches away so it shouldnt move much
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> follower.followPath(shootPile6, true))
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // pile 7
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.followPath(intakePile7, false);
                        })
                        // iteration 1: updating ONCE! not multiple times, so we go to corner originally
                        .transition(new Transition(() -> follower.getPose().getX() < 30)),
                new State()
                        .onEnter(() -> {
                            double optimalX = robot.vision.getLargestClusterX();
                            double currentY = follower.getPose().getY();
                            double currentX = follower.getPose().getX();
                            intakePile7 = follower.pathBuilder()
                                    .addPath(
                                            new BezierCurve(
                                                    new Pose(currentX, 9),
                                                    new Pose(currentX - 4, MathUtil.clamp(currentY + optimalX, 9, 40)),
                                                    new Pose(currentX - 6, MathUtil.clamp(currentY + optimalX, 9, 40)),
                                                    new Pose(9, MathUtil.clamp(currentY + optimalX, 9, 40))
                                            )
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180))
                                    .setVelocityConstraint(20)
                                    .setTValueConstraint(0.9)
                                    .setHeadingConstraint(Math.toRadians(5))

                                    .build();
                            shootPile7 = follower.pathBuilder()
                                    .addPath(
                                            new BezierCurve(
                                                    new Pose(9, MathUtil.clamp(currentY + optimalX, 9, 40)),
                                                    new Pose(46, 9)
                                            )
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180))

                                    .build();
                            follower.breakFollowing();
                            follower.followPath(intakePile7, false);
                        })
                        // iteration 3: just go till the end, the intake is literally like 10 inches away so it shouldnt move much
                        .maxTime(2000)
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> follower.followPath(shootPile7, true))
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                new State()
                        .onEnter(() -> {
                            follower.followPath(park, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy()))
                        .onExit(() -> blackboard.put(RobotConstants.END_POSE_KEY, follower.getPose())),
                new State()
                        .onEnter(() -> {
                            // just in case
                            blackboard.put(RobotConstants.END_POSE_KEY, follower.getPose());
                        })
                        .maxTime(100)

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
