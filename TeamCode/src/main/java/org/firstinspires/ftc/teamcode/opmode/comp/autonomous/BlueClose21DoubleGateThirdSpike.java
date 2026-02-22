//package org.firstinspires.ftc.teamcode.opmode.comp.autonomous;
//
//import static java.lang.Thread.sleep;
//
//import android.util.Log;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.BezierPoint;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.math.Vector;
//import com.pedropathing.paths.HeadingInterpolator;
//import com.pedropathing.paths.PathChain;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//import org.firstinspires.ftc.teamcode.robot.constants.PoseConstants;
//import org.firstinspires.ftc.teamcode.robot.constants.RobotConstants;
//import org.firstinspires.ftc.teamcode.robot.robots.AutonomousRobot;
//import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
//import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
//import org.firstinspires.ftc.teamcode.util.decodeutil.Alliance;
//import org.firstinspires.ftc.teamcode.util.decodeutil.MathUtil;
//import org.firstinspires.ftc.teamcode.util.decodeutil.SOTM;
//import org.firstinspires.ftc.teamcode.util.fsm.State;
//import org.firstinspires.ftc.teamcode.util.fsm.StateMachine;
//import org.firstinspires.ftc.teamcode.util.fsm.Transition;
//
//import java.util.ArrayList;
//
//@Autonomous(name="Blue Close 21 Double Gate Third Spike COMP", group="!")
//public class BlueClose21DoubleGateThirdSpike extends OpMode {
//    private Follower follower;
//    private StateMachine stateMachine;
//    private AutonomousRobot robot;
//    private SOTM sotm2;
//    private final Pose startPose = PoseConstants.BLUE_CLOSE_AUTO_POSE;
//    private final Pose goalPose = PoseConstants.BLUE_GOAL_POSE;
//    private final Pose shootPose = new Pose(60, 72, Math.toRadians(-144));
//    private Pose currentShootPose = new Pose(60, 72, Math.toRadians(-144));
//    private Pose finalGateCycleEndPose = new Pose(60, 72, Math.toRadians(-105));
//    private PathChain shootPreloadIntakeSecond, openGate1, shootSecond, intakeGate1, shootGate1, intakeGate2, shootGate2, openGate2, intakeGate3, shootGate3, intakeGate4, shootGate4, intakeFirst, shootFirst;
//    private boolean isSOTMing = true;
//
//    public void buildPaths() {
//        double k1 = 10; // i actually forgot what these were check desmos
//        double k2 = 20;
//        // idk abt this just check it again and do for control point 2
//        Pose controlPoint1 = new Pose(shootPose.getX() + k1 * Math.cos(shootPose.getHeading()), shootPose.getY() + k1 * Math.sin(shootPose.getHeading()));
//        Pose controlPoint2 = new Pose(PoseConstants.BLUE_GATE_AUTO_POSE.getX() - k2 * Math.cos(PoseConstants.BLUE_GATE_AUTO_POSE.getHeading()), PoseConstants.BLUE_GATE_AUTO_POSE.getY() - k2 * Math.sin(PoseConstants.BLUE_GATE_AUTO_POSE.getHeading()));
//        Pose controlPoint3 = new Pose(finalGateCycleEndPose.getX() + k1 * Math.cos(finalGateCycleEndPose.getHeading()), finalGateCycleEndPose.getY() + k1 * Math.sin(finalGateCycleEndPose.getHeading()));
//
//        shootPreloadIntakeSecond = follower.pathBuilder().addPath(
//                        new BezierLine(
//                                startPose,
//                                new Pose(54.000, 90.000)
//                        )
//                ).setConstantHeadingInterpolation(Math.toRadians(-110))
//                .addPath(
//                        new BezierCurve(
//                                new Pose(54, 90),
//                                new Pose(40, 60),
//                                new Pose(12, 60)
//                        )
//                )
//                .setTangentHeadingInterpolation()
//                .build();
//
//        // also move slow while opening gate
//        openGate1 = follower.pathBuilder().addPath(
//                        new BezierCurve(
//                                new Pose(12.000, 60.000),
//                                new Pose(24.457, 58.947),
//                                new Pose(24.383, 63.138),
//                                PoseConstants.BLUE_GATE_POSE
//                        )
//                ).setConstantHeadingInterpolation(Math.toRadians(180))
//                .setNoDeceleration()
//                .build();
//
//        shootSecond = follower.pathBuilder().addPath(
//                        new BezierLine(
//                                PoseConstants.BLUE_GATE_POSE,
//                                new Pose(60.000, 72.000)
//                        )
//                ).setConstantHeadingInterpolation(Math.toRadians(180))
//                .build();
//
//        HeadingInterpolator toGateAfterOpen = HeadingInterpolator.piecewise(
//                new HeadingInterpolator.PiecewiseNode(
//                        0,
//                        0.75,
//                        HeadingInterpolator.linear(shootSecond.getFinalHeadingGoal(), PoseConstants.BLUE_GATE_AUTO_POSE.getHeading())
//                ),
//                new HeadingInterpolator.PiecewiseNode(
//                        0.75,
//                        1,
//                        HeadingInterpolator.constant(PoseConstants.BLUE_GATE_AUTO_POSE.getHeading())
//                )
//        );
//
//        intakeGate1 = follower.pathBuilder()
//                .addPath(
//                        new BezierCurve(
//                                new Pose(60.000, 72.000),
//                                new Pose(45, PoseConstants.BLUE_GATE_AUTO_POSE.getY()),
//                                PoseConstants.BLUE_GATE_AUTO_POSE
//                        )
//                )
//                .setHeadingInterpolation(toGateAfterOpen)
//                .setTValueConstraint(0.99)
//                .build();
//
//        // optimal coordinates trust. both the second intake and this path end at the same heading.
//        shootGate1 = follower.pathBuilder().addPath(
//                        new BezierCurve(
//                                PoseConstants.BLUE_GATE_AUTO_POSE,
//                                controlPoint2,
//                                controlPoint1,
//                                new Pose(60.000, 72.000)
//                        )
//                ).setTangentHeadingInterpolation()
//                .setReversed()
//                .build();
//
//        HeadingInterpolator toGate = HeadingInterpolator.piecewise(
//                new HeadingInterpolator.PiecewiseNode(
//                        0,
//                        0.75,
//                        HeadingInterpolator.linear(shootGate1.getFinalHeadingGoal(), PoseConstants.BLUE_GATE_AUTO_POSE.getHeading())
//                ),
//                new HeadingInterpolator.PiecewiseNode(
//                        0.75,
//                        1,
//                        HeadingInterpolator.constant(PoseConstants.BLUE_GATE_AUTO_POSE.getHeading())
//                )
//        );
//
//        intakeGate2 = follower.pathBuilder()
//                .addPath(
//                        new BezierCurve(
//                                new Pose(60.000, 72.000),
//                                new Pose(45, PoseConstants.BLUE_GATE_AUTO_POSE.getY()),
//                                PoseConstants.BLUE_GATE_AUTO_POSE
//                        )
//                )
//                .setHeadingInterpolation(toGate)
//                .setTValueConstraint(0.99)
//                .build();
//
//
//
//        openGate2 = follower.pathBuilder().addPath(
//                        new BezierCurve(
//                                PoseConstants.BLUE_GATE_AUTO_POSE,
//                                new Pose(31.806, 63.266),
//                                PoseConstants.BLUE_GATE_POSE
//                        )
//                ).setLinearHeadingInterpolation(Math.toRadians(148), Math.toRadians(180))
//                .build();
//
//
//        shootGate2 = follower.pathBuilder().addPath(
//                        new BezierLine(
//                                PoseConstants.BLUE_GATE_POSE,
//                                new Pose(60.000, 72.000)
//                        )
//                ).setConstantHeadingInterpolation(Math.toRadians(180))
//                .build();
//
//
//        intakeGate3 = follower.pathBuilder()
//                .addPath(
//                        new BezierCurve(
//                                new Pose(60.000, 72.000),
//                                new Pose(45, PoseConstants.BLUE_GATE_AUTO_POSE.getY()),
//                                PoseConstants.BLUE_GATE_AUTO_POSE
//                        )
//                )
//                .setHeadingInterpolation(toGateAfterOpen)
//                .setTValueConstraint(0.99)
//                .build();
//
//        // optimal coordinates trust. both the second intake and this path end at the same heading.
//        shootGate3 = follower.pathBuilder().addPath(
//                        new BezierCurve(
//                                PoseConstants.BLUE_GATE_AUTO_POSE,
//                                controlPoint2,
//                                controlPoint1,
//                                new Pose(60.000, 72.000)
//                        )
//                ).setTangentHeadingInterpolation()
//                .setReversed()
//                .build();
//
//        intakeThird = follower.pathBuilder().addPath(
//                        new BezierCurve(
//                                new Pose(60.000, 72.000),
//                                new Pose(50.000, 34.000),
//                                new Pose(15.000, 36.000)
//                        )
//                ).setTangentHeadingInterpolation()
//                .build();
//
//        shootThird = follower.pathBuilder().addPath(
//                new BezierLine(
//                        new Pose(15)
//                )
//        )
//
//
//        public static class Paths {
//            public PathChain Path1;
//            public PathChain Path2;
//            public PathChain Path3;
//            public PathChain Path4;
//            public PathChain Path5;
//            public PathChain Path6;
//            public PathChain Path7;
//            public PathChain Path8;
//            public PathChain Path9;
//
//            public Paths(Follower follower) {
//                Path1 = follower.pathBuilder().addPath(
//                                new BezierLine(
//                                        new Pose(30.000, 135.000),
//
//                                        new Pose(54.000, 90.000)
//                                )
//                        ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-110))
//
//                        .build();
//
//                Path2 = follower.pathBuilder().addPath(
//                                new BezierCurve(
//                                        new Pose(54.000, 90.000),
//                                        new Pose(58.053, 61.191),
//                                        new Pose(29.053, 46.160),
//                                        new Pose(13.100, 56.000)
//                                )
//                        ).setTangentHeadingInterpolation()
//
//                        .build();
//
//                Path3 = follower.pathBuilder().addPath(
//                                new BezierLine(
//                                        new Pose(13.100, 56.000),
//
//                                        new Pose(60.000, 72.000)
//                                )
//                        ).setConstantHeadingInterpolation(Math.toRadians(180))
//
//                        .build();
//
//                Path4 = follower.pathBuilder().addPath(
//                                new BezierLine(
//                                        new Pose(60.000, 72.000),
//
//                                        new Pose(13.000, 56.000)
//                                )
//                        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(148))
//
//                        .build();
//
//                Path5 = follower.pathBuilder().addPath(
//                                new BezierLine(
//                                        new Pose(13.000, 56.000),
//
//                                        new Pose(60.000, 72.000)
//                                )
//                        ).setConstantHeadingInterpolation(Math.toRadians(180))
//
//                        .build();
//
//                Path6 = follower.pathBuilder().addPath(
//                                new BezierCurve(
//                                        new Pose(60.000, 72.000),
//                                        new Pose(50.000, 34.000),
//                                        new Pose(15.000, 36.000)
//                                )
//                        ).setTangentHeadingInterpolation()
//
//                        .build();
//
//                Path7 = follower.pathBuilder().addPath(
//                                new BezierLine(
//                                        new Pose(15.000, 36.000),
//
//                                        new Pose(46.000, 9.000)
//                                )
//                        ).setConstantHeadingInterpolation(Math.toRadians(180))
//
//                        .build();
//
//                Path8 = follower.pathBuilder().addPath(
//                                new BezierLine(
//                                        new Pose(46.000, 9.000),
//
//                                        new Pose(12.000, 9.000)
//                                )
//                        ).setConstantHeadingInterpolation(Math.toRadians(180))
//
//                        .build();
//
//                Path9 = follower.pathBuilder().addPath(
//                                new BezierLine(
//                                        new Pose(12.000, 9.000),
//
//                                        new Pose(46.000, 9.000)
//                                )
//                        ).setConstantHeadingInterpolation(Math.toRadians(180))
//
//                        .build();
//            }
//        }
//
//
//        intakeGate4 = follower.pathBuilder()
//                .addPath(
//                        new BezierCurve(
//                                new Pose(60.000, 72.000),
//                                new Pose(45, PoseConstants.BLUE_GATE_AUTO_POSE.getY()),
//                                PoseConstants.BLUE_GATE_AUTO_POSE
//                        )
//                )
//                .setHeadingInterpolation(toGate)
//                .setTValueConstraint(0.99)
//                .build();
//
//        shootGate4 = follower.pathBuilder()
//                .addPath(
//                        new BezierCurve(
//                                PoseConstants.BLUE_GATE_AUTO_POSE,
//                                new Pose(48,70),
//                                new Pose(54, 84)
//                        )
//                )
//                .setLinearHeadingInterpolation(PoseConstants.BLUE_GATE_AUTO_POSE.getHeading(), Math.toRadians(180))
//                .build();
//
//        intakeFirst = follower.pathBuilder()
//                .addPath(
//                        new BezierLine(new Pose(54.000, 84.000), new Pose(18.000, 84.000))
//                )
//                .setConstantHeadingInterpolation(Math.toRadians(180))
//                .build();
//
//
//        shootFirst = follower.pathBuilder()
//                .addPath(
//                        new BezierLine(new Pose(18.000, 84.000), new Pose(48, 114))
//                )
//                .setTangentHeadingInterpolation()
//                .setReversed()
//                .build();
//    }
//
//    @Override
//    public void init() {
//        follower = Constants.createFollower(hardwareMap);
//        follower.usePredictiveBraking = true;
//        follower.setStartingPose(startPose);
//        robot = new AutonomousRobot(hardwareMap, Alliance.BLUE);
//        sotm2 = new SOTM(goalPose);
//        buildPaths();
//
//        stateMachine = new StateMachine(
//                new State()
//                        .onEnter(() -> {
//                            follower.setMaxPower(0.75);
//                            follower.followPath(shootPreloadIntakeSecond, true);
//                            robot.intake.state = Intake.IntakeState.INTAKE_SLOW;
//                        })
//                        .maxTime(1200),
//                new State()
//                        .onEnter(() -> robot.shootCommand.start())
//                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
//                new State()
//                        .onEnter(() -> {
//                            follower.setMaxPower(1);
//                            robot.intakeCommand.start();
//                        })
//                        .transition(new Transition(() -> !follower.isBusy())),
//                new State()
//                        .onEnter(() -> {
//                            follower.setMaxPower(0.65);
//                            follower.followPath(openGate1, true);
//                            robot.turret.setPDCoefficients(0.01, 0.0005);
//                            currentShootPose = new Pose(60, 72, Math.toRadians(180));
//                        })
//                        .transition(new Transition(() -> !follower.isBusy())),
//                new State()
//                        .onEnter(() -> follower.setMaxPower(1))
//                        .maxTime(100),
//                // second
//                new State()
//                        .onEnter(() -> {
//                            follower.followPath(shootSecond, true);
//                            isSOTMing = false;
//                            robot.intakeCommand.start();
//                        })
//                        .transition(new Transition(() -> follower.atParametricEnd())),
//                new State()
//                        .onEnter(() -> robot.shootCommand.start())
//                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
//                // gate cycle 1
//                new State()
//                        .onEnter(() -> {
//                            robot.intakeCommand.start();
//                            follower.followPath(intakeGate1, true);
//                        })
//                        .transition(new Transition(() -> !follower.isBusy())),
//                new State()
//                        .onEnter(() -> {
//                            follower.holdPoint(new BezierPoint(PoseConstants.BLUE_GATE_AUTO_POSE_IN), PoseConstants.BLUE_GATE_AUTO_POSE_IN.getHeading());
//                            currentShootPose = new Pose(60, 72, Math.toRadians(-144));
//                        })
//                        .minTime(600)
//                        .transition(new Transition(() -> robot.intake.intakeFull()))
//                        .maxTime(1700),
//                new State()
//                        .onEnter(() -> {
//                            follower.setMaxPower(1);
//                            follower.followPath(shootGate1, true);
//                        })
//                        .transition(new Transition(() -> follower.atParametricEnd())),
//                new State()
//                        .onEnter(() -> robot.shootCommand.start())
//                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
//                // gate cycle 2
//                new State()
//                        .onEnter(() -> {
//                            robot.intakeCommand.start();
//                            follower.followPath(intakeGate2, true);
//                        })
//                        .transition(new Transition(() -> !follower.isBusy())),
//                new State()
//                        .onEnter(() -> {
//                            follower.holdPoint(new BezierPoint(PoseConstants.BLUE_GATE_AUTO_POSE_IN), PoseConstants.BLUE_GATE_AUTO_POSE_IN.getHeading());
//                        })
//                        .minTime(600)
//                        .transition(new Transition(() -> robot.intake.intakeFull()))
//                        .maxTime(1700),
//                new State()
//                        .onEnter(() -> {
//                            follower.setMaxPower(0.65);
//                            follower.followPath(openGate2, true);
//                        })
//                        .transition(new Transition(() -> !follower.isBusy())),
//                new State()
//                        .maxTime(100),
//                new State()
//                        .onEnter(() -> {
//                            follower.setMaxPower(1);
//                            follower.followPath(shootGate2, true);
//                            currentShootPose = new Pose(60, 72, Math.toRadians(180));
//                        })
//                        .transition(new Transition(() -> follower.atParametricEnd())),
//                new State()
//                        .onEnter(() -> robot.shootCommand.start())
//                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
//                // gate cycle 3
//                new State()
//                        .onEnter(() -> {
//                            robot.intakeCommand.start();
//                            follower.followPath(intakeGate3, true);
//                        })
//                        .transition(new Transition(() -> !follower.isBusy())),
//                new State()
//                        .onEnter(() -> {
//                            follower.holdPoint(new BezierPoint(PoseConstants.BLUE_GATE_AUTO_POSE_IN), PoseConstants.BLUE_GATE_AUTO_POSE_IN.getHeading());
//                            currentShootPose = new Pose(60, 72, Math.toRadians(-144));
//                        })
//                        .minTime(600)
//                        .transition(new Transition(() -> robot.intake.intakeFull()))
//                        .maxTime(1700),
//                new State()
//                        .onEnter(() -> {
//                            follower.setMaxPower(1);
//                            follower.followPath(shootGate3, true);
//                        })
//                        .transition(new Transition(() -> follower.atParametricEnd())),
//                new State()
//                        .onEnter(() -> robot.shootCommand.start())
//                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
//
//                // gate cycle 4
//                new State()
//                        .onEnter(() -> {
//                            robot.intakeCommand.start();
//                            follower.followPath(intakeGate4, true);
//                        })
//                        .transition(new Transition(() -> !follower.isBusy())),
//                new State()
//                        .onEnter(() -> {
//                            follower.holdPoint(new BezierPoint(PoseConstants.BLUE_GATE_AUTO_POSE_IN), PoseConstants.BLUE_GATE_AUTO_POSE_IN.getHeading());
//                            currentShootPose = new Pose(54, 84, Math.toRadians(180));
//                        })
//                        .minTime(600)
//                        .transition(new Transition(() -> robot.intake.intakeFull()))
//                        .maxTime(1700),
//                new State()
//                        .onEnter(() -> {
//                            follower.setMaxPower(1);
//                            follower.followPath(shootGate4, true);
//                        })
//                        .transition(new Transition(() -> follower.atParametricEnd())),
//                new State()
//                        .onEnter(() -> robot.shootCommand.start())
//                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
//
//                // intake 1
//                new State()
//                        .onEnter(() -> {
//                            robot.intakeCommand.start();
//                            follower.followPath(intakeFirst, false);
//                            currentShootPose = new Pose(48, 114, Math.toRadians(-135));
//                        })
//                        .transition(new Transition(() -> follower.atParametricEnd())),
//                new State()
//                        .onEnter(() -> {
//                            follower.setMaxPower(1);
//                            follower.followPath(shootFirst, true);
//                        })
//                        .transition(new Transition(() -> follower.atParametricEnd())),
//                new State()
//                        .onEnter(() -> {
//                            robot.shootCommand.start();
//                            blackboard.put(RobotConstants.END_POSE_KEY, follower.getPose());
//                        })
//                        .onExit(() -> blackboard.put(RobotConstants.END_POSE_KEY, follower.getPose()))
//                        .transition(new Transition(() -> robot.shootCommand.isFinished()))
//        );
//
//        try {
//            sleep(500);
//        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
//        }
//        robot.initPositions();
//        robot.turret.resetEncoderWithAbsoluteReading();
//        robot.turret.setUseExternal(false);
//    }
//    @Override
//    public void loop() {
//        if (isSOTMing) {
//            double[] values = sotm2.calculateAzimuthThetaVelocityFRCBetter(follower.getPose(), follower.getVelocity(), follower.getAngularVelocity());
//            robot.setAzimuthThetaVelocity(new double[] {values[0], values[1], values[2]});
//            robot.turret.setFeedforward(values[3]);
//        } else {
//            double[] values = sotm2.calculateAzimuthThetaVelocityFRCBetter(currentShootPose, new Vector(), follower.getAngularVelocity());
//            robot.setAzimuthThetaVelocity(new double[] {values[0], values[1], values[2]});
//            robot.turret.setFeedforward(0);
//        }
//
//        stateMachine.update();
//        follower.update();
//
//        robot.update();
//        blackboard.put(RobotConstants.END_POSE_KEY, follower.getPose());
//        telemetry.update();
//    }
//
//    @Override
//    public void start() {
//        robot.shooter.state = Shooter.ShooterState.SHOOTER_ON;
//        stateMachine.start();
//        robot.start();
//    }
//
//    @Override
//    public void stop() {
//        blackboard.put(RobotConstants.END_POSE_KEY, follower.getPose());
//    }
//}


