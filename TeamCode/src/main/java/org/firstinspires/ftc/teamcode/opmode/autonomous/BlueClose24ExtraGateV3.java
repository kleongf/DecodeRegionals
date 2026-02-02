package org.firstinspires.ftc.teamcode.opmode.autonomous;
// try the custom heading interpolator to move tangentially to and from the goal.
// alternatively just be straight until a point then turn

// this should increase speed.
// leave goal when 3 balls
// sotm last shot (and use a better point)

// sotm on all the time?
// maybe use the zoneutil? when in zone, shoot, start next path right away? maybe too risky
// just remove all shooting time with sotm

// todo:
// make shoot at beginning
// better linear pathing to and from gate
// add back the hold point
// calculate the tangent point derivative so that we know the linear pathing

import static java.lang.Thread.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.constants.PoseConstants;
import org.firstinspires.ftc.teamcode.robot.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.robots.AutonomousRobot;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.decodeutil.Alliance;
import org.firstinspires.ftc.teamcode.util.decodeutil.SOTM;
import org.firstinspires.ftc.teamcode.util.fsm.State;
import org.firstinspires.ftc.teamcode.util.fsm.StateMachine;
import org.firstinspires.ftc.teamcode.util.fsm.Transition;
@Autonomous(name="Blue Close 24 Extra Gate, v3 optimized pathing and predictive", group="!")
public class BlueClose24ExtraGateV3 extends OpMode {
    private Follower follower;
    private StateMachine stateMachine;
    private AutonomousRobot robot;
    private SOTM sotm2;
    // Mathematically the correct pose based on the robot's length and width.
    private final Pose startPose = new Pose(29.4664028463, 133.308918506, Math.toRadians(144));
    private final Pose goalPose = PoseConstants.BLUE_GOAL_POSE;
    private PathChain shootPreload, intakeSecond, shootSecond, intakeGate1, shootGate1, intakeGate2, shootGate2, intakeGate3, shootGate3, intakeGate4, shootGate4, intakeGate5, shootGate5, intakeFirst, shootFirst;

    public void buildPaths() {
        intakeSecond = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(60.000, 75.000),
                                new Pose(50.000, 48.000),
                                new Pose(15.000, 60.000)
                        )
                ).setTangentHeadingInterpolation()
                .build();

        shootPreload = follower.pathBuilder().addPath(
                        new BezierLine(
                                startPose,
                                new Pose(60, 75)
                        )
                )
                // this interpolates it so that we are at the correct heading already at t=0
                .setLinearHeadingInterpolation(startPose.getHeading(), intakeSecond.getHeadingGoal(new PathChain.PathT(0, 0)))
                .build();

        // Mathematically proven to end at the correct angle
        intakeGate1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(60.000, 75.000),
                                new Pose(24.721, 52.051),
                                new Pose(12.000, 60.000)
                        )
                ).setTangentHeadingInterpolation()
                .build();

        // TODO: make linear heading interpolation so we don't jolt at the start of next path
        shootSecond = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(15.000, 60.000),
                                new Pose(60.000, 75.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), intakeGate1.getHeadingGoal(new PathChain.PathT(0, 0)))
                .setReversed()
                .build();

        intakeGate1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(60.000, 75.000),
                                new Pose(24.721, 52.051),
                                new Pose(12.000, 60.000)
                        )
                ).setTangentHeadingInterpolation()
                .setBrakingStart(2)
                .setTValueConstraint(0.99)
                .build();

        shootGate1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(60.000, 75.000),
                                new Pose(24.721, 52.051),
                                new Pose(12.000, 60.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        intakeGate2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(60.000, 75.000),
                                new Pose(24.721, 52.051),
                                new Pose(12.000, 60.000)
                        )
                ).setTangentHeadingInterpolation()
                .setBrakingStart(2)
                .setTValueConstraint(0.99)
                .build();

        shootGate2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(60.000, 75.000),
                                new Pose(24.721, 52.051),
                                new Pose(12.000, 60.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        intakeGate3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(60.000, 75.000),
                                new Pose(24.721, 52.051),
                                new Pose(12.000, 60.000)
                        )
                ).setTangentHeadingInterpolation()
                .setBrakingStart(2)
                .setTValueConstraint(0.99)
                .build();

        shootGate3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(60.000, 75.000),
                                new Pose(24.721, 52.051),
                                new Pose(12.000, 60.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        intakeGate4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(60.000, 75.000),
                                new Pose(24.721, 52.051),
                                new Pose(12.000, 60.000)
                        )
                ).setTangentHeadingInterpolation()
                .setBrakingStart(2)
                .setTValueConstraint(0.99)
                .build();

        shootGate4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(60.000, 75.000),
                                new Pose(24.721, 52.051),
                                new Pose(12.000, 60.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        intakeGate5 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(60.000, 75.000),
                                new Pose(24.721, 52.051),
                                new Pose(12.000, 60.000)
                        )
                ).setTangentHeadingInterpolation()
                .setBrakingStart(2)
                .setTValueConstraint(0.99)
                .build();

        shootGate5 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(60.000, 75.000),
                                new Pose(24.721, 52.051),
                                new Pose(12.000, 60.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        shootGate5 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(12.000, 60.000),
                                new Pose(50,60),
                                new Pose(54, 84)
                        )
                )
                .setLinearHeadingInterpolation(PoseConstants.BLUE_SHOOT_AUTO_POSE.getHeading(), Math.toRadians(180))
                .build();

        intakeFirst = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(54.000, 84.000), new Pose(18.000, 84.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();


        shootFirst = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(18.000, 84.000), new Pose(48, 114))
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.usePredictiveBraking = true;
        follower.setStartingPose(startPose);
        robot = new AutonomousRobot(hardwareMap, Alliance.BLUE);
        sotm2 = new SOTM(goalPose);
        buildPaths();

        stateMachine = new StateMachine(
                // preload
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootPreload, true);
                            robot.intake.state = Intake.IntakeState.INTAKE_SLOW;
                        })
                        .transition(new Transition(() -> follower.atParametricEnd())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // intake second
                new State()
                        .onEnter(() -> {
                            follower.followPath(intakeSecond, true);
                            robot.intakeCommand.start();
                        })
                        .transition(new Transition(() -> follower.atParametricEnd())),
                // second
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootSecond, true);
                        })
                        .transition(new Transition(() -> follower.atParametricEnd())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // gate cycle 1
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.followPath(intakeGate1, true);
                        })
                        .transition(new Transition(() -> follower.atParametricEnd())),
                new State()
                        .onEnter(() -> {
                            follower.holdPoint(new BezierPoint(PoseConstants.BLUE_GATE_AUTO_POSE_IN), PoseConstants.BLUE_GATE_AUTO_POSE_IN.getHeading());
                        })
                        // currently setting all to 1000. if it is possible at 1s then it is possible. if not i should prob give up.
                        .maxTime(1000),
                new State()
                        .onEnter(() -> {
                            follower.setMaxPower(1);
                            follower.followPath(shootGate1, true);
                        })
                        .transition(new Transition(() -> follower.atParametricEnd())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // gate cycle 2
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.followPath(intakeGate2, true);
                        })
                        .transition(new Transition(() -> follower.atParametricEnd())),
                new State()
                        .onEnter(() -> {
                            follower.holdPoint(new BezierPoint(PoseConstants.BLUE_GATE_AUTO_POSE_IN), PoseConstants.BLUE_GATE_AUTO_POSE_IN.getHeading());
                        })
                        .maxTime(1000),
                new State()
                        .onEnter(() -> {
                            follower.setMaxPower(1);
                            follower.followPath(shootGate2, true);
                        })
                        .transition(new Transition(() -> follower.atParametricEnd())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // gate cycle 3
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.followPath(intakeGate3, true);
                        })
                        .transition(new Transition(() -> follower.atParametricEnd())),
                new State()
                        .onEnter(() -> {
                            follower.holdPoint(new BezierPoint(PoseConstants.BLUE_GATE_AUTO_POSE_IN), PoseConstants.BLUE_GATE_AUTO_POSE_IN.getHeading());
                        })
//                        .minTime(600)
//                        .transition(new Transition(() -> robot.intake.intakeFull()))
                        .maxTime(1000),
                new State()
                        .onEnter(() -> {
                            follower.setMaxPower(1);
                            follower.followPath(shootGate3, true);
                        })
                        .transition(new Transition(() -> follower.atParametricEnd())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),

                // gate cycle 4
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.followPath(intakeGate4, true);
                        })
                        .transition(new Transition(() -> follower.atParametricEnd())),
                new State()
                        .onEnter(() -> {
                            follower.holdPoint(new BezierPoint(PoseConstants.BLUE_GATE_AUTO_POSE_IN), PoseConstants.BLUE_GATE_AUTO_POSE_IN.getHeading());
                        })
//                        .minTime(600)
//                        .transition(new Transition(() -> robot.intake.intakeFull()))
                        .maxTime(1000),
                new State()
                        .onEnter(() -> {
                            follower.setMaxPower(1);
                            follower.followPath(shootGate4, true);
                        })
                        .transition(new Transition(() -> follower.atParametricEnd())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),

                // gate cycle 5
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.followPath(intakeGate5, true);
                        })
                        .transition(new Transition(() -> follower.atParametricEnd())),
                new State()
                        .onEnter(() -> {
                            follower.holdPoint(new BezierPoint(PoseConstants.BLUE_GATE_AUTO_POSE_IN), PoseConstants.BLUE_GATE_AUTO_POSE_IN.getHeading());
                        })
//                        .minTime(600)
//                        .transition(new Transition(() -> robot.intake.intakeFull()))
                        .maxTime(1000),
                new State()
                        .onEnter(() -> {
                            follower.setMaxPower(1);
                            follower.followPath(shootGate5, true);
                        })
                        .transition(new Transition(() -> follower.atParametricEnd())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // intake 1
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.followPath(intakeFirst, false);
                        })
                        .transition(new Transition(() -> follower.atParametricEnd())),
                new State()
                        .onEnter(() -> {
                            follower.setMaxPower(1);
                            follower.followPath(shootFirst, true);
                        })
                        .transition(new Transition(() -> follower.atParametricEnd())),
                new State()
                        .onEnter(() -> {
                            robot.shootCommand.start();
                            blackboard.put(RobotConstants.END_POSE_KEY, follower.getPose());
                        })
                        .onExit(() -> blackboard.put(RobotConstants.END_POSE_KEY, follower.getPose()))
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
        values = sotm2.calculateAzimuthThetaVelocityFRC(follower.getPose(), follower.getVelocity());
        robot.setAzimuthThetaVelocity(values);
//        if (follower.getPose() != null && follower.getVelocity() != null) {
//            values = sotm2.calculateAzimuthThetaVelocity(follower.getPose(), follower.getVelocity());
//            robot.setAzimuthThetaVelocity(values);
//        }

        stateMachine.update();
        follower.update();
        robot.update();
        telemetry.update();
    }

    @Override
    public void start() {
        robot.shooter.state = Shooter.ShooterState.SHOOTER_ON;
        stateMachine.start();
        robot.start();
    }

    @Override
    public void stop() {
        blackboard.put(RobotConstants.END_POSE_KEY, follower.getPose());
    }
}
