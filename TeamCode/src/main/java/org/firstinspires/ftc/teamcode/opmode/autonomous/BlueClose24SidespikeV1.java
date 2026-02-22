package org.firstinspires.ftc.teamcode.opmode.autonomous;

// sidespike:
// shoot first, intake three (since they sometimes bounce away anyways), go to shoot pose, do an intakeGate path:
// this intakeGate path will simultaneously open gate while getting the three balls. set no deceleration on this one. it will also interpolate later at 0.8 instead of 0.75 maybe but not for now
// shoot

// by this time we have shot 9

// gate cycle: do 4 gate cycles and 1 area cycle

// by now we should have shot 21

// now we do the "pile" cycle

// i think this is the important auto to get working
// if this works, then we can just do 3rd intake
// and for the 24 solo, save 3rd spike and don't open gate several times

// we need: 24 open gate, 24 open gate w/3rd spike, 24 solo no open gate, 30 far
// tbh though idk if we can 2x open gate with 24 or if its needed

import static java.lang.Thread.sleep;

import android.util.Log;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.constants.PoseConstants;
import org.firstinspires.ftc.teamcode.robot.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.robots.AutonomousRobot;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.decodeutil.Alliance;
import org.firstinspires.ftc.teamcode.util.decodeutil.MathUtil;
import org.firstinspires.ftc.teamcode.util.decodeutil.SOTM;
import org.firstinspires.ftc.teamcode.util.fsm.State;
import org.firstinspires.ftc.teamcode.util.fsm.StateMachine;
import org.firstinspires.ftc.teamcode.util.fsm.Transition;

import java.util.ArrayList;

@Autonomous(name="Blue Close 24 Sidespike no vision", group="!")
public class BlueClose24SidespikeV1 extends OpMode {
    private Follower follower;
    private StateMachine stateMachine;
    private AutonomousRobot robot;
    private SOTM sotm2;
    private final Pose startPose = PoseConstants.BLUE_CLOSE_AUTO_POSE;
    private final Pose goalPose = PoseConstants.BLUE_GOAL_POSE;
    private final Pose shootPose = new Pose(60, 72, Math.toRadians(-144));
    private Pose currentShootPose = new Pose(60, 72, Math.toRadians(-144));
    private PathChain shootPreload, intakeFirst, shootFirst, intakeSecond, shootSecond, openGate, intakeGate1, shootGate1, intakeGate2, shootGate2, intakeGate3, shootGate3, intakeGate4, shootGate4, intakePile, shootPile;

    public void buildPaths() {
        double k1 = 10; // i actually forgot what these were check desmos
        double k2 = 20;
        double k3 = 20; // used for random cycle thingy, but we're gonna use vision later so its fine

        // idk abt this just check it again and do for control point 2
        Pose controlPoint1 = new Pose(shootPose.getX() + k1 * Math.cos(shootPose.getHeading()), shootPose.getY() + k1 * Math.sin(shootPose.getHeading()));
        Pose controlPoint2 = new Pose(PoseConstants.BLUE_GATE_AUTO_POSE.getX() - k2 * Math.cos(PoseConstants.BLUE_GATE_AUTO_POSE.getHeading()), PoseConstants.BLUE_GATE_AUTO_POSE.getY() - k2 * Math.sin(PoseConstants.BLUE_GATE_AUTO_POSE.getHeading()));

        shootPreload = follower.pathBuilder().addPath(
                new BezierLine(
                        startPose,
                        new Pose(24.000, 105.000)
                )
        ).setConstantHeadingInterpolation(startPose.getHeading()).build();

        intakeFirst = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(24.000, 105.000),
                        new Pose(24.000, 88.000)
                )
        ).setConstantHeadingInterpolation(startPose.getHeading()).build();

        shootFirst = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(24.000, 88.000),

                                new Pose(24.000, 105.000)
                        )
                ).setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        intakeSecond = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(24.000, 105.000),
                                new Pose(24.000, 65.000)
                        )
                ).setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        openGate = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(24.000, 65.000),
                                PoseConstants.BLUE_SIDE_GATE_POSE
                        )
                ).setConstantHeadingInterpolation(PoseConstants.BLUE_SIDE_GATE_POSE.getHeading())
                .setNoDeceleration()
                .build();


        shootSecond = follower.pathBuilder().addPath(
                        new BezierLine(
                                PoseConstants.BLUE_SIDE_GATE_POSE,
                                new Pose(60.000, 72.000)
                        )
                ).setLinearHeadingInterpolation(PoseConstants.BLUE_SIDE_GATE_POSE.getHeading(), Math.toRadians(-144))
                .build();

        shootGate1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                PoseConstants.BLUE_GATE_AUTO_POSE,
                                controlPoint2,
                                controlPoint1,
                                new Pose(60.000, 72.000)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        HeadingInterpolator toGate = HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(
                        0,
                        0.75,
                        HeadingInterpolator.linear(shootGate1.getFinalHeadingGoal(), PoseConstants.BLUE_GATE_AUTO_POSE.getHeading())
                ),
                new HeadingInterpolator.PiecewiseNode(
                        0.75,
                        1,
                        HeadingInterpolator.constant(PoseConstants.BLUE_GATE_AUTO_POSE.getHeading())
                )
        );

        intakeGate1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(60.000, 72.000),
                                new Pose(45, PoseConstants.BLUE_GATE_AUTO_POSE.getY()),
                                PoseConstants.BLUE_GATE_AUTO_POSE
                        )
                )
                .setHeadingInterpolation(toGate)
                .setTValueConstraint(0.99)
                .build();

        intakeGate2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(60.000, 72.000),
                                new Pose(45, PoseConstants.BLUE_GATE_AUTO_POSE.getY()),
                                PoseConstants.BLUE_GATE_AUTO_POSE
                        )
                )
                .setHeadingInterpolation(toGate)
                .setTValueConstraint(0.99)
                .build();

        shootGate2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                PoseConstants.BLUE_GATE_AUTO_POSE,
                                controlPoint2,
                                controlPoint1,
                                new Pose(60.000, 72.000)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        intakeGate3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(60.000, 72.000),
                                new Pose(45, PoseConstants.BLUE_GATE_AUTO_POSE.getY()),
                                PoseConstants.BLUE_GATE_AUTO_POSE
                        )
                )
                .setHeadingInterpolation(toGate)
                .setTValueConstraint(0.99)
                .build();

        shootGate3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                PoseConstants.BLUE_GATE_AUTO_POSE,
                                controlPoint2,
                                controlPoint1,
                                new Pose(60.000, 72.000)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        intakeGate4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(60.000, 72.000),
                                new Pose(45, PoseConstants.BLUE_GATE_AUTO_POSE.getY()),
                                PoseConstants.BLUE_GATE_AUTO_POSE
                        )
                )
                .setHeadingInterpolation(toGate)
                .setTValueConstraint(0.99)
                .build();

        shootGate4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                PoseConstants.BLUE_GATE_AUTO_POSE,
                                controlPoint2,
                                controlPoint1,
                                new Pose(60.000, 72.000)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        // btw: 60 + 20 cos (-144), 72 + 20 sin -144, 12-15 cos(180), 48-15 sin 180 = 48
        intakePile = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(60.000, 72.000),
                                new Pose(43.820, 60.244),
                                new Pose(27.000, 48.000),
                                new Pose(12.000, 48.000)
                        )
                ).setTangentHeadingInterpolation()
                .build();

        shootPile = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(12.000, 48.000),
                                new Pose(27.000, 48.000),
                                new Pose(43.820, 60.244),
                                new Pose(60.000, 72.000)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        // no park for now its easy, we'll just move forward a bit
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
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootPreload, true);
                            currentShootPose = new Pose(24, 105, Math.toRadians(-90));
                            robot.intake.state = Intake.IntakeState.INTAKE_SLOW;
                        })
                        .transition(new Transition(() -> !follower.isBusy() && robot.shooter.atTarget(40))),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                new State()
                        .onEnter(() -> {
                            follower.followPath(intakeFirst, true);
                            robot.intakeCommand.start();
                        })
                        .transition(new Transition(() -> follower.atParametricEnd())),
                new State()
                        .onEnter(() -> follower.followPath(shootFirst, true))
                        .transition(new Transition(() -> follower.atParametricEnd())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                new State()
                        .onEnter(() -> {
                            follower.followPath(intakeSecond, true);
                            currentShootPose = new Pose(60, 72, Math.toRadians(-144));
                            robot.intakeCommand.start();
                        })
                        .transition(new Transition(() -> follower.atParametricEnd())),
                new State()
                        .onEnter(() -> {
                            follower.followPath(openGate, true);
                        })
                        .transition(new Transition(() -> follower.atParametricEnd())),
                new State()
                        .maxTime(100),
                new State()
                        .onEnter(() -> follower.followPath(shootSecond, true))
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
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            follower.holdPoint(new BezierPoint(PoseConstants.BLUE_GATE_AUTO_POSE_IN), PoseConstants.BLUE_GATE_AUTO_POSE_IN.getHeading());
                        })
                        .minTime(600)
                        .transition(new Transition(() -> robot.intake.intakeFull()))
                        .maxTime(1700),
                new State()
                        .onEnter(() -> {
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
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            follower.holdPoint(new BezierPoint(PoseConstants.BLUE_GATE_AUTO_POSE_IN), PoseConstants.BLUE_GATE_AUTO_POSE_IN.getHeading());
                        })
                        .minTime(600)
                        .transition(new Transition(() -> robot.intake.intakeFull()))
                        .maxTime(1700),
                new State()
                        .onEnter(() -> {
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
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            follower.holdPoint(new BezierPoint(PoseConstants.BLUE_GATE_AUTO_POSE_IN), PoseConstants.BLUE_GATE_AUTO_POSE_IN.getHeading());
                        })
                        .minTime(600)
                        .transition(new Transition(() -> robot.intake.intakeFull()))
                        .maxTime(1700),
                new State()
                        .onEnter(() -> {
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
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            follower.holdPoint(new BezierPoint(PoseConstants.BLUE_GATE_AUTO_POSE_IN), PoseConstants.BLUE_GATE_AUTO_POSE_IN.getHeading());
                        })
                        .minTime(600)
                        .transition(new Transition(() -> robot.intake.intakeFull()))
                        .maxTime(1700),
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootGate4, true);
                        })
                        .transition(new Transition(() -> follower.atParametricEnd())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),

                // intake pile
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.followPath(intakePile, false);
                        })
                        .transition(new Transition(() -> follower.atParametricEnd())),
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootPile, true);
                        })
                        .transition(new Transition(() -> follower.atParametricEnd())),
                new State()
                        .onEnter(() -> {
                            robot.shootCommand.start();
                        })
                        .transition(new Transition(() -> robot.shootCommand.isFinished()))
        );

        try {
            sleep(500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        robot.initPositions();
        robot.turret.resetEncoderWithAbsoluteReading();
        robot.turret.setUseExternal(false);
        // TODO: remove?
        robot.turret.setPDCoefficients(0.008, 0.0002);
    }
    @Override
    public void loop() {
        // TODO: test with sotm on
        // ALSO TODO: change the position to be a bit lower as we are slightly over the line
        double[] values = sotm2.calculateAzimuthThetaVelocityFRCBetter(currentShootPose, new Vector(), follower.getAngularVelocity());
        robot.setAzimuthThetaVelocity(new double[] {values[0], values[1], values[2]});
        robot.turret.setFeedforward(0);

        stateMachine.update();
        follower.update();

        robot.update();
        blackboard.put(RobotConstants.END_POSE_KEY, follower.getPose());
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

