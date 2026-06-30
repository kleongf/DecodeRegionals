package org.firstinspires.ftc.teamcode.decode2026.opmode.autonomous;

import static java.lang.Thread.sleep;

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

import org.firstinspires.ftc.teamcode.decode2026.CurrentRobot;
import org.firstinspires.ftc.teamcode.decode2026.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.decode2026.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.decode2026.constants.ShootingConstants;
import org.firstinspires.ftc.teamcode.decode2026.subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.decodeutil.Alliance;
import org.firstinspires.ftc.teamcode.util.decodeutil.SOTMUtil;
import org.firstinspires.ftc.teamcode.util.fsm.State;
import org.firstinspires.ftc.teamcode.util.fsm.StateMachine;
import org.firstinspires.ftc.teamcode.util.fsm.Transition;

// last one is going to be a pile cycle, it will always be a pile cycle


@Autonomous(name="Blue Close 27 Vision", group="!")
public class BlueClose27Vision extends OpMode {
    private Pose lockedPose = new Pose();
    private boolean lockShooter = true;
    private double turretOffset = 0;
    private final double pathSOTMTValue = 0.76;
    private Follower follower;
    private StateMachine stateMachine;
    private CurrentRobot robot;
    private SOTMUtil sotm;
    private Intake.DetectionState prevState;
    private PathChain shootPreload, intakeFirst, shootFirst, intakeSecond, shootSecond, intakeGate, shootGate, intakePile, shootPile;
    private int numGateCyclesCompleted = 0;
    private final int numGateCyclesTarget = 5;
    private final double pileCycleMinY = 12; // adjust based on partner
    private final double pileCycleMaxY = 50;

    // important: to flip any pose, use Flipper.flip(Pose)
    public void buildPaths() {
        shootPreload = follower.pathBuilder().addPath(
                new BezierLine(
                        FieldConstants.BLUE_CLOSE_START_AUTO_POSE,
                        new Pose(32, 108.000)
                )
        ).setConstantHeadingInterpolation(FieldConstants.BLUE_CLOSE_START_AUTO_POSE.getHeading()).build();

        intakeFirst = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(32, 108.000),
                        new Pose(24, 97),
                        new Pose(24, 92),
                        new Pose(23.500, 83.000)
                )
        ).setConstantHeadingInterpolation(FieldConstants.BLUE_CLOSE_START_AUTO_POSE.getHeading()).build();

        shootFirst = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(23.500, 83.000),
                                new Pose(57.000, 76.000)
                        )
                )
                // optimal angle trust defined by deriv of curve
                .setConstantHeadingInterpolation(Math.toRadians(-130.37))
                .build();

        intakeSecond = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(57.000, 76.000),
                                new Pose(46.741, 65.108),
                                new Pose(31.688, 59.731),
                                new Pose(16.000, 63.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        shootSecond = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(24.00, 67.000),
                                new Pose(57.000, 76.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-160))
                .build();

        HeadingInterpolator toGate = HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(
                        0,
                        0.25,
                        HeadingInterpolator.constant(Math.toRadians(-160))
                ),
                new HeadingInterpolator.PiecewiseNode(
                        0.25,
                        1,
                        HeadingInterpolator.constant(FieldConstants.BLUE_GATE_AUTO_POSE_27.getHeading())
                )
        );

        intakeGate = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(57.000, 76.000),
                                FieldConstants.BLUE_GATE_AUTO_POSE_27
                        )
                )
                .setHeadingInterpolation(toGate)
                .setTValueConstraint(0.99)
                .build();

        shootGate = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                FieldConstants.BLUE_GATE_AUTO_POSE_27,
                                new Pose(57, 76)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        HeadingInterpolator pileCycle = HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(
                        0,
                        0.5,
                        HeadingInterpolator.constant(Math.toRadians(-130))
                ),
                new HeadingInterpolator.PiecewiseNode(
                        0.5,
                        1,
                        HeadingInterpolator.linear(Math.toRadians(-130), Math.toRadians(-90))
                )
        );

        intakePile = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(57.000, 76.000),
                                new Pose(12, pileCycleMinY)
                        )
                )
                .setHeadingInterpolation(pileCycle)
                .build();

        shootPile = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(12.000, pileCycleMinY),
                                new Pose(58.000, 120.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.usePredictiveBraking = true;
        follower.setMaxPower(1);
        follower.setStartingPose(FieldConstants.BLUE_CLOSE_START_AUTO_POSE);
        robot = new CurrentRobot(hardwareMap);
        sotm = new SOTMUtil(FieldConstants.BLUE_GOAL_POSE);
        lockedPose = new Pose(32, 108, FieldConstants.BLUE_CLOSE_START_AUTO_POSE.getHeading());
        turretOffset = Math.toRadians(-4);
        buildPaths();

        stateMachine = new StateMachine(
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootPreload, 0.7,true);
                            robot.prepareShootCommand.start();
                        })
                        .transition(new Transition(() -> follower.atParametricEnd() && robot.shooter.atTarget(40))),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                new State()
                        .onEnter(() -> {
                            follower.setMaxPower(1);
                            follower.followPath(intakeFirst, true);
                            robot.intakeCommand.start();
                            //ShootingConstants.tofMultiplier = 0.5;
                            lockedPose = new Pose(57, 76);
                            turretOffset = Math.toRadians(0);
                            lockShooter = false;
                        })
                        .transition(new Transition(() -> follower.getCurrentTValue() > pathSOTMTValue)),
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootFirst, true);
                        })
                        .transition(new Transition(() -> follower.getCurrentTValue() > pathSOTMTValue)),
                new State()
                        .onEnter(() -> {
                            robot.shootCommand.start();
                        })
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                new State()
                        .onEnter(() -> {
                            follower.followPath(intakeSecond, true);
                            robot.intakeCommand.start();
                        })
                        .transition(new Transition(() -> follower.getCurrentTValue() > 0.85)),
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootSecond, true);
                        })
                        .transition(new Transition(() -> follower.getCurrentTValue() > pathSOTMTValue)),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // gate cycle 1
                new State("gateCycleStart")
                        .onEnter(() -> {
                            numGateCyclesCompleted++;
                            robot.intakeCommand.start();
                            follower.followPath(intakeGate, 0.8, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            follower.holdPoint(new BezierPoint(FieldConstants.BLUE_GATE_AUTO_POSE_IN), FieldConstants.BLUE_GATE_AUTO_POSE_IN.getHeading());
                        })
                        .minTime(600)
                        .transition(new Transition(() -> robot.intake.isFull))
                        .maxTime(1900),
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootGate, true);
                            robot.prepareShootCommandLonger.start();
                        })
                        .transition(new Transition(() -> follower.getCurrentTValue() > pathSOTMTValue)),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .maxTime(100),
                new State()
                        .transition(new Transition(() -> numGateCyclesCompleted < numGateCyclesTarget, "gateCycleStart"))
                        .transition(new Transition(() -> numGateCyclesCompleted >= numGateCyclesTarget, "pileCycle")),
                // pile intake
                new State("pileCycle")
                        .maxTime(300), // wait a bit to finish shooting before starting path
                new State()
                        .onEnter(() -> {
                            double bestY = robot.artifactVision.findBestYPosition(follower.getPose(), pileCycleMinY, pileCycleMaxY);
                            if (bestY != -1) {
                                intakePile = follower.pathBuilder()
                                        .addPath(
                                                new BezierCurve(
                                                        follower.getPose(),
                                                        new Pose(28, pileCycleMinY),
                                                        new Pose(12, pileCycleMinY)
                                                )
                                        )
                                        .setTangentHeadingInterpolation()
                                        .build();

                                shootPile = follower.pathBuilder()
                                        .addPath(
                                                new BezierLine(
                                                        new Pose(12, pileCycleMinY),
                                                        new Pose(58.000, 120.000)
                                                )
                                        )
                                        .setTangentHeadingInterpolation()
                                        .setReversed()
                                        .build();
                            }
                            robot.intakeCommand.start();
                            follower.followPath(intakePile, false);
                        })
                        .maxTime(2000) // so we don't get stuck
                        .transition(new Transition(() -> follower.getCurrentTValue() > 0.9)),
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootPile, true);
                            ShootingConstants.tofMultiplier = 0.93;
                        })
                        .maxTime(300)
                        .transition(new Transition(() -> robot.intake.isFull)),
                new State()
                        .onEnter(() -> robot.prepareShootCommandLonger.start())
                        .transition(new Transition(() -> follower.getCurrentTValue() > 0.6)),
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
        robot.reset();
    }
    @Override
    public void loop() {
        ShootingConstants.ShooterOutputs shooterOutputs;

        if (lockShooter) {
            shooterOutputs = sotm.calculateShooterOutputsTele(lockedPose, new Vector(), new Vector(), 0, RobotConstants.dt, Alliance.BLUE);
        } else {
            shooterOutputs =
                    RobotConstants.useShootOnTheMove ?
                            sotm.calculateShooterOutputsTele(follower.getPose(), follower.getVelocity(), follower.getAcceleration(), follower.getAngularVelocity(), RobotConstants.dt, Alliance.BLUE) :
                            sotm.calculateShooterOutputsTele(follower.getPose(), new Vector(), new Vector(), 0, RobotConstants.dt, Alliance.BLUE);
        }

        robot.shooter.wantedVelocity = shooterOutputs.wheelVelocity;
        robot.shooter.wantedAcceleration = shooterOutputs.wheelFeedforward;
        robot.shooter.wantedPitch = shooterOutputs.hoodAngle;
        robot.turret.wantedAngle = shooterOutputs.turretAngle + turretOffset;
        robot.turret.wantedAngularVelocity = shooterOutputs.turretFeedforward;

        if ((robot.intake.detectionState == Intake.DetectionState.THIRD_TRIGGERED && prevState == Intake.DetectionState.SECOND_TRIGGERED)) {
            robot.ledIndicator.indicateIntakeFull();
        }
        prevState = robot.intake.detectionState;

        stateMachine.update();
        follower.update();
        robot.update();
        blackboard.put(FieldConstants.END_POSE_KEY, follower.getPose());
    }

    @Override
    public void start() {
        stateMachine.start();
        robot.start();
    }

    @Override
    public void stop() {
        blackboard.put(FieldConstants.END_POSE_KEY, follower.getPose());
    }
}
