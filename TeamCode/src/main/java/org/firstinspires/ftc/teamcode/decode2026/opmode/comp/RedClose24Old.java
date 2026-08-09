package org.firstinspires.ftc.teamcode.decode2026.opmode.comp;

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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.decode2026.CurrentRobot;
import org.firstinspires.ftc.teamcode.decode2026.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.decode2026.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.decode2026.constants.ShootingConstants;
import org.firstinspires.ftc.teamcode.decode2026.subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.decodeutil.Alliance;
import org.firstinspires.ftc.teamcode.util.decodeutil.Flipper;
import org.firstinspires.ftc.teamcode.util.decodeutil.SOTMUtil;
import org.firstinspires.ftc.teamcode.util.fsm.State;
import org.firstinspires.ftc.teamcode.util.fsm.StateMachine;
import org.firstinspires.ftc.teamcode.util.fsm.Transition;

// last one is going to be a pile cycle, it will always be a pile cycle
@Disabled
@Autonomous(name="Red Close 24 Old", group="!")
public class RedClose24Old extends OpMode {
    private Pose lockedPose = new Pose();
    private boolean lockShooter = true;
    private double turretOffset = 0;
    private Follower follower;
    private StateMachine stateMachine;
    private CurrentRobot robot;
    private SOTMUtil sotm;
    private Intake.DetectionState prevState;
    private PathChain shootPreload, intakeFirst, shootFirst, shootFirstOpenGate, intakeSecondOpenGate, intakeSecond, shootSecond, shootSecondOpenGate, intakeGate, shootGate, park;
    private int numGateCyclesCompleted = 0;
    private final int numGateCyclesWanted = 5;
    private boolean openGate = true;

    // important: to flip any pose, use Flipper.flip(Pose)
    public void buildPaths() {
        shootPreload = follower.pathBuilder().addPath(
                new BezierLine(
                        FieldConstants.RED_CLOSE_START_AUTO_POSE,
                        Flipper.flip(new Pose(32, 108.000))
                )
        ).setConstantHeadingInterpolation(FieldConstants.RED_CLOSE_START_AUTO_POSE.getHeading()).build();

        intakeFirst = follower.pathBuilder().addPath(
                new BezierCurve(
                        Flipper.flip(new Pose(32, 108.000)),
                        Flipper.flip(new Pose(24, 97)),
                        Flipper.flip(new Pose(24, 92)),
                        Flipper.flip(new Pose(23.500, 83.000))
                )
        ).setConstantHeadingInterpolation(FieldConstants.RED_CLOSE_START_AUTO_POSE.getHeading()).build();


        shootFirstOpenGate = follower.pathBuilder().addPath(
                        new BezierLine(
                                Flipper.flip(new Pose(23.500, 83.000)),
                                Flipper.flip(new Pose(57.000, 77.000))
                        )
                )
                // optimal angle trust defined by deriv of curve
                .setConstantHeadingInterpolation(Flipper.flipAngle(Math.toRadians(-130.37)))
                .build();

        shootFirst = follower.pathBuilder().addPath(
                        new BezierLine(
                                Flipper.flip(new Pose(23.500, 83.000)),
                                Flipper.flip(new Pose(32, 108))
                        )
                )
                .setConstantHeadingInterpolation(FieldConstants.RED_CLOSE_START_AUTO_POSE.getHeading())
                .build();



        intakeSecondOpenGate = follower.pathBuilder().addPath(
                        new BezierCurve(
                                Flipper.flip(new Pose(57.000, 77.000)),
                                Flipper.flip(new Pose(46.741, 65.108)),
                                Flipper.flip(new Pose(31.688, 59.731)),
                                Flipper.flip(new Pose(16.000, 63.000))
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        intakeSecond = follower.pathBuilder().addPath(
                        new BezierCurve(
                                Flipper.flip(new Pose(32.00, 108.000)),
                                Flipper.flip(new Pose(22.5, 82.000)),
                                Flipper.flip(new Pose(22.5, 72.000)),
                                Flipper.flip(new Pose(24.5, 62.000))
                        )
                ).setConstantHeadingInterpolation(FieldConstants.RED_CLOSE_START_AUTO_POSE.getHeading())
                .build();

        shootSecondOpenGate = follower.pathBuilder().addPath(
                        new BezierLine(
                                Flipper.flip(new Pose(24.00, 67.000)),
                                Flipper.flip(new Pose(57.000, 77.000))
                        )
                )
                .setConstantHeadingInterpolation(Flipper.flipAngle(Math.toRadians(-160)))
                .build();

        shootSecond = follower.pathBuilder().addPath(
                        new BezierLine(
                                Flipper.flip(new Pose(24.500, 62.000)),
                                Flipper.flip(new Pose(57.000, 77.000))
                        )
                ).setConstantHeadingInterpolation(Flipper.flipAngle(Math.toRadians(-160)))
                .build();


        HeadingInterpolator toGate = HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(
                        0,
                        0.25,
                        HeadingInterpolator.constant(Flipper.flipAngle(Math.toRadians(-160)))
                ),
                new HeadingInterpolator.PiecewiseNode(
                        0.25,
                        1,
                        HeadingInterpolator.constant(FieldConstants.RED_GATE_AUTO_POSE.getHeading())
                )
        );

        intakeGate = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                Flipper.flip(new Pose(57.000, 77.000)),
                                FieldConstants.RED_GATE_AUTO_POSE
                        )
                )
                .setHeadingInterpolation(toGate)
                .setTValueConstraint(0.95)
                .build();

        shootGate = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                FieldConstants.RED_GATE_AUTO_POSE,
                                Flipper.flip(new Pose(56, 75))
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        park = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                Flipper.flip(new Pose(56.000, 75.000)),
                                Flipper.flip(new Pose(40.000, 78.000))
                        )
                )
                .setLinearHeadingInterpolation(Flipper.flipAngle(Math.toRadians(-160)), Flipper.flipAngle(Math.toRadians(180)))
                .build();
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.usePredictiveBraking = true;
        follower.setMaxPower(1);
        follower.setStartingPose(FieldConstants.RED_CLOSE_START_AUTO_POSE);
        robot = new CurrentRobot(hardwareMap);
        sotm = new SOTMUtil(FieldConstants.RED_GOAL_POSE);
        lockedPose = Flipper.flip(new Pose(32, 108, FieldConstants.RED_CLOSE_START_AUTO_POSE.getHeading()));
        turretOffset = Math.toRadians(0);
        buildPaths();

        stateMachine = new StateMachine(
                new State()
                        .onEnter(() -> {
                            follower.setMaxPower(0.7);
                            follower.followPath(shootPreload, true);
                            robot.prepareShootCommand.start();

                            // setting the paths to the correct ones
                            if (openGate) {
                                shootFirst = shootFirstOpenGate;
                                intakeSecond = intakeSecondOpenGate;
                                shootSecond = shootSecondOpenGate;
                            }
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
                            ShootingConstants.tofMultiplier = 0.35;
                        })
                        .maxTime(25),
                new State()
                        .onEnter(() -> {
                            if (openGate) {
                                lockShooter = true;
                                lockedPose = Flipper.flip(new Pose(57,76, Math.toRadians(-130.37)));
                                turretOffset = Math.toRadians(0);
                            }
                            else{
                                lockShooter = false;
                            }
                        })
                        .transition(new Transition(() -> follower.getCurrentTValue() > 0.75)),
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootFirst, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                new State()
                        .onEnter(() -> {
                            follower.followPath(intakeSecond, true);
                            robot.intakeCommand.start();
                        })
                        .maxTime(50),
                new State()
                        .onEnter(() -> {
                            lockShooter = true;
                            lockedPose = new Pose(57,76, Math.toRadians(-160));
                            turretOffset = Math.toRadians(3);
                        })
                        .transition(new Transition(() -> follower.getCurrentTValue() > 0.85)),
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootSecond, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // gate cycle 1
                new State("gateCycleStart")
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.followPath(intakeGate, 0.8, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            follower.holdPoint(new BezierPoint(FieldConstants.RED_GATE_AUTO_POSE_IN), FieldConstants.RED_GATE_AUTO_POSE_IN.getHeading());
                        })
                        .minTime(600)
                        .transition(new Transition(() -> robot.intake.isFull))
                        .maxTime(2200),
                new State()
                        .onEnter(() -> {
                            turretOffset = Math.toRadians(0);
                            follower.followPath(shootGate, true);
                            robot.prepareShootCommandLonger.start();
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            robot.shootCommand.start();
                            numGateCyclesCompleted++;
                        })
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                new State()
                        .transition(new Transition(() -> numGateCyclesCompleted < numGateCyclesWanted, "gateCycleStart"))
                        .transition(new Transition(() -> numGateCyclesCompleted >= numGateCyclesWanted, "park")),
                // park
                new State("park")
                        .onEnter(() -> follower.followPath(park, true))
                        .transition(new Transition(() -> !follower.isBusy()))

        );

        try {
            sleep(500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        robot.reset();
    }

    @Override
    public void init_loop() {
        telemetry.addLine("Press dpad left/right buttons to configure opening gate.");
        telemetry.addData("Open Gate", openGate);
        if (gamepad1.dpadRightWasPressed()) {
            openGate = true;
        }
        if (gamepad1.dpadLeftWasPressed()) {
            openGate = false;
        }
        telemetry.update();
    }

    @Override
    public void loop() {
        ShootingConstants.ShooterOutputs shooterOutputs;

        if (lockShooter) {
            shooterOutputs = sotm.calculateShooterOutputs(lockedPose, new Vector(), new Vector(), 0, RobotConstants.dt, Alliance.RED);
        } else {
            shooterOutputs =
                    RobotConstants.useShootOnTheMove ?
                            sotm.calculateShooterOutputs(follower.getPose(), follower.getVelocity(), follower.getAcceleration(), follower.getAngularVelocity(), RobotConstants.dt, Alliance.RED) :
                            sotm.calculateShooterOutputs(follower.getPose(), new Vector(), new Vector(), 0, RobotConstants.dt, Alliance.RED);
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
