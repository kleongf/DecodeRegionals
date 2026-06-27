package org.firstinspires.ftc.teamcode.decode2026.opmode.custom;

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
import org.firstinspires.ftc.teamcode.util.decodeutil.Flipper;
import org.firstinspires.ftc.teamcode.util.decodeutil.SOTMUtil;
import org.firstinspires.ftc.teamcode.util.fsm.State;
import org.firstinspires.ftc.teamcode.util.fsm.StateMachine;
import org.firstinspires.ftc.teamcode.util.fsm.Transition;

// last one is going to be a pile cycle, it will always be a pile cycle

@Autonomous(name="Red Close 24 Crow Force V2", group="!")
public class RedClose24Overcharged extends OpMode {
    private boolean openGate = true;
    private Pose lockedPose = new Pose();
    private boolean lockShooter = true;
    private double turretOffset = 0;
    private Follower follower;
    private StateMachine stateMachine;
    private CurrentRobot robot;
    private SOTMUtil sotm;
    private Intake.DetectionState prevState;
    private PathChain shootPreload, intakeFirst, shootFirst, intakeSecond, shootSecond, intakeGate1, shootGate1, intakeGate2, shootGate2, intakeGate3, shootGate3, intakeGate4, shootGate4, intakeGate5, shootGate5, intakePile, shootPile;
    private PathChain shootFirstOpenGate, intakeSecondOpenGate, shootSecondOpenGate;

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
                        HeadingInterpolator.constant(FieldConstants.RED_GATE_AUTO_POSE_24.getHeading())
                )
        );

        intakeGate1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                Flipper.flip(new Pose(57.000, 77.000)),
                                FieldConstants.RED_GATE_AUTO_POSE_24
                        )
                )
                .setHeadingInterpolation(toGate)
                .setTValueConstraint(0.95)
                .build();

        shootGate1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                FieldConstants.RED_GATE_AUTO_POSE_24,
                                Flipper.flip(new Pose(56, 75))
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        intakeGate2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                Flipper.flip(new Pose(57.000, 77.000)),
                                new Pose(FieldConstants.RED_GATE_AUTO_POSE_24.getX(),FieldConstants.RED_GATE_AUTO_POSE_24.getY())
                        )
                )
                .setHeadingInterpolation(toGate)
                .setTValueConstraint(0.95)
                .build();

        shootGate2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                FieldConstants.RED_GATE_AUTO_POSE_24,
                                Flipper.flip(new Pose(57.000, 77.000))
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        intakeGate3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                Flipper.flip(new Pose(57.000, 77.000)),
                                new Pose(FieldConstants.RED_GATE_AUTO_POSE_24.getX(),FieldConstants.RED_GATE_AUTO_POSE_24.getY()-.2)

                        )
                )
                .setHeadingInterpolation(toGate)
                .setTValueConstraint(0.95)
                .build();

        shootGate3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                FieldConstants.RED_GATE_AUTO_POSE_24,
                                Flipper.flip(new Pose(57.000, 77.000))
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        intakeGate4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                Flipper.flip(new Pose(57.000, 77.000)),
                                new Pose(FieldConstants.RED_GATE_AUTO_POSE_24.getX(),FieldConstants.RED_GATE_AUTO_POSE_24.getY()-.2)
                        )
                )
                .setHeadingInterpolation(toGate)
                .setTValueConstraint(0.95)
                .build();

        shootGate4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                FieldConstants.RED_GATE_AUTO_POSE_24,
                                Flipper.flip(new Pose(57.000, 77.000))
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        intakeGate5 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                Flipper.flip(new Pose(57.000, 77.000)),
                                new Pose(FieldConstants.RED_GATE_AUTO_POSE_24.getX(),FieldConstants.RED_GATE_AUTO_POSE_24.getY()-.2)
                        )
                )
                .setHeadingInterpolation(toGate)
                .setTValueConstraint(0.95)
                .build();

        shootGate5 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                FieldConstants.RED_GATE_AUTO_POSE_24,
                                Flipper.flip(new Pose(57.000, 77.000))
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        HeadingInterpolator pileCycle = HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(
                        0,
                        0.5,
                        HeadingInterpolator.constant(Flipper.flipAngle(Math.toRadians(-130)))
                ),
                new HeadingInterpolator.PiecewiseNode(
                        0.5,
                        1,
                        HeadingInterpolator.linear(Flipper.flipAngle(Math.toRadians(-130)), Flipper.flipAngle(Math.toRadians(-90)))
                )
        );

//        intakePile = follower.pathBuilder()
//                .addPath(
//                        new BezierLine(
//                                Flipper.flip(new Pose(57.000, 77.000)),
//                                Flipper.flip(new Pose(8, 12))
//                        )
//                )
//                .setHeadingInterpolation(pileCycle)
//                .build();
//
//        shootPile = follower.pathBuilder()
//                .addPath(
//                        new BezierLine(
//                                Flipper.flip(new Pose(8, 12)),
//                                Flipper.flip(new Pose(58.000, 120.000))
//                        )
//                )
//                .setTangentHeadingInterpolation()
//                .setReversed()
//                .build();
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
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.usePredictiveBraking = true;
        follower.setMaxPower(1);
        follower.setStartingPose(FieldConstants.RED_CLOSE_START_AUTO_POSE);
        robot = new CurrentRobot(hardwareMap);
        sotm = new SOTMUtil(FieldConstants.RED_GOAL_POSE);
        lockedPose = Flipper.flip(new Pose(32, 108, FieldConstants.RED_CLOSE_START_AUTO_POSE.getHeading()));
        turretOffset = Math.toRadians(4);
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
                            //lockedPose = Flipper.flip(new Pose(56, 75);
                            lockShooter = false;
                        })
                        .maxTime(25),
                new State()
                        .onEnter(() -> {
                            if(openGate) {
                                lockShooter = true;
                                lockedPose = Flipper.flip(new Pose(57,77, Math.toRadians(-130.37)));
                                turretOffset = Math.toRadians(3);
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
                            lockedPose = Flipper.flip(new Pose(57,77, Math.toRadians(-160)));
                            turretOffset = Math.toRadians(-3);
                        })
                        .transition(new Transition(() -> follower.getCurrentTValue() > 0.85)),
                new State()
                        .onEnter(() -> follower.holdPoint(Flipper.flip(new Pose(16, 64, Math.toRadians(180))), false))
                        .maxTime(800),
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootSecond, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // gate cycle 1
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.setMaxPower(.8);
                            follower.followPath(intakeGate1, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            follower.holdPoint(FieldConstants.RED_GATE_AUTO_POSE_IN, false);
                        })
                        .minTime(600)
                        .transition(new Transition(() -> robot.intake.isFull))
                        .maxTime(2200),
                new State()
                        .onEnter(() -> {
                            turretOffset = Math.toRadians(4);
                            follower.setMaxPower(1);
                            follower.followPath(shootGate1, true);
                            robot.prepareShootCommandLonger.start();
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // gate cycle 2
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.setMaxPower(.7);
                            follower.followPath(intakeGate2, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            follower.holdPoint(FieldConstants.RED_GATE_AUTO_POSE_IN, false);
                        })
                        .minTime(600)
                        .transition(new Transition(() -> robot.intake.isFull))
                        .maxTime(2200),
                new State()
                        .onEnter(() -> {
                            follower.setMaxPower(1);
                            follower.followPath(shootGate2, true);
                            robot.prepareShootCommandLonger.start();
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // gate cycle 3
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.setMaxPower(.8);
                            follower.followPath(intakeGate3, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            follower.holdPoint(FieldConstants.RED_GATE_AUTO_POSE_IN, false);
                        })
                        .minTime(600)
                        .transition(new Transition(() -> robot.intake.isFull))
                        .maxTime(2200),
                new State()
                        .onEnter(() -> {
                            follower.setMaxPower(1);
                            follower.followPath(shootGate3, true);
                            robot.prepareShootCommandLonger.start();
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // gate cycle 4
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.setMaxPower(.8);
                            follower.followPath(intakeGate4, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            follower.holdPoint(FieldConstants.RED_GATE_AUTO_POSE_IN, false);
                        })
                        .minTime(600)
                        .transition(new Transition(() -> robot.intake.isFull))
                        .maxTime(2200),
                new State()
                        .onEnter(() -> {
                            follower.setMaxPower(1);
                            follower.followPath(shootGate4, true);
                            robot.prepareShootCommandLonger.start();
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // gate cycle 5
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.setMaxPower(.8);
                            follower.followPath(intakeGate5, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            follower.holdPoint(FieldConstants.RED_GATE_AUTO_POSE_IN, false);
                        })
                        .minTime(600)
                        .transition(new Transition(() -> robot.intake.isFull))
                        .maxTime(2200),
                new State()
                        .onEnter(() -> {
                            follower.setMaxPower(1);
                            follower.followPath(shootGate5, true);
                            robot.prepareShootCommandLonger.start();
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
        robot.reset();
    }
    @Override
    public void loop() {
        ShootingConstants.ShooterOutputs shooterOutputs;

        if (lockShooter) {
            shooterOutputs = sotm.calculateShooterOutputs(lockedPose, new Vector(), new Vector(), 0, RobotConstants.dt);
        } else {
            shooterOutputs =
                    RobotConstants.useShootOnTheMove ?
                            sotm.calculateShooterOutputs2(follower.getPose(), follower.getVelocity(), follower.getAcceleration(), follower.getAngularVelocity(), RobotConstants.dt, Alliance.RED) :
                            sotm.calculateShooterOutputs2(follower.getPose(), new Vector(), new Vector(), 0, RobotConstants.dt, Alliance.RED);
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