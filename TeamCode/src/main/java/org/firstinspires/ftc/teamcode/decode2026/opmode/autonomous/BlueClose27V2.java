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
import org.firstinspires.ftc.teamcode.util.decodeutil.SOTMUtil;
import org.firstinspires.ftc.teamcode.util.decodeutil.ZoneUtil;
import org.firstinspires.ftc.teamcode.util.fsm.State;
import org.firstinspires.ftc.teamcode.util.fsm.StateMachine;
import org.firstinspires.ftc.teamcode.util.fsm.Transition;

// last one is going to be a pile cycle, it will always be a pile cycle


@Autonomous(name="Blue Close 27 V2", group="!")
public class BlueClose27V2 extends OpMode {
    private boolean doOpenGate = true; // trust...
    private Pose lockedPose = new Pose();
    private boolean lockShooter = true;
    private Follower follower;
    private StateMachine stateMachine;
    private CurrentRobot robot;
    private SOTMUtil sotm;
    private Intake.DetectionState prevState;
    private PathChain shootPreload, intakeFirst, intakeFirstOpenGate, shootFirst, shootFirstOpenGate, intakeSecond, shootSecond, intakeGate1, shootGate1, intakeGate2, shootGate2, intakeThird, shootThird, intakeGate3, shootGate3, intakeGate4, shootGate4, intakeGate5, shootGate5, intakePile, shootPile;

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

        intakeFirstOpenGate = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(32.000, 108.000),
                        new Pose(24.000, 97.000),
                        new Pose(24.000, 92.000),
                        new Pose(24.000, 78.000),
                        new Pose(16.000, 70.000)
                )
        ).setConstantHeadingInterpolation(FieldConstants.BLUE_CLOSE_START_AUTO_POSE.getHeading()).build();

        shootFirst = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(23.500, 83.000),
                                new Pose(32, 108)
                        )
                )
                .setConstantHeadingInterpolation(FieldConstants.BLUE_CLOSE_START_AUTO_POSE.getHeading())
                .build();

        shootFirstOpenGate = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(23.500, 83.000),
                                new Pose(32, 108)
                        )
                )
                .setConstantHeadingInterpolation(FieldConstants.BLUE_CLOSE_START_AUTO_POSE.getHeading())
                .build();

        intakeSecond = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(32.00, 108.000),
                                new Pose(22, 82.000),
                                new Pose(22, 72.000),
                                new Pose(23.500, 62.000)
                        )
                ).setConstantHeadingInterpolation(FieldConstants.BLUE_CLOSE_START_AUTO_POSE.getHeading())
                .build();

        shootSecond = follower.pathBuilder().addPath(
                        new BezierLine(
                                FieldConstants.BLUE_SIDE_GATE_POSE,
                                new Pose(56.000, 75.000)
                        )
                )
                //.setLinearHeadingInterpolation(FieldConstants.BLUE_SIDE_GATE_POSE.getHeading(), Math.toRadians(-160))
                .build();


        HeadingInterpolator toGate = HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(
                        0,
                        0.5,
                        //testing straightline path cuz faster
                        HeadingInterpolator.constant(Math.toRadians(-160))
                        //HeadingInterpolator.linear(Math.toRadians(-160), FieldConstants.BLUE_GATE_AUTO_POSE.getHeading())
                ),
                new HeadingInterpolator.PiecewiseNode(
                        0.5,
                        1,
                        HeadingInterpolator.constant(FieldConstants.BLUE_GATE_AUTO_POSE_24.getHeading())
                )
        );

        intakeGate1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(56.000, 75.000),
                                //new Pose(45, FieldConstants.BLUE_GATE_AUTO_POSE.getY()),
                                FieldConstants.BLUE_GATE_AUTO_POSE_24
                        )
                )
                .setHeadingInterpolation(toGate)
                .setTValueConstraint(0.99)
                .build();

        shootGate1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                FieldConstants.BLUE_GATE_AUTO_POSE_24,
                                new Pose(56.000, 75.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        intakeGate2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(56.000, 75.000),
                                //new Pose(45, FieldConstants.BLUE_GATE_AUTO_POSE.getY()),
                                new Pose(FieldConstants.BLUE_GATE_AUTO_POSE_24.getX(),FieldConstants.BLUE_GATE_AUTO_POSE_24.getY())
                        )
                )
                .setHeadingInterpolation(toGate)
                .setTValueConstraint(0.99)
                .build();

        shootGate2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                FieldConstants.BLUE_GATE_AUTO_POSE_24,
                                new Pose(56.000, 75.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        intakeGate3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(56.000, 75.000),
                                //new Pose(45, FieldConstants.BLUE_GATE_AUTO_POSE.getY()),
                                new Pose(FieldConstants.BLUE_GATE_AUTO_POSE_24.getX(),FieldConstants.BLUE_GATE_AUTO_POSE_24.getY())
                        )
                )
                .setHeadingInterpolation(toGate)
                .setTValueConstraint(0.99)
                .build();

        shootGate3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                FieldConstants.BLUE_GATE_AUTO_POSE_24,
                                new Pose(56.000, 75.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        intakeGate4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(56.000, 75.000),
                                //new Pose(45, FieldConstants.BLUE_GATE_AUTO_POSE.getY()),
                                new Pose(FieldConstants.BLUE_GATE_AUTO_POSE_24.getX(),FieldConstants.BLUE_GATE_AUTO_POSE_24.getY())
                        )
                )
                .setHeadingInterpolation(toGate)
                .setTValueConstraint(0.99)
                .build();

        shootGate4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                FieldConstants.BLUE_GATE_AUTO_POSE_24,
                                new Pose(56.000, 75.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
        intakeGate5 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(56.000, 75.000),
                                //new Pose(45, FieldConstants.BLUE_GATE_AUTO_POSE.getY()),
                                new Pose(FieldConstants.BLUE_GATE_AUTO_POSE_24.getX(),FieldConstants.BLUE_GATE_AUTO_POSE_24.getY())
                        )
                )
                .setHeadingInterpolation(toGate)
                .setTValueConstraint(0.99)
                .build();

        shootGate5 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                FieldConstants.BLUE_GATE_AUTO_POSE_24,
                                new Pose(56.000, 75.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        intakeThird = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(56.000, 75.000),
                                new Pose(55, 36),
                                new Pose(12.000, 36.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-160), Math.toRadians(-180))
                .build();

        shootThird = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(13.000, 36.000),
                                new Pose(56.000, 75.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-160))
                .build();

        intakePile = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(56.000, 75.000),
                                new Pose(19.864, 51.595),
                                new Pose(14.892, 42.784),
                                new Pose(12.000, 30.000),
                                new Pose(12.000, 25.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        shootPile = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(12.000, 20.000),
                                new Pose(58.000, 120.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpadUpWasPressed()) {
            doOpenGate = !doOpenGate;
        }
        telemetry.addLine("Press the dpad up/down buttons to configure gate.");
        telemetry.addData("Do Open Gate", doOpenGate);
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.usePredictiveBraking = true;
        follower.setMaxPower(1);
        follower.setStartingPose(FieldConstants.BLUE_CLOSE_START_AUTO_POSE);
        robot = new CurrentRobot(hardwareMap);
        sotm = new SOTMUtil(FieldConstants.BLUE_GOAL_POSE);
        ShootingConstants.tofMultiplier = 0.35; // todo: remember to change back for tele as it is a static variable
        lockedPose = new Pose(32, 108, FieldConstants.BLUE_CLOSE_START_AUTO_POSE.getHeading());
        buildPaths();

        stateMachine = new StateMachine(
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootPreload, true);
                            robot.prepareShootCommand.start();
                        })
                        .transition(new Transition(() -> follower.atParametricEnd() && robot.shooter.atTarget(40))),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                new State()
                        .onEnter(() -> {
                            intakeFirst = doOpenGate ? intakeFirstOpenGate : intakeFirst;
                            follower.followPath(intakeFirst, true);
                            robot.intakeCommand.start();
                        })
                        .transition(new Transition(() -> follower.atParametricEnd())),
                new State()
                        .onEnter(() -> {
                            shootFirst = doOpenGate ? shootFirstOpenGate : shootFirst;
                            follower.followPath(shootFirst, true);
                        })
                        .transition(new Transition(() -> follower.atParametricEnd())),
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
                        .transition(new Transition(() -> follower.atParametricEnd())),
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootSecond, true);
                            lockedPose = new Pose(56, 75, Math.toRadians(-160));
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // gate cycle 1
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            lockShooter = false;
                            follower.setMaxPower(.8);
                            follower.followPath(intakeGate1, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            follower.holdPoint(new BezierPoint(FieldConstants.BLUE_GATE_AUTO_POSE_IN), FieldConstants.BLUE_GATE_AUTO_POSE_IN.getHeading());
                        })
                        .minTime(600)
                        .transition(new Transition(() -> robot.intake.isFull))
                        .maxTime(1500),
                new State()
                        .onEnter(() -> {
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
                            follower.setMaxPower(.8);
                            follower.followPath(intakeGate2, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            follower.holdPoint(new BezierPoint(FieldConstants.BLUE_GATE_AUTO_POSE_IN), FieldConstants.BLUE_GATE_AUTO_POSE_IN.getHeading());
                        })
                        .minTime(600)
                        .transition(new Transition(() -> robot.intake.isFull))
                        .maxTime(1500),
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
                            follower.holdPoint(new BezierPoint(FieldConstants.BLUE_GATE_AUTO_POSE_IN), FieldConstants.BLUE_GATE_AUTO_POSE_IN.getHeading());
                        })
                        .minTime(600)
                        .transition(new Transition(() -> robot.intake.isFull))
                        .maxTime(1500),
                new State()
                        .onEnter(() -> {
                            follower.setMaxPower(1);
                            follower.followPath(shootGate3, true);
                            robot.prepareShootCommandLonger.start();
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                // gate cycle 4
                new State("gateCycle4")
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.setMaxPower(.8);
                            follower.followPath(intakeGate4, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            follower.holdPoint(new BezierPoint(FieldConstants.BLUE_GATE_AUTO_POSE_IN), FieldConstants.BLUE_GATE_AUTO_POSE_IN.getHeading());
                        })
                        .minTime(600)
                        .transition(new Transition(() -> robot.intake.isFull))
                        .maxTime(1600),
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
                            follower.holdPoint(new BezierPoint(FieldConstants.BLUE_GATE_AUTO_POSE_IN), FieldConstants.BLUE_GATE_AUTO_POSE_IN.getHeading());
                        })
                        .minTime(600)
                        .transition(new Transition(() -> robot.intake.isFull))
                        .maxTime(1600),
                new State()
                        .onEnter(() -> {
                            follower.setMaxPower(1);
                            follower.followPath(shootGate5, true);
                            robot.prepareShootCommandLonger.start();
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),

                // pile intake
                new State("intakePile")
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.followPath(intakePile, false);
                        })
                        .transition(new Transition(() -> follower.atParametricEnd())),
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootPile, true);
                            ShootingConstants.tofMultiplier = .8;
                        })
                        .maxTime(300)
                        .transition(new Transition(() -> robot.intake.isFull)),
                new State()
                        .onEnter(() -> robot.prepareShootCommandLonger.start())
                        // todo: test this on normal shooting as well because funny
                        .transition(new Transition(() -> ZoneUtil.inCloseZone(follower.getPose()))),//CRAZY SOTTMMM LESGOO
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
            shooterOutputs = sotm.calculateShooterOutputs(lockedPose, new Vector(), new Vector(), 0, RobotConstants.dt);
        } else {
            shooterOutputs =
                    RobotConstants.useShootOnTheMove ?
                            sotm.calculateShooterOutputs(follower.getPose(), follower.getVelocity(), follower.getAcceleration(), follower.getAngularVelocity(), RobotConstants.dt) :
                            sotm.calculateShooterOutputs(follower.getPose(), new Vector(), new Vector(), 0, RobotConstants.dt);
        }

        robot.shooter.wantedVelocity = shooterOutputs.wheelVelocity;
        robot.shooter.wantedAcceleration = shooterOutputs.wheelFeedforward;
        robot.shooter.wantedPitch = shooterOutputs.hoodAngle;
        robot.turret.wantedAngle = shooterOutputs.turretAngle;
        robot.turret.wantedAngularVelocity = shooterOutputs.turretFeedforward;

        if (robot.intake.detectionState == Intake.DetectionState.THIRD_TRIGGERED && prevState == Intake.DetectionState.SECOND_TRIGGERED) {
            robot.ledIndicator.indicateIntakeFull();
        }
        prevState = robot.intake.detectionState;

        stateMachine.update();
        follower.update();
        robot.update();
        blackboard.put(FieldConstants.END_POSE_KEY, follower.getPose());
        telemetry.update();
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
