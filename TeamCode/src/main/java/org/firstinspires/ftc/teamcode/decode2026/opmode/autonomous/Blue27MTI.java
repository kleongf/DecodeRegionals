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

import org.firstinspires.ftc.teamcode.decode2026.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.decode2026.subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.decode2026.CurrentRobot;
import org.firstinspires.ftc.teamcode.util.decodeutil.SOTMUtil;
import org.firstinspires.ftc.teamcode.util.fsm.State;
import org.firstinspires.ftc.teamcode.util.fsm.StateMachine;
import org.firstinspires.ftc.teamcode.util.fsm.Transition;


@Autonomous(name="Blue Close and Far 27 Side Spike MTI", group="!")
public class Blue27MTI extends OpMode {

    private Intake.DetectionState prevDetectState;
    private Follower follower;
    private StateMachine stateMachine;
    private CurrentRobot robot;
    private SOTMUtil sotm2;
    private final Pose startPose = FieldConstants.BLUE_CLOSE_AUTO_POSE;
    private final Pose goalPose = FieldConstants.BLUE_GOAL_POSE;
    private Pose currentShootPose = new Pose(32, 108, Math.toRadians(-90));
    private PathChain shootPreload, intakeFirst, shootFirst, intakeSecond, shootSecond, intakeGate1, shootGate1, intakeThird, shootThird, intakePile1, shootPile1, intakeGate2, shootGate2, intakePile2, shootPile2, intakePile3, shootPile3;

    public void buildPaths() {
        shootPreload = follower.pathBuilder().addPath(
                new BezierLine(
                        startPose,
                        new Pose(32, 108.000)
                )
        ).setBrakingStrength(4).setTimeoutConstraint(100).setConstantHeadingInterpolation(startPose.getHeading()).setTValueConstraint(0.95).build();

        intakeFirst = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(32, 108.000),
                        // new Pose(24, 104),
                        new Pose(24, 97),
                        new Pose(24, 92),
                        new Pose(23.500, 83.000)
                )
        ).setConstantHeadingInterpolation(startPose.getHeading()).setTValueConstraint(0.95).build();

        shootFirst = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(23.500, 83.000),
                                new Pose(32, 108)
                        )
                )
                .setConstantHeadingInterpolation(startPose.getHeading())
                .setBrakingStrength(0.5)
                .setTValueConstraint(0.95)
                .build();

        intakeSecond = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(32.00, 108.000),
                                new Pose(25, 82.000),
                                new Pose(24, 72.000),
                                new Pose(23.500, 62.000)
                        )
                ).setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        shootSecond = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(23.500, 64.000),
                                new Pose(56.000, 75.000)
                        )
                )
//                .setTangentHeadingInterpolation()
//                .setReversed()
                .setLinearHeadingInterpolation(FieldConstants.BLUE_SIDE_GATE_POSE.getHeading(), Math.toRadians(-160))
                .build();


        HeadingInterpolator toGate = HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(
                        0,
                        0.75,
                        HeadingInterpolator.linear(Math.toRadians(-160), FieldConstants.BLUE_GATE_AUTO_POSE.getHeading())
                ),
                new HeadingInterpolator.PiecewiseNode(
                        0.75,
                        1,
                        HeadingInterpolator.constant(FieldConstants.BLUE_GATE_AUTO_POSE.getHeading())
                )
        );

        intakeGate1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(56.000, 75.000),
                                new Pose(45, FieldConstants.BLUE_GATE_AUTO_POSE.getY()),
                                FieldConstants.BLUE_GATE_AUTO_POSE
                        )
                )
                .setHeadingInterpolation(toGate)
                .setTValueConstraint(0.99)
                .build();

        shootGate1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                FieldConstants.BLUE_GATE_AUTO_POSE,
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
                ).setLinearHeadingInterpolation(Math.toRadians(-160), Math.toRadians(180))
                .build();

        shootThird = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(12.000, 36.000),
                                new Pose(60.000, 12.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        intakePile1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(60, 12),
                                new Pose(12, 12)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        shootPile1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(12, 12.000),
                                new Pose(60.000, 12.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
//                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(148))
                .build();

        intakeGate2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(60, 12),
                                new Pose(38,41),
                                FieldConstants.BLUE_GATE_AUTO_POSE
                        )
                )
                .setTangentHeadingInterpolation()
                .setTValueConstraint(0.99)
                .build();

        shootGate2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                FieldConstants.BLUE_GATE_AUTO_POSE,
                                new Pose(38,41),
                                new Pose(60,12)
                        )
                )
                .setLinearHeadingInterpolation(FieldConstants.BLUE_GATE_AUTO_POSE.getHeading(), Math.toRadians(180))
//                .setTangentHeadingInterpolation()
//                .setReversed()
                .build();

        intakePile2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(60, 12),
                                new Pose(40,24),
                                new Pose(12, 25)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        shootPile2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(12, 25),
                                new Pose(60, 12)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        intakePile3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(60, 12),
                                new Pose(12.000, 12.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        shootPile3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(12.000, 12.000),
                                new Pose(60, 12)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.usePredictiveBraking = true;
        follower.setStartingPose(startPose);
        robot = new CurrentRobot(hardwareMap);
        sotm2 = new SOTMUtil(goalPose);
        buildPaths();

        stateMachine = new StateMachine(
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootPreload, true);
                            currentShootPose = new Pose(32, 108, Math.toRadians(-90));
                        })
                        .transition(new Transition(() -> follower.atParametricEnd() && robot.shooter.atTarget(200))),
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
                        .onEnter(() -> {
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
                        .maxTime(400),
                new State()
                        .onEnter(() -> {
                            currentShootPose = new Pose(56, 75, Math.toRadians(-160));
                        })
                        // if no open gate, go to shoot second state
                        .transition(new Transition(() -> follower.atParametricEnd())),
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
                            follower.followPath(intakeGate1, true);
                            currentShootPose = new Pose(56, 75, Math.toRadians(-160)-Math.toRadians(2));
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            follower.holdPoint(new BezierPoint(FieldConstants.BLUE_GATE_AUTO_POSE_IN), FieldConstants.BLUE_GATE_AUTO_POSE_IN.getHeading());
                        })
                        .minTime(600)
                        .transition(new Transition(() -> robot.intake.detectionState == Intake.DetectionState.SECOND_TRIGGERED || robot.intake.detectionState == Intake.DetectionState.THIRD_TRIGGERED))
                        .maxTime(1200),
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootGate1, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // gate cycle 2
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.followPath(intakeThird, false);
                        })
                        .transition(new Transition(() -> follower.atParametricEnd())),
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootThird, true);
                            currentShootPose = new Pose(60,12,Math.toRadians(180));
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .maxTime(100),
                new State()
                        .onEnter(() -> robot.shootCommandSlow.start())
                        .transition(new Transition(() -> robot.shootCommandSlow.isFinished())),
                // intake pile 1
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.followPath(intakePile1, false);
                        })
                        .maxTime(1300)
                        .transition(new Transition(() -> follower.atParametricEnd())),
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootPile1, true);
                            //currentShootPose = new Pose(60, 12, Math.toRadians(180+3));
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .maxTime(100),
                new State()
                        .onEnter(() -> {
                            robot.shootCommandSlow.start();
                        })
                        .transition(new Transition(() -> robot.shootCommandSlow.isFinished())),
                // gate cycle 3
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.followPath(intakeGate2, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            follower.holdPoint(new BezierPoint(FieldConstants.BLUE_GATE_AUTO_POSE_IN), FieldConstants.BLUE_GATE_AUTO_POSE_IN.getHeading());
                        })
                        .minTime(600)
                        .transition(new Transition(() -> robot.intake.detectionState == Intake.DetectionState.SECOND_TRIGGERED || robot.intake.detectionState == Intake.DetectionState.THIRD_TRIGGERED))
                        .maxTime(1500),
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootGate2, true);
                            currentShootPose = new Pose(60,12,Math.toRadians(180));
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .maxTime(200),
                new State()
                        .onEnter(() -> robot.shootCommandSlow.start())
                        .transition(new Transition(() -> robot.shootCommandSlow.isFinished())),

                // pile intake 2
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.followPath(intakePile2, false);
                        })
                        .maxTime(1300)
                        .transition(new Transition(() -> follower.atParametricEnd())),
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootPile2, true);
                            //currentShootPose = new Pose(52,16,Math.toRadians(180+3));
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .maxTime(100),
                new State()
                        .onEnter(() -> {
                            robot.shootCommandSlow.start();
                        })
                        .transition(new Transition(() -> robot.shootCommandSlow.isFinished())),
                // pile intake 3
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.followPath(intakePile3, false);
                        })
                        .maxTime(1300)
                        .transition(new Transition(() -> follower.atParametricEnd())),
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootPile3, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .maxTime(100),
                new State()
                        .onEnter(() -> {
                            robot.shootCommandSlow.start();
                        })
                        .transition(new Transition(() -> robot.shootCommandSlow.isFinished()))
        );

        try {
            sleep(500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
    @Override
    public void loop() {
        double[] values = sotm2.calculateAzimuthFeedforwardThetaVelocity(currentShootPose, new Vector(), 0.0, 0.02);
        robot.turret.wantedAngle = values[0];
        robot.turret.angularVelocityToGoal = values[1];
        robot.shooter.wantedPitch = values[2];
        robot.shooter.wantedVelocity = values[3];

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
