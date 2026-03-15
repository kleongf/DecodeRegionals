package org.firstinspires.ftc.teamcode.opmode.comp.autonomous;
// the main class for the sidespike
// different choices based on the initial selection
// can choose to intake third or not and to do cycle or not
// TODO: if we are doing third do differently, go closer

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

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.constants.PoseConstants;
import org.firstinspires.ftc.teamcode.robot.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.robots.AutonomousRobot;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.WebcamLocalizer;
import org.firstinspires.ftc.teamcode.util.decodeutil.Alliance;
import org.firstinspires.ftc.teamcode.util.decodeutil.SOTM;
import org.firstinspires.ftc.teamcode.util.fsm.State;
import org.firstinspires.ftc.teamcode.util.fsm.StateMachine;
import org.firstinspires.ftc.teamcode.util.fsm.Transition;


@Autonomous(name="Blue Close and Far 27 Side Spike MTI V3", group="!")
public class BlueClose27MTIV3 extends OpMode {

    private Intake.DetectionState prevDetectState;
    private Follower follower;
    private StateMachine stateMachine;
    private AutonomousRobot robot;
    private SOTM sotm2;
    private double firstSpikesOffset = Math.toRadians(5);
    private boolean holdingTurret = true;
    private boolean isTrackingFar = false;
    private final Pose startPose = PoseConstants.BLUE_CLOSE_AUTO_POSE;
    private final Pose goalPose = PoseConstants.BLUE_GOAL_POSE;
    private Pose currentShootPose = new Pose(32, 108, Math.toRadians(-90));
    // TODO: final pos is in between the stuff
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
                                new Pose(23, 62.000)
                        )
                ).setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        shootSecond = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(23, 62.000),
                                new Pose(56.000, 75.000)
                        )
                )
//                .setTangentHeadingInterpolation()
//                .setReversed()
                .setLinearHeadingInterpolation(PoseConstants.BLUE_SIDE_GATE_POSE.getHeading(), Math.toRadians(-160))
                .build();


        HeadingInterpolator toGate = HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(
                        0,
                        0.75,
                        HeadingInterpolator.linear(Math.toRadians(-160), PoseConstants.BLUE_GATE_AUTO_POSE.getHeading())
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
                                new Pose(56.000, 75.000),
                                new Pose(45, PoseConstants.BLUE_GATE_AUTO_POSE.getY()),
                                PoseConstants.BLUE_GATE_AUTO_POSE
                        )
                )
                .setHeadingInterpolation(toGate)
                .setTValueConstraint(0.99)
                .build();

        shootGate1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                PoseConstants.BLUE_GATE_AUTO_POSE,
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
                                new Pose(15.000, 36.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-160), Math.toRadians(180))
                .build();

        shootThird = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(15.000, 36.000),
                                new Pose(60.000, 12.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        intakePile1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(60, 12),
                                new Pose(10, 12)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        shootPile1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(10, 12.000),
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
                                PoseConstants.BLUE_GATE_AUTO_POSE
                        )
                )
                .setTangentHeadingInterpolation()
                .setTValueConstraint(0.99)
                .build();

        shootGate2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                PoseConstants.BLUE_GATE_AUTO_POSE,
                                new Pose(38,41),
                                new Pose(60,12)
                        )
                )
                .setLinearHeadingInterpolation(PoseConstants.BLUE_GATE_AUTO_POSE.getHeading(), Math.toRadians(180))
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
                                new Pose(12, 12.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        shootPile3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(12, 12.000),
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
        robot = new AutonomousRobot(hardwareMap, Alliance.BLUE);
        sotm2 = new SOTM(goalPose);
        buildPaths();

        stateMachine = new StateMachine(
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootPreload, true);
                            currentShootPose = new Pose(32, 108, Math.toRadians(-90));
                            robot.intake.state = Intake.IntakeState.INTAKE_SLOW;
                            robot.turret.setPDCoefficients(0.009, 0.0003);
                        })
                        .transition(new Transition(() -> follower.atParametricEnd() && robot.shooter.atTarget(200))),
                new State()
                        .onEnter(() -> {
                            robot.shootCommand.start();
                            robot.turret.resetEncoderWithAbsoluteReading();
                        })
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
                            robot.turret.resetEncoderWithAbsoluteReading();
                        })
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                new State()
                        .onEnter(() -> {
                            follower.followPath(intakeSecond, true);
                            robot.intakeCommand.start();
                            robot.turret.setPDCoefficients(0.01, 0.0004);
                        })
                        .maxTime(400),
                new State()
                        .onEnter(() -> {
                            holdingTurret = false;
                            currentShootPose = new Pose(56, 75, Math.toRadians(-160-6));
                        })
                        // if no open gate, go to shoot second state
                        .transition(new Transition(() -> follower.atParametricEnd())),
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootSecond, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            robot.shootCommand.start();
                            robot.turret.resetEncoderWithAbsoluteReading();
                        })
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                // gate cycle 1
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.followPath(intakeGate1, true);
                            currentShootPose = new Pose(56, 75, Math.toRadians(-160-6));
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            follower.holdPoint(new BezierPoint(PoseConstants.BLUE_GATE_AUTO_POSE_IN), PoseConstants.BLUE_GATE_AUTO_POSE_IN.getHeading());
                        })
                        .minTime(600)
                        .transition(new Transition(() -> robot.intake.intakeMostlyFull()))
                        .maxTime(1200),
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootGate1, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            robot.shootCommand.start();
                            robot.turret.resetEncoderWithAbsoluteReading();
                        })
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
                            isTrackingFar = true;
                            robot.turret.setPDCoefficients(0.01, 0.0004);
                            sotm2.latencyScaleFactor = 0;
                            sotm2.latencyScaleFactorRadial = 0;
                            currentShootPose = new Pose(60,12,Math.toRadians(180-5));
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .maxTime(100),
                new State()
                        .onEnter(() -> {
                            robot.shootCommandSlow.start();
                            robot.turret.resetEncoderWithAbsoluteReading();
                        })
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
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .maxTime(100),
                new State()
                        .onEnter(() -> {
                            robot.shootCommandSlow.start();
                            robot.turret.resetEncoderWithAbsoluteReading();
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
                            follower.holdPoint(new BezierPoint(PoseConstants.BLUE_GATE_AUTO_POSE_IN), PoseConstants.BLUE_GATE_AUTO_POSE_IN.getHeading());
                        })
                        .minTime(600)
                        .transition(new Transition(() -> robot.intake.intakeMostlyFull()))
                        .maxTime(1500),
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootGate2, true);
                            currentShootPose = new Pose(60,12,Math.toRadians(180-5));
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .maxTime(200),
                new State()
                        .onEnter(() -> {
                            robot.shootCommandSlow.start();
                            robot.turret.resetEncoderWithAbsoluteReading();
                        })
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
                            robot.turret.resetEncoderWithAbsoluteReading();
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
        robot.initPositions();
        robot.turret.resetEncoderWithAbsoluteReading();
        robot.turret.setUseExternal(false);
        sotm2.latencyScaleFactor = 0.5;
    }
    @Override
    public void loop() {
//        if (robot.intake.isFull() && prevDetectState != robot.intake.detectionState) {
//            robot.webcamLocalizer.ledState = WebcamLocalizer.LedState.INTAKE_FULL;
//            robot.webcamLocalizer.flashLEDMultipleTimes();
//        }
        // ALSO TODO: change the position to be a bit lower as we are slightly over the line

        if (holdingTurret) {
            double[] shooterValues = sotm2.calculateAzimuthThetaVelocityFRCBetter(currentShootPose, new Vector(), follower.getAngularVelocity());
            // now it applies feedforward but target is same, so it will update slightly when shooting kinda like sotm.
            robot.setAzimuthThetaVelocity(new double[] {shooterValues[0]+firstSpikesOffset, shooterValues[1], shooterValues[2]});
            robot.turret.setFeedforward(0);
        } else {
            if (isTrackingFar) {
                double[] values = sotm2.calculateAzimuthThetaVelocityFRCBetter(new Pose(currentShootPose.getX(), currentShootPose.getY(), follower.getPose().getHeading()), new Vector(), follower.getAngularVelocity());
                values[2] += 40; //in case inconsistency --> undershooting
                robot.setAzimuthThetaVelocity(values);
            }
            else {
                double[] values = sotm2.calculateAzimuthThetaVelocityFRCBetter(currentShootPose, new Vector(), follower.getAngularVelocity());
                robot.setAzimuthThetaVelocity(values);
            }
            robot.turret.setFeedforward(0);
        }

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
