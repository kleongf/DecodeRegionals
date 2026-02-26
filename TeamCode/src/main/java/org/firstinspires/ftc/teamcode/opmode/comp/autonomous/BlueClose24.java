package org.firstinspires.ftc.teamcode.opmode.comp.autonomous;
// the main class for the sidespike
// different choices based on the initial selection
// can choose to intake third or not and to do cycle or not
// TODO: if we are doing third do differently, go closer

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
import com.qualcomm.robotcore.hardware.Gamepad;
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

@Autonomous(name="Blue Close 24 Side Spike Main", group="!")
public class BlueClose24 extends OpMode {
    private boolean doThirdSpike = false;
    private boolean doOpenGate = true;
    private double pileCycleY = 36; // idk just in case we need to change it? unsure. i might just make a diff program for that?
    private Follower follower;
    private StateMachine stateMachine;
    private AutonomousRobot robot;
    private SOTM sotm2;
    private boolean holdingTurret = true;
    private final Pose startPose = PoseConstants.BLUE_CLOSE_AUTO_POSE;
    private final Pose goalPose = PoseConstants.BLUE_GOAL_POSE;
    private Pose currentShootPose = new Pose(27, 104, Math.toRadians(-90));
    // TODO: final pos is in between the stuff
    private PathChain shootPreload, intakeFirst, shootFirst, intakeSecond, shootSecond, openGate, intakeGate1, shootGate1, intakeGate2, shootGate2, intakeThird, shootThird, intakeGate3, shootGate3, intakeGate4, shootGate4, intakePile, shootPile;

    public void buildPaths() {
        shootPreload = follower.pathBuilder().addPath(
                new BezierLine(
                        startPose,
                        new Pose(32, 102.000)
                )
        ).setConstantHeadingInterpolation(startPose.getHeading()).setTValueConstraint(0.93).build();

        intakeFirst = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(32, 102.000),
                        new Pose(23.5, 97),
                        new Pose(23.5, 92),
                        new Pose(23.500, 83.000)
                )
        ).setConstantHeadingInterpolation(startPose.getHeading()).setTValueConstraint(0.95).build();

        shootFirst = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(23.500, 83.000),
                                new Pose(32, 102)
                        )
                )
                .setBrakingStrength(0.5) // equal to zpam 2 try this i guess
                .setTValueConstraint(0.95)
                // i also decreased t value so it will stop earlier hopefully
                // .setTValueConstraint(0.9)
                .setConstantHeadingInterpolation(startPose.getHeading())
                // TODO: play around with max power, i dont think its going to the correct spot
                // .setBrakingStart(10)
                // TODO: idk maybe give it some time to correct?
                // .setTimeoutConstraint(200)
                // TODO: or turn off predictive for the first two paths
                .build();


        intakeSecond = follower.pathBuilder().addPath(
                        new BezierCurve(
                                // TODO: also artificially changed this lol
                                new Pose(32.00, 102.000),
                                // i also changed this control point a bit
                                // new Pose(28.00, 92.000),
                                new Pose(25.00, 82.000),
                                new Pose(23.500, 62.000)
                        )
                ).setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        openGate = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(23.500, 62.000),
                                PoseConstants.BLUE_SIDE_GATE_POSE
                        )
                ).setConstantHeadingInterpolation(PoseConstants.BLUE_SIDE_GATE_POSE.getHeading())
                .setNoDeceleration()
                .build();

        // TODO: should we change gate? to simulate the drift?


        shootSecond = follower.pathBuilder().addPath(
                        new BezierLine(
                                PoseConstants.BLUE_SIDE_GATE_POSE,
                                new Pose(56.000, 75.000)
                        )
                ).setLinearHeadingInterpolation(PoseConstants.BLUE_SIDE_GATE_POSE.getHeading(), Math.toRadians(-160))
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

        intakeGate2 = follower.pathBuilder()
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

        shootGate2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                PoseConstants.BLUE_GATE_AUTO_POSE,
                                new Pose(56.000, 75.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        intakeGate3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(56.000, 75.000),
                                new Pose(45, PoseConstants.BLUE_GATE_AUTO_POSE.getY()),
                                new Pose(PoseConstants.BLUE_GATE_AUTO_POSE.getX(), PoseConstants.BLUE_GATE_AUTO_POSE.getY()+0.5)
                                // PoseConstants.BLUE_GATE_AUTO_POSE
                        )
                )
                .setHeadingInterpolation(toGate)
                .setTValueConstraint(0.99)
                .build();

        shootGate3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                PoseConstants.BLUE_GATE_AUTO_POSE,
                                new Pose(56.000, 75.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        // if we are solo, do the third in between
        intakeThird = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(56.000, 72.000),
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

        intakeGate4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(56.000, 75.000),
                                new Pose(45, PoseConstants.BLUE_GATE_AUTO_POSE.getY()),
                                new Pose(PoseConstants.BLUE_GATE_AUTO_POSE.getX(), PoseConstants.BLUE_GATE_AUTO_POSE.getY()+1)
                                // PoseConstants.BLUE_GATE_AUTO_POSE
                        )
                )
                .setHeadingInterpolation(toGate)
                .setTValueConstraint(0.99)
                .build();

        shootGate4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                PoseConstants.BLUE_GATE_AUTO_POSE,
                                new Pose(56.000, 75.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();


        intakePile = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(56.000, 72.000),
                                new Pose(40.000, 36.000),
                                new Pose(12.000, 36.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-160), Math.toRadians(-180))
                .build();

        shootPile = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(12.000, 36.000),
                                new Pose(58, 96)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        // TODO: when porting to red use FIELD_WIDTH not 144 btw
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpadUpWasPressed()) {
            doThirdSpike = !doThirdSpike;
        }
        if (gamepad1.dpadDownWasPressed()) {
            doOpenGate = !doOpenGate;
        }
        telemetry.addLine("Press the dpad up/down buttons to configure spike and gate.");
        telemetry.addData("Do Third Spike", doThirdSpike);
        telemetry.addData("Do Open Gate", doOpenGate);
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
                            // follower.usePredictiveBraking = false;
                            // actually it seems to work on the first path
                            // also this pose isn't accurate?
                            follower.followPath(shootPreload, true);
                            currentShootPose = new Pose(27, 102, Math.toRadians(-90));
                            robot.intake.state = Intake.IntakeState.INTAKE_SLOW;
                        })
                        .transition(new Transition(() -> follower.atParametricEnd() && robot.shooter.atTarget(40))),
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
                            // follower.usePredictiveBraking = true;
                            follower.followPath(intakeSecond, true);
                            robot.intakeCommand.start();
                        })
                        .maxTime(400),
                new State()
                        .onEnter(() -> {
                            holdingTurret = false;
                            // TODO: maybe compensate by subtracting like 2 degrees? or test more with sotm on
                            // i added 2 degrees
                            currentShootPose = new Pose(56, 75, Math.toRadians(-160));
                        })
                        // if no open gate, go to shoot second state
                        .transition(new Transition(() -> follower.atParametricEnd() && !doOpenGate, "shootSecond"))
                        .transition(new Transition(() -> follower.atParametricEnd() && doOpenGate)),
                new State()
                        .onEnter(() -> {
                            follower.followPath(openGate, true);
                        })
                        .transition(new Transition(() -> follower.atParametricEnd())),
                new State()
                        .maxTime(100),
                new State("shootSecond")
                        .onEnter(() -> {
                            if (doOpenGate) {
                                shootSecond = follower.pathBuilder().addPath(
                                                new BezierLine(
                                                        PoseConstants.BLUE_SIDE_GATE_POSE,
                                                        new Pose(56.000, 75.000)
                                                )
                                        ).setLinearHeadingInterpolation(PoseConstants.BLUE_SIDE_GATE_POSE.getHeading(), Math.toRadians(-160))
                                        .build();
                            } else {
                                shootSecond = follower.pathBuilder().addPath(
                                                new BezierLine(
                                                        new Pose(23.500, 64.000),
                                                        new Pose(56.000, 75.000)
                                                )
                                        ).setLinearHeadingInterpolation(PoseConstants.BLUE_SIDE_GATE_POSE.getHeading(), Math.toRadians(-160))
                                        .build();
                            }
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
                        .transition(new Transition(() -> !follower.isBusy())),
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
                        .transition(new Transition(() -> !follower.isBusy())),
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
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished() && doThirdSpike, "thirdSpike"))
                        .transition(new Transition(() -> robot.shootCommand.isFinished() && !doThirdSpike, "gateCycle4")),

                new State("thirdSpike")
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.followPath(intakeThird, false);
                        })
                        .transition(new Transition(() -> follower.atParametricEnd())),
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootThird, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            robot.shootCommand.start();
                        })
                        // intake pile after third
                        .transition(new Transition(() -> robot.shootCommand.isFinished(), "intakePile")),

                // gate cycle 4
                new State("gateCycle4")
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
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),

                // intake pile
                new State("intakePile")
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.followPath(intakePile, false);
                        })
                        .transition(new Transition(() -> follower.atParametricEnd())),
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootPile, true);
                            currentShootPose = new Pose(58, 96, Math.toRadians(-127));
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
        // TODO: test, should be a bit better. will not overcorrect.
        sotm2.latencyScaleFactor = 0.5;
        // robot.turret.setPDCoefficients(0.005, 0);
    }
    @Override
    public void loop() {
        // ALSO TODO: change the position to be a bit lower as we are slightly over the line

        if (holdingTurret) {
            double[] shooterValues = sotm2.calculateAzimuthThetaVelocityFRCBetter(currentShootPose, new Vector(), follower.getAngularVelocity());
            // now it applies feedforward but target is same, so it will update slightly when shooting kinda like sotm.
            robot.setAzimuthThetaVelocity(new double[] {shooterValues[0], shooterValues[1], shooterValues[2]});
            robot.turret.setFeedforward(0);
        } else {
            double[] turretValues = sotm2.calculateAzimuthThetaVelocityFRCBetter(currentShootPose, follower.getVelocity(), follower.getAngularVelocity());
            double[] shooterValues = sotm2.calculateAzimuthThetaVelocityFRCBetter(currentShootPose, new Vector(), follower.getAngularVelocity());
            // now it applies feedforward but target is same, so it will update slightly when shooting kinda like sotm.
            // TODO: testing no move turret
            robot.setAzimuthThetaVelocity(new double[] {turretValues[0], shooterValues[1], shooterValues[2]});
            robot.turret.setFeedforward(turretValues[3]*0);
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
