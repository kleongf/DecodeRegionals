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

// we need: 24 open gate no 3rd spike, 24 open gate 3rd spike, 24 no open gate with 3rd spike (solo), 30 far
// no 24 no open gate no 3rd spike because it doesn't make sense? that would suggest we have a bad partner who cannot cycle from the pile
// yet for some reason takes the 3rd spike
// i guess it is possible, in mecha auto they shoot the 9 and then don't cycle so i guess it is needed

// so autos we need (and their situations) are:
// 24 open gate 3rd spike (partner doesn't intake 3rd but pile cycles)
// 24 open gate no 3rd spike (partner takes 3rd and pile cycles)
// 24 no open gate 3rd spike (solo auto),
// 24 no open gate no 3rd spike (partner intakes the 3rd but doesn't pile cycle)
// 30 far

// 23 and 7/8 * 6, -3/4 total (3/8 both sides) = 142.5

// 24 open gate 3rd spike: omit the last gate cycle and replace. USE TANGENTIAL and different shoot pose going back as to be more efficient.
// 24 no open gate 3rd spike: omit the path to open gate, omit last gate cycle and replace.
// 24 no open gate no 3rd spike: same as this but omit the open gate part

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

@Autonomous(name="Red Close 24 Sidespike no vision", group="!")
public class RedClose24SidespikeV1 extends OpMode {
    private Follower follower;
    private StateMachine stateMachine;
    private AutonomousRobot robot;
    private SOTM sotm2;
    private final Pose startPose = PoseConstants.RED_CLOSE_AUTO_POSE;
    private final Pose goalPose = PoseConstants.RED_GOAL_POSE;
    // in the future it might be better to do 56 72 -150
    private final Pose shootPose = new Pose(PoseConstants.FIELD_WIDTH-60, 72, Math.toRadians(180-(-144)));
    private Pose currentShootPose = new Pose(PoseConstants.FIELD_WIDTH-60, 72, Math.toRadians(180-(-144)));
    private PathChain shootPreload, intakeFirst, shootFirst, intakeSecond, shootSecond, openGate, intakeGate1, shootGate1, intakeGate2, shootGate2, intakeGate3, shootGate3, intakeGate4, shootGate4, intakePile, shootPile;

    public void buildPaths() {
        double k1 = 10; // i actually forgot what these were check desmos
        double k2 = 20;
        double k3 = 20; // used for random cycle thingy, but we're gonna use vision later so its fine

        // idk abt this just check it again and do for control point 2
        Pose controlPoint1 = new Pose(PoseConstants.FIELD_WIDTH-shootPose.getX() + k1 * Math.cos(shootPose.getHeading()), shootPose.getY() + k1 * Math.sin(shootPose.getHeading()));
        Pose controlPoint2 = new Pose(PoseConstants.FIELD_WIDTH-PoseConstants.RED_GATE_AUTO_POSE.getX() - k2 * Math.cos(PoseConstants.RED_GATE_AUTO_POSE.getHeading()), PoseConstants.RED_GATE_AUTO_POSE.getY() - k2 * Math.sin(PoseConstants.RED_GATE_AUTO_POSE.getHeading()));

        shootPreload = follower.pathBuilder().addPath(
                new BezierLine(
                        startPose,
                        new Pose(PoseConstants.FIELD_WIDTH-23.500, 105.000)
                )
        ).setConstantHeadingInterpolation(startPose.getHeading()).build();

        intakeFirst = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(PoseConstants.FIELD_WIDTH-23.500, 105.000),
                        new Pose(PoseConstants.FIELD_WIDTH-23.500, 83.000)
                )
        ).setConstantHeadingInterpolation(startPose.getHeading()).build();

        shootFirst = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(PoseConstants.FIELD_WIDTH-23.500, 83.000),
                                new Pose(PoseConstants.FIELD_WIDTH-23.500, 105.000)
                        )
                ).setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        intakeSecond = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(PoseConstants.FIELD_WIDTH-23.500, 105.000),
                                new Pose(PoseConstants.FIELD_WIDTH-23.500, 62.000)
                        )
                ).setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        openGate = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(PoseConstants.FIELD_WIDTH-23.500, 62.000),
                                PoseConstants.RED_SIDE_GATE_POSE
                        )
                ).setConstantHeadingInterpolation(PoseConstants.RED_SIDE_GATE_POSE.getHeading())
                .setNoDeceleration()
                .build();


        shootSecond = follower.pathBuilder().addPath(
                        new BezierLine(
                                PoseConstants.RED_SIDE_GATE_POSE,
                                new Pose(PoseConstants.FIELD_WIDTH-60.000, 72.000)
                        )
                ).setLinearHeadingInterpolation(PoseConstants.RED_SIDE_GATE_POSE.getHeading(), Math.toRadians(180-(-144)))
                .build();

        shootGate1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                PoseConstants.RED_GATE_AUTO_POSE,
                                controlPoint2,
                                controlPoint1,
                                new Pose(PoseConstants.FIELD_WIDTH-60.000, 72.000)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        HeadingInterpolator toGate = HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(
                        0,
                        0.75,
                        HeadingInterpolator.linear(shootGate1.getFinalHeadingGoal(), PoseConstants.RED_GATE_AUTO_POSE.getHeading())
                ),
                new HeadingInterpolator.PiecewiseNode(
                        0.75,
                        1,
                        HeadingInterpolator.constant(PoseConstants.RED_GATE_AUTO_POSE.getHeading())
                )
        );

        intakeGate1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(PoseConstants.FIELD_WIDTH-60.000, 72.000),
                                new Pose(PoseConstants.FIELD_WIDTH-45, PoseConstants.RED_GATE_AUTO_POSE.getY()),
                                PoseConstants.RED_GATE_AUTO_POSE
                        )
                )
                .setHeadingInterpolation(toGate)
                .setTValueConstraint(0.99)
                .build();

        intakeGate2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(PoseConstants.FIELD_WIDTH-60.000, 72.000),
                                new Pose(PoseConstants.FIELD_WIDTH-45, PoseConstants.RED_GATE_AUTO_POSE.getY()),
                                PoseConstants.RED_GATE_AUTO_POSE
                        )
                )
                .setHeadingInterpolation(toGate)
                .setTValueConstraint(0.99)
                .build();

        shootGate2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                PoseConstants.RED_GATE_AUTO_POSE,
                                controlPoint2,
                                controlPoint1,
                                new Pose(PoseConstants.FIELD_WIDTH-60.000, 72.000)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        intakeGate3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(PoseConstants.FIELD_WIDTH-60.000, 72.000),
                                new Pose(PoseConstants.FIELD_WIDTH-45, PoseConstants.RED_GATE_AUTO_POSE.getY()),
                                PoseConstants.RED_GATE_AUTO_POSE
                        )
                )
                .setHeadingInterpolation(toGate)
                .setTValueConstraint(0.99)
                .build();

        shootGate3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                PoseConstants.RED_GATE_AUTO_POSE,
                                controlPoint2,
                                controlPoint1,
                                new Pose(PoseConstants.FIELD_WIDTH-60.000, 72.000)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        intakeGate4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(PoseConstants.FIELD_WIDTH-60.000, 72.000),
                                new Pose(PoseConstants.FIELD_WIDTH-45, PoseConstants.RED_GATE_AUTO_POSE.getY()),
                                PoseConstants.RED_GATE_AUTO_POSE
                        )
                )
                .setHeadingInterpolation(toGate)
                .setTValueConstraint(0.99)
                .build();

        shootGate4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                PoseConstants.RED_GATE_AUTO_POSE,
                                controlPoint2,
                                controlPoint1,
                                new Pose(PoseConstants.FIELD_WIDTH-60.000, 72.000)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        // btw: 60 + 20 cos (-144), 72 + 20 sin -144, 12-15 cos(180), 48-15 sin 180 = 48
        // TODO: maybe go to the end of the last tile far down at an angle, and strafe up? to cover all space
        intakePile = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(PoseConstants.FIELD_WIDTH-60.000, 72.000),
                                new Pose(PoseConstants.FIELD_WIDTH-12.000, 34.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180-(-144)), Math.toRadians(180-125))
                .addPath(
                        new BezierLine(
                                new Pose(PoseConstants.FIELD_WIDTH-12.000, 34.000),
                                new Pose(PoseConstants.FIELD_WIDTH-12.000, 54.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180-125))
                .build();

        // TODO: in fact, we should have time to make a double path to bounce on both tiles. bounce far, then close so the close groups up the far
        // this will ensure that we cover both tiles, leaving nothing behind

        shootPile = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(PoseConstants.FIELD_WIDTH-12.000, 54.000),
                                new Pose(PoseConstants.FIELD_WIDTH-52.000, 88.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180-125), Math.toRadians(180-(-135)))

                .build();
//        follower.pathBuilder().addPath(
//                        new BezierCurve(
//                                new Pose(PoseConstants.FIELD_WIDTH-12.000, 48.000),
//                                new Pose(PoseConstants.FIELD_WIDTH-27.000, 48.000),
//                                new Pose(PoseConstants.FIELD_WIDTH-43.820, 60.244),
//                                new Pose(PoseConstants.FIELD_WIDTH-60.000, 72.000)
//                        )
//                ).setTangentHeadingInterpolation()
//                .setReversed()
//                .build();

        // TODO: no park for now its easy, we'll just move forward a bit
        // TODO: when porting to red use FIELD_WIDTH not 144 btw
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.usePredictiveBraking = true;
        follower.setStartingPose(startPose);
        robot = new AutonomousRobot(hardwareMap, Alliance.RED);
        sotm2 = new SOTM(goalPose);
        buildPaths();

        stateMachine = new StateMachine(
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootPreload, true);
                            currentShootPose = new Pose(PoseConstants.FIELD_WIDTH-24, 105, Math.toRadians(180-(-90)));
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
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                new State()
                        .onEnter(() -> {
                            follower.followPath(intakeSecond, true);
                            currentShootPose = new Pose(PoseConstants.FIELD_WIDTH-60, 72, Math.toRadians(180-(-144)));
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
                            follower.holdPoint(new BezierPoint(PoseConstants.RED_GATE_AUTO_POSE_IN), PoseConstants.RED_GATE_AUTO_POSE_IN.getHeading());
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
                            follower.holdPoint(new BezierPoint(PoseConstants.RED_GATE_AUTO_POSE_IN), PoseConstants.RED_GATE_AUTO_POSE_IN.getHeading());
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
                            follower.holdPoint(new BezierPoint(PoseConstants.RED_GATE_AUTO_POSE_IN), PoseConstants.RED_GATE_AUTO_POSE_IN.getHeading());
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
                            follower.holdPoint(new BezierPoint(PoseConstants.RED_GATE_AUTO_POSE_IN), PoseConstants.RED_GATE_AUTO_POSE_IN.getHeading());
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
                            currentShootPose = new Pose(PoseConstants.FIELD_WIDTH-52.000, 88.000, Math.toRadians(180-(-135)));
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
    }
    @Override
    public void loop() {
        // ALSO TODO: change the position to be a bit lower as we are slightly over the line
        // ALSO TODO: strengthen translational pid for pedro maybe it will make gate even more consistent
        // this will be better as we will still make it but not have to worry about inconsistent speed
        double[] turretValues = sotm2.calculateAzimuthThetaVelocityFRCBetter(currentShootPose, follower.getVelocity(), follower.getAngularVelocity());
        double[] shooterValues = sotm2.calculateAzimuthThetaVelocityFRCBetter(currentShootPose, new Vector(), follower.getAngularVelocity());
        // now it applies feedforward but target is same, so it will update slightly when shooting kinda like sotm.
        robot.setAzimuthThetaVelocity(new double[] {turretValues[0], shooterValues[1], shooterValues[2]});
        robot.turret.setFeedforward(turretValues[3]);

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
