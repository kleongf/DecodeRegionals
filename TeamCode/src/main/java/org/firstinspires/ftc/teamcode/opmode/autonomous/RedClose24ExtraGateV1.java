package org.firstinspires.ftc.teamcode.opmode.autonomous;

// set no decel on first intake so we just take it all in
// we also need to set up the timer thing, see how long it takes for us to do a gate cycle and finish with the spike mark
// i will put the states here but this we will have to tune

// also when porting to red: do we do 144- or 141- ?
// because if it goes to the wrong spot on the vis but its the correct spot irl
// then we are alright

// upon thinking about if for a little while, we should do 144-
// this would make our auto alright because the start pose is correct so stuff doesn't matter

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

@Autonomous(name="Red Close 24 Extra Gate V1", group="!")
public class RedClose24ExtraGateV1 extends OpMode {
    private Follower follower;
    private StateMachine stateMachine;
    private AutonomousRobot robot;
    private SOTM sotm2;
    private final Pose startPose = PoseConstants.RED_CLOSE_AUTO_POSE;
    private final Pose goalPose = PoseConstants.RED_GOAL_POSE;
    private final Pose shootPose = new Pose(144-60, 72, Math.toRadians(180-(-144)));
    private Pose currentShootPose = new Pose(144-60, 72, Math.toRadians(180-(-144)));
    private PathChain shootPreloadIntakeSecond, shootSecond, intakeGate1, shootGate1, intakeGate2, shootGate2, intakeGate3, shootGate3, intakeGate4, shootGate4, intakeGate5, shootGate5, intakeFirst, shootFirst;
    private ArrayList<Vector> rollingVelocities;
    private ElapsedTime elapsedTime;
    private Pose prevPose;
    private int numVelocities = 4;
    private boolean isSOTMing = true;

    private Vector getRollingVelocity() {
        Vector currentSum = new Vector();
        if (rollingVelocities.isEmpty()) {
            return currentSum;
        }
        for (Vector v: rollingVelocities) {
            currentSum = MathUtil.addVectors(currentSum, v);
        }
        Vector movingAverage = MathUtil.scalarMultiplyVector(currentSum, 1d / rollingVelocities.size());
        return movingAverage;
    }

    public void buildPaths() {
        double k1 = 10; // i actually forgot what these were check desmos
        double k2 = 20;
        // idk abt this just check it again and do for control point 2
        Pose controlPoint1 = new Pose(shootPose.getX() + k1 * Math.cos(shootPose.getHeading()), shootPose.getY() + k1 * Math.sin(shootPose.getHeading()));
        Pose controlPoint2 = new Pose(PoseConstants.RED_GATE_AUTO_POSE.getX() - k2 * Math.cos(PoseConstants.RED_GATE_AUTO_POSE.getHeading()), PoseConstants.RED_GATE_AUTO_POSE.getY() - k2 * Math.sin(PoseConstants.RED_GATE_AUTO_POSE.getHeading()));


        shootPreloadIntakeSecond = follower.pathBuilder().addPath(
                        new BezierLine(
                                startPose,
                                new Pose(144-54.000, 90.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180-(-110)))
                .addPath(
                        new BezierCurve(
                                new Pose(144-54, 90),
                                new Pose(144-40, 60),
                                new Pose(144-12, 60)
                        )
                )
                .setTangentHeadingInterpolation()
                .setNoDeceleration()
                .build();

        shootSecond = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(144-15.000, 60.000),
                                new Pose(144-47.8647450844, 63.1832212156), // k = 15: 60+cos-144, 72 + sin 144
                                new Pose(144-60.000, 72.000)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        HeadingInterpolator toGate = HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(
                        0,
                        0.75,
                        HeadingInterpolator.linear(shootSecond.getFinalHeadingGoal(), PoseConstants.RED_GATE_AUTO_POSE.getHeading())
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
                                new Pose(144-60.000, 72.000),
                                new Pose(144-45, PoseConstants.RED_GATE_AUTO_POSE.getY()),
                                PoseConstants.RED_GATE_AUTO_POSE
                        )
                )
                .setHeadingInterpolation(toGate)
                .setTValueConstraint(0.99)
                .build();

        // optimal coordinates trust. both the second intake and this path end at the same heading.
        shootGate1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                PoseConstants.RED_GATE_AUTO_POSE,
                                controlPoint2,
                                controlPoint1,
                                new Pose(144-60.000, 72.000)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        intakeGate2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(144-60.000, 72.000),
                                new Pose(144-45, PoseConstants.RED_GATE_AUTO_POSE.getY()),
                                PoseConstants.RED_GATE_AUTO_POSE
                        )
                )
                .setHeadingInterpolation(toGate)
                .setTValueConstraint(0.99)
                .build();

        // optimal coordinates trust. both the second intake and this path end at the same heading.
        shootGate2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                PoseConstants.RED_GATE_AUTO_POSE,
                                controlPoint2,
                                controlPoint1,
                                new Pose(144-60.000, 72.000)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        intakeGate3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(144-60.000, 72.000),
                                new Pose(144-45, PoseConstants.RED_GATE_AUTO_POSE.getY()),
                                PoseConstants.RED_GATE_AUTO_POSE
                        )
                )
                .setHeadingInterpolation(toGate)
                .setTValueConstraint(0.99)
                .build();

        // optimal coordinates trust. both the second intake and this path end at the same heading.
        shootGate3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                PoseConstants.RED_GATE_AUTO_POSE,
                                controlPoint2,
                                controlPoint1,
                                new Pose(144-60.000, 72.000)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        intakeGate4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(144-60.000, 72.000),
                                new Pose(144-45, PoseConstants.RED_GATE_AUTO_POSE.getY()),
                                PoseConstants.RED_GATE_AUTO_POSE
                        )
                )
                .setHeadingInterpolation(toGate)
                .setTValueConstraint(0.99)
                .build();

        // optimal coordinates trust. both the second intake and this path end at the same heading.
        shootGate4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                PoseConstants.RED_GATE_AUTO_POSE,
                                controlPoint2,
                                controlPoint1,
                                new Pose(144-60.000, 72.000)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        intakeGate5 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(144-60.000, 72.000),
                                new Pose(144-45, PoseConstants.RED_GATE_AUTO_POSE.getY()),
                                PoseConstants.RED_GATE_AUTO_POSE
                        )
                )
                .setHeadingInterpolation(toGate)
                .setTValueConstraint(0.99)
                .build();

        shootGate5 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                PoseConstants.RED_GATE_AUTO_POSE,
                                new Pose(144-48,70),
                                new Pose(144-54, 84)
                        )
                )
                .setLinearHeadingInterpolation(PoseConstants.RED_GATE_AUTO_POSE.getHeading(), Math.toRadians(180-180))
                .build();

        intakeFirst = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(144-54.000, 84.000), new Pose(144-18.000, 84.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180-180))
                .build();


        shootFirst = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(144-18.000, 84.000), new Pose(144-48, 114))
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
        robot = new AutonomousRobot(hardwareMap, Alliance.RED);
        sotm2 = new SOTM(goalPose);
        rollingVelocities = new ArrayList<>();
        elapsedTime = new ElapsedTime();
        prevPose = startPose;
        buildPaths();

        stateMachine = new StateMachine(
                // TODO: when pathing is finalized, next priority is SOTM and driver automations
                // i believe we can save at least a second with sotm at beginning.
                // shoot preload and intake second
                new State()
                        .onEnter(() -> {
                            follower.setMaxPower(0.75);
                            follower.followPath(shootPreloadIntakeSecond, false);
                            robot.intake.state = Intake.IntakeState.INTAKE_SLOW;
                        })
                        .maxTime(1000),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .transition(new Transition(() -> robot.shootCommand.isFinished())),
                new State()
                        .onEnter(() -> {
                            follower.setMaxPower(1);
                            robot.intakeCommand.start();
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                // second
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootSecond, true);
                            isSOTMing = false;
                            robot.turret.setPDCoefficients(0.01, 0.0005);
                            robot.intakeCommand.start();
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
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            follower.holdPoint(new BezierPoint(PoseConstants.RED_GATE_AUTO_POSE_IN), PoseConstants.RED_GATE_AUTO_POSE_IN.getHeading());
                        })
                        .minTime(600)
                        .transition(new Transition(() -> robot.intake.intakeFull()))
                        // currently setting all to 1000. if it is possible at 1s then it is possible. if not i should prob give up.
                        .maxTime(3000),
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
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            follower.holdPoint(new BezierPoint(PoseConstants.RED_GATE_AUTO_POSE_IN), PoseConstants.RED_GATE_AUTO_POSE_IN.getHeading());
                        })
                        .minTime(600)
                        .transition(new Transition(() -> robot.intake.intakeFull()))
                        // currently setting all to 1000. if it is possible at 1s then it is possible. if not i should prob give up.
                        .maxTime(3000),
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
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            follower.holdPoint(new BezierPoint(PoseConstants.RED_GATE_AUTO_POSE_IN), PoseConstants.RED_GATE_AUTO_POSE_IN.getHeading());
                        })
//                        .minTime(600)
//                        .transition(new Transition(() -> robot.intake.intakeFull()))
                        .minTime(600)
                        .transition(new Transition(() -> robot.intake.intakeFull()))
                        // currently setting all to 1000. if it is possible at 1s then it is possible. if not i should prob give up.
                        .maxTime(3000),
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
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            follower.holdPoint(new BezierPoint(PoseConstants.RED_GATE_AUTO_POSE_IN), PoseConstants.RED_GATE_AUTO_POSE_IN.getHeading());
                        })
//                        .minTime(600)
//                        .transition(new Transition(() -> robot.intake.intakeFull()))
                        .minTime(600)
                        .transition(new Transition(() -> robot.intake.intakeFull()))
                        // currently setting all to 1000. if it is possible at 1s then it is possible. if not i should prob give up.
                        .maxTime(3000),
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
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            follower.holdPoint(new BezierPoint(PoseConstants.RED_GATE_AUTO_POSE_IN), PoseConstants.RED_GATE_AUTO_POSE_IN.getHeading());
                            currentShootPose = new Pose(144-54, 84, Math.toRadians(180-180));
                        })
//                        .minTime(600)
//                        .transition(new Transition(() -> robot.intake.intakeFull()))
                        .minTime(600)
                        .transition(new Transition(() -> robot.intake.intakeFull()))
                        // currently setting all to 1000. if it is possible at 1s then it is possible. if not i should prob give up.
                        .maxTime(3000),
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
                            currentShootPose = new Pose(144-48, 114, Math.toRadians(180-(-135)));
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
        if (isSOTMing) {
            double[] values = sotm2.calculateAzimuthThetaVelocityFRCBetter(follower.getPose(), follower.getVelocity(), follower.getAngularVelocity());
            robot.setAzimuthThetaVelocity(new double[] {values[0], values[1], values[2]});
            robot.turret.setFeedforward(values[3]);
        } else {
            double[] values = sotm2.calculateAzimuthThetaVelocityFRCBetter(currentShootPose, new Vector(), follower.getAngularVelocity());
            robot.setAzimuthThetaVelocity(new double[] {values[0], values[1], values[2]});
            robot.turret.setFeedforward(0);
        }

        // I am starting to believe that the turret is off because it's not reaching its target
        // let's confirm, the p and d values are really low
        Log.d("turret current pos", "" + robot.turret.getCurrent());
        Log.d("turret target pos", "" + robot.turret.getTarget());

        stateMachine.update();
        follower.update();

        double dt = elapsedTime.seconds();

        Vector currentVelocity;
        if (dt > 0) {
            currentVelocity = follower.getPose().minus(prevPose).div(dt).getAsVector();
        } else {
            currentVelocity = follower.getVelocity();
        }

        if (rollingVelocities.size() < numVelocities) {
            for (int i = 0; i < numVelocities; i++) {
                rollingVelocities.add(currentVelocity);
            }
        } else {
            rollingVelocities.add(currentVelocity);
            rollingVelocities.remove(0);
        }

        robot.update();
        // TODO: BRUH why did i never do this? of course we should update every loop just in case
        blackboard.put(RobotConstants.END_POSE_KEY, follower.getPose());

        elapsedTime.reset();
        prevPose = follower.getPose();
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