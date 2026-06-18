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
import org.firstinspires.ftc.teamcode.util.fsm.State;
import org.firstinspires.ftc.teamcode.util.fsm.StateMachine;
import org.firstinspires.ftc.teamcode.util.fsm.Transition;

// last one is going to be a pile cycle, it will always be a pile cycle

// combinations:
// 24 third spike open gate
// 24 third spike no open gate
// 24 pile cycle (extra gate cycle) open gate
// 24 pile cycle (extra gate cycle) no open gate

@Autonomous(name="Blue Close 24 Just Paths", group="!")
public class BlueClose24JustPaths extends OpMode {
    private boolean doThirdSpike = false;
    private boolean doOpenGate = true;//todo: just testing for now
    private Follower follower;
    private StateMachine stateMachine;
    private CurrentRobot robot;
    private SOTMUtil sotm;
    private Intake.DetectionState prevState;
    private PathChain shootPreload, intakeFirst, shootFirst, intakeSecond, shootSecond, intakeGate1, shootGate1, intakeGate2, shootGate2, intakeGate3, shootGate3, intakeGate4, shootGate4,intakePile, shootPile;

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
                                new Pose(23.500, 64.000),
                                new Pose(56.000, 75.000)
                        )
                ).setLinearHeadingInterpolation(FieldConstants.BLUE_SIDE_GATE_POSE.getHeading(), Math.toRadians(-160))
                .build();

        HeadingInterpolator toGate = HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(
                        0,
                        0.3,
                        //testing straightline path cuz faster
                        HeadingInterpolator.constant(Math.toRadians(-160))
                        //HeadingInterpolator.linear(Math.toRadians(-160), FieldConstants.BLUE_GATE_AUTO_POSE.getHeading())
                ),
                new HeadingInterpolator.PiecewiseNode(
                        0.3,
                        1,
                        HeadingInterpolator.constant(FieldConstants.BLUE_GATE_AUTO_POSE_24.getHeading())
                )
        );

        intakeGate1 = follower.pathBuilder()
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



        intakePile = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(56.000, 75.000),
                                new Pose(16.759, 44.568),
                                new Pose(12.000, 25.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        shootPile = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(12.000, 20.000),
                                new Pose(56.000, 100.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
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

        telemetry.addLine("x:" + follower.getPose().getX());
        telemetry.addLine("y:" + follower.getPose().getY());
        telemetry.addLine("heading:" + follower.getPose().getHeading());
        telemetry.addLine("total heading:" + follower.getTotalHeading());
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.usePredictiveBraking = true;
        follower.setMaxPower(1);
        follower.setStartingPose(FieldConstants.BLUE_CLOSE_START_AUTO_POSE);
        robot = new CurrentRobot(hardwareMap);
        sotm = new SOTMUtil(FieldConstants.BLUE_GOAL_POSE);
        ShootingConstants.tofMultiplier = 0.35;
        buildPaths();

        stateMachine = new StateMachine(
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootPreload, true);
                        })
                        .transition(new Transition(() -> follower.atParametricEnd())),
                new State()
                        .maxTime(500),//added wait so less sotm cuz (i think) we have time
                new State()
                        .onEnter(() -> {
                            follower.followPath(intakeFirst, true);
                        })
                        .transition(new Transition(() -> follower.atParametricEnd())),
                new State()
                        .maxTime(300),
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootFirst, true);
                        })
                        .transition(new Transition(() -> follower.atParametricEnd())),
                new State()
                        .maxTime(100),
                new State()
                        .onEnter(() -> {
                            follower.followPath(intakeSecond, true);
                        })
                        .maxTime(400),
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootSecond, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .maxTime(100),
                // gate cycle 1
                new State()
                        .onEnter(() -> {
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
                            
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .maxTime(100),
                
                // gate cycle 2
                new State()
                        .onEnter(() -> {
                            
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
                            
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .maxTime(100),
                

                // gate cycle 3
                new State()
                        .onEnter(() -> {
                            
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
                            
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                // DO THIRD SPIKE?
                new State()
                        .maxTime(100),
                new State()
                        .onEnter(() -> {
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
                            
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .maxTime(100),
                // pile intake
                new State()
                        .onEnter(() -> {
                            
                            follower.followPath(intakePile, false);
                        })
                        .transition(new Transition(() -> follower.atParametricEnd())),
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootPile, true);
                            ShootingConstants.tofMultiplier = .8;
                        })
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
    public void loop() {
        ShootingConstants.ShooterOutputs shooterOutputs =
                RobotConstants.useShootOnTheMove ?
                        sotm.calculateShooterOutputs(follower.getPose(), follower.getVelocity(), follower.getAcceleration(), follower.getAngularVelocity(), RobotConstants.dt) :
                        sotm.calculateShooterOutputs(follower.getPose(), new Vector(), new Vector(), 0, RobotConstants.dt);

        robot.shooter.wantedVelocity = shooterOutputs.wheelVelocity;
        robot.shooter.wantedAcceleration = shooterOutputs.wheelFeedforward;
        robot.shooter.wantedPitch = shooterOutputs.hoodAngle;
        
        if (robot.intake.detectionState == Intake.DetectionState.THIRD_TRIGGERED && prevState == Intake.DetectionState.SECOND_TRIGGERED) {
            robot.ledIndicator.indicateIntakeFull();
        }
        prevState = robot.intake.detectionState;

        robot.turret.wantedAngle = shooterOutputs.turretAngle;
        robot.turret.wantedAngularVelocity = shooterOutputs.turretFeedforward;

        stateMachine.update();
        follower.update();
        //robot.update();
        blackboard.put(FieldConstants.END_POSE_KEY, follower.getPose());
        telemetry.addLine("x:" + follower.getPose().getX());
        telemetry.addLine("y:" + follower.getPose().getY());
        telemetry.addLine("heading:" + follower.getPose().getHeading());
        telemetry.addLine("total heading:" + follower.getTotalHeading());
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