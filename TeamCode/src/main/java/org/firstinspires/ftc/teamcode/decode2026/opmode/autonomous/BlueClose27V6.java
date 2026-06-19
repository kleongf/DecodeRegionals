package org.firstinspires.ftc.teamcode.decode2026.opmode.autonomous;

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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Disabled

@Autonomous(name="Blue Close 27 V6 Consistent", group="!")
public class BlueClose27V6 extends OpMode {
    private boolean doThirdSpike = false;
    private boolean doOpenGate = false;//no time to do this for 27
    private Pose lockedPose = new Pose();
    private boolean lockTurret = true;
    private boolean lockSpeed = true;
    private Follower follower;
    private StateMachine stateMachine;
    private CurrentRobot robot;
    private SOTMUtil sotm;
    private Intake.DetectionState prevState;
    private PathChain shootPreload, intakeFirst, shootFirst, intakeSecond, shootSecond, shootSecondNoOpenGate, openGate, intakeGate1, shootGate1, intakeGate2, shootGate2, intakeThird, shootThird, intakeGate3, shootGate3, intakeGate4, shootGate4, intakeGate5, shootGate5, intakePile, shootPile;

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
                                new Pose(56.000, 76.000)
                        )
                )
                // optimal angle trust defined by deriv of curve
                .setConstantHeadingInterpolation(Math.toRadians(-130.37))
                .build();

        intakeSecond = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(56.000, 76.000),
                                new Pose(46.741, 65.108),
                                new Pose(31.688, 59.731),
                                new Pose(13.000, 67.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();
//        shootPreload = follower.pathBuilder()
//                // say the bumper is 1 inch off, idk what it really is tho
//                .addPath(
//                        new BezierLine(
//                                new Pose(14.811+MathUtil.mmToIn(50), FieldConstants.FIELD_WIDTH * (5/6d) - FieldConstants.ROBOT_BACK_TO_CENTER_DISTANCE + 1),
//                                new Pose(24.000, 110.000)
//                        )
//
//        ).setConstantHeadingInterpolation(FieldConstants.BLUE_CLOSE_START_AUTO_POSE.getHeading()).build();
//
//        intakeFirst = follower.pathBuilder().addPath(
//                new BezierLine(
//                        new Pose(24.000, 110.000),
//                        new Pose(24.000, 83.000)
//                )
//        ).setConstantHeadingInterpolation(FieldConstants.BLUE_CLOSE_START_AUTO_POSE.getHeading()).build();
//
//        shootFirst = follower.pathBuilder().addPath(
//                        new BezierLine(
//                                new Pose(24.000, 83.000),
//                                new Pose(24.000, 110.000)
//                        )
//                )
//                .setConstantHeadingInterpolation(FieldConstants.BLUE_CLOSE_START_AUTO_POSE.getHeading())
//                .build();
//
//        intakeSecond = follower.pathBuilder().addPath(
//                        new BezierLine(
//                                new Pose(24.000, 110.000),
//                                new Pose(24.000, 62.000)
//                        )
//                ).setConstantHeadingInterpolation(FieldConstants.BLUE_CLOSE_START_AUTO_POSE.getHeading())
//                .build();

        openGate = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(24.00, 62.000),
                                FieldConstants.BLUE_SIDE_GATE_POSE
                        )
                ).setConstantHeadingInterpolation(FieldConstants.BLUE_SIDE_GATE_POSE.getHeading())
                .setNoDeceleration()
                .build();

        shootSecond = follower.pathBuilder().addPath(
                        new BezierLine(
                                FieldConstants.BLUE_SIDE_GATE_POSE,
                                new Pose(56.000, 75.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-160))
                //.setLinearHeadingInterpolation(FieldConstants.BLUE_SIDE_GATE_POSE.getHeading(), Math.toRadians(-160))
                .build();

        shootSecondNoOpenGate = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(24.00, 64.000),
                                new Pose(56.000, 75.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-160))
                // .setLinearHeadingInterpolation(FieldConstants.BLUE_SIDE_GATE_POSE.getHeading(), Math.toRadians(-160))
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

        HeadingInterpolator fromGate = HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(
                        0,
                        0.1,
                        //testing straightline path cuz faster
                        HeadingInterpolator.constant(FieldConstants.BLUE_GATE_AUTO_POSE_24.getHeading())
                        //HeadingInterpolator.linear(Math.toRadians(-160), FieldConstants.BLUE_GATE_AUTO_POSE.getHeading())
                ),
                new HeadingInterpolator.PiecewiseNode(
                        0.1,
                        1,
                        HeadingInterpolator.constant(Math.toRadians(-160))
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
                .setHeadingInterpolation(fromGate)
                // .setTangentHeadingInterpolation()
                // .setReversed()
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
                .setHeadingInterpolation(fromGate)
                // .setTangentHeadingInterpolation()
                // .setReversed()
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
                .setHeadingInterpolation(fromGate)
                //.setTangentHeadingInterpolation()
                //.setReversed()
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
                .setHeadingInterpolation(fromGate)
                // .setTangentHeadingInterpolation()
                // .setReversed()
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
                .setHeadingInterpolation(fromGate)
                // .setTangentHeadingInterpolation()
                // .setReversed()
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
                        new BezierLine(
                                new Pose(56, 75),
                                new Pose(12, 20)
                        )
//                        new BezierCurve(
//                                new Pose(56.000, 75.000),
//                                new Pose(19.864, 51.595),
//                                new Pose(17, 42.784),
//                                new Pose(15.000, 30.000),
//                                new Pose(15.000, 25.000)
//                        )
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
        follower.setMaxPower(1);
        follower.setStartingPose(FieldConstants.BLUE_CLOSE_START_AUTO_POSE);
        robot = new CurrentRobot(hardwareMap);
        sotm = new SOTMUtil(FieldConstants.BLUE_GOAL_POSE);
        lockedPose = new Pose(32, 108, FieldConstants.BLUE_CLOSE_START_AUTO_POSE.getHeading());
        ShootingConstants.tofMultiplier = 0.35;
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
                            follower.followPath(intakeFirst, true);
                            robot.intakeCommand.start();
                        })
                        .transition(new Transition(() -> follower.atParametricEnd())),
                new State()
                        .onEnter(() -> {
                            lockedPose = new Pose(56, 75);
                            lockSpeed = true;
                            lockTurret = false;
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
                // OPEN GATE?
                new State()
                        .onEnter(() -> {})
                        // if no open gate, go to shoot second state
                        .transition(new Transition(() -> follower.getCurrentTValue() > 0.85 && !doOpenGate, "shootSecond"))
                        .transition(new Transition(() -> follower.getCurrentTValue() > 0.85 && doOpenGate)),
                new State()
                        .onEnter(() -> {
                            follower.setMaxPower(1);
                            follower.followPath(openGate, true);
                        })
                        .transition(new Transition(() -> follower.atParametricEnd())),
                new State()
                        .maxTime(500),
                new State("shootSecond")
                        .onEnter(() -> {
                            if (doOpenGate) {
                                follower.followPath(shootSecond, true);
                            } else {
                                follower.followPath(shootSecondNoOpenGate, true);
                            }
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
                // DO THIRD SPIKE?
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
                            robot.prepareShootCommandLonger.start();
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
                            ShootingConstants.tofMultiplier = 0.75;
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
        if (lockSpeed && lockTurret) {
            shooterOutputs = sotm.calculateShooterOutputs(lockedPose, new Vector(), new Vector(), 0, RobotConstants.dt);
            shooterOutputs.wheelVelocity += 20;
        } else if (lockSpeed && !lockTurret) {
            shooterOutputs =
                    RobotConstants.useShootOnTheMove ?
                            sotm.calculateShooterOutputs2(lockedPose, new Vector(), new Vector(), follower.getAngularVelocity(), RobotConstants.dt, Alliance.BLUE) :
                            sotm.calculateShooterOutputs2(follower.getPose(), new Vector(), new Vector(), 0, RobotConstants.dt, Alliance.BLUE);

            ShootingConstants.ShooterOutputs shooterOutputsTurret =
                    RobotConstants.useShootOnTheMove ?
                            sotm.calculateShooterOutputs2(follower.getPose(), follower.getVelocity(), follower.getAcceleration(), follower.getAngularVelocity(), RobotConstants.dt, Alliance.BLUE) :
                            sotm.calculateShooterOutputs2(follower.getPose(), new Vector(), new Vector(), 0, RobotConstants.dt, Alliance.BLUE);

            shooterOutputs.turretAngle = shooterOutputsTurret.turretAngle;
            shooterOutputs.turretFeedforward = shooterOutputsTurret.turretFeedforward;
            // Log.d("turret vel", String.valueOf(robot.turret.currentVelocityTicks));
            // shooterOutputsTurret.turretFeedforward;
        } else {
            shooterOutputs =
                    RobotConstants.useShootOnTheMove ?
                            sotm.calculateShooterOutputs2(follower.getPose(), follower.getVelocity(), new Vector(), follower.getAngularVelocity(), RobotConstants.dt, Alliance.BLUE) :
                            sotm.calculateShooterOutputs2(follower.getPose(), new Vector(), new Vector(), 0, RobotConstants.dt, Alliance.BLUE);
        }

//        if (lockShooter) {
//            shooterOutputs = sotm.calculateShooterOutputs(lockedPose, new Vector(), new Vector(), 0, RobotConstants.dt);
//        } else {
//            shooterOutputs =
//                    RobotConstants.useShootOnTheMove ?
//                            sotm.calculateShooterOutputs(follower.getPose(), follower.getVelocity(), follower.getAcceleration(), follower.getAngularVelocity(), RobotConstants.dt) :
//                            sotm.calculateShooterOutputs(follower.getPose(), new Vector(), new Vector(), 0, RobotConstants.dt);
//        }

        robot.shooter.wantedVelocity = shooterOutputs.wheelVelocity;
        robot.shooter.wantedAcceleration = shooterOutputs.wheelFeedforward;
        robot.shooter.wantedPitch = shooterOutputs.hoodAngle;
        robot.turret.wantedAngle = shooterOutputs.turretAngle;
        robot.turret.wantedAngularVelocity = shooterOutputs.turretFeedforward;

        if ((robot.intake.detectionState == Intake.DetectionState.THIRD_TRIGGERED && prevState == Intake.DetectionState.SECOND_TRIGGERED)) {
            robot.ledIndicator.indicateIntakeFull();
        }
        prevState = robot.intake.detectionState;

        stateMachine.update();
        follower.update();
        robot.update();
        blackboard.put(FieldConstants.END_POSE_KEY, follower.getPose());
        Log.d("turret vel", String.valueOf(robot.turret.currentVelocityTicks));
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
