package org.firstinspires.ftc.teamcode.decode2026.opmode.autonomous;

import static java.lang.Thread.sleep;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Curve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.decode2026.CurrentRobot;
import org.firstinspires.ftc.teamcode.decode2026.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.decode2026.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.decode2026.constants.ShootingConstants;
import org.firstinspires.ftc.teamcode.decode2026.subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.decodeutil.Alliance;
import org.firstinspires.ftc.teamcode.util.decodeutil.Copier;
import org.firstinspires.ftc.teamcode.util.decodeutil.SOTMUtil;
import org.firstinspires.ftc.teamcode.util.fsm.State;
import org.firstinspires.ftc.teamcode.util.fsm.StateMachine;
import org.firstinspires.ftc.teamcode.util.fsm.Transition;

@Autonomous(name="Blue 30 Incapable Partner", group="!")
public class Blue30IncapablePartner extends OpMode {
    private Pose lockedPose = new Pose();
    private boolean lockShooter = false;
    private double turretOffset = 0;
    private double speedOffset = 0;
    private double minY = 8;
    private double maxY = 48;
    private Follower follower;
    private StateMachine stateMachine;
    private CurrentRobot robot;
    private SOTMUtil sotm;
    private PathChain intakeCorner, shootCorner, intakeThird, shootThird, intakeSecond, shootSecond, intakeFirst, shootFirst, intakeGate1, shootGate1, intakeGate2, shootGate2, intakePilePath, shootPilePath, park;
    private HeadingInterpolator toPile, fromPile;
    private boolean goodDetection = false;
    private int numPileCyclesWanted = 3;
    private int numPileCyclesCompleted = 0;

    public void buildPaths() {
        intakeThird = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                FieldConstants.BLUE_FAR_START_AUTO_SIDESPIKE_POSE,
                                new Pose(26.000, 15.000),
                                new Pose(26.000, 30.000)
                        )
                )
                .setConstantHeadingInterpolation(FieldConstants.BLUE_FAR_START_AUTO_SIDESPIKE_POSE.getHeading())
                .build();

        shootThird = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(26.000, 30.000),
                                new Pose(50.000, 12.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        intakeCorner = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(50.000, 12.000), new Pose(9, 8)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        // this one is different: 160 degrees, this is so we can see stuff better
        shootCorner = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(9, 8), new Pose(50, 16)))
                .setConstantHeadingInterpolation(Math.toRadians(160))
                .build();

        intakeSecond = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(50.000, 16.000),
                                new Pose(40.000, 63.000),
                                new Pose(12.000, 63.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        shootSecond = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(12.000, 63.000),
                                new Pose(50.000, 82.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        intakeFirst = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(50.000, 82.000),
                                new Pose(17.000, 82.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        shootFirst = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(17.000, 82.000),
                                new Pose(57, 76)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        HeadingInterpolator toGate = HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(
                        0,
                        0.25,
                        HeadingInterpolator.constant(Math.toRadians(-160))
                ),
                new HeadingInterpolator.PiecewiseNode(
                        0.25,
                        1,
                        HeadingInterpolator.constant(FieldConstants.BLUE_GATE_AUTO_POSE_24.getHeading())
                )
        );

        intakeGate1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(57.000, 76.000),
                                FieldConstants.BLUE_GATE_AUTO_POSE_24
                        )
                )
                .setHeadingInterpolation(toGate)
                .setTValueConstraint(0.95)
                .build();

        shootGate1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                FieldConstants.BLUE_GATE_AUTO_POSE_24,
                                new Pose(57, 76)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        intakeGate2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(57.000, 76.000),
                                FieldConstants.BLUE_GATE_AUTO_POSE_24
                        )
                )
                .setHeadingInterpolation(toGate)
                .setTValueConstraint(0.95)
                .build();

        shootGate2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                FieldConstants.BLUE_GATE_AUTO_POSE_24,
                                new Pose(50, 16)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
        park = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(50, 16), new Pose(30, 12))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    // tangent heading interp then linear to 180 at end for paths

    // idea 2 is to not use the new spline, just go to the x-coordinate of the largest one
    // idea 3 is to find the 2 largest ones, go to the x-pos of the first, and sweep up to the second

    // i like idea 2 more if we can shoot 2 balls
    // if after a state we still have 1 ball or less, go camp at gate, do the gate camp pathing
    // after this, this puts us in the same state as the no detection state: waiting at gate for balls
    // wait for like 3s idk or until 3 balls

    // retune flywheel kA

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(FieldConstants.BLUE_FAR_START_AUTO_SIDESPIKE_POSE);
        follower.usePredictiveBraking = true;
        robot = new CurrentRobot(hardwareMap);
        sotm = new SOTMUtil(FieldConstants.BLUE_GOAL_POSE);
        turretOffset = Math.toRadians(1.75);
        speedOffset = 30;
        // dont compensate for velo but pos
        // ShootingConstants.tofMultiplier = 0;
        buildPaths();

        stateMachine = new StateMachine(
                // preload
                new State()
                        .maxTime(3000) // in case it takes too long.
                        // .onEnter(() -> robot.prepareShootCommandLonger.start())
                        .transition(new Transition(() -> robot.shooter.atTarget(30) && !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            if(robot.intake.isFull){
                                robot.shootCommandSlow.start();
                            }
                            else{
                                robot.shootCommandFast.start();
                            }
                            follower.setMaxPower(1);
                        })
                        .maxTime(500),
                // corner
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.followPath(intakeThird, true);
                        })
                        .transition(new Transition(() -> follower.atParametricEnd()))
                        .maxTime(1200),
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootThird, true);
                            // turretOffset = Math.toRadians(2);
                            speedOffset = -10;
                            // lockedPose = new Pose(50, 16, Math.toRadians(180));
                            lockShooter = false;
                        })
                        .transition(new Transition(() -> !follower.isBusy())),

                new State()
                        .maxTime(100),
                new State()
                        .onEnter(() -> {
                            robot.shootCommandSlow.start();
                        })
                        .maxTime(500),
                // third
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.followPath(intakeCorner, true);
                        })
                        .transition(new Transition(() -> follower.atParametricEnd()))
                        .maxTime(1200),
                new State()
                        .onEnter(() -> follower.followPath(shootCorner, true))
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .maxTime(100),
                new State()
                        .onEnter(() -> {
                            robot.shootCommandSlow.start();
                        })
                        .maxTime(500),
                // second
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.followPath(intakeSecond, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootSecond, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            robot.shootCommand.start();
                        })
                        .maxTime(300),
                // first
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.followPath(intakeFirst, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootFirst, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            robot.shootCommand.start();
                        })
                        .maxTime(300),
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
                        .maxTime(1900),
                new State()
                        .onEnter(() -> {
                            follower.setMaxPower(1);
                            follower.followPath(shootGate1, true);
                            robot.prepareShootCommandLonger.start();
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .maxTime(300),
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
                        .maxTime(2000),
                new State()
                        .onEnter(() -> {
                            follower.setMaxPower(1);
                            follower.followPath(shootGate2, true);
                            robot.prepareShootCommandLonger.start();
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommand.start())
                        .maxTime(300),
                // 3 pile cycles
                new State("startCycle")
                        .onEnter(() -> {
                            double intakeY = robot.artifactVision.findBestYPosition(follower.getPose(), minY, maxY);
                            if (intakeY == -1) {
                                goodDetection = false;
                                intakePilePath = follower.pathBuilder()
                                        .addPath(
                                                new BezierCurve(
                                                        new Pose(50.000, 16.000),
                                                        new Pose(28.000, minY),
                                                        new Pose(10, minY)
                                                )
                                        )
                                        .setTangentHeadingInterpolation()
                                        .build();
                            } else {
                                goodDetection = true;
                                intakePilePath = follower.pathBuilder()
                                        .addPath(
                                                new BezierCurve(
                                                        follower.getPose(),
                                                        new Pose(28, intakeY),
                                                        new Pose(10, intakeY)
                                                )
                                        )
                                        .setTangentHeadingInterpolation()
                                        .build();
                            }
                            follower.followPath(intakePilePath, true);
                        })
                        .maxTime(1500)
                        .transition(new Transition(() -> numPileCyclesCompleted >= numPileCyclesWanted, "park"))
                        .transition(new Transition(() -> follower.getCurrentTValue() > 0.9, "shootPath"))
                        .transition(new Transition(() -> robot.intake.isFull && follower.getCurrentTValue() > 0.5, "shootPath")),
                new State("shootPath")
                        .onEnter(() -> {
                            if (robot.intake.isFull) {
                                robot.intake.wantedMode = Intake.Mode.INTAKE_OFF;
                            }
                            shootPilePath = follower.pathBuilder()
                                    .addPath(
                                            new BezierLine(
                                                    follower.getPose(),
                                                    new Pose(50, 16)
                                            )
                                    )
                                    .setHeadingInterpolation(fromPile)
                                    .build();
                            follower.followPath(shootPilePath, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .maxTime(100),
                new State()
                        .onEnter(() -> {
                            robot.shootCommandSlow.start();
                            numPileCyclesCompleted += 1;
                        })
                        .transition(new Transition(() -> robot.shootCommandSlow.isFinished(), "startCycle")),
                new State("park")
                        .onEnter(() -> {
                            follower.followPath(park, true);
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
        ShootingConstants.ShooterOutputs shooterOutputs;

        if (lockShooter) {
            shooterOutputs = sotm.calculateShooterOutputs(lockedPose, new Vector(), new Vector(), 0, RobotConstants.dt);
        } else {
            shooterOutputs =
                    RobotConstants.useShootOnTheMove ?
                            sotm.calculateShooterOutputs2(follower.getPose(), follower.getVelocity(), follower.getAcceleration(), follower.getAngularVelocity(), RobotConstants.dt, Alliance.BLUE) :
                            sotm.calculateShooterOutputs2(follower.getPose(), new Vector(), new Vector(), 0, RobotConstants.dt, Alliance.BLUE);
        }

        robot.shooter.wantedVelocity = shooterOutputs.wheelVelocity + speedOffset;
        robot.shooter.wantedAcceleration = shooterOutputs.wheelFeedforward;
        robot.shooter.wantedPitch = shooterOutputs.hoodAngle;
        robot.turret.wantedAngle = shooterOutputs.turretAngle + turretOffset;
        robot.turret.wantedAngularVelocity = shooterOutputs.turretFeedforward;

        stateMachine.update();
        follower.update();
        robot.update();
        blackboard.put(FieldConstants.END_POSE_KEY, follower.getPose());
        // telemetry.update();
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
