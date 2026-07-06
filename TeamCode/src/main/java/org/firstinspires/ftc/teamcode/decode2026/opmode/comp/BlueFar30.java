package org.firstinspires.ftc.teamcode.decode2026.opmode.comp;

import static java.lang.Thread.sleep;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.decode2026.CurrentRobot;
import org.firstinspires.ftc.teamcode.decode2026.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.decode2026.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.decode2026.constants.ShootingConstants;
import org.firstinspires.ftc.teamcode.decode2026.subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.decodeutil.Alliance;
import org.firstinspires.ftc.teamcode.util.decodeutil.MathUtil;
import org.firstinspires.ftc.teamcode.util.decodeutil.SOTMUtil;
import org.firstinspires.ftc.teamcode.util.fsm.State;
import org.firstinspires.ftc.teamcode.util.fsm.StateMachine;
import org.firstinspires.ftc.teamcode.util.fsm.Transition;

@Autonomous(name="Blue Far 30/33 Vision + park", group="!")
public class BlueFar30 extends OpMode {
    private Pose lockedPose = new Pose();
    private boolean lockShooter = false;
    private double turretOffset = 0;
    private double speedOffset = 0;
    private final double minY = 8;
    private final double maxY = 48;
    private final int numCyclesWanted = 8;
    private int numCyclesCompleted = 0;
    private Follower follower;
    private StateMachine stateMachine;
    private CurrentRobot robot;
    private SOTMUtil sotm;
    private PathChain intakeCorner, shootCorner, intakeThird, shootThird, intakeGate, currentIntakePath, currentShootPath, park;
    private HeadingInterpolator fromPile;
    private ElapsedTime elapsedTime;

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

        intakeGate = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(50.000, 16.000),
                                new Pose(28.000, maxY),
                                new Pose(10.000, maxY)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        park = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(50, 16), new Pose(48, 24))
                )
                .setConstantHeadingInterpolation(Math.toRadians(135))
                .build();

        fromPile = HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(
                        0,
                        0.5,
                        HeadingInterpolator.tangent.reverse()
                ),
                new HeadingInterpolator.PiecewiseNode(
                        0.5,
                        1,
                        HeadingInterpolator.constant(Math.toRadians(165))
                )
        );
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(FieldConstants.BLUE_FAR_START_AUTO_SIDESPIKE_POSE);
        follower.usePredictiveBraking = true;
        robot = new CurrentRobot(hardwareMap);
        sotm = new SOTMUtil(FieldConstants.BLUE_GOAL_POSE);
        elapsedTime = new ElapsedTime();
        turretOffset = Math.toRadians(0);
        speedOffset = 0;
        buildPaths();

        stateMachine = new StateMachine(
                // preload
                new State()
                        .onEnter(() -> elapsedTime.reset())
                        .maxTime(3000) // in case it takes too long.
                        .transition(new Transition(() -> robot.shooter.atTarget(30) && !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            if (robot.intake.isFull) {
                                robot.shootCommandSlow.start();
                            } else {
                                robot.shootCommandFast.start();
                            }
                            follower.setMaxPower(1);
                        })
                        .maxTime(500),
                // third
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.followPath(intakeThird, true);
                        })
                        .transition(new Transition(() -> follower.atParametricEnd()))
                        .maxTime(1200),
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootThird, 0.8, true);
                            // turretOffset = Math.toRadians(2);
                            speedOffset = 0;
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
                // corner
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
                // cycles
                new State("startCycle")
                        .onEnter(() -> {
                            double intakeY = MathUtil.clamp(robot.artifactVision.findBestYPosition(follower.getPose(), 0, maxY), minY, maxY);
                            if (intakeY == -1) {
                                currentIntakePath = intakeGate;
                            } else {
                                currentIntakePath = follower.pathBuilder()
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
                            follower.followPath(currentIntakePath, true);
                        })
                        .maxTime(1500)
                        .transition(new Transition(() -> follower.getCurrentTValue() > 0.9, "shootPath"))
                        .transition(new Transition(() -> robot.intake.isFull && follower.getCurrentTValue() > 0.5, "shootPath")),
                new State("shootPath")
                        .onEnter(() -> {
                            if (robot.intake.isFull) {
                                robot.intake.wantedMode = Intake.Mode.INTAKE_OFF;
                            }
                            currentShootPath = follower.pathBuilder()
                                    .addPath(
                                            new BezierLine(
                                                    follower.getPose(),
                                                    new Pose(50, 16)
                                            )
                                    )
                                    .setHeadingInterpolation(fromPile)
                                    .build();
                            follower.followPath(currentShootPath, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                // maybe take away, will give us extra 0.7s
                new State()
                        .maxTime(100),
                new State()
                        .onEnter(() -> {
                            robot.shootCommandSlow.start();
                            numCyclesCompleted++;
                        })
                        .transition(new Transition(() -> robot.shootCommandSlow.isFinished())),
                new State()
                        .transition(new Transition(() -> elapsedTime.seconds() > 27 || numCyclesCompleted >= numCyclesWanted, "park"))
                        .transition(new Transition(() -> elapsedTime.seconds() <= 27 && numCyclesCompleted < numCyclesWanted, "startCycle")),
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
            shooterOutputs = sotm.calculateShooterOutputs(lockedPose, new Vector(), new Vector(), 0, RobotConstants.dt, Alliance.BLUE);
        } else {
            shooterOutputs =
                    RobotConstants.useShootOnTheMove ?
                            sotm.calculateShooterOutputs(follower.getPose(), follower.getVelocity(), follower.getAcceleration(), follower.getAngularVelocity(), RobotConstants.dt, Alliance.BLUE) :
                            sotm.calculateShooterOutputs(follower.getPose(), new Vector(), new Vector(), 0, RobotConstants.dt, Alliance.BLUE);
        }

        robot.shooter.wantedVelocity = -400;
        robot.shooter.wantedAcceleration = shooterOutputs.wheelFeedforward;
        robot.shooter.wantedPitch = shooterOutputs.hoodAngle;
        robot.turret.wantedAngle = Math.toRadians(180);
        robot.turret.wantedAngularVelocity = 0;

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