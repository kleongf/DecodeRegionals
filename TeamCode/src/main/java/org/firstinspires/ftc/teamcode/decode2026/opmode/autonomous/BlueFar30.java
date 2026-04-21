package org.firstinspires.ftc.teamcode.decode2026.opmode.autonomous;

import static java.lang.Thread.sleep;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.decode2026.CurrentRobot;
import org.firstinspires.ftc.teamcode.decode2026.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.decode2026.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.decode2026.constants.ShootingConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.decodeutil.Copier;
import org.firstinspires.ftc.teamcode.util.decodeutil.SOTMUtil;
import org.firstinspires.ftc.teamcode.util.fsm.State;
import org.firstinspires.ftc.teamcode.util.fsm.StateMachine;
import org.firstinspires.ftc.teamcode.util.fsm.Transition;

@Autonomous(name="Blue Far 30", group="!")
public class BlueFar30 extends OpMode {
    // TODO: we might want to do 30 or park, idk how much time we have.
    private Follower follower;
    private StateMachine stateMachine;
    private CurrentRobot robot;
    private SOTMUtil sotm;
    private PathChain intakeCorner, shootCorner, intakeThird, shootThird, intakePileLowCycle, shootPileLowCycle, intakePileHighCycle, shootPileHighCycle, intakePile1, shootPile1, intakePile2, shootPile2, intakePile3, shootPile3, intakePile4, shootPile4, intakePile5, shootPile5, intakePile6, shootPile6, intakePile7, shootPile7, park;

    public void buildPaths() {
        intakeCorner = follower.pathBuilder()
                .addPath(new BezierLine(FieldConstants.BLUE_FAR_AUTO_START_POSE, new Pose(9, 10)))
                .setConstantHeadingInterpolation(FieldConstants.BLUE_FAR_AUTO_START_POSE.getHeading())
                .build();

        shootCorner = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(9, 10), new Pose(50, 10)))
                .setConstantHeadingInterpolation(FieldConstants.BLUE_FAR_AUTO_START_POSE.getHeading())
                .build();

        intakeThird = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(50, 10),
                                new Pose(40.000, 35.000),
                                new Pose(30.000, 35.000),
                                new Pose(10.000, 35.000)
                        )
                )
                .setConstantHeadingInterpolation(FieldConstants.BLUE_FAR_AUTO_START_POSE.getHeading())
                .build();

        shootThird = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(10.000, 35.000), new Pose(50, 10))
                )
                .setConstantHeadingInterpolation(FieldConstants.BLUE_FAR_AUTO_START_POSE.getHeading())
                .build();

        intakePileLowCycle = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(50.000, 10.000),
                                new Pose(9.000, 10.000)
                        )
                )
                .setConstantHeadingInterpolation(FieldConstants.BLUE_FAR_AUTO_START_POSE.getHeading())
                .build();

        shootPileLowCycle = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(9.000, 10.000),
                                new Pose(50.000, 10.000)
                        )
                )
                .setConstantHeadingInterpolation(FieldConstants.BLUE_FAR_AUTO_START_POSE.getHeading())
                .build();

        intakePileHighCycle = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(50.000, 10.000),
                                new Pose(40.000, 32.000),
                                new Pose(30.000, 32.000),
                                new Pose(9.000, 32.000)
                        )
                )
                .setConstantHeadingInterpolation(FieldConstants.BLUE_FAR_AUTO_START_POSE.getHeading())
                .build();

        shootPileHighCycle = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(9.000, 32.000),
                                new Pose(50.000, 10.000)
                        )
                )
                .setConstantHeadingInterpolation(FieldConstants.BLUE_FAR_AUTO_START_POSE.getHeading())
                .build();

        intakePile1 = Copier.copy(follower, intakePileLowCycle);
        shootPile1 = Copier.copy(follower, shootPileLowCycle);

        intakePile2 = Copier.copy(follower, intakePileHighCycle);
        shootPile2 = Copier.copy(follower, shootPileHighCycle);

        intakePile3 = Copier.copy(follower, intakePileLowCycle);
        shootPile3 = Copier.copy(follower, shootPileLowCycle);

        intakePile4 = Copier.copy(follower, intakePileHighCycle);
        shootPile4 = Copier.copy(follower, shootPileHighCycle);

        intakePile5 = Copier.copy(follower, intakePileLowCycle);
        shootPile5 = Copier.copy(follower, shootPileLowCycle);

        intakePile6 = Copier.copy(follower, intakePileHighCycle);
        shootPile6 = Copier.copy(follower, shootPileHighCycle);

        intakePile7 = Copier.copy(follower, intakePileLowCycle);
        shootPile7 = Copier.copy(follower, shootPileLowCycle);

        park = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(50, 10), new Pose(30, 10))
                )
                .setConstantHeadingInterpolation(FieldConstants.BLUE_FAR_AUTO_START_POSE.getHeading())
                .build();
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(FieldConstants.BLUE_FAR_AUTO_START_POSE);
        follower.usePredictiveBraking = true;
        robot = new CurrentRobot(hardwareMap);
        sotm = new SOTMUtil(FieldConstants.BLUE_GOAL_POSE);
        buildPaths();

        stateMachine = new StateMachine(
                // preload
                new State()
                        .maxTime(2000) // in case it takes too long
                        .transition(new Transition(() -> robot.shooter.atTarget(20) && !follower.isBusy())),
                new State()
                        .onEnter(() -> {
                            robot.shootCommandSlow.start();
                            follower.setMaxPower(1);
                        })
                        .transition(new Transition(() -> robot.shootCommandSlow.isFinished())),
                // corner
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.followPath(intakeCorner, false);
                        })
                        .transition(new Transition(() -> follower.atParametricEnd()))
                        .maxTime(1200),
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootCorner, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommandSlow.start())
                        .transition(new Transition(() -> robot.shootCommandSlow.isFinished())),
                // third
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.followPath(intakeThird, false);
                        })
                        .transition(new Transition(() -> follower.atParametricEnd()))
                        .maxTime(1200),
                new State()
                        .onEnter(() -> follower.followPath(shootThird, true))
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommandSlow.start())
                        .transition(new Transition(() -> robot.shootCommandSlow.isFinished())),
                // pile 1
                new State()
                        .onEnter(() -> {
                            follower.followPath(intakePile1, false);
                        })
                        .maxTime(1200)
                        .transition(new Transition(() -> follower.atParametricEnd())),
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootPile1, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommandSlow.start())
                        .transition(new Transition(() -> robot.shootCommandSlow.isFinished())),
                // pile 2
                new State()
                        .onEnter(() -> {
                            follower.followPath(intakePile2, false);
                        })
                        .maxTime(1200)
                        .transition(new Transition(() -> follower.atParametricEnd())),
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootPile2, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommandSlow.start())
                        .transition(new Transition(() -> robot.shootCommandSlow.isFinished())),
                // pile 3
                new State()
                        .onEnter(() -> {
                            follower.followPath(intakePile3, false);
                        })
                        .maxTime(1200)
                        .transition(new Transition(() -> follower.atParametricEnd())),
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootPile3, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommandSlow.start())
                        .transition(new Transition(() -> robot.shootCommandSlow.isFinished())),
                // pile 4
                new State()
                        .onEnter(() -> {
                            follower.followPath(intakePile4, false);
                        })
                        .maxTime(1200)
                        .transition(new Transition(() -> follower.atParametricEnd())),
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootPile4, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommandSlow.start())
                        .transition(new Transition(() -> robot.shootCommandSlow.isFinished())),
                // pile 5
                new State()
                        .onEnter(() -> {
                            follower.followPath(intakePile5, false);
                        })
                        .maxTime(1200)
                        .transition(new Transition(() -> follower.atParametricEnd())),
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootPile5, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommandSlow.start())
                        .transition(new Transition(() -> robot.shootCommandSlow.isFinished())),
                // pile 6
                new State()
                        .onEnter(() -> {
                            follower.followPath(intakePile6, false);
                        })
                        .maxTime(1200)
                        .transition(new Transition(() -> follower.atParametricEnd())),
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootPile6, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommandSlow.start())
                        .transition(new Transition(() -> robot.shootCommandSlow.isFinished())),
                // pile 7
                new State()
                        .onEnter(() -> {
                            follower.followPath(intakePile7, false);
                        })
                        .maxTime(1200)
                        .transition(new Transition(() -> follower.atParametricEnd())),
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootPile7, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .onEnter(() -> robot.shootCommandSlow.start())
                        .transition(new Transition(() -> robot.shootCommandSlow.isFinished())),
                new State()
                        .onEnter(() -> follower.followPath(park, true))
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
        robot.turret.wantedAngle = shooterOutputs.turretAngle;
        robot.turret.wantedAngularVelocity = shooterOutputs.turretFeedforward;

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
