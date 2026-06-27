package org.firstinspires.ftc.teamcode.decode2026.opmode.comp;

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
import org.firstinspires.ftc.teamcode.decode2026.subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.decodeutil.Alliance;
import org.firstinspires.ftc.teamcode.util.decodeutil.Copier;
import org.firstinspires.ftc.teamcode.util.decodeutil.SOTMUtil;
import org.firstinspires.ftc.teamcode.util.fsm.State;
import org.firstinspires.ftc.teamcode.util.fsm.StateMachine;
import org.firstinspires.ftc.teamcode.util.fsm.Transition;

@Autonomous(name="Blue Far 33 sidespike Comp", group="!")
public class BlueFar30NewComp extends OpMode {
    private Pose lockedPose = new Pose();
    private boolean lockShooter = false;
    private double turretOffset = 0;
    private double speedOffset = 0;
    private double highPileCycleHeight = 40; //todo subtract smth like 5 inches if we hit teammate
    private Follower follower;
    private StateMachine stateMachine;
    private CurrentRobot robot;
    private SOTMUtil sotm;
    private PathChain intakeCorner, shootCorner, intakeThird, shootThird, intakePileLowCycle, shootPileLowCycle, intakePileHighCycle, shootPileHighCycle, intakePile1, shootPile1, intakePile2, shootPile2, intakePile3, shootPile3, intakePile4, shootPile4, intakePile5, shootPile5, intakePile6, shootPile6, intakePile7, shootPile7, intakePile8, shootPile8, intakePile9, shootPile9, park;

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
                .addPath(new BezierLine(new Pose(50.000, 12.000), new Pose(9, 9)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        shootCorner = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(9, 10), new Pose(50, 16)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();


        intakePileLowCycle = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(50.000, 16),
                                new Pose(30.000, 11),
                                new Pose(9.000, 9.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        shootPileLowCycle = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(9.000, 9.000),
                                new Pose(50.000, 12)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        intakePileHighCycle = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(50.000, 12),
                                new Pose(40.000, 34.000),
                                new Pose(30.000, 34.000),
                                new Pose(9.000, 34.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        shootPileHighCycle = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(9.000, 34.000),
                                new Pose(50.000, 12)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

//        intakePile2 = follower.pathBuilder()
//                .addPath(
//                        new BezierCurve(
//                                new Pose(50.000, 12.000),
//                                new Pose(24.000, 20.000),
//                                new Pose(14.000, 28.000),
//                                new Pose(14.000, highPileCycleHeight)
//                        )
//                )
//                .setTangentHeadingInterpolation()
//                .build();
//
//        shootPile2 = follower.pathBuilder()
//                .addPath(
//                        new BezierLine(
//                                new Pose(10.000, highPileCycleHeight),
//                                new Pose(50.000, 12.000)
//                        )
//                )
//                // .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
//                .setTangentHeadingInterpolation()
//                .setReversed()
//                .build();

        intakePile4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(50.000, 12.000),
                                new Pose(24.000, 20.000),
                                new Pose(14.000, 28.000),
                                new Pose(14.000, highPileCycleHeight)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        shootPile4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(10.000, highPileCycleHeight),
                                new Pose(50.000, 12.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        intakePile6 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(50.000, 12.000),
                                new Pose(24.000, 20.000),
                                new Pose(14.000, 28.000),
                                new Pose(14.000, highPileCycleHeight)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        shootPile6 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(10.000, highPileCycleHeight),
                                new Pose(50.000, 12.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        intakePile8 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(50.000, 12.000),
                                new Pose(24.000, 20.000),
                                new Pose(14.000, 28.000),
                                new Pose(14.000, highPileCycleHeight)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        shootPile8 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(10.000, highPileCycleHeight),
                                new Pose(50.000, 12.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        intakePile1 = Copier.copy(follower, intakePileLowCycle);
        shootPile1 = Copier.copy(follower, shootPileLowCycle);

        intakePile2 = Copier.copy(follower, intakePileHighCycle);
        shootPile2 = Copier.copy(follower, shootPileHighCycle);

        intakePile3 = Copier.copy(follower, intakePileLowCycle);
        shootPile3 = Copier.copy(follower, shootPileLowCycle);

//        intakePile4 = Copier.copy(follower, intakePileHighCycle);
//        shootPile4 = Copier.copy(follower, shootPileHighCycle);

        intakePile5 = Copier.copy(follower, intakePileLowCycle);
        shootPile5 = Copier.copy(follower, shootPileLowCycle);

//        intakePile6 = Copier.copy(follower, intakePileHighCycle);
//        shootPile6 = Copier.copy(follower, shootPileHighCycle);

        intakePile7 = Copier.copy(follower, intakePileLowCycle);
        shootPile7 = Copier.copy(follower, shootPileLowCycle);

        park = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(50, 16), new Pose(30, 12))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

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
                        // .onExit(() -> robot.prepareShootCommandLonger.start())
                        .maxTime(1200),
                new State()
                        .onEnter(() -> {
                            follower.followPath(shootThird, true);
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
                // third
                new State()
                        .onEnter(() -> {
                            robot.intakeCommand.start();
                            follower.followPath(intakeCorner, true);
                        })
                        .transition(new Transition(() -> follower.atParametricEnd()))
                        // .onExit(() -> robot.prepareShootCommandLonger.start())
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
                // pile 1
                new State()
                        .onEnter(() -> {
                            follower.followPath(intakePile1, false);
                        })
                        .maxTime(1200)
                        // .onExit(() -> robot.prepareShootCommandLonger.start())
                        .transition(new Transition(() -> follower.getCurrentTValue() > 0.7)),
                new State()
                        .onEnter(() -> {
                            if(robot.intake.isFull){
                                robot.intake.wantedMode = Intake.Mode.INTAKE_OFF;
                            }
                            follower.followPath(shootPile1, true);
                        })

                        .transition(new Transition(() -> !follower.isBusy())),

                new State()
                        .maxTime(100),
                new State()
                        .onEnter(() -> {
                            robot.shootCommandSlow.start();
                        })
                        .maxTime(500),
                // pile 2
                new State()
                        .onEnter(() -> {
                            follower.followPath(intakePile2, false);
                        })
                        .maxTime(1200)
                        // .onExit(() -> robot.prepareShootCommandLonger.start())
                        .transition(new Transition(() -> follower.getCurrentTValue() > 0.7)),
                new State()
                        .onEnter(() -> {
                            if(robot.intake.isFull){
                                robot.intake.wantedMode = Intake.Mode.INTAKE_OFF;
                            }
                            follower.followPath(shootPile2, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),

                new State()
                        .maxTime(100),
                new State()
                        .onEnter(() -> {
                            robot.shootCommandSlow.start();
                        })
                        .maxTime(500),
                // pile 3
                new State()
                        .onEnter(() -> {
                            follower.followPath(intakePile3, false);
                        })
                        .maxTime(1200)
                        // .onExit(() -> robot.prepareShootCommandLonger.start())
                        .transition(new Transition(() -> follower.getCurrentTValue() > 0.7)),
                new State()
                        .onEnter(() -> {
                            if(robot.intake.isFull){
                                robot.intake.wantedMode = Intake.Mode.INTAKE_OFF;
                            }
                            follower.followPath(shootPile3, true);
                        })

                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .maxTime(100),
                new State()
                        .onEnter(() -> {
                            robot.shootCommandSlow.start();})
                        .maxTime(500),
                // pile 4
                new State()
                        .onEnter(() -> {
                            follower.followPath(intakePile4, false);
                        })
                        .maxTime(1200)
                        // .onExit(() -> robot.prepareShootCommandLonger.start())
                        .transition(new Transition(() -> follower.getCurrentTValue() > 0.7)),
                new State()
                        .onEnter(() -> {
                            if(robot.intake.isFull){
                                robot.intake.wantedMode = Intake.Mode.INTAKE_OFF;
                            }
                            follower.followPath(shootPile4, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .maxTime(100),
                new State()
                        .onEnter(() -> {
                            robot.shootCommandSlow.start();})
                        .maxTime(500),
                // pile 5
                new State()
                        .onEnter(() -> {
                            follower.followPath(intakePile5, false);
                        })
                        .maxTime(1200)
                        // .onExit(() -> robot.prepareShootCommandLonger.start())
                        .transition(new Transition(() -> follower.getCurrentTValue() > 0.7)),
                new State()
                        .onEnter(() -> {
                            if(robot.intake.isFull){
                                robot.intake.wantedMode = Intake.Mode.INTAKE_OFF;
                            }
                            follower.followPath(shootPile5, true);
                        })

                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .maxTime(100),
                new State()
                        .onEnter(() -> {
                            robot.shootCommandSlow.start();})
                        .maxTime(500),
                // pile 6
                new State()
                        .onEnter(() -> {
                            follower.followPath(intakePile6, false);
                        })
                        .maxTime(1200)
                        // .onExit(() -> robot.prepareShootCommandLonger.start())
                        .transition(new Transition(() -> follower.getCurrentTValue() > 0.7)),
                new State()
                        .onEnter(() -> {
                            if(robot.intake.isFull){
                                robot.intake.wantedMode = Intake.Mode.INTAKE_OFF;
                            }
                            follower.followPath(shootPile6, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .maxTime(100),
                new State()
                        .onEnter(() -> {
                            robot.shootCommandSlow.start();})
                        .maxTime(500),
                // pile 7
                new State()
                        .onEnter(() -> {
                            follower.followPath(intakePile7, false);
                        })
                        .maxTime(1200)
                        // .onExit(() -> robot.prepareShootCommandLonger.start())
                        .transition(new Transition(() -> follower.getCurrentTValue() > 0.7)),
                new State()
                        .onEnter(() -> {
                            if(robot.intake.isFull){
                                robot.intake.wantedMode = Intake.Mode.INTAKE_OFF;
                            }
                            follower.followPath(shootPile7, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),

                new State()
                        .maxTime(100),
                new State()
                        .onEnter(() -> {
                            robot.shootCommandSlow.start();
                        })
                        .maxTime(500),
//                new State()
//                        .onEnter(() -> follower.followPath(park, true))
//                        .transition(new Transition(() -> !follower.isBusy())),
                // pile 8
                new State()
                        .onEnter(() -> {
                            follower.followPath(intakePile8, false);
                        })
                        .maxTime(1200)
                        // .onExit(() -> robot.prepareShootCommandLonger.start())
                        .transition(new Transition(() -> follower.getCurrentTValue() > 0.7)),
                new State()
                        .onEnter(() -> {
                            if(robot.intake.isFull){
                                robot.intake.wantedMode = Intake.Mode.INTAKE_OFF;
                            }
                            follower.followPath(shootPile8, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .maxTime(100),
                new State()
                        .onEnter(() -> {
                            robot.shootCommandSlow.start();})
                        .maxTime(500)

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

        // multipossession logic
        if (robot.intake.detectionState == Intake.DetectionState.THIRD_TRIGGERED && robot.shootCommandSlow.isFinished()) {
            robot.intake.wantedMode = Intake.Mode.INTAKE_OFF;
        }

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

