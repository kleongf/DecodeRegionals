package org.firstinspires.ftc.teamcode.decode2026.opmode.autonomous;

import static java.lang.Thread.sleep;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Curve;
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

@Autonomous(name="Blue Far 30 Vision", group="!")
public class BlueFar30Vision extends OpMode {
    private Pose lockedPose = new Pose();
    private boolean lockShooter = false;
    private double turretOffset = 0;
    private double speedOffset = 0;
    private double highPileCycleHeight = 44; //todo subtract smth like 5 inches if we hit teammate
    private Follower follower;
    private StateMachine stateMachine;
    private CurrentRobot robot;
    private SOTMUtil sotm;
    private PathChain intakeCorner, shootCorner, intakeThird, shootThird, intakeGate, currentIntakePath, currentShootPath;

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
                                new Pose(24.000, 20.000),
                                new Pose(14.000, 28.000),
                                new Pose(14.000, highPileCycleHeight)
                        )
                )
                .setTangentHeadingInterpolation()
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
                // cycles
                new State("startCycle")
                        .onEnter(() -> {
                            Curve intakePath = robot.artifactVision.constructSpline(
                                    follower.getPose(),
                                    8,
                                    30,
                                    8,
                                    50
                            );
                            if (intakePath == null) {
                                currentIntakePath = follower.pathBuilder()
                                        .addPath(
                                                new BezierCurve(
                                                        new Pose(50.000, 16.000),
                                                        new Pose(24.000, 20.000),
                                                        new Pose(14.000, 28.000),
                                                        new Pose(14.000, highPileCycleHeight)
                                                )
                                        )
                                        .setTangentHeadingInterpolation()
                                        .build();
                            } else {
                                currentIntakePath = follower.pathBuilder()
                                        .addPath(intakePath)
                                        .setTangentHeadingInterpolation()
                                        .build();
                            }
                            follower.followPath(currentIntakePath, true);
                        })
                        .maxTime(1500)
                        .transition(new Transition(() -> follower.getCurrentTValue() > 0.9))
                        .transition(new Transition(() -> robot.intake.isFull && follower.getCurrentTValue() > 0.5)),
                new State()
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
                                    .setConstantHeadingInterpolation(Math.toRadians(160))
                                    .build();
                            follower.followPath(currentShootPath, true);
                        })
                        .transition(new Transition(() -> !follower.isBusy())),
                new State()
                        .maxTime(100),
                new State()
                        .onEnter(() -> {
                            robot.shootCommandSlow.start();
                        })
                        .transition(new Transition(() -> robot.shootCommandSlow.isFinished(), "startCycle"))
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
//        if (robot.intake.detectionState == Intake.DetectionState.THIRD_TRIGGERED && robot.shootCommandSlow.isFinished()) {
//            robot.intake.wantedMode = Intake.Mode.INTAKE_OFF;
//        }

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


