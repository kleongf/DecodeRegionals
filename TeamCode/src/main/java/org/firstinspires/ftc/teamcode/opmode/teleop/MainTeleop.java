package org.firstinspires.ftc.teamcode.opmode.teleop;

import static com.qualcomm.robotcore.eventloop.opmode.OpMode.blackboard;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.constants.PoseConstants;
import org.firstinspires.ftc.teamcode.robot.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.robots.TeleopRobot;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.decodeutil.Alliance;
import org.firstinspires.ftc.teamcode.util.decodeutil.TeleopDrivetrain;
import org.firstinspires.ftc.teamcode.util.decodeutil.SOTM;
import com.pedropathing.paths.Path;
import org.firstinspires.ftc.teamcode.util.decodeutil.MathUtil;
import org.firstinspires.ftc.teamcode.util.decodeutil.Zone;
import org.firstinspires.ftc.teamcode.util.decodeutil.ZoneUtil;


public class MainTeleop {
    public enum RobotState {
        IDLE,
        SHOOTING
    }
    // TODO: also have it so when driver 2 has a trigger down then go slow
    // also side quest kalman filter stuff
    private RobotState robotState;
    private TeleopDrivetrain drivetrain;
    private double turretOffset = 0;
    private double speedScaler = 1;
    private double longitudinalSpeed = 1, lateralSpeed = 1, rotationSpeed = 0.2;
    public TeleopRobot robot;
    private Pose gatePose, parkPose, goalPose, gateIntakePose;
    private Gamepad gamepad1, gamepad2;
    public SOTM sotm;
    private boolean automateRobot = false;
    private Telemetry telemetry;
    private Alliance alliance;
    private PathBuilder pathBuilder;
    private Zone currentZone;
    private ZoneUtil zoneUtil;

    public MainTeleop(Pose startPose, Alliance alliance, HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2, boolean resetEncoders) {
        drivetrain = new TeleopDrivetrain(hardwareMap, alliance);
        drivetrain.setStartingPose(startPose);
        pathBuilder = drivetrain.follower.pathBuilder();

        robot = new TeleopRobot(hardwareMap);
        if (resetEncoders) {
            robot.turret.resetEncoder();
        }

        this.goalPose = alliance == Alliance.BLUE ? PoseConstants.BLUE_GOAL_POSE :  PoseConstants.RED_GOAL_POSE;
        this.parkPose = alliance == Alliance.BLUE ? PoseConstants.BLUE_PARK_POSE :  PoseConstants.RED_PARK_POSE;
        this.gatePose = alliance == Alliance.BLUE ? PoseConstants.BLUE_GATE_POSE : PoseConstants.RED_GATE_POSE;
        this.gateIntakePose = alliance == Alliance.BLUE ? PoseConstants.BLUE_GATE_AUTO_POSE : PoseConstants.RED_GATE_AUTO_POSE;

        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        this.telemetry = telemetry;
        this.alliance = alliance;

        this.sotm = new SOTM(goalPose);
        this.currentZone = Zone.CLOSE;
        this.zoneUtil = new ZoneUtil(8); // 8 inch radius seems right
    }
    private double normalizeInput(double input) {
        return 1.2 * Math.signum(input) * Math.sqrt(Math.abs(input));
    }

    private double getDistance(Pose a, Pose b) {
        return Math.hypot(a.getX()-b.getX(), a.getY()-b.getY());
    }

    public void loop() {
        if (alliance == Alliance.BLUE) {
            // TODO: no idea, might have to add the 180 offset for red
            drivetrain.update(speedScaler *-normalizeInput(gamepad1.left_stick_y*longitudinalSpeed),
                    speedScaler *-normalizeInput(gamepad1.left_stick_x*lateralSpeed),
                    speedScaler *-normalizeInput(gamepad1.right_stick_x*rotationSpeed));
        } else if (alliance == Alliance.RED) {
            drivetrain.update(speedScaler *-normalizeInput(gamepad1.left_stick_y*longitudinalSpeed),
                    speedScaler *-normalizeInput(gamepad1.left_stick_x*lateralSpeed),
                    speedScaler *-normalizeInput(gamepad1.right_stick_x*rotationSpeed));
        }

        Pose currentPose = drivetrain.getPose();
        Pose closestPose = zoneUtil.closestPose(drivetrain.follower.getPose(), currentZone);
        Vector currentVelocity = drivetrain.getVelocity();

        // SETTING ROBOT STATE
        if (robot.shootCommand.isFinished()) {
            robotState = RobotState.IDLE;
        } else {
            robotState = RobotState.SHOOTING;
        }

        // TODO: WILL BE USED LATER. CAN BE OPTIMIZED: STOP INTAKE WHEN 3 BALLS.

//        if (robotState == RobotState.IDLE && robot.intake.intakeFull()) {
//            robot.intake.state = Intake.IntakeState.INTAKE_SLOW;
//        } else if (robotState == RobotState.IDLE && !robot.intake.intakeFull()) {
//            robot.intake.state = Intake.IntakeState.INTAKE_FAST;
//        }

        if (automateRobot) {
            // TODO: implement with new functions and stuff, with an enum of 1, 2, 3 cases
            // robot not in shooting zone, intake is full, drivetrain not busy, and not shooting
            if (!zoneUtil.inZone(currentPose, currentZone) && robot.intake.intakeFull() && !drivetrain.isBusy() && robotState == RobotState.IDLE) {
                // case 1: the current pose is close to the closestPose, in this case no heading change is best. say it's 20 inches idk
                if (getDistance(currentPose, closestPose) < 20) {
                    drivetrain.kick(true, false, closestPose);
                } else {
                    // case 2: the current pose is NOT close to the closestPose, in which case we need to find the closest angle
                    double targetAngle = Math.atan2(closestPose.getY()- currentPose.getY(), closestPose.getX()- currentPose.getX());
                    double currentAngle = currentPose.getHeading();
                    // case 2a: tangential is closer, so we need to turn less
                    if (Math.abs(MathUtil.normalizeAngle(targetAngle-currentAngle)) < Math.abs(MathUtil.normalizeAngle(Math.PI-(targetAngle-currentAngle)))) {
                        drivetrain.kick(false, false, closestPose);
                    } else {
                        drivetrain.kick(false, true, closestPose);
                    }
                }
            }
        }

        // GAMEPAD 1 (DRIVER)

        // shoot: right bumper
        if (gamepad1.rightBumperWasPressed()) {
            robot.shootCommand.start();
        }

        // gate intake: x
        if (gamepad1.xWasPressed()) {
//            PathChain intakeGate = pathBuilder
//                    .addPath(
//                            new Path(
//                                    new BezierLine(
//                                            currentPose,
//                                            new Pose((alliance == Alliance.BLUE ? gateIntakePose.getX()+20: gateIntakePose.getX()-20), gateIntakePose.getY())
//                                    )
//                            )
//                    )
//                    .setLinearHeadingInterpolation(currentPose.getHeading(), gateIntakePose.getHeading())
//                    .addPath(
//                            new Path(
//                                    new BezierLine(
//                                            new Pose((alliance == Alliance.BLUE ? gateIntakePose.getX()+20: gateIntakePose.getX()-20), gateIntakePose.getY()),
//                                            gateIntakePose
//                                    )
//                            )
//                    )
//                    .setConstantHeadingInterpolation(gateIntakePose.getHeading())
//                    .build();
            drivetrain.intakeGate();
        }

        // gate open: y
        if (gamepad1.yWasPressed()) {
//            PathChain openGate = pathBuilder
//                    .addPath(
//                            new Path(
//                                    new BezierLine(
//                                            currentPose,
//                                            new Pose((alliance == Alliance.BLUE ? gatePose.getX()+15: gatePose.getX()-15), gatePose.getY())
//                                    )
//                            )
//                    )
//                    .setLinearHeadingInterpolation(currentPose.getHeading(), gatePose.getHeading())
//                    .addPath(
//                            new Path(
//                                    new BezierLine(
//                                            new Pose((alliance == Alliance.BLUE ? gatePose.getX()+15: gatePose.getX()-15), gatePose.getY()),
//                                            gatePose
//                                    )
//                            )
//                    )
//                    .setConstantHeadingInterpolation(gatePose.getHeading())
//                    .build();
            drivetrain.openGate();
        }

        // we don't want to park blindly. want to park either up or down, based on where we currently are.
        // usually we are the second bot to park, so either the top or bottom edge.

        // park: b
        if (gamepad1.bWasPressed()) {
            // if our y coord is low then go on bottom
            if (currentPose.getY() < 32) {
                PathChain park = pathBuilder
                        .addPath(
                                new Path(
                                        new BezierLine(
                                                currentPose,
                                                parkPose
                                        )
                                )
                        )
                        .setLinearHeadingInterpolation(currentPose.getHeading(), parkPose.getHeading())
                        .build();
                drivetrain.park();
            } else { // y coord is high so go on top
                PathChain park = pathBuilder
                        .addPath(
                                new Path(
                                        new BezierLine(
                                                currentPose,
                                                new Pose(parkPose.getX(), parkPose.getY() + 36)
                                        )
                                )
                        )
                        .setLinearHeadingInterpolation(currentPose.getHeading(), parkPose.getHeading()+Math.toRadians(180))
                        .build();
                drivetrain.park();
            }
        }

        // GAMEPAD 2 (OPERATOR)

        // slow mo for good park, 0.3x speed
        if (Math.abs(gamepad2.left_trigger) > 0.05 || Math.abs(gamepad2.right_trigger) > 0.05) {
            speedScaler = 0.3;
        } else {
            speedScaler = 1;
        }

        // field-centric safety
        if (gamepad2.bWasPressed()) {
            drivetrain.setRobotCentric(!drivetrain.getRobotCentric());
        }

        // relocalization
        if (gamepad2.xWasPressed()) {
            Pose llPose = robot.limelightLocalizer.getCurrentPose(currentPose);
            if (llPose.getX() != currentPose.getX() && llPose.getY() != currentPose.getY()) {
                gamepad1.rumble(300);
                drivetrain.follower.setPose(llPose);
            }
        }

        // corner relocalization
        if (gamepad2.yWasPressed()) {
            if (alliance == Alliance.BLUE) {
                drivetrain.follower.setPose(PoseConstants.BLUE_RELOCALIZATION_POSE);
            } else {
                drivetrain.follower.setPose(PoseConstants.RED_RELOCALIZATION_POSE);
            }
        }

        // turret offset
        if (gamepad2.rightBumperWasPressed()) {
            turretOffset -= Math.toRadians(4);
        }
        if (gamepad2.leftBumperWasPressed()) {
            turretOffset += Math.toRadians(4);
        }

        // close zone: up
        if (gamepad2.dpadUpWasPressed()) {
            currentZone = Zone.CLOSE;
        }
        if (gamepad2.dpadDownWasPressed()) {
            currentZone = Zone.FAR;
        }

        // safety for auto drive: both gamepads so that if one driver doesn't realize, the other will
        if (gamepad1.leftStickButtonWasPressed() || gamepad1.rightStickButtonWasPressed() || gamepad2.leftStickButtonWasPressed() || gamepad2.rightStickButtonWasPressed()) {
            drivetrain.breakFollowing();
        }

        double[] values = sotm.calculateAzimuthThetaVelocityFRCBetter(currentPose, currentVelocity, drivetrain.getAngularVelocity());
        robot.shooter.setShooterPitch(values[1]);
        robot.shooter.setTargetVelocity(values[2]);
        robot.turret.setFeedforward(values[3]);
        robot.turret.setTarget(values[0]+turretOffset);

        robot.update();

        telemetry.addData("Pose", currentPose );
        telemetry.addData("Current state", drivetrain.getState());
        telemetry.addData("Angle to goal", Math.atan2(-(goalPose.getX()-currentPose.getX()), (goalPose.getY()- currentPose.getY())));
        telemetry.addLine("Robot in shooting zone: " + zoneUtil.inZone(currentPose, currentZone));
        telemetry.addLine("Intake full: " + robot.intake.intakeFull());
        telemetry.addLine("Drivetrain Busy: " + drivetrain.isBusy());
        telemetry.addLine("Robot idle: " + (robotState == RobotState.IDLE));
        telemetry.addLine("Current turret pos: " + robot.turret.getCurrent());
        telemetry.addLine("Target turret pos: " + robot.turret.getTarget());
        telemetry.update();
    }


    public void start() {
        robot.initPositions();
        robot.start();
    }

    public void stop() {
        blackboard.put(RobotConstants.END_POSE_KEY, drivetrain.follower.getPose());
    }
}
