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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.decodeutil.MathUtil;
import org.firstinspires.ftc.teamcode.util.decodeutil.Zone;
import org.firstinspires.ftc.teamcode.util.decodeutil.ZoneUtil;

import java.util.ArrayList;


public class MainTeleop {
    public enum RobotState {
        IDLE,
        SHOOTING
    }
    private RobotState robotState;
    private ArrayList<Vector> rollingVelocities;
    private double numVelocities = 4;
    private Intake.DetectionState prevDetectState;
    public TeleopDrivetrain drivetrain;
    private double turretOffset = 0;
    private double speedScaler = 1;
    private double longitudinalSpeed = 1, lateralSpeed = 1, rotationSpeed = 0.2;
    public TeleopRobot robot;
    private Pose gatePose, parkPose, goalPose, gateIntakePose;
    private Gamepad gamepad1, gamepad2;
    public SOTM sotm;
    private boolean automateRobot = true;
    private Telemetry telemetry;
    private Alliance alliance;
    private PathBuilder pathBuilder;
    private Zone currentZone;
    private Pose prevPose;
    private ElapsedTime elapsedTime;
    private ElapsedTime relocalizationTimer;
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
        this.currentZone = Zone.FAR;
        this.zoneUtil = new ZoneUtil(8); // 8 inch radius seems right

        this.prevDetectState = Intake.DetectionState.EMPTY;
        this.rollingVelocities = new ArrayList<>();
        this.elapsedTime = new ElapsedTime();
        this.relocalizationTimer = new ElapsedTime();
        this.prevPose = startPose;
    }
    private double normalizeInput(double input) {
        return 1.2 * Math.signum(input) * Math.sqrt(Math.abs(input));
    }

    private double getDistance(Pose a, Pose b) {
        return Math.hypot(a.getX()-b.getX(), a.getY()-b.getY());
    }

    private void relocalize(Pose pinpointPose, Pose cameraPose, double velocity, double angularVelocity, double distanceEpsilon) {
        // normal relocalization: checks velocity, angular velocity, and distance AND the timer to see if its over 10s
        relocalizationTimer.reset();
    }

    private void overrideRelocalize(Pose pinpointPose, Pose cameraPose) {
        // completely overrides the pinpoint with the heading and position.
    }

    public Vector getRollingVelocity() {
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

        double dt = elapsedTime.seconds();

        Pose currentPose = drivetrain.getPose();
        Pose closestPose = zoneUtil.closestPose(drivetrain.follower.getPose(), currentZone);

        Vector currentVelocity;
        if (dt > 0) {
            currentVelocity = drivetrain.getPose().minus(prevPose).div(dt).getAsVector();
        } else {
            currentVelocity = drivetrain.getVelocity();
        }

        if (rollingVelocities.size() < numVelocities) {
            for (int i = 0; i < numVelocities; i++) {
                rollingVelocities.add(currentVelocity);
            }
        } else {
            rollingVelocities.add(currentVelocity);
            rollingVelocities.remove(0);
        }

        // SETTING ROBOT STATE
        if (robot.shootCommand.isFinished()) {
            robotState = RobotState.IDLE;
        } else {
            robotState = RobotState.SHOOTING;
        }

        // TODO: WILL BE USED LATER. CAN BE OPTIMIZED: STOP INTAKE WHEN 3 BALLS.
//        if (robotState == RobotState.IDLE && robot.intake.isFull()) {
//            robot.intake.state = Intake.IntakeState.INTAKE_SLOW;
//        } else if (robotState == RobotState.IDLE && !robot.intake.isFull()) {
//            robot.intake.state = Intake.IntakeState.INTAKE_FAST;
//        }

        if (automateRobot) {
            // we could also keep track of the previous state. if the prev state was also full then don't kick back again, since shoot did not happen
            // if prev few states were the same, then we didn't shoot anything, therefore no need to autodrive again


            if (!zoneUtil.inZone(currentPose, currentZone) && robot.intake.isFull() && !drivetrain.isBusy() && robotState == RobotState.IDLE && prevDetectState != robot.intake.detectionState) {
                // case 1: the current pose is close to the closestPose, in this case no heading change is best. say it's 20 inches idk
                if (getDistance(currentPose, closestPose) < 50) {
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

        // added just for this

        if (gamepad1.dpadUpWasPressed()) {
            automateRobot = !automateRobot;
        }

        // gate intake: x
        if (gamepad1.xWasPressed()) {
            drivetrain.intakeGate();
        }

        // gate open: y
        if (gamepad1.yWasPressed()) {
            drivetrain.openGate();
        }

        // we don't want to park blindly. want to park either up or down, based on where we currently are.
        // usually we are the second bot to park, so either the top or bottom edge.

        // park: b TODO: map this to a better button, not really sure which one though
        if (gamepad1.bWasPressed()) {
            drivetrain.park();
        }

        if (gamepad1.leftStickButtonWasPressed() || gamepad1.rightStickButtonWasPressed()) {
            drivetrain.breakFollowing();
        }

        // GAMEPAD 2 (OPERATOR)

        // slow mo for good park, 0.3x speed
        if (Math.abs(gamepad2.left_trigger) > 0.05 || Math.abs(gamepad2.right_trigger) > 0.05) {
            speedScaler = 0.4;
        } else {
            speedScaler = 1;
        }

        // x: field-centric (toggle)
        if (gamepad2.bWasPressed()) {
            drivetrain.setRobotCentric(!drivetrain.getRobotCentric());
        }

        // y: reset turret
        if (gamepad2.yWasPressed()) {
            robot.resetTurretCommand.start();
        }

        // relocalization: left stick
        if (gamepad2.leftStickButtonWasPressed()) { // map to o
            Pose llPose = robot.limelightLocalizer.getCurrentPose(currentPose);
            if (llPose.getX() != currentPose.getX() && llPose.getY() != currentPose.getY()) {
                gamepad1.rumble(300);
                drivetrain.follower.setPose(llPose);
            }
        }

        // corner relocalization: right stick
        if (gamepad2.rightStickButtonWasPressed()) {
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

        // close zone: dpad up
        if (gamepad2.dpadUpWasPressed()) {
            currentZone = Zone.CLOSE;
        }
        // far zone: dpad down
        if (gamepad2.dpadDownWasPressed()) {
            currentZone = Zone.FAR;
        }


        double[] values = sotm.calculateAzimuthThetaVelocityFRCBetter(currentPose, drivetrain.getVelocity(), drivetrain.getAngularVelocity());

        // new stuff: if we have a large distance then hold it TODO: uncomment later
//        boolean isFar = MathUtil.distance(currentPose, goalPose) > 120; // at 120 or more inches, we switch to the far coefficients so we don't move.
//        if (isFar) {
//            robot.turret.setPDCoefficients(0.01, 0.0005);
//            robot.turret.setFeedforward(0);
//        } else {
//            robot.turret.setPDCoefficients(0.005, 0);
//            robot.turret.setFeedforward(values[3]);
//        }
        if (!robot.resetTurretCommand.isFinished()) { // we are in the middle of trying to reset encoder, so don't set the target to something else
            robot.turret.setFeedforward(0);
        } else {
            robot.turret.setTarget(values[0]+turretOffset);
            robot.turret.setFeedforward(values[3]);
        }

        robot.shooter.setShooterPitch(values[1]);
        robot.shooter.setTargetVelocity(values[2]);

        prevDetectState = robot.intake.detectionState;
        prevPose = currentPose;
        elapsedTime.reset();

        robot.update();

        telemetry.addData("Pose", currentPose );
        telemetry.addData("Current state", drivetrain.getState());
        telemetry.addData("Angle to goal", Math.atan2(-(goalPose.getX()-currentPose.getX()), (goalPose.getY()- currentPose.getY())));
        telemetry.addLine("Robot in shooting zone: " + zoneUtil.inZone(currentPose, currentZone));
        telemetry.addLine("Intake full: " + robot.intake.intakeFull());
        telemetry.addLine("Intake state: "+ robot.intake.getState());
        telemetry.addLine("Drivetrain Busy: " + drivetrain.isBusy());
        telemetry.addLine("Robot idle: " + (robotState == RobotState.IDLE));
        telemetry.addLine("Current turret pos: " + robot.turret.getCurrent());
        telemetry.addLine("Target turret pos: " + robot.turret.getTarget());
        telemetry.update();
    }


    public void start() {
        robot.initPositions();
        robot.start();
        elapsedTime.reset();
    }

    public void stop() {
        blackboard.put(RobotConstants.END_POSE_KEY, drivetrain.follower.getPose());
    }
}
