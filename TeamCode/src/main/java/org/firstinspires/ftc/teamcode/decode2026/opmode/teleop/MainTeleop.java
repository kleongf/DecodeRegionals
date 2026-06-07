package org.firstinspires.ftc.teamcode.decode2026.opmode.teleop;

import static com.qualcomm.robotcore.eventloop.opmode.OpMode.blackboard;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.decode2026.CurrentRobot;
import org.firstinspires.ftc.teamcode.decode2026.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.decode2026.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.decode2026.constants.ShootingConstants;
import org.firstinspires.ftc.teamcode.decode2026.constants.TurretConstants;
import org.firstinspires.ftc.teamcode.decode2026.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.decodeutil.Alliance;
import org.firstinspires.ftc.teamcode.util.decodeutil.SOTMUtil;
import org.firstinspires.ftc.teamcode.util.decodeutil.TeleopDrivetrain;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.util.decodeutil.MathUtil;
import org.firstinspires.ftc.teamcode.util.decodeutil.ZoneUtil;

public class MainTeleop {
    public enum RobotState {
        IDLE,
        SHOOTING
    }

    private Intake.DetectionState prevDetectState;
    public TeleopDrivetrain drivetrain;
    private double turretOffset = 0;
    public CurrentRobot robot;
    private final Pose goalPose;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;
    private final SOTMUtil sotmUtil;
    private final Telemetry telemetry;
    private final Alliance alliance;
    private ZoneUtil.Zone currentZone;
    private final ElapsedTime relocalizationTimer;
    private final ElapsedTime turretResetTimer;
    private final ElapsedTime intakeTimer;

    public MainTeleop(Pose startPose, Alliance alliance, HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        drivetrain = new TeleopDrivetrain(hardwareMap, alliance);
        drivetrain.setStartingPose(startPose);

        robot = new CurrentRobot(hardwareMap);
        robot.cameraLocalizer.setAlliance(alliance);
        robot.reset();

        this.goalPose = alliance == Alliance.BLUE ? FieldConstants.BLUE_GOAL_POSE :  FieldConstants.RED_GOAL_POSE;

        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        this.telemetry = telemetry;
        this.alliance = alliance;

        this.sotmUtil = new SOTMUtil(this.goalPose);
        this.currentZone = ZoneUtil.Zone.CLOSE;

        this.prevDetectState = Intake.DetectionState.EMPTY;
        this.relocalizationTimer = new ElapsedTime();
        this.turretResetTimer = new ElapsedTime();
        this.intakeTimer = new ElapsedTime();
    }
    private double normalizeInput(double input) {
        return 1.1 * input;
    }
    private void shoot(Pose currentPose, Pose goalPose) {
        if (MathUtil.distance(currentPose, goalPose) > RobotConstants.farShootingDistanceThreshold)  {
            robot.shootCommandSlow.start();
        } else {
            robot.shootCommand.start();
        }
    }

    public void loop() {
        Pose currentPose = drivetrain.getPose();
        Pose closestPose = currentZone == ZoneUtil.Zone.CLOSE ?
                alliance == Alliance.BLUE ?
                        FieldConstants.BLUE_CLOSE_ZONE_POSE : FieldConstants.RED_CLOSE_ZONE_POSE :
                alliance == Alliance.BLUE ?
                        FieldConstants.BLUE_FAR_ZONE_POSE : FieldConstants.RED_FAR_ZONE_POSE;

        RobotState robotState = robot.shootCommand.isFinished() && robot.shootCommandSlow.isFinished() ? RobotState.IDLE : RobotState.SHOOTING;
        boolean inZone =
                (ZoneUtil.inCloseZone(currentPose) && currentZone == ZoneUtil.Zone.CLOSE) ||
                (ZoneUtil.inFarZone(currentPose) && currentZone == ZoneUtil.Zone.FAR);


        if (
                robot.intake.isFull &&
                        (prevDetectState == Intake.DetectionState.SECOND_TRIGGERED && robot.intake.detectionState == Intake.DetectionState.THIRD_TRIGGERED) &&
                        // prevDetectState != robot.intake.detectionState
                robotState != RobotState.SHOOTING
        ) {
            robot.ledIndicator.indicateIntakeFull();
            intakeTimer.reset();
        }

        if (prevDetectState == Intake.DetectionState.SECOND_TRIGGERED && robot.intake.detectionState == Intake.DetectionState.THIRD_TRIGGERED && robotState == RobotState.IDLE) {
            // start preparation sequence
            robot.prepareShootCommandLonger.start();
        }

        // if 3 and shoot, no close latch
        // if 0 and shoot, closes latch

//        if (robot.intake.isFull && robotState == RobotState.IDLE) {
//            robot.intake.wantedMode = Intake.Mode.INTAKE_SLOW;
//        } else {
//            robot.intake.wantedMode = Intake.Mode.INTAKE_FAST;
//        }

        // when changes from 2nd to 3rd detection: trigger the preparation sequence
        // does nothing for 0.2s then opens the latch

//        robot.intake.wantedMode = Intake.Mode.INTAKE_FAST;
//
//        if (intakeTimer.seconds() > 0.2 && robot.intake.isFull && robotState != RobotState.SHOOTING) {
//            robot.intake.wantedMode = Intake.Mode.INTAKE_OFF;
//        }
//        if(intakeTimer.seconds() > 0.4 && robot.intake.isFull && robotState != RobotState.SHOOTING) {
//            robot.shooter.openLatch();
//        }

        // we want to not necessarily turn to the closest pose as that could end badly but rather a certain constant pose.
        // automatically kick the robot in the correct direction
        // problem:
        if (RobotConstants.useAutomateRobotDrive) {
            // if prev few states were the same, then we didn't shoot anything, therefore no need to auto drive again
            // not in zone, intake full, not currently auto driving, and intake JUST became full (so we don't
            if (!inZone &&
                    robot.intake.isFull &&
                    !drivetrain.isBusy() &&
                    (prevDetectState == Intake.DetectionState.SECOND_TRIGGERED && robot.intake.detectionState == Intake.DetectionState.THIRD_TRIGGERED) &&
                    // prevDetectState != robot.intake.detectionState &&
                    robotState != RobotState.SHOOTING) {
                drivetrain.kick(closestPose);
            }
        }

        // auto shoot if in zone, intake full, and stuff is at the right positions
        if (
                // if we are not using far zone auto shoot OR we are in far zone and using far zone shooting
                // i know there is a redundant statement but its more clear to me
                (!RobotConstants.useFarZoneAutoShoot || (RobotConstants.useFarZoneAutoShoot && ZoneUtil.inFarZone(currentPose))) &&
                inZone &&
                        (robot.intake.isFull) &&
                        robotState != RobotState.SHOOTING &&
                Math.abs(robot.shooter.wantedVelocity - robot.shooter.currentVelocity) < RobotConstants.autoShootWheelSpeedEpsilonTicks &&
                Math.abs(robot.turret.errorTicks) < RobotConstants.autoShootTurretTicksEpsilon &&
                        robot.turret.currentPositionTicks > -TurretConstants.ticksPerRevolution + RobotConstants.autoShootTurretRangeEpsilon &&
                        robot.turret.currentPositionTicks < -RobotConstants.autoShootTurretRangeEpsilon &&
                        currentPose.distanceFrom(goalPose) > RobotConstants.MIN_SHOOTING_DISTANCE
        ) {
            shoot(currentPose, this.goalPose);
        }//todo: commented out for now

//        if (relocalizationTimer.seconds() > relocalizationTime) {
//            // TODO: test automatic relocalization
//            // we could also try a larger dist constraint and avging the poses
//
//            // wait hold up:
//            // we want to know the angle of the camera relative to the goal
//            // the camera is same as the fucking robot's heading like am i dumb or something
//
//            // if we are on blue: camera angle is 135 deg, red is 45 deg
//            // check if robot heading roughly equal to angle to goal
//
//            double dx = goalPose.getX() - currentPose.getX();
//            double dy = goalPose.getY() - currentPose.getY();
//            double angleToGoal = Math.atan2(-dx, dy) + Math.toRadians(90);
//
//            double headingDiffFromGoal = MathFunctions.getSmallestAngleDifference(currentPose.getHeading(), angleToGoal);
//
//            if (drivetrain.getVelocity().getMagnitude() < 3 && Math.abs(drivetrain.getAngularVelocity()) < 0.1 && getDistance(currentPose, goalPose) < 100 && Math.abs(headingDiffFromGoal) < Math.toRadians(30)) {
//                Pose relocalizedPose = robot.webcamLocalizer.getCurrentPose();
//                if (MathUtil.distance(relocalizedPose, currentPose) < 10) { // threshold: 10 inches
//                    drivetrain.follower.setPose(new Pose(relocalizedPose.getX(), relocalizedPose.getY(), currentPose.getHeading()));
//                    double averageX = (relocalizedPose.getX() + currentPose.getX()) / 2;
//                    double averageY = (relocalizedPose.getY() + currentPose.getY()) / 2;
//                    double theta1 = currentPose.getHeading();
//                    double theta2 = relocalizedPose.getHeading();
//                    double averageAngle = Math.atan2(Math.cos(theta1) + Math.cos(theta2), Math.sin(theta1) + Math.sin(theta2));
//                    Pose averagePose = new Pose(averageX, averageY, averageAngle);
//                    // something is cooked here.
//                    // drivetrain.follower.setPose(averagePose);
//                    robot.webcamLocalizer.flashLED();
//                    relocalizationTimer.reset();
//                }
//            }
//        }

        // hopefully this will not be necessary todo: fix encoder from slipping
        if (RobotConstants.useAutomaticTurretRelocalization) {
            if (turretResetTimer.seconds() > RobotConstants.turretResetTimeSeconds) {
                if (Math.abs(robot.turret.currentVelocityTicks) < 30 && robot.turret.currentPositionTicks > -1200 && robot.turret.currentPositionTicks < -200) {
                    robot.turret.resetEncoderWithAbsoluteReading();
                    turretResetTimer.reset();
                }
            }
        }

        /** GAMEPAD 1 (DRIVER) **/

        // shoot: right bumper
        if (gamepad1.rightBumperWasPressed()) {
            shoot(currentPose, this.goalPose);
        }

        // RIGHT TRIGGER: hold for gate heading lock
        drivetrain.gateHeadingLock = Math.abs(gamepad1.right_trigger) > 0.05;
        // LEFT TRIGGER: hold for gate open heading lock
        drivetrain.openGateHeadingLock = Math.abs(gamepad1.left_trigger) > 0.05;

        // park: x
        if (gamepad1.xWasPressed()) {
            drivetrain.park();
        }

        // toggle tilt: y
        if (gamepad1.yWasPressed()) {
            if (robot.tilt.tilted) {
                robot.tilt.unTilt();
            } else {
                robot.tilt.tilt();
            }
        }

        // stop auto drive: left/right stick
        if (gamepad1.leftStickButtonWasPressed() || gamepad1.rightStickButtonWasPressed()) {
            drivetrain.breakFollowing();
        }

        drivetrain.update(-normalizeInput(gamepad1.left_stick_y),
                -normalizeInput(gamepad1.left_stick_x),
                -normalizeInput(gamepad1.right_stick_x));

        /** GAMEPAD 2 (OPERATOR) **/

        // x: reset turret
        if (gamepad2.xWasPressed()) {
            robot.turret.resetEncoderWithAbsoluteReading();
        }

        // y: set robot centric todo: test robot centric
        if (gamepad2.yWasPressed()) {
            drivetrain.setRobotCentric(!drivetrain.getRobotCentric());
        }

        // left stick: webcam relocalization
        // maybe it is a good idea to have reloc and reset turret on same button idk
        if (gamepad2.leftStickButtonWasPressed()) {
            Pose webcamPose = robot.cameraLocalizer.currentPose;
            if (webcamPose.getX() != 0 && webcamPose.getY() != 0) {
                robot.ledIndicator.indicateRelocalization();
                drivetrain.follower.setPose(webcamPose);
            }
        }

        // right stick: corner relocalization:
        if (gamepad2.rightStickButtonWasPressed()) {
            if (alliance == Alliance.BLUE) {
                drivetrain.follower.setPose(FieldConstants.BLUE_RELOCALIZATION_POSE);
            } else {
                drivetrain.follower.setPose(FieldConstants.RED_RELOCALIZATION_POSE);
            }
        }

        // turret offset
        if (gamepad2.rightBumperWasPressed()) {
            turretOffset -= Math.toRadians(2);
        }
        if (gamepad2.leftBumperWasPressed()) {
            turretOffset += Math.toRadians(2);
        }

        // close zone: dpad up
        if (gamepad2.dpadUpWasPressed()) {
            currentZone = ZoneUtil.Zone.CLOSE;
        }
        // far zone: dpad down
        if (gamepad2.dpadDownWasPressed()) {
            currentZone = ZoneUtil.Zone.FAR;
        }

        ShootingConstants.ShooterOutputs shooterOutputs =
                RobotConstants.useShootOnTheMove ?
                        sotmUtil.calculateShooterOutputs(
                                drivetrain.getPose(),
                                drivetrain.getVelocity(),
                                drivetrain.getAcceleration(),
                                drivetrain.getAngularVelocity(),
                                RobotConstants.dt) :
                        sotmUtil.calculateShooterOutputs(drivetrain.getPose(),
                                new Vector(),
                                new Vector(),
                                0,
                                RobotConstants.dt);

        robot.shooter.wantedVelocity = shooterOutputs.wheelVelocity;
        robot.shooter.wantedAcceleration = shooterOutputs.wheelFeedforward;
        robot.shooter.wantedPitch = shooterOutputs.hoodAngle;
        robot.turret.wantedAngle = shooterOutputs.turretAngle + turretOffset;
        robot.turret.wantedAngularVelocity = shooterOutputs.turretFeedforward;

        prevDetectState = robot.intake.detectionState;
        robot.update();

        blackboard.put(FieldConstants.END_POSE_KEY, drivetrain.follower.getPose());

        telemetry.addData("Loop time", robot.dt);
        telemetry.addData("Pose", currentPose);
        telemetry.addData("Current state", drivetrain.getState());
        telemetry.addData("Angle to goal", Math.atan2(-(goalPose.getX()-currentPose.getX()), (goalPose.getY()- currentPose.getY())));
        telemetry.addLine("Robot in shooting zone: " + inZone);
        telemetry.addLine("Intake full: " + robot.intake.isFull);
        telemetry.addLine("Intake state: "+ robot.intake.detectionState);
        telemetry.addLine("Drivetrain Busy: " + drivetrain.isBusy());
        telemetry.addLine("Robot idle: " + (robotState == RobotState.IDLE));
        telemetry.addLine("Is full or 2nd triggered: " + (robot.intake.isFull || robot.intake.detectionState == Intake.DetectionState.SECOND_TRIGGERED));
        telemetry.addLine("Shooter wheel error ticks: " + Math.abs(robot.shooter.wantedVelocity - robot.shooter.currentVelocity));
        telemetry.addLine("Turret ticks error: " + robot.turret.errorTicks);

        telemetry.update();
    }


    public void start() {
        robot.start();
        robot.intake.wantedMode = Intake.Mode.INTAKE_FAST;
    }

    public void stop() {
        blackboard.put(FieldConstants.END_POSE_KEY, drivetrain.follower.getPose());
    }
}
