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
    private boolean isParking = false;
    private double speedScaleFactor = 1.0;

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
        ShootingConstants.tofMultiplier = ShootingConstants.teleTofMultiplier;
    }
    private double normalizeInput(double input) {
        return 1.1 * input * speedScaleFactor;
    }
    private void shoot(Pose currentPose, Pose goalPose) {
        if (MathUtil.distance(currentPose, goalPose) > RobotConstants.farShootingDistanceThreshold)  {
            robot.shootCommandSlow.start();
        } else {
            robot.shootCommandFast.start();
        }
    }

    public void init_loop() {
        // right bumper: close zone
        if (gamepad2.rightBumperWasPressed()) {
            currentZone = ZoneUtil.Zone.CLOSE;
        }
        // left bumper: far zone
        if (gamepad2.leftBumperWasPressed()) {
            currentZone = ZoneUtil.Zone.FAR;
        }
    }

    public void loop() {
        RobotConstants.useAutomateRobotDrive = (currentZone == ZoneUtil.Zone.CLOSE);

        Pose currentPose = drivetrain.getPose();
        Pose closestPose = currentZone == ZoneUtil.Zone.CLOSE ?
                alliance == Alliance.BLUE ?
                        FieldConstants.BLUE_CLOSE_ZONE_POSE : FieldConstants.RED_CLOSE_ZONE_POSE :
                alliance == Alliance.BLUE ?
                        FieldConstants.BLUE_FAR_ZONE_POSE : FieldConstants.RED_FAR_ZONE_POSE;

        RobotState robotState = robot.shootCommandFast.isFinished() && robot.shootCommandSlow.isFinished() ? RobotState.IDLE : RobotState.SHOOTING;
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
            //robot.prepareShootCommandLonger.start();
            robot.prepareShootCommand.start();
        }

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
        //todo: drivers don't want this anymore
//        if (
//                // if we are not using far zone auto shoot OR we are in far zone and using far zone shooting
//                // i know there is a redundant statement but its more clear to me
//                (!RobotConstants.useFarZoneAutoShoot || (RobotConstants.useFarZoneAutoShoot && ZoneUtil.inFarZone(currentPose))) &&
//                currentZone == ZoneUtil.Zone.CLOSE &&
//                inZone &&
//                        (robot.intake.isFull) &&
//                        robotState != RobotState.SHOOTING &&
//                Math.abs(robot.shooter.wantedVelocity - robot.shooter.currentVelocity) < RobotConstants.autoShootWheelSpeedEpsilonTicks &&
//                Math.abs(robot.turret.errorTicks) < RobotConstants.autoShootTurretTicksEpsilon &&
//                        robot.turret.currentPositionTicks > -TurretConstants.ticksPerRevolution + RobotConstants.autoShootTurretRangeEpsilon &&
//                        robot.turret.currentPositionTicks < -RobotConstants.autoShootTurretRangeEpsilon &&
//                        currentPose.distanceFrom(goalPose) > RobotConstants.MIN_SHOOTING_DISTANCE
//        ) {
//            shoot(currentPose, this.goalPose);
//        }

        // hopefully this will not be necessary
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
            isParking = !isParking;
            Pose parkPose = currentZone == ZoneUtil.Zone.CLOSE ?
                    alliance == Alliance.BLUE ? FieldConstants.BLUE_CLOSE_PARK_POSE : FieldConstants.RED_CLOSE_PARK_POSE :
                    alliance == Alliance.BLUE ? FieldConstants.BLUE_PARK_POSE : FieldConstants.RED_PARK_POSE;
            //drivetrain.park(parkPose);
        }

        // toggle tilt: y
        if (gamepad1.yWasPressed()) {
            isParking = true;
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
        //todo amahl doesn't want this rn
//        // hold pose: right stick
//        if (gamepad1.rightStickButtonWasPressed()) {
//            drivetrain.holdPose(drivetrain.getPose());
//        }

        drivetrain.update(-normalizeInput(gamepad1.left_stick_y),
                -normalizeInput(gamepad1.left_stick_x),
                -normalizeInput(gamepad1.right_stick_x));

        /** GAMEPAD 2 (OPERATOR) **/

        // x: reset turret
        if (gamepad2.xWasPressed()) {
            robot.turret.resetEncoderWithAbsoluteReading();
        }

        // y: set robot centric todo: test robot centric AHHH still need to do but no worry
        if (gamepad2.yWasPressed()) {
            drivetrain.setRobotCentric(!drivetrain.getRobotCentric());
        }

        // left stick: webcam relocalization and reset turret, also left bumper
        if (gamepad2.leftStickButtonWasPressed() || gamepad1.leftBumperWasPressed()) {
            // robot.turret.resetEncoderWithAbsoluteReading();
            Pose webcamPose = robot.cameraLocalizer.getPoseFromApriltag(robot.turret.currentAngle);
            if (webcamPose.getX() != 0 && webcamPose.getY() != 0) {
                robot.ledIndicator.indicateRelocalization();
                turretOffset = 0;
                ShootingConstants.wheelSpeedMultiplier = 1.0;
                // if large angle diff, trust the camera bc that means imu drifted, otherwise use pinpoint heading
                // also magic number here but wtf
                if (Math.abs(MathUtil.getSmallestAngleDifference(webcamPose.getHeading(), currentPose.getHeading())) > Math.toRadians(20)) {
                    drivetrain.follower.setPose(webcamPose);
                } else {
                    drivetrain.follower.setPose(new Pose(webcamPose.getX(), webcamPose.getY(), currentPose.getHeading()));
                }
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

        // right bumper: close zone
        if (gamepad2.rightBumperWasPressed()) {
            currentZone = ZoneUtil.Zone.CLOSE;
        }
        // left bumper: far zone
        if (gamepad2.leftBumperWasPressed()) {
            currentZone = ZoneUtil.Zone.FAR;
        }

        // trim speed up
        if (gamepad2.dpadUpWasPressed()) {
            ShootingConstants.wheelSpeedMultiplier += 0.005;
        }
        // trim speed down
        if (gamepad2.dpadDownWasPressed()) {
            ShootingConstants.wheelSpeedMultiplier -= 0.005;
        }

        // dpad left: trim turret
        if (gamepad2.dpadLeftWasPressed()) {
            turretOffset += Math.toRadians(2);
        }
        // dpad right: trim turret
        if (gamepad2.dpadRightWasPressed()) {
            turretOffset -= Math.toRadians(2);
        }

        if (gamepad2.left_trigger > 0.1) {
            robot.intake.wantedMode = Intake.Mode.INTAKE_FAST;
        }

//        if (gamepad2.right_trigger > 0.1) {
//            robot.intake.wantedMode = Intake.Mode.INTAKE_BACKWARD;
//        }

        ShootingConstants.ShooterOutputs shooterOutputs =
                RobotConstants.useShootOnTheMove ?
                        sotmUtil.calculateShooterOutputs(
                                drivetrain.getPose(),
                                drivetrain.getVelocity(),
                                drivetrain.getAcceleration(),
                                drivetrain.getAngularVelocity(),
                                RobotConstants.dt, alliance) :
                        sotmUtil.calculateShooterOutputs(drivetrain.getPose(),
                                new Vector(),
                                new Vector(),
                                0,
                                RobotConstants.dt, alliance);

        robot.shooter.wantedVelocity = shooterOutputs.wheelVelocity;
        robot.shooter.wantedAcceleration = shooterOutputs.wheelFeedforward;
        robot.shooter.wantedPitch = shooterOutputs.hoodAngle;

        if(isParking){
            robot.turret.wantedAngle = Math.toRadians(180);
            robot.turret.wantedAngularVelocity = 0;
            speedScaleFactor = 0.45;
        }
        else {
            robot.turret.wantedAngle = shooterOutputs.turretAngle + turretOffset;
            robot.turret.wantedAngularVelocity = shooterOutputs.turretFeedforward;
            speedScaleFactor = 1.0;
        }

        prevDetectState = robot.intake.detectionState;
        robot.update();

        blackboard.put(FieldConstants.END_POSE_KEY, drivetrain.follower.getPose());

        telemetry.addData("Loop time", robot.dt);
        telemetry.addData("Pose", currentPose);
        telemetry.addData("Offset", turretOffset);
        telemetry.addData("Distance", currentPose.distanceFrom(goalPose));
//        telemetry.addData("Current state", drivetrain.getState());
//        telemetry.addData("Angle to goal", Math.atan2(-(goalPose.getX()-currentPose.getX()), (goalPose.getY()- currentPose.getY())));
//        telemetry.addLine("Robot in shooting zone: " + inZone);
//        telemetry.addLine("Intake full: " + robot.intake.isFull);
//        telemetry.addLine("Top triggered" + robot.intake.topTriggered());
//        telemetry.addLine("Middle triggered" + robot.intake.middleTriggered());
//        telemetry.addLine("Bottom triggered" + robot.intake.bottomTriggered());
//        telemetry.addLine("Intake state: "+ robot.intake.detectionState);
//        telemetry.addLine("Drivetrain Busy: " + drivetrain.isBusy());
//        telemetry.addLine("Robot idle: " + (robotState == RobotState.IDLE));
//        telemetry.addLine("Shooter wheel error ticks: " + Math.abs(robot.shooter.wantedVelocity - robot.shooter.currentVelocity));
//        telemetry.addLine("Turret ticks error: " + robot.turret.errorTicks);

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
