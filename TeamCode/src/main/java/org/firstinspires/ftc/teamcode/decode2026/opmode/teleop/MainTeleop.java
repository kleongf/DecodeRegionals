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
import org.firstinspires.ftc.teamcode.decode2026.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.decodeutil.Alliance;
import org.firstinspires.ftc.teamcode.util.decodeutil.SOTMUtil;
import org.firstinspires.ftc.teamcode.util.decodeutil.TeleopDrivetrain;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.util.decodeutil.MathUtil;
import org.firstinspires.ftc.teamcode.util.decodeutil.Zone;
import org.firstinspires.ftc.teamcode.util.decodeutil.ZoneUtil;

public class MainTeleop {
    public enum RobotState {
        IDLE,
        SHOOTING
    }

    private Intake.DetectionState prevDetectState;
    public TeleopDrivetrain drivetrain;
    private double turretOffset = 0;
    private double longitudinalSpeed = 1, lateralSpeed = 1, rotationSpeed = 0.35;
    public CurrentRobot robot;
    private final Pose goalPose;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;
    private final SOTMUtil sotmUtil;
    private final Telemetry telemetry;
    private final Alliance alliance;
    private Zone currentZone;
    private final ElapsedTime loopTimer;
    private final ElapsedTime relocalizationTimer;
    private final ElapsedTime turretResetTimer;
    private final ZoneUtil zoneUtil;
    private boolean endgameOn = false;

    public MainTeleop(Pose startPose, Alliance alliance, HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        drivetrain = new TeleopDrivetrain(hardwareMap, alliance);
        drivetrain.setStartingPose(startPose);

        robot = new CurrentRobot(hardwareMap);
        robot.reset();

        this.goalPose = alliance == Alliance.BLUE ? FieldConstants.BLUE_GOAL_POSE :  FieldConstants.RED_GOAL_POSE;

        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        this.telemetry = telemetry;
        this.alliance = alliance;

        this.sotmUtil = new SOTMUtil(this.goalPose);
        this.currentZone = Zone.FAR;
        this.zoneUtil = new ZoneUtil(RobotConstants.robotRadius);

        this.prevDetectState = Intake.DetectionState.EMPTY;
        this.relocalizationTimer = new ElapsedTime();
        this.turretResetTimer = new ElapsedTime();
        this.loopTimer = new ElapsedTime();
    }
    private double normalizeInput(double input) {
        return 1.1 * input;
    }

    public void loop() {
        double dt = loopTimer.seconds() > 0 ? loopTimer.seconds() : 0.02;
        Pose currentPose = drivetrain.getPose();
        Pose closestPose = zoneUtil.closestPose(drivetrain.follower.getPose(), currentZone);

        RobotState robotState;

        if (robot.shootCommand.isFinished() && robot.shootCommandSlow.isFinished()) {
            robotState = RobotState.IDLE;
        } else {
            robotState = RobotState.SHOOTING;
        }

        if (robot.intake.isFull && prevDetectState != robot.intake.detectionState) {
            robot.ledIndicator.indicateIntakeFull();
        }

        if (RobotConstants.useAutomateRobotDrive) {
            // if prev few states were the same, then we didn't shoot anything, therefore no need to auto drive again
            if (!zoneUtil.inZone(currentPose, currentZone) && robot.intake.isFull && !drivetrain.isBusy() && robotState == RobotState.IDLE && prevDetectState != robot.intake.detectionState) {
                drivetrain.kick(closestPose);
            }
        }

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
            if (MathUtil.distance(currentPose, goalPose) > RobotConstants.farShootingDistanceThreshold)  {
                robot.shootCommandSlow.start();
            } else {
                robot.shootCommand.start();
            }
        }

        // RIGHT TRIGGER: hold for gate heading lock
        drivetrain.gateHeadingLock = Math.abs(gamepad1.right_trigger) > 0.01;

        drivetrain.openGateHeadingLock = Math.abs(gamepad1.left_trigger) > 0.01;

        // set robot centric: x
        if (gamepad1.xWasPressed()) {
            drivetrain.setRobotCentric(!drivetrain.getRobotCentric());
        }

        // gate open: y
        if (gamepad1.yWasPressed()) {
            drivetrain.openGate();
        }

        // stop auto drive: left/right stick
        if (gamepad1.leftStickButtonWasPressed() || gamepad1.rightStickButtonWasPressed()) {
            drivetrain.breakFollowing();
        }

        /** GAMEPAD 2 (OPERATOR) **/

        // x: reset turret
        if (gamepad2.xWasPressed()) {
            robot.turret.resetEncoderWithAbsoluteReading();
        }

        // y: park
        if (gamepad2.yWasPressed()) {
            drivetrain.park();
            endgameOn = !endgameOn;
        }

        // left stick: webcam relocalization
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
            currentZone = Zone.CLOSE;
        }
        // far zone: dpad down
        if (gamepad2.dpadDownWasPressed()) {
            currentZone = Zone.FAR;
        }

        if (alliance == Alliance.BLUE) {
            drivetrain.update(-normalizeInput(gamepad1.left_stick_y*longitudinalSpeed),
                    -normalizeInput(gamepad1.left_stick_x*lateralSpeed),
                    -normalizeInput(gamepad1.right_stick_x*rotationSpeed));
        } else if (alliance == Alliance.RED) {
            drivetrain.update(-normalizeInput(gamepad1.left_stick_y*longitudinalSpeed),
                    -normalizeInput(gamepad1.left_stick_x*lateralSpeed),
                    -normalizeInput(gamepad1.right_stick_x*rotationSpeed));
        }
        double[] values = RobotConstants.useShootOnTheMove ?
                sotmUtil.calculateAzimuthFeedforwardThetaVelocity(currentPose, drivetrain.getVelocity(), drivetrain.getAngularVelocity(), dt) :
                sotmUtil.calculateAzimuthFeedforwardThetaVelocity(currentPose, new Vector(), 0, dt);

        robot.turret.wantedAngle = values[0]+turretOffset;
        robot.turret.angularVelocityToGoal = values[1];
        robot.shooter.wantedPitch = values[2];
        robot.shooter.wantedVelocity = values[3];

        prevDetectState = robot.intake.detectionState;
        loopTimer.reset();
        robot.update();

        telemetry.addData("Loop time", dt);
        telemetry.addData("Pose", currentPose);
        telemetry.addData("Current state", drivetrain.getState());
        telemetry.addData("Angle to goal", Math.atan2(-(goalPose.getX()-currentPose.getX()), (goalPose.getY()- currentPose.getY())));
        telemetry.addLine("Robot in shooting zone: " + zoneUtil.inZone(currentPose, currentZone));
        telemetry.addLine("Intake full: " + robot.intake.isFull);
        telemetry.addLine("Intake state: "+ robot.intake.detectionState);
        telemetry.addLine("Drivetrain Busy: " + drivetrain.isBusy());
        telemetry.addLine("Robot idle: " + (robotState == RobotState.IDLE));
        telemetry.update();
    }


    public void start() {
        robot.start();
    }

    public void stop() {
        blackboard.put(FieldConstants.END_POSE_KEY, drivetrain.follower.getPose());
    }
}
