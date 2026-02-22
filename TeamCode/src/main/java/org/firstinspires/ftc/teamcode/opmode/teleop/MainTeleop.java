package org.firstinspires.ftc.teamcode.opmode.teleop;

import static com.qualcomm.robotcore.eventloop.opmode.OpMode.blackboard;

import com.pedropathing.geometry.Pose;
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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.decodeutil.MathUtil;
import org.firstinspires.ftc.teamcode.util.decodeutil.Zone;
import org.firstinspires.ftc.teamcode.util.decodeutil.ZoneUtil;

public class MainTeleop {
    public enum RobotState {
        IDLE,
        SHOOTING
    }
    private RobotState robotState;
    private Intake.DetectionState prevDetectState;
    public TeleopDrivetrain drivetrain;
    private double turretOffset = 0;
    private double speedScaler = 1;
    private double longitudinalSpeed = 1, lateralSpeed = 1, rotationSpeed = 0.15;
    public TeleopRobot robot;
    private Pose gatePose, parkPose, goalPose, gateIntakePose;
    private Gamepad gamepad1, gamepad2;
    public SOTM sotm;
    private boolean automateRobot = true;
    private Telemetry telemetry;
    private Alliance alliance;
    private Zone currentZone;
    private ElapsedTime relocalizationTimer;
    private ZoneUtil zoneUtil;
    private double relocalizationTime = 10; // 10 seconds
    private boolean endgameOn = false;

    public MainTeleop(Pose startPose, Alliance alliance, HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2, boolean resetEncoders) {
        drivetrain = new TeleopDrivetrain(hardwareMap, alliance);
        drivetrain.setStartingPose(startPose);

        robot = new TeleopRobot(hardwareMap);
        if (resetEncoders) {
            robot.turret.resetEncoder();
        }
        robot.turret.resetEncoderWithAbsoluteReading();
        robot.turret.setUseExternal(false);

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
        this.relocalizationTimer = new ElapsedTime();
    }
    private double normalizeInput(double input) {
        return 1.1 * input;
        // Math.signum(input) * Math.sqrt(Math.abs(input));
    }

    private double getDistance(Pose a, Pose b) {
        return Math.hypot(a.getX()-b.getX(), a.getY()-b.getY());
    }


    public void loop() {
        Pose currentPose = drivetrain.getPose();
        Pose closestPose = zoneUtil.closestPose(drivetrain.follower.getPose(), currentZone);

        // SETTING ROBOT STATE
        if (robot.shootCommand.isFinished() && robot.shootCommandSlow.isFinished()) {
            robotState = RobotState.IDLE;
        } else {
            robotState = RobotState.SHOOTING;
        }

        // TODO: STOP INTAKE WHEN 3 BALLS.
//        if (robotState == RobotState.IDLE && robot.intake.isFull()) {
//            robot.intake.state = Intake.IntakeState.INTAKE_SLOW;
//        } else if (robotState == RobotState.IDLE && !robot.intake.isFull()) {
//            robot.intake.state = Intake.IntakeState.INTAKE_FAST;
//        }

        if (automateRobot) {
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

        if (relocalizationTimer.seconds() > relocalizationTime) {
            // TODO: automatic relocalization
//            Pose webcamPose = robot.webcamLocalizer.getCurrentPose();
//            Log.d("Webcam pose", webcamPose.toString());
//            drivetrain.follower.setPose(webcamPose);
//            relocalizationTimer.reset();
            // normal relocalization: checks velocity, angular velocity, and distance AND the timer to see if its over 10s
//            if (drivetrain.getVelocity().getMagnitude() > 5 && drivetrain.getAngularVelocity() > 0.1 && getDistance(currentPose, goalPose) < 100) {
//                Pose relocalizedPose = robot.webcamLocalizer.getCurrentPose();
//                if (MathUtil.distance(relocalizedPose, currentPose) < 5) { // threshold: 5 inches
//                    drivetrain.follower.setPose(new Pose(relocalizedPose.getX(), relocalizedPose.getY(), currentPose.getHeading()));
//                }
//            }
        }

        // GAMEPAD 1 (DRIVER)

        // shoot: right bumper
        if (gamepad1.rightBumperWasPressed()) {
            // i would consider 120 to be far
            if (MathUtil.distance(currentPose, goalPose) > 120)  {
                robot.shootCommandSlow.start();
            } else {
                robot.shootCommand.start();
            }
        }

        // RIGHT TRIGGER: hold for gate heading lock
        if (Math.abs(gamepad1.right_trigger) > 0.01)  {
            drivetrain.setGateHeadingLock(true);
        } else {
            drivetrain.setGateHeadingLock(false);
        }

        // LEFT TRIGGER: slow mo
        if (Math.abs(gamepad1.left_trigger) > 0.01) {
            speedScaler = 0.4;
        } else {
            speedScaler = 1;
        }

        // added just for this
        if (gamepad1.dpadUpWasPressed()) {
            automateRobot = !automateRobot;
        }

        // set robot centric: x (used to be intake gate but not anymore)
        if (gamepad1.xWasPressed()) {
            drivetrain.setRobotCentric(!drivetrain.getRobotCentric());
            // drivetrain.intakeGate();
        }

        // gate open: y
        if (gamepad1.yWasPressed()) {
            drivetrain.openGate();
        }

        if (gamepad1.leftStickButtonWasPressed() || gamepad1.rightStickButtonWasPressed()) {
            drivetrain.breakFollowing();
        }

        // GAMEPAD 2 (OPERATOR)

        // x: reset turret
        if (gamepad2.xWasPressed()) {
            robot.resetTurretCommand.start();
        }

        // y: park
        if (gamepad2.yWasPressed()) {
            drivetrain.park();
            endgameOn = !endgameOn;
        }

        // slow mo for good park, 0.4x speed
        // TODO: set up hang buttons. folded up = 0.73 and down = 0.5, so 0.5 when parking
//        if (Math.abs(gamepad2.right_trigger) > 0.05) {
//            robot.pivot.setPower(-gamepad2.right_trigger);
//        }
//
//        if (Math.abs(gamepad2.left_trigger) > 0.05) {
//            robot.pivot.setPower(gamepad2.left_trigger);
//        }
//
//        if (Math.abs(gamepad2.right_trigger) < 0.05 && Math.abs(gamepad2.left_trigger) < 0.05) {
//            robot.pivot.setPower(0);
//        }

        // relocalization: left stick OVERRIDE with webcam
        if (gamepad2.leftStickButtonWasPressed()) {
            Pose webcamPose = robot.webcamLocalizer.getCurrentPose();
            if (webcamPose.getX() != 0 && webcamPose.getY() != 0) {
                drivetrain.follower.setPose(webcamPose);
            }
            // TODO: CHANGE TO WEBCAM
//            if (robot.webcamLocalizer.getIsGoodDetection()) {
//                Pose webcamPose = robot.webcamLocalizer.getCurrentPose();
//                Log.d("Webcam pose", webcamPose.toString());
//                drivetrain.follower.setPose(webcamPose);
//            }
//            Pose webcamPose = robot.webcamLocalizer.getCurrentPose();
//            drivetrain.follower.setPose(webcamPose);
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

        double[] values = sotm.calculateAzimuthThetaVelocityFRCBetter(currentPose, drivetrain.getVelocity(), drivetrain.getAngularVelocity());

        if (endgameOn) {
            robot.turret.setTarget(Math.toRadians(-15));
            robot.turret.setFeedforward(0);
            robot.shooter.setShooterPitch(RobotConstants.PITCH_I);
            robot.shooter.setTargetVelocity(0);
        } else {
            if (robot.resetTurretCommand.isFinished()) {
                robot.turret.setTarget(values[0]+turretOffset);
                robot.turret.setFeedforward(values[3]);
                robot.shooter.setShooterPitch(values[1]);
                robot.shooter.setTargetVelocity(values[2]);
            } else {
                robot.shooter.setShooterPitch(values[1]);
                robot.turret.setFeedforward(0);
            }
        }

        prevDetectState = robot.intake.detectionState;

        robot.update();

        telemetry.addData("Pose", currentPose);
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
    }

    public void stop() {
        blackboard.put(RobotConstants.END_POSE_KEY, drivetrain.follower.getPose());
    }
}
