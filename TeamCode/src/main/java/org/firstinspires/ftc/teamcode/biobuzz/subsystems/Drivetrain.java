package org.firstinspires.ftc.teamcode.biobuzz.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.biobuzz.constants.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.lib.Alliance;
import org.firstinspires.ftc.teamcode.lib.controllers.WeightedSetpointPIDController;
import org.firstinspires.ftc.teamcode.lib.robot.Subsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.MathUtil;

public class Drivetrain extends Subsystem {
    public final Follower follower;

    public double driveX, driveY, driveTurn;
    public boolean robotCentric = false;

    public boolean xLocked, yLocked, headingLocked;
    private double lockedX, lockedY, lockedHeading;

    // BLUE and RED driver stations face opposite directions across a mirrored field, so
    // "stick forward" corresponds to a different fixed field angle per alliance.
    private final double allianceHeadingOffset;

    private final WeightedSetpointPIDController xController;
    private final WeightedSetpointPIDController yController;
    private final WeightedSetpointPIDController headingController;

    public Drivetrain(HardwareMap hardwareMap, Alliance alliance) {
        follower = Constants.createFollower(hardwareMap);
        allianceHeadingOffset = alliance == Alliance.BLUE ? Math.PI : 0;
        xController = new WeightedSetpointPIDController(DrivetrainConstants.xP, DrivetrainConstants.xI, DrivetrainConstants.xD);
        yController = new WeightedSetpointPIDController(DrivetrainConstants.yP, DrivetrainConstants.yI, DrivetrainConstants.yD);
        headingController = new WeightedSetpointPIDController(DrivetrainConstants.headingP, DrivetrainConstants.headingI, DrivetrainConstants.headingD);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    public void driveManual(double x, double y, double turn) {
        driveX = x;
        driveY = y;
        driveTurn = turn;
    }

    public void lockX(double targetX) {
        xLocked = true;
        lockedX = targetX;
    }

    public void lockY(double targetY) {
        yLocked = true;
        lockedY = targetY;
    }

    public void lockHeading(double targetHeadingRadians) {
        headingLocked = true;
        lockedHeading = targetHeadingRadians;
    }

    public void unlockX() { xLocked = false; }
    public void unlockY() { yLocked = false; }
    public void unlockHeading() { headingLocked = false; }
    public void unlockAll() { xLocked = yLocked = headingLocked = false; }

    public boolean atXTarget(double tolerance) {
        return !xLocked || Math.abs(follower.getPose().getX() - lockedX) < tolerance;
    }

    public boolean atYTarget(double tolerance) {
        return !yLocked || Math.abs(follower.getPose().getY() - lockedY) < tolerance;
    }

    public boolean atHeadingTarget(double tolerance) {
        return !headingLocked || Math.abs(MathFunctions.getSmallestAngleDifference(follower.getPose().getHeading(), lockedHeading)) < tolerance;
    }

    @Override
    public void periodic() {
        follower.update();

        if (follower.isBusy()) {
            // an autonomous PathChain is actively driving the follower — don't fight it
            return;
        }

        double heading = follower.getPose().getHeading();

        // Locked axes are absolute field-frame PID targets, so they must never be rotated
        // by the alliance offset (only raw manual joystick input needs that). Manual input
        // is rotated into field frame here — by the live heading in robot-centric mode
        // (a no-op once the final rotation below undoes it), or by the fixed alliance
        // offset in field-centric mode — so that locked and manual axes can be combined
        // in a single shared field frame before one final rotation into robot frame.
        double manualX = driveX * DrivetrainConstants.MOVEMENT_SPEED_MULTIPLIER;
        double manualY = driveY * DrivetrainConstants.MOVEMENT_SPEED_MULTIPLIER;
        double manualRotation = robotCentric ? heading : allianceHeadingOffset;
        double fieldManualX = manualX * Math.cos(manualRotation) - manualY * Math.sin(manualRotation);
        double fieldManualY = manualX * Math.sin(manualRotation) + manualY * Math.cos(manualRotation);

        double fieldX = xLocked
                ? MathUtil.clamp(xController.calculate(follower.getPose().getX(), lockedX), -1, 1)
                : fieldManualX;
        double fieldY = yLocked
                ? MathUtil.clamp(yController.calculate(follower.getPose().getY(), lockedY), -1, 1)
                : fieldManualY;

        double turn = driveTurn * DrivetrainConstants.TURN_SPEED_MULTIPLIER;
        if (headingLocked) {
            double headingError = MathFunctions.getTurnDirection(heading, lockedHeading)
                    * MathFunctions.getSmallestAngleDifference(heading, lockedHeading);
            turn = MathUtil.clamp(headingController.calculate(0, headingError), -1, 1);
        }

        // Rotate the combined field-frame vector into robot frame ourselves, then always
        // drive the follower in robot-centric mode — this keeps us in full control of when
        // the alliance offset applies instead of relying on Follower's own field-centric
        // rotation, which would apply uniformly to locked-axis output too.
        double robotX = fieldX * Math.cos(-heading) - fieldY * Math.sin(-heading);
        double robotY = fieldX * Math.sin(-heading) + fieldY * Math.cos(-heading);

        follower.setTeleOpDrive(robotX, robotY, turn, true);
    }
}
