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

        double manualX = driveX * DrivetrainConstants.MOVEMENT_SPEED_MULTIPLIER;
        double manualY = driveY * DrivetrainConstants.MOVEMENT_SPEED_MULTIPLIER;
        double manualTurn = driveTurn * DrivetrainConstants.TURN_SPEED_MULTIPLIER;

        double xPidOutput = xLocked
                ? MathUtil.clamp(xController.calculate(follower.getPose().getX(), lockedX), -1, 1)
                : 0;
        double yPidOutput = yLocked
                ? MathUtil.clamp(yController.calculate(follower.getPose().getY(), lockedY), -1, 1)
                : 0;
        double headingPidOutput = 0;
        if (headingLocked) {
            double headingError = MathFunctions.getTurnDirection(heading, lockedHeading)
                    * MathFunctions.getSmallestAngleDifference(heading, lockedHeading);
            headingPidOutput = MathUtil.clamp(headingController.calculate(0, headingError), -1, 1);
        }

        double[] powers = computeDrivePowers(
                heading, allianceHeadingOffset, robotCentric,
                manualX, manualY, manualTurn,
                xLocked, xPidOutput,
                yLocked, yPidOutput,
                headingLocked, headingPidOutput
        );

        follower.setTeleOpDrive(powers[0], powers[1], powers[2], true);
    }

    /**
     * Pure geometry: blends locked-axis PID output (already absolute field-frame, alliance-
     * invariant) with manual joystick input (rotated into field frame by the live heading in
     * robot-centric mode, or by the fixed alliance offset in field-centric mode), then rotates
     * the combined field-frame vector into robot frame for {@code Follower#setTeleOpDrive}.
     * Extracted as a static pure function (no Follower/hardware access) so it's unit-testable.
     */
    static double[] computeDrivePowers(
            double heading, double allianceHeadingOffset, boolean robotCentric,
            double manualX, double manualY, double manualTurn,
            boolean xLocked, double xPidOutput,
            boolean yLocked, double yPidOutput,
            boolean headingLocked, double headingPidOutput
    ) {
        double manualRotation = robotCentric ? heading : allianceHeadingOffset;
        double fieldManualX = manualX * Math.cos(manualRotation) - manualY * Math.sin(manualRotation);
        double fieldManualY = manualX * Math.sin(manualRotation) + manualY * Math.cos(manualRotation);

        double fieldX = xLocked ? xPidOutput : fieldManualX;
        double fieldY = yLocked ? yPidOutput : fieldManualY;
        double turn = headingLocked ? headingPidOutput : manualTurn;

        double robotX = fieldX * Math.cos(-heading) - fieldY * Math.sin(-heading);
        double robotY = fieldX * Math.sin(-heading) + fieldY * Math.cos(-heading);

        return new double[] {robotX, robotY, turn};
    }
}
