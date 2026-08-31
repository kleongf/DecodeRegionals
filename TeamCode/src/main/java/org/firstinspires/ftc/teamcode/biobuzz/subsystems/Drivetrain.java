package org.firstinspires.ftc.teamcode.biobuzz.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.biobuzz.constants.DrivetrainConstants;
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

    private final WeightedSetpointPIDController xController;
    private final WeightedSetpointPIDController yController;
    private final WeightedSetpointPIDController headingController;

    public Drivetrain(HardwareMap hardwareMap) {
        follower = Constants.createFollower(hardwareMap);
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

        double x = driveX * DrivetrainConstants.MOVEMENT_SPEED_MULTIPLIER;
        double y = driveY * DrivetrainConstants.MOVEMENT_SPEED_MULTIPLIER;
        double turn = driveTurn * DrivetrainConstants.TURN_SPEED_MULTIPLIER;

        if (xLocked) {
            x = MathUtil.clamp(xController.calculate(follower.getPose().getX(), lockedX), -1, 1);
        }
        if (yLocked) {
            y = MathUtil.clamp(yController.calculate(follower.getPose().getY(), lockedY), -1, 1);
        }
        if (headingLocked) {
            double headingError = MathFunctions.getTurnDirection(follower.getPose().getHeading(), lockedHeading)
                    * MathFunctions.getSmallestAngleDifference(follower.getPose().getHeading(), lockedHeading);
            turn = MathUtil.clamp(headingController.calculate(0, headingError), -1, 1);
        }

        follower.setTeleOpDrive(x, y, turn, robotCentric);
    }
}
