package org.firstinspires.ftc.teamcode.decode2026.constants;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
// corner, run up, run up,

public class CameraLocalizerConstants {
    /**
     * Variables to store the position and orientation of the camera on the robot. Setting these
     * values requires a definition of the axes of the camera and robot:
     *
     * Camera axes:
     * Origin location: Center of the lens
     * Axes orientation: +x right, +y down, +z forward (from camera's perspective)
     *
     * Robot axes (this is typical, but you can define this however you want):
     * Origin location: Center of the robot at field height
     * Axes orientation: +x right, +y forward, +z upward
     *
     * Position:
     * If all values are zero (no translation), that implies the camera is at the center of the
     * robot. Suppose your camera is positioned 5 inches to the left, 7 inches forward, and 12
     * inches above the ground - you would need to set the position to (-5, 7, 12).
     *
     * Orientation:
     * If all values are zero (no rotation), that implies the camera is pointing straight up. In
     * most cases, you'll need to set the pitch to -90 degrees (rotation about the x-axis), meaning
     * the camera is horizontal. Use a yaw of 0 if the camera is pointing forwards, +90 degrees if
     * it's pointing straight left, -90 degrees for straight right, etc. You can also set the roll
     * to +/-90 degrees if it's vertical, or 180 degrees if it's upside-down.
     */

    public static final Position cameraPositionRight = new Position(DistanceUnit.MM,
            169, -120, 130, 0);
    public static final YawPitchRollAngles cameraOrientationRight = new YawPitchRollAngles(AngleUnit.DEGREES,
            -90, -70, 0, 0);
    public static final Position cameraPositionLeft = new Position(DistanceUnit.MM,
            -169, -120, 130, 0);
    public static final YawPitchRollAngles cameraOrientationLeft = new YawPitchRollAngles(AngleUnit.DEGREES,
            90, -70, 0, 0);
//  [[938.60008119   0.         602.16868966]
// [  0.         937.20744359 353.82015718]
// [  0.           0.           1.        ]]

    // right
//     [[912.59997673   0.         657.3040482 ]
//             [  0.         910.71508727 408.55061061]
//             [  0.           0.           1.        ]]

    public static final double fxLeft = 909.26403429;
    public static final double fyLeft = 905.63651776;
    public static final double cxLeft = 628.49810599;
    public static final double cyLeft = 413.74093954;
    // todo: needs a bit more tuning but should be fine
    public static final double fxRight = 912.59997673; //915.89533774;
    public static final double fyRight = 910.71508727; //916.57002166;
    public static final double cxRight = 657.3040482; // 665.64617643;
    public static final double cyRight = 398.55061061; //423.48045066;
    public static final Size cameraResolutionRight = new Size(1280, 800);
    public static final Size cameraResolutionLeft = new Size(1280, 800);
}
