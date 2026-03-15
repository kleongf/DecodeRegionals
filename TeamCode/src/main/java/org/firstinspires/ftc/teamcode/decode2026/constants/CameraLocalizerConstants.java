package org.firstinspires.ftc.teamcode.decode2026.constants;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class CameraLocalizerConstants {
    public static final Position cameraPosition = new Position(DistanceUnit.MM,
            138, 119, 236, 0);
    public static final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -70, 0, 0);
    public static final double fx = 915.89533774;
    public static final double fy = 916.57002166;
    public static final double cx = 665.64617643;
    public static final double cy = 423.48045066;
    public static final Size cameraResolution = new Size(1280, 800);
}
