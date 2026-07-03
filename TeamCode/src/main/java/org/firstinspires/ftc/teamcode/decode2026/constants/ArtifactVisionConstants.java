package org.firstinspires.ftc.teamcode.decode2026.constants;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.util.decodeutil.Matrix;
@Config
public class ArtifactVisionConstants {
    public static final double MIN_AREA = 300;
    public static final double MAX_AREA = 20000;
    public static final double fx = 212;
    public static final double fy = 212;
    public static final double cx = 320;
    public static final double cy = 240;
    public static final double IMAGE_WIDTH = 640;
    public static final double IMAGE_HEIGHT = 480;
    public static final double CAMERA_HEIGHT = 8.0; // in
    public static final double FORWARD_OFFSET = 8.0; // in
    public static final double CAMERA_PITCH = Math.toRadians(15); // 35 degrees tilted down
    // update this
    public static final Matrix cameraMatrix = new Matrix(
            new double[][] {
                    {fx, 0, cx},
                    {0, fy, cy},
                    {0, 0,  1 },
            }
    );

    public static final Matrix cameraMatrixInverse = new Matrix(
            new double[][] {
                    {1/fx, 0,    -cx/fx},
                    {0,    1/fy, -cy/fy},
                    {0,    0,     1    },
            }
    );

    public static final Matrix M0 = new Matrix(
            new double[][] {
                    {1, 0, 0},
                    {0, 0, 1},
                    {0, 1, 0},
            }
    );

    public static Matrix createRotationX(double angle) {
        double c = Math.cos(angle);
        double s = Math.sin(angle);
        return new Matrix(
                new double[][] {
                        {1, 0,  0},
                        {0, c, -s},
                        {0, s,  c},
                }
        );
    }

    public static Matrix createRotationZ(double angle) {
        double c = Math.cos(angle);
        double s = Math.sin(angle);
        return new Matrix(
                new double[][] {
                        {c, -s, 0},
                        {s,  c, 0},
                        {0,  0, 1},
                }
        );
    }
}
