package org.firstinspires.ftc.teamcode.decode2026.constants;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.util.decodeutil.MathUtil;
import org.firstinspires.ftc.teamcode.util.decodeutil.Matrix;
@Config
public class ArtifactVisionConstants {
    public static final double fx = 212;
    public static final double fy = 212;
    public static final double cx = 320;
    public static final double cy = 240;
    public static final double CAMERA_HEIGHT = MathUtil.mmToIn(237); // in
    public static final double FORWARD_OFFSET = 8.0; // in
    public static final double CAMERA_PITCH = Math.toRadians(12); // 12 degrees tilted down
    public static final double CORNER_MAX_MULTIPLIER = 1.7;
    public static final double WINDOW_SIZE = 15.0; // intake width (roughly)
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
