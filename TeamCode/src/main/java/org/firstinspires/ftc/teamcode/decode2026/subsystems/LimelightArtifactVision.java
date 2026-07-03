package org.firstinspires.ftc.teamcode.decode2026.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.decode2026.constants.ArtifactVisionConstants;
import org.firstinspires.ftc.teamcode.lib.robot.Subsystem;
import org.firstinspires.ftc.teamcode.util.decodeutil.Matrix;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

public class LimelightArtifactVision extends Subsystem {
    private final Limelight3A limelight;

    public LimelightArtifactVision(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(7);
        limelight.start();
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void reset() {
        limelight.start();
    }

    @Override
    public void update() {
        super.update();
    }

    private static Matrix cameraToWorldMatrix(Pose robotPose) {
        double pitch = ArtifactVisionConstants.CAMERA_PITCH;
        double roll = Math.toRadians(0);
        double heading = robotPose.getHeading();

        return ArtifactVisionConstants.createRotationZ(heading - Math.PI / 2)
                .multiply(ArtifactVisionConstants.createRotationX(-pitch))
                .multiply(ArtifactVisionConstants.M0)
                .multiply(ArtifactVisionConstants.createRotationZ(-roll));
    }

    // Converts Limelight tx/ty angles (degrees) to a world ground-plane point.
    // Limelight ty is already camera-y-up, so no sign flip is needed.
    private Point getWorldPosition(double txDeg, double tyDeg, Pose robotPose) {
        Matrix cameraDirection = new Matrix(
                new double[][] {
                        {Math.tan(Math.toRadians(txDeg))},
                        {Math.tan(Math.toRadians(tyDeg))},
                        {1.0}
                }
        );

        Matrix rotationMatrix = cameraToWorldMatrix(robotPose);
        Matrix worldDirection = rotationMatrix.multiply(cameraDirection);

        if (worldDirection.get(2, 0) >= -1e-9) {
            return null;
        }

        double t = -ArtifactVisionConstants.CAMERA_HEIGHT / worldDirection.get(2, 0);

        if (t <= 0) {
            return null;
        }

        Matrix origin = new Matrix(
                new double[][] {
                        {robotPose.getX() + ArtifactVisionConstants.FORWARD_OFFSET * Math.cos(robotPose.getHeading())},
                        {robotPose.getY() + ArtifactVisionConstants.FORWARD_OFFSET * Math.sin(robotPose.getHeading())},
                        {ArtifactVisionConstants.CAMERA_HEIGHT}
                }
        );

        Matrix worldPoint = origin.add(worldDirection.multiply(t));

        return new Point(worldPoint.get(0, 0), worldPoint.get(1, 0));
    }

    // Returns a list of {tx, ty} pairs from the latest Limelight detector results.
    public List<double[]> getBlobs() {
        List<double[]> blobs = new ArrayList<>();
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            for (LLResultTypes.DetectorResult dr : result.getDetectorResults()) {
                blobs.add(new double[]{dr.getTargetXDegrees(), dr.getTargetYDegrees()});
            }
        }
        return blobs;
    }

    public List<Point> getArtifactWorldPoints(Pose robotPose) {
        List<double[]> blobs = getBlobs();
        List<Point> worldPoints = new ArrayList<>();

        for (double[] txty : blobs) {
            Point worldPoint = getWorldPosition(txty[0], txty[1], robotPose);
            if (worldPoint != null) {
                worldPoints.add(worldPoint);
            }
        }

        return worldPoints;
    }

    // Returns the start y of the 15-inch window that contains the most detections.
    // Each detection's y is used as a candidate window start; returns -1 if none found.
    public double findBestYPosition(Pose robotPose, double minY, double maxY) {
        List<Point> worldPoints = getArtifactWorldPoints(robotPose);
        List<Point> filtered = worldPoints.stream()
                .filter(p -> p.y >= minY && p.y <= maxY)
                .collect(Collectors.toList());

        if (filtered.isEmpty()) {
            return -1;
        }

        final double WINDOW = 15.0;
        double bestY = -1;
        int bestCount = 0;

        for (Point candidate : filtered) {
            // Clamp window start so the window fits within [minY, maxY].
            final double windowStart = Math.min(candidate.y, maxY);
            int count = (int) filtered.stream()
                    .filter(p -> p.y >= windowStart && p.y <= windowStart + WINDOW)
                    .count();

            if (count > bestCount) {
                bestCount = count;
                bestY = windowStart;
            }
        }

        return bestY;
    }
}
