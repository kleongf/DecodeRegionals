package org.firstinspires.ftc.teamcode.decode2026.subsystems;

import android.util.Size;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Curve;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.decode2026.constants.ArtifactVisionConstants;
import org.firstinspires.ftc.teamcode.lib.robot.Subsystem;
import org.firstinspires.ftc.teamcode.util.decodeutil.CatmullRomSpline;
import org.firstinspires.ftc.teamcode.util.decodeutil.MathUtil;
import org.firstinspires.ftc.teamcode.util.decodeutil.Matrix;
import org.firstinspires.ftc.teamcode.util.decodeutil.vision.ArtifactProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

public class ArtifactVision extends Subsystem {
    private final ArtifactProcessor artifactLocator;
    private final VisionPortal portal;
    private final double MIN_BLOB_SIZE = 200;
    private final double MAX_BLOB_SIZE = 30000;
    public ArtifactVision(HardwareMap hardwareMap) {
        artifactLocator = new ArtifactProcessor.Builder()
                .build();

        portal = new VisionPortal.Builder()
                .addProcessor(artifactLocator)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setCameraResolution(new Size(640, 480))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .enableLiveView(true)
                // .setLiveViewContainerId(0) // idk about this one but ok
                .build();

    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void reset() {
        super.reset();
    }

    @Override
    public void update() {
        // we don't need to actually update anything
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
    private Point getWorldPosition(Point point, Pose robotPose) {
        Matrix pixel = new Matrix(
                new double[][] {
                        {point.x},
                        {point.y},
                        {1.0}
                }
        );

        Matrix cameraDirection = ArtifactVisionConstants.cameraMatrixInverse.multiply(pixel);
        cameraDirection.set(1, 0, cameraDirection.get(1, 0) * -1);

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
                new double[][]{
                        {robotPose.getX() + ArtifactVisionConstants.FORWARD_OFFSET * Math.cos(robotPose.getHeading())},
                        {robotPose.getY() + ArtifactVisionConstants.FORWARD_OFFSET * Math.sin(robotPose.getHeading())},
                        {ArtifactVisionConstants.CAMERA_HEIGHT}
                }
        );

        Matrix worldPoint = origin.add(worldDirection.multiply(t));

        return new Point(worldPoint.get(0, 0), worldPoint.get(1, 0));
    }

    public List<Point> getBlobs() {
        List<ArtifactProcessor.Blob> blobs = artifactLocator.getBlobs();
        if (blobs == null) {
            return new ArrayList<>();
        }

        // filter out very small blobs
        ArtifactProcessor.Util.filterByCriteria(
                ArtifactProcessor.BlobCriteria.BY_CONTOUR_AREA,
                MIN_BLOB_SIZE, MAX_BLOB_SIZE, blobs);

        if (blobs.isEmpty()) {
            return new ArrayList<>();
        }

        List<Point> cameraPoints = new ArrayList<>();

        for (ArtifactProcessor.Blob b : blobs) {
            RotatedRect boxFit = b.getBoxFit();
            if (boxFit != null) {
                Point cameraPoint = new Point(boxFit.center.x, boxFit.center.y + boxFit.size.height / 2);
                cameraPoints.add(cameraPoint);
            }
        }

        return cameraPoints;
    }

    public List<Point> getArtifactWorldPoints(Pose robotPose) {
        // get points
        // convert all points into coordinates
        List<Point> cameraPoints = getBlobs();
        List<Point> worldPoints = new ArrayList<>();

        for (Point cameraPoint: cameraPoints) {
            Point worldPoint = getWorldPosition(cameraPoint, robotPose);
            if (worldPoint != null) {
                worldPoints.add(worldPoint);
            }
        }

        return worldPoints;
    }
    public Curve constructSpline(Pose robotPose, double xMin, double xMax, double yMin, double yMax) {
        // TODO: sort by the 3 LARGEST whoops actually it already did that LMAO,
        //  since we sorted by contour size
        List<Point> artifactPoints = getArtifactWorldPoints(robotPose);
        // is this best practice to return null? idk ask ai but in this case,
        // its prob best to make a path to sit at gate, so maybe return that instead.
        if (artifactPoints.isEmpty()) {
            return null;
        }

        // idk what happens if the list is too short, or if its inclusive or exclusive, ill assume exclusive
        List<Point> bestPoints = artifactPoints.subList(0,3);
        List<Pose> splinePoses = new ArrayList<>();

        splinePoses.add(robotPose);
        for (Point p: bestPoints) {
            splinePoses.add(new Pose(MathUtil.clamp(p.x, xMin, xMax), MathUtil.clamp(p.y, yMin, yMax)));
        }

        return new CatmullRomSpline(splinePoses);
        // btw the go back path should be a constant heading interpolation
        // so we can ensure camera gets a good enough fov

        // if the spline stuff ends up being a bad idea
        // we can just get these world points and just go to their x positions

        // also all paths should be dynamic
        // like if at any point we have 3 we should transition to the leave to go back state

        // so we can actually have a really cool continuous loop in the state machine
        // and if we ever have not enough time we park. this is checked before the intake path and return path.
    }

    public double findBestYPosition(Pose robotPose, double minY, double maxY) {
        List<Point> artifactWorldPoints = getArtifactWorldPoints(robotPose);
        // filter blobs between y-values
        List<Point> filteredWorldPoints = artifactWorldPoints.stream()
                .filter(p -> p.y < maxY && p.y > minY)
                .collect(Collectors.toList());
        if (filteredWorldPoints.isEmpty()) {
            return -1; // no artifacts detected between these heights
        }
        // largest blob is already first because of the filter
        return filteredWorldPoints.get(0).y;
    }
}
