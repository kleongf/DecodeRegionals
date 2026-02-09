package org.firstinspires.ftc.teamcode.robot.subsystems;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.util.decodeutil.Alliance;
import org.firstinspires.ftc.teamcode.util.decodeutil.vision.ArtifactProcessor;
import org.firstinspires.ftc.teamcode.util.decodeutil.Subsystem;
import org.firstinspires.ftc.teamcode.util.decodeutil.Matrix;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;

import java.util.ArrayList;
import java.util.List;

// RAY projection but better
public class ArtifactVision2 extends Subsystem {
    private VisionPortal portal;
    private ArtifactProcessor colorLocator;
    private double bestX = -19;
    private double cameraAngle = Math.toRadians(0); // if angled down, -10 degrees or something
    // camera intrinsic matrix. since focusing the new lens it may need some retuning.
    private Matrix K = new Matrix(new double[][] {
            {214.1037056, 0, 313},
            {0, 212.72822576, 254.488},
            {0, 0, 1}
    });
    // R0: permutation matrix to convert camera coordinates into real-life ones.
    private Matrix R0 = new Matrix(new double[][] {
            {-1, 0, 0},
            {0, 0, 1},
            {0, -1, 0}
    });
    // R: rotation matrix to rotate if we choose to pitch camera down
    private Matrix R = new Matrix(new double[][] {
            {Math.cos(cameraAngle), 0, Math.sin(cameraAngle)},
            {0, 1, 0},
            {-Math.sin(cameraAngle), 0, Math.cos(cameraAngle)}
    });
    private Matrix RCombined = R0.multiply(R);

    private Matrix T = new Matrix(new double[][] {
            {-4.80315, -9.05512, 5.59055} // x: +right, y: +down, z: +forward
    }).transpose();
    public ArtifactVision2(HardwareMap hardwareMap, Alliance alliance) {
        colorLocator = new ArtifactProcessor.Builder()
                .build();

        portal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                // .setStreamFormat(VisionPortal.StreamFormat.MJPEG) // TODO: sometimes faster?
                .setCameraResolution(new Size(640, 480))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

//        private Position cameraPosition = new Position(DistanceUnit.MM,
//                -122, 142, 230, 0);

        if (alliance == Alliance.BLUE) {
            T = new Matrix(new double[][] {
                    {-4.80315, -9.05512, 5.59055}
            }).transpose();

        }

        if (alliance == Alliance.RED) {
            T = new Matrix(new double[][] {
                    {4.80315, -9.05512, 5.59055} // TODO: invert the x position
            }).transpose();
        }
    }

    private Point imageToWorld(double u, double v) {
        double fx = K.get(0, 0);
        double fy = K.get(1, 1);
        double cx = K.get(0, 2);
        double cy = K.get(1, 2);

        // camera ray
        Matrix dc = new Matrix(new double[][] {
                {(u-cx)/fx, (v-cy)/fy, 1}
        }).transpose();

        // rotate ray into world frame
        Matrix dw = RCombined.transpose().multiply(dc);

        Matrix negativeOne = new Matrix(new double[][] {
                {-1, 0, 0},
                {0, -1, 0},
                {0, 0, -1}
        });

        // camera pose in world coordinates
        Matrix C = RCombined.transpose().multiply(negativeOne).multiply(T);

        // solve for lambda
        double lambda = -C.get(2, 0) / dw.get(2, 0);

        // so i can multiply
        Matrix lambdaMatrix = new Matrix(new double[][] {
                {lambda, 0, 0},
                {0, lambda, 0},
                {0, 0, lambda}
        });
        // C + lambda * dw
        Matrix pt = C.add(lambdaMatrix.multiply(dw));

        return new Point(pt.get(0, 0), pt.get(1, 0));
    }

    @Override
    public void update() {

        List<ArtifactProcessor.Blob> blobs = colorLocator.getBlobs();
        if (blobs == null) {
            return;
        }

        ArtifactProcessor.Util.filterByCriteria(
                ArtifactProcessor.BlobCriteria.BY_CONTOUR_AREA,
                300, 20000, blobs);  // filter out very small blobs.

        if (blobs.isEmpty()) { return; }

        // new idea method: find world x and y, loop through a range, range with most total area wins
        // may want to subtract a bit, because balls usually have a bit of downward velocity

        List<Double> distances = new ArrayList<>();
        List<Double> areas = new ArrayList<>(); // calculating these first to save on computations
        for(ArtifactProcessor.Blob b : blobs)
        {
            RotatedRect boxFit = b.getBoxFit();
            // filtering out any blobs that are too high, as they might be a person's clothes. this works with opencv coord system.
            // idk if this is good anymore tho now that we detect from other spots
            if (boxFit != null) {
                // making sure it's not in the center
                if (boxFit.center.y + boxFit.size.height / 2 > 254.488) {
                    // just x dist
                    // for the ray algorithm, we want the BOTTOM of the ball. therefore we add height/2.
                    distances.add(imageToWorld(boxFit.center.x, boxFit.center.y + boxFit.size.height / 2).x);
                    // TODO: THIS IS IMPORTANT: IF THE Y COORDINATE IS TOO SMALL, THEN CAP THE MAX AREA (or just ignore)
                    // Importantly, object size is roughly inversely proportional from distance from camera
                    // however, because this is size and i measure area, it's about d^2
                    Point pt = imageToWorld(boxFit.center.x, boxFit.center.y);
                    // dealing with very close distances
                    double distance = Math.hypot(pt.x, pt.y) < 8 ? 8 : Math.hypot(pt.x, pt.y);
                    double proportionalArea = b.getContourArea() * Math.pow(distance, 2);
                    areas.add(proportionalArea);
                }
            }
        }

        double maxAreaLoc = -24;
        double maxArea = 0;

        for (int i = -240; i < 240; i++) {
            double area = calculateArea(distances, areas, i/10d-5, i/10d+5);
            if (area > maxArea) {
                maxArea = area;
                maxAreaLoc = i/10d;
            }
        }
        // making sure it doesn't aim too low!
        // TODO: set conditions in the auto to clamp it to x=8. not doing it here as idk x pos
        if (maxAreaLoc < -24) { maxAreaLoc = -24; }

        bestX = maxAreaLoc;
    }

    @Override
    public void start() {

    }

    public double getLargestClusterX() {
        return bestX;
    }
    private double calculateArea(List<Double> dists, List<Double> areas, double lower, double upper) {
        double totalArea = 0;
        for (int i = 0; i < dists.size(); i++) {
            double x = dists.get(i);
            if (x >= lower && x <= upper) {
                totalArea += areas.get(i);
            }
        }
        return totalArea;
    }
}
