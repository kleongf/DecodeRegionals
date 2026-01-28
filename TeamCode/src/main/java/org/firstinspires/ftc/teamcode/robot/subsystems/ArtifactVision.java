package org.firstinspires.ftc.teamcode.robot.subsystems;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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


public class ArtifactVision extends Subsystem {
    private VisionPortal portal;
    private ArtifactProcessor colorLocator;
    private double bestX = -19;
    private Mat H;
    private Matrix K = new Matrix(new double[][] {
            {214.1037056, 0, 313},
            {0, 212.72822576, 254.488},
            {0, 0, 1}
    });
    /*
    [[214.1037056    0.         313.19451923]
 [  0.         212.72822576 254.48870311]
 [  0.           0.           1.        ]]
     */
    private Matrix R = new Matrix(new double[][] {
        {1, 0, 0},
        {0, -1, 0},
        {0, 0, 1}
    });

    private Matrix C = new Matrix(new double[][] {
        {-4, 6, 0} // for blue it's to the left for red idk
    }).transpose();

    private double planeY = 1.0;

    public ArtifactVision(HardwareMap hardwareMap, Alliance alliance) {
        colorLocator = new ArtifactProcessor.Builder()
                .build();

        portal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setCameraResolution(new Size(640, 480))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        // portal.setProcessorEnabled(colorLocator, true); ok cool beans

        double[][] homographyBlue = {
                { -4.98885709e-02, 2.91967072e-02, 9.88258249e+00 },
                { 3.80623143e-03,  5.18877858e-02, -2.55240985e+01 },
                { -6.01290894e-05, -4.77537046e-03,  1.00000000e+00 }
        };

        double[][] homographyRed = {
                { 3.56423511e-02, -3.54066283e-02, -3.46991136e+00},
                { 3.79738251e-03,  2.79198165e-02, -1.59729952e+01},
                { 9.16521855e-05, -4.75490455e-03,  1.00000000e+00}
        };
        if (alliance == Alliance.BLUE) {
             C = new Matrix(new double[][] {
                    {-4, 6, 0} // for blue it's to the left for red idk
            }).transpose();
        }

        if (alliance == Alliance.RED) {
             C = new Matrix(new double[][] {
                    {4, 6, 0} // for blue it's to the left for red idk
            }).transpose();
        }

        if (alliance == Alliance.BLUE) {
            this.H = new Mat(3, 3, CvType.CV_64F);
            for (int r = 0; r < 3; r++) {
                for (int c = 0; c < 3; c++) {
                    H.put(r, c, homographyBlue[r][c]);
                }
            }
        } else {
            this.H = new Mat(3, 3, CvType.CV_64F);
            for (int r = 0; r < 3; r++) {
                for (int c = 0; c < 3; c++) {
                    H.put(r, c, homographyRed[r][c]);
                }
            }
        }

    }

    private Point imageToWorld(double x, double y) {
        Matrix pt = new Matrix(new double[][] {
                {x, y, 1.0}
        }).transpose();

        Matrix ray_cam = K.inverse().multiply(pt).normalize();
        Matrix dir = R.multiply(ray_cam);

        double t = -1 * C.get(1, 0) / dir.get(1, 0);
        // i was too lazy to add scalar multiplication, so i made it a matrix. ez way to multiply by scalar
        Matrix t_scaled =  new Matrix(new double[][] {
                {t, 0, 0},
                {0, t, 0},
                {0, 0, t}
        });
        Matrix X = C.add(t_scaled.multiply(dir));
        return new Point(X.get(0, 0), X.get(2, 0));
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
                if (boxFit.center.y > 200) {
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
