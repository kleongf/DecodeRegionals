package org.firstinspires.ftc.teamcode.util.decodeutil;

import com.pedropathing.geometry.Pose;
import java.util.ArrayList;

// handles: closest point in zone AND if u are in zone or not.
public class ZoneUtil {
    private int robotRadius;
    private ArrayList<Pose> closePoses = new ArrayList<>();
    private ArrayList<Pose> farPoses = new ArrayList<>();
    public ZoneUtil(int robotRadius) {
        this.robotRadius = robotRadius;

        // CLOSE POSES
        // starting at 24 inches so that we don't include the edges of the blue goal
        for (int i = 24; i < 72; i++) {
            closePoses.add(new Pose(i, (144-i)-robotRadius, Math.toRadians(0)));
        }
        // ending at 120 inches so that we don't include the edges of the red goal
        for (int i = 72; i < 120; i++) {
            closePoses.add(new Pose(i, i-robotRadius, Math.toRadians(0)));
        }

        // FAR POSES
        // starting at 48-robotRadius to include more areas. however robot cannot be all up against the wall
        // so if the y < robotRadius then make y = robotRadius
        for (int i = 48-robotRadius; i < 72; i++) {
            if (-48 + robotRadius + i < robotRadius) {
                farPoses.add(new Pose(i, robotRadius, Math.toRadians(0)));
            } else {
                farPoses.add(new Pose(i, -48+robotRadius + i, Math.toRadians(0)));
            }
        }
        for (int i = 72; i < 96+robotRadius; i++) {
            if (96+robotRadius - i < robotRadius) {
                farPoses.add(new Pose(i, robotRadius, Math.toRadians(0)));
            } else {
                farPoses.add(new Pose(i, 96+robotRadius - i, Math.toRadians(0)));
            }
        }
    }

    public void setRobotRadius(int robotRadius) {
        this.robotRadius = robotRadius;
    }

    // think of this like a coordinate system quadrant thing
    public boolean inQ1(Pose pose) {
        return pose.getY() > (-robotRadius) + pose.getX() && pose.getX() > 72;
    }
    public boolean inQ2(Pose pose) {
        return pose.getY() > (144-robotRadius) - pose.getX() && pose.getX() <= 72;
    }

    public boolean inQ3(Pose pose) {
        return pose.getY() < (-48+robotRadius) + pose.getX() && pose.getX() <= 72;
    }

    public boolean inQ4(Pose pose) {
        return pose.getY() < (96+robotRadius) - pose.getX() && pose.getX() >= 72;
    }

    public boolean inZone(Pose pose, Zone zone) {
        if (zone == Zone.CLOSE) {
            return inQ1(pose) || inQ2(pose);
        }
        return inQ3(pose) || inQ4(pose);
    }

    public Pose closestPose(Pose pose, Zone zone) {
        if (zone == Zone.CLOSE) {
            return closestPose(pose, closePoses);
        }
        return closestPose(pose, farPoses);
    }

    private static Pose closestPose(Pose robotPose, ArrayList<Pose> poses) {
        ArrayList<Double> distances = new ArrayList<>();
        for (Pose p: poses) {
            distances.add(MathUtil.distance(robotPose, p));
        }

        int minIndex = 0;
        double minValue = distances.get(0);

        for (int i = 1; i < distances.size(); i++) {
            if (distances.get(i) < minValue) {
                minValue = distances.get(i);
                minIndex = i;
            }
        }

        return poses.get(minIndex);
    }



//    y > -radius + x
//    y > (144-radius) - x
//    y < 48+radius + x
//    y < 96 + radius -x
}
