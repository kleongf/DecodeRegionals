package org.firstinspires.ftc.teamcode.util.decodeutil;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.decode2026.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.decode2026.constants.RobotConstants;

import java.util.ArrayList;

// handles: closest point in zone AND if u are in zone or not.
public class ZoneUtil {
    public enum Zone {
        CLOSE,
        FAR
    }
    private static boolean lineLine(
            double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
        double uA =
                ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3))
                        / ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1));
        double uB =
                ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3))
                        / ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1));
        return uA >= 0 && uA <= 1 && uB >= 0 && uB <= 1;
    }

    private static boolean pointRect(double x, double y, double cx, double cy, double w, double h) {
        return Math.abs(x - cx) <= w / 2 && Math.abs(y - cy) <= h / 2;
    }

    private static boolean lineRect(
            double x1, double y1, double x2, double y2, double cx, double cy, double w, double h) {
        if (pointRect(x1, y1, cx, cy, w, h) || pointRect(x2, y2, cx, cy, w, h)) {
            return true;
        }
        double xMin = cx - w / 2, xMax = cx + w / 2;
        double yMin = cy - h / 2, yMax = cy + h / 2;
        boolean topBottom =
                lineLine(x1, y1, x2, y2, xMin, yMin, xMax, yMin)
                        || lineLine(x1, y1, x2, y2, xMin, yMax, xMax, yMax);
        boolean leftRight =
                lineLine(x1, y1, x2, y2, xMin, yMin, xMin, yMax)
                        || lineLine(x1, y1, x2, y2, xMax, yMin, xMax, yMax);
        return topBottom || leftRight;
    }
    private static boolean robotIntersectsLine(Pose pose, double x1, double y1, double x2, double y2) {
        double cosH = Math.cos(pose.getHeading());
        double sinH = Math.sin(pose.getHeading());

        double topRightX = FieldConstants.ROBOT_EFFECTIVE_LENGTH / 2 * cosH - FieldConstants.ROBOT_WIDTH * sinH + pose.getX();
        double topRightY = FieldConstants.ROBOT_EFFECTIVE_LENGTH / 2 * sinH + FieldConstants.ROBOT_WIDTH * cosH + pose.getY();

        double topLeftX = -FieldConstants.ROBOT_EFFECTIVE_LENGTH / 2 * cosH - FieldConstants.ROBOT_WIDTH * sinH + pose.getX();
        double topLeftY = -FieldConstants.ROBOT_EFFECTIVE_LENGTH / 2 * sinH + FieldConstants.ROBOT_WIDTH * cosH + pose.getY();

        double bottomRightX = FieldConstants.ROBOT_EFFECTIVE_LENGTH / 2 * cosH - (-FieldConstants.ROBOT_WIDTH) * sinH + pose.getX();
        double bottomRightY = FieldConstants.ROBOT_EFFECTIVE_LENGTH / 2 * sinH + (-FieldConstants.ROBOT_WIDTH) * cosH + pose.getY();

        double bottomLeftX = -FieldConstants.ROBOT_EFFECTIVE_LENGTH / 2 * cosH - (-FieldConstants.ROBOT_WIDTH) * sinH + pose.getX();
        double bottomLeftY = -FieldConstants.ROBOT_EFFECTIVE_LENGTH / 2 * sinH + (-FieldConstants.ROBOT_WIDTH) * cosH + pose.getY();

        // connecting lines:
        // topRight to topLeft
        // topLeft to bottomLeft
        // bottomLeft to bottomRight
        // bottomRight to topLeft

        return (lineLine(x1, y1, x2, y2, topRightX, topRightY, topLeftX, topLeftY) ||
                (lineLine(x1, y1, x2, y2, topLeftX, topLeftY, bottomLeftX, bottomLeftY)) ||
                (lineLine(x1, y1, x2, y2, bottomLeftX, bottomLeftY, bottomRightX, bottomRightY)) ||
                (lineLine(x1, y1, x2, y2, bottomRightX, bottomRightY, topRightX, topRightY)));
    }

    private static boolean intersectsCloseZoneLeftLine(Pose pose) {
        return robotIntersectsLine(pose, 0, FieldConstants.FIELD_WIDTH, FieldConstants.FIELD_WIDTH / 2, FieldConstants.FIELD_WIDTH / 2);
    }

    private static boolean intersectsCloseZoneRightLine(Pose pose) {
        return robotIntersectsLine(pose, FieldConstants.FIELD_WIDTH / 2, FieldConstants.FIELD_WIDTH / 2, FieldConstants.FIELD_WIDTH, FieldConstants.FIELD_WIDTH);
    }

    private static boolean intersectsFarZoneLeftLine(Pose pose) {
        return robotIntersectsLine(pose, FieldConstants.FIELD_WIDTH / 3, 0, FieldConstants.FIELD_WIDTH / 2, FieldConstants.FIELD_WIDTH / 6);
    }

    private static boolean intersectsFarZoneRightLine(Pose pose) {
        return robotIntersectsLine(pose, FieldConstants.FIELD_WIDTH / 2, FieldConstants.FIELD_WIDTH / 6, FieldConstants.FIELD_WIDTH * (2/3d), 0);
    }

    private static boolean inBetween(double value, double min, double max) {
        return value < max && value > min;
    }

    public static boolean inCloseZone(Pose pose) {
        boolean inTopLeftZone = inBetween(pose.getX(), 0, FieldConstants.FIELD_WIDTH / 2) &&
                inBetween(pose.getY(), FieldConstants.FIELD_WIDTH / 2, FieldConstants.FIELD_WIDTH) &&
                pose.getY() > FieldConstants.FIELD_WIDTH - pose.getX();

        boolean intersectsTopLeftLine = intersectsCloseZoneLeftLine(pose);

        boolean inTopRightZone = inBetween(pose.getX(), FieldConstants.FIELD_WIDTH / 2, FieldConstants.FIELD_WIDTH) &&
                inBetween(pose.getY(), FieldConstants.FIELD_WIDTH / 2, FieldConstants.FIELD_WIDTH) &&
                pose.getY() > pose.getX();
        boolean intersectsTopRightLine = intersectsCloseZoneRightLine(pose);

        return inTopLeftZone || intersectsTopLeftLine || inTopRightZone || intersectsTopRightLine;
    }

    public static boolean inFarZone(Pose pose) {
        boolean inBottomLeftZone = inBetween(pose.getX(), FieldConstants.FIELD_WIDTH / 3, FieldConstants.FIELD_WIDTH / 2) &&
                inBetween(pose.getY(), 0, FieldConstants.FIELD_WIDTH / 6) &&
                pose.getY() < pose.getX() - FieldConstants.FIELD_WIDTH / 3;

        boolean intersectsBottomLeftLine = intersectsFarZoneLeftLine(pose);

        boolean inBottomRightZone = inBetween(pose.getX(), FieldConstants.FIELD_WIDTH / 2, FieldConstants.FIELD_WIDTH * (2/3d)) &&
                inBetween(pose.getY(), 0, FieldConstants.FIELD_WIDTH / 6) &&
                pose.getY() < pose.getX() + FieldConstants.FIELD_WIDTH * (2/3d);
        boolean intersectsBottomRightLine = intersectsFarZoneRightLine(pose);

        return inBottomLeftZone || intersectsBottomLeftLine || inBottomRightZone || intersectsBottomRightLine;
    }
}
