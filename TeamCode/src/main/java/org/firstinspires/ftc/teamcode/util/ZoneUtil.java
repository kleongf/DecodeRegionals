package org.firstinspires.ftc.teamcode.util;

public class ZoneUtil {
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
}
