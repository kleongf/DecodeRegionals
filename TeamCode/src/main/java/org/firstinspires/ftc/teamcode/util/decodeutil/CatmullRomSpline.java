package org.firstinspires.ftc.teamcode.util.decodeutil;
import com.pedropathing.geometry.CustomCurve;
import com.pedropathing.geometry.FuturePose;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathConstraints;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * A piecewise cubic Hermite spline that passes through every control point,
 * with tangents estimated automatically via a non-uniform Catmull-Rom
 * formula. No heading input is required at the waypoints -- tangents are
 * derived purely from neighboring point positions.
 *
 * Works for any number of control points >= 2:
 *   - 2 points  -> degenerates to a straight line (tangent = chord direction
 *                  at both ends, matching LinearSpline's behavior)
 *   - 3+ points -> smooth curve through all points, tangent-continuous (G1),
 *                  using non-uniform tangent weighting so unevenly spaced
 *                  waypoints don't produce loops/overshoot.
 *
 * Mirrors LinearSpline's lazy-init pattern: CustomCurve.initialize() calls
 * approximateLength() (which calls getPose(0)) BEFORE the initialization()
 * hook runs, so setup can't safely live there. Instead, like LinearSpline,
 * setup happens on first call to getPose/getDerivative/getSecondDerivative.
 */
public class CatmullRomSpline extends CustomCurve {

    private List<Pose> points;
    private List<Pose> tangents; // each tangent stored as a Pose(dx, dy) for convenience
    private int numSegments;

    public CatmullRomSpline(Pose... controlPoints) {
        super(controlPoints);
    }

    public CatmullRomSpline(FuturePose... controlPoints) {
        super(controlPoints);
    }

    public CatmullRomSpline(List<Pose> controlPoints, PathConstraints constraints) {
        super(controlPoints, constraints);
    }

    public CatmullRomSpline(PathConstraints constraints, FuturePose... controlPoints) {
        super(constraints, controlPoints);
    }

    // Convenience overload -- not a CustomCurve constructor, just routes into
    // the varargs constructor above so callers can pass a List<Pose> directly.
    public CatmullRomSpline(List<Pose> controlPoints) {
        this(controlPoints.toArray(new Pose[0]));
    }

    @Override
    public String pathType() {
        return "Catmull-Rom Spline";
    }

    @Override
    public CatmullRomSpline getReversed() {
        List<Pose> reversed = new ArrayList<>(getControlPoints());
        Collections.reverse(reversed);
        CatmullRomSpline curve = new CatmullRomSpline(reversed, this.getPathConstraints());
        curve.initialize();
        return curve;
    }

    private void init() {
        points = new ArrayList<>(getControlPoints());
        int n = points.size();
        if (n < 2) {
            throw new IllegalArgumentException("CatmullRomSpline needs at least 2 control points.");
        }
        numSegments = n - 1;
        tangents = computeTangents(points);
    }

    /**
     * Estimates a tangent at every control point. Endpoints use the direction
     * to their single neighbor; interior points use a distance-weighted
     * (non-uniform) Catmull-Rom average of both neighbors, which avoids the
     * looping/overshoot that plain uniform Catmull-Rom can produce when
     * waypoints are spaced unevenly.
     */
    private List<Pose> computeTangents(List<Pose> pts) {
        int n = pts.size();
        List<Pose> result = new ArrayList<>(n);

        if (n == 2) {
            Pose diff = pts.get(1).minus(pts.get(0));
            result.add(diff);
            result.add(diff);
            return result;
        }

        for (int i = 0; i < n; i++) {
            if (i == 0) {
                result.add(pts.get(1).minus(pts.get(0)));
            } else if (i == n - 1) {
                result.add(pts.get(n - 1).minus(pts.get(n - 2)));
            } else {
                Pose prev = pts.get(i - 1);
                Pose cur = pts.get(i);
                Pose next = pts.get(i + 1);

                double dPrev = prev.distanceFrom(cur);
                double dNext = cur.distanceFrom(next);
                if (dPrev < 1e-9) dPrev = 1e-9;
                if (dNext < 1e-9) dNext = 1e-9;

                double tx = (dNext * (cur.getX() - prev.getX()) / dPrev
                        + dPrev * (next.getX() - cur.getX()) / dNext) / (dPrev + dNext);
                double ty = (dNext * (cur.getY() - prev.getY()) / dPrev
                        + dPrev * (next.getY() - cur.getY()) / dNext) / (dPrev + dNext);

                result.add(new Pose(tx, ty));
            }
        }
        return result;
    }

    /** Maps global t in [0,1] to {segmentIndex, localU in [0,1]}. */
    private double[] resolveSegment(double t) {
        double clampedT = Math.max(0.0, Math.min(1.0, t));
        double scaled = clampedT * numSegments;
        int segIndex = (int) Math.floor(scaled);
        double u;
        if (segIndex >= numSegments) {
            segIndex = numSegments - 1;
            u = 1.0;
        } else {
            u = scaled - segIndex;
        }
        return new double[]{segIndex, u};
    }

    @Override
    public Pose getPose(double t) {
        if (points == null) init();

        double[] seg = resolveSegment(t);
        int segIndex = (int) seg[0];
        double u = seg[1];

        Pose p0 = points.get(segIndex);
        Pose p1 = points.get(segIndex + 1);
        Pose m0 = tangents.get(segIndex);
        Pose m1 = tangents.get(segIndex + 1);

        double u2 = u * u;
        double u3 = u2 * u;

        double h00 = 2 * u3 - 3 * u2 + 1;
        double h10 = u3 - 2 * u2 + u;
        double h01 = -2 * u3 + 3 * u2;
        double h11 = u3 - u2;

        double x = h00 * p0.getX() + h10 * m0.getX() + h01 * p1.getX() + h11 * m1.getX();
        double y = h00 * p0.getY() + h10 * m0.getY() + h01 * p1.getY() + h11 * m1.getY();

        return new Pose(x, y);
    }

    @Override
    public Vector getDerivative(double t) {
        if (points == null) init();

        double[] seg = resolveSegment(t);
        int segIndex = (int) seg[0];
        double u = seg[1];

        Pose p0 = points.get(segIndex);
        Pose p1 = points.get(segIndex + 1);
        Pose m0 = tangents.get(segIndex);
        Pose m1 = tangents.get(segIndex + 1);

        double u2 = u * u;

        double dh00 = 6 * u2 - 6 * u;
        double dh10 = 3 * u2 - 4 * u + 1;
        double dh01 = -6 * u2 + 6 * u;
        double dh11 = 3 * u2 - 2 * u;

        // chain rule: u = t * numSegments, so du/dt = numSegments
        double dx = (dh00 * p0.getX() + dh10 * m0.getX() + dh01 * p1.getX() + dh11 * m1.getX()) * numSegments;
        double dy = (dh00 * p0.getY() + dh10 * m0.getY() + dh01 * p1.getY() + dh11 * m1.getY()) * numSegments;

        return new Vector(dx, dy);
    }

    @Override
    public Vector getSecondDerivative(double t) {
        if (points == null) init();

        double[] seg = resolveSegment(t);
        int segIndex = (int) seg[0];
        double u = seg[1];

        Pose p0 = points.get(segIndex);
        Pose p1 = points.get(segIndex + 1);
        Pose m0 = tangents.get(segIndex);
        Pose m1 = tangents.get(segIndex + 1);

        double ddh00 = 12 * u - 6;
        double ddh10 = 6 * u - 4;
        double ddh01 = -12 * u + 6;
        double ddh11 = 6 * u - 2;

        // chain rule: second derivative scales by (du/dt)^2 = numSegments^2
        double scale = (double) numSegments * numSegments;
        double ddx = (ddh00 * p0.getX() + ddh10 * m0.getX() + ddh01 * p1.getX() + ddh11 * m1.getX()) * scale;
        double ddy = (ddh00 * p0.getY() + ddh10 * m0.getY() + ddh01 * p1.getY() + ddh11 * m1.getY()) * scale;

        return new Vector(ddx, ddy);
    }
}
