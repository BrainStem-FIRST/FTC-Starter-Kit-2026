package org.firstinspires.ftc.teamcode.utils.bezierCurveDrive.buildingBlocks;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.Vector2d;

/**
 * Represents a cubic Bézier curve defined by a start point, two control/anchor points,
 * and an end point.
 */
public class BezierCurve {

    private final Vector2d start;
    private final Vector2d control1;
    private final Vector2d control2;
    private final Vector2d end;

    public BezierCurve(Vector2d start, Vector2d control1, Vector2d control2, Vector2d end) {
        this.start = start;
        this.control1 = control1;
        this.control2 = control2;
        this.end = end;
    }

    public BezierCurve(Vector2d start, Vector2d end) {
        this.start = start;
        this.end = end;
        Vector2d startToEnd = end.minus(start);
        this.control1 = start.plus(startToEnd.times(0.333));
        this.control2 = start.plus(startToEnd.times(0.667));
    }

    public Vector2d getPoint(double t) {
        t = clamp01(t);
        double u = 1.0 - t;
        double u2 = u * u;
        double u3 = u2 * u;
        double t2 = t * t;
        double t3 = t2 * t;

        return start.times(u3)
                .plus(control1.times(3.0 * u2 * t))
                .plus(control2.times(3.0 * u * t2))
                .plus(end.times(t3));
    }

    public Vector2d getDerivative(double t) {
        t = clamp01(t);
        double u = 1.0 - t;
        double u2 = u * u;
        double t2 = t * t;

        return control1.minus(start).times(3.0 * u2)
                .plus(control2.minus(control1).times(6.0 * u * t))
                .plus(end.minus(control2).times(3.0 * t2));
    }

    public Vector2d getStart() {
        return start;
    }

    public Vector2d getControl1() {
        return control1;
    }

    public Vector2d getControl2() {
        return control2;
    }

    public Vector2d getEnd() {
        return end;
    }

    private static double clamp01(double v) {
        return Math.max(0.0, Math.min(1.0, v));
    }

    public void draw(Canvas canvas, int numPoints) {
        double[] xPoints = new double[numPoints];
        double[] yPoints = new double[numPoints];
        for (int i = 0; i < numPoints; i++) {
            double t = i * 1.0 / (numPoints - 1);
            Vector2d point = getPoint(t);
            xPoints[i] = point.x;
            yPoints[i] = point.y;
        }
        canvas.setStrokeWidth(1);
        canvas.strokePolyline(xPoints, yPoints);
    }
}
