package org.firstinspires.ftc.teamcode.utils.math;

import com.acmerobotics.roadrunner.Vector2d;

import java.util.ArrayList;

public class GeometryUtils {

    public static Vector2d rotateVector(Vector2d vector, double angle) {
        double cos = Math.cos(angle);
        double sin = Math.sin(angle);
        // cos, -sin
        // sin,  cos
        return new Vector2d(vector.x * cos - vector.y * sin, vector.x * sin + vector.y * cos);
    }

    // returns if a circle is fully inside a polygon
    public static boolean isCircleInsidePolygon(ArrayList<Vector2d> polygon, Vector2d circleCenter, double circleRadius) {
        int n = polygon.size();
        if (n < 3) return false;

        boolean inside = false;
        for (int i = 0, j = n - 1; i < n; j = i++) {
            Vector2d pi = polygon.get(i);
            Vector2d pj = polygon.get(j);

            boolean intersect = ((pi.y > circleCenter.y) != (pj.y > circleCenter.y)) &&
                    (circleCenter.x < (pj.x - pi.x) * (circleCenter.y - pi.y) / (pj.y - pi.y) + pi.x);

            if (intersect) inside = !inside;
        }
        if (!inside) return false;

        for (int i = 0; i < n; i++) {
            Vector2d a = polygon.get(i);
            Vector2d b = polygon.get((i + 1) % n);

            if (distancePointToSegment(circleCenter, a, b) < circleRadius) {
                return false;
            }
        }

        return true;
    }
    public static double distancePointToSegment(Vector2d p, Vector2d a, Vector2d b) {
        Vector2d ab = b.minus(a);
        Vector2d ap = p.minus(a);

        double t = ap.dot(ab) / ab.dot(ab);
        t = Math.max(0, Math.min(1, t));

        Vector2d closest = a.plus(ab.times(t));
        return Math.hypot(p.x - closest.x, p.y - closest.y);
    }



    // ensures the point is inside the field
    // if it isn't, it projects it onto the field by finding the line between the origin and the vector
    // then finding the intersection of that line and the field wall + some buffer distance
    // that intersection is the projected point
    public static Vector2d projectOntoField(Vector2d origin, Vector2d vecToProject, double extraBufferDistance) {
        final double fieldMin = -72. + extraBufferDistance;
        final double fieldMax = 72. - extraBufferDistance;

        double x0 = origin.x;
        double y0 = origin.y;

        double dx = vecToProject.x - x0;
        double dy = vecToProject.y - y0;

        // If already inside field, return as-is
        if (vecToProject.x >= fieldMin && vecToProject.x <= fieldMax &&
                vecToProject.y >= fieldMin && vecToProject.y <= fieldMax) {
            return vecToProject;
        }

        double tMin = Double.POSITIVE_INFINITY;

        // Intersect with vertical walls
        if (dx != 0) {
            double tLeft  = (fieldMin - x0) / dx;
            double tRight = (fieldMax - x0) / dx;

            if (tLeft > 0)  tMin = Math.min(tMin, tLeft);
            if (tRight > 0) tMin = Math.min(tMin, tRight);
        }

        // Intersect with horizontal walls
        if (dy != 0) {
            double tBottom = (fieldMin - y0) / dy;
            double tTop    = (fieldMax - y0) / dy;

            if (tBottom > 0) tMin = Math.min(tMin, tBottom);
            if (tTop > 0)    tMin = Math.min(tMin, tTop);
        }

        // Safety check (should not happen if origin is inside field)
        if (!Double.isFinite(tMin)) {
            return origin;
        }

        // Intersection point
        double ix = x0 + dx * tMin;
        double iy = y0 + dy * tMin;

        // Clamp for numerical safety
        ix = Math.max(fieldMin, Math.min(fieldMax, ix));
        iy = Math.max(fieldMin, Math.min(fieldMax, iy));

        return new Vector2d(ix, iy);
    }

}

