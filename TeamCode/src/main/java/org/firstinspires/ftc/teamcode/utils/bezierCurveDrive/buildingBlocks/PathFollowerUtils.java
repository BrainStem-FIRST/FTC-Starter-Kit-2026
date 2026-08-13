package org.firstinspires.ftc.teamcode.utils.bezierCurveDrive.buildingBlocks;

import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.utils.math.MathUtils;
import org.firstinspires.ftc.teamcode.utils.pilotAutoBuilder.helperClasses.PilotGeometry;

import java.util.List;

/**
 * Static utility class containing all calculations needed to follow a {@link BezierCurve}.
 */
public final class PathFollowerUtils {
    private PathFollowerUtils() {}

    public static double findClosestT(BezierCurve curve, Vector2d robotPos, int coarseSamples, int maxIterations, double tolerance) {
        double bestT = 0.0;
        double bestDist = Double.MAX_VALUE;

        for (int i = 0; i <= coarseSamples; i++) {
            double t = (double) i / coarseSamples;
            double dist = MathUtils.vecDist(curve.getPoint(t), robotPos);
            if (dist < bestDist) {
                bestDist = dist;
                bestT = t;
            }
        }

        double step = 1.0 / coarseSamples;
        double lo = Math.max(0.0, bestT - step);
        double hi = Math.min(1.0, bestT + step);

        double prevMinDist = bestDist;

        for (int i = 0; i < maxIterations; i++) {
            double span = hi - lo;
            double m1 = lo + span / 3.0;
            double m2 = hi - span / 3.0;

            double d1 = MathUtils.vecDist(curve.getPoint(m1), robotPos);
            double d2 = MathUtils.vecDist(curve.getPoint(m2), robotPos);

            if (d1 < d2) {
                hi = m2;
            } else {
                lo = m1;
            }

            double newMinDist = Math.min(d1, d2);
            if (Math.abs(prevMinDist - newMinDist) < tolerance) break;
            prevMinDist = newMinDist;
        }

        return (lo + hi) / 2.0;
    }

    public static Vector2d getLookaheadPoint(BezierCurve curve, double closestT, double lookaheadT) {
        return curve.getPoint(Math.min(1.0, closestT + lookaheadT));
    }

    public static double estimateRemainingLength(BezierCurve curve, double fromT, int samples) {
        double length = 0.0;
        Vector2d prev = curve.getPoint(fromT);

        for (int i = 1; i <= samples; i++) {
            double t = fromT + (1.0 - fromT) * i / samples;
            Vector2d curr = curve.getPoint(t);
            length += MathUtils.vecDist(curr, prev);
            prev = curr;
        }

        return length;
    }

    public static Vector2d calculateDriveVector(
            BezierCurve curve,
            Vector2d robotPos,
            Vector2d lookaheadPoint,
            double closestT,
            double remainingLength,
            double speedKP,
            double speedKF,
            double correctiveStrength) {

        double speed = remainingLength * speedKP + speedKF;

        Vector2d toTarget = lookaheadPoint.minus(robotPos);
        double toTargetNorm = toTarget.norm();

        if (toTargetNorm < 1e-6) {
            return new Vector2d(0, 0);
        }

        Vector2d driveVec = toTarget.times(speed / toTargetNorm);

        Vector2d tangent = curve.getDerivative(closestT);
        double tangentNorm = tangent.norm();

        if (tangentNorm < 1e-6) {
            return driveVec;
        }

        Vector2d tangentUnit = tangent.times(1.0 / tangentNorm);
        Vector2d perpUnit = new Vector2d(-tangentUnit.y, tangentUnit.x);

        double parallelMag = dot(driveVec, tangentUnit);
        double perpMag = dot(driveVec, perpUnit);

        Vector2d parallelComponent = tangentUnit.times(parallelMag);
        Vector2d perpComponent = perpUnit.times(perpMag * correctiveStrength);

        return parallelComponent.plus(perpComponent);
    }

    public static double getTargetRotation(List<RotationPoint> rotationPoints, double t, double entryHeadingRad) {
        if (rotationPoints == null || rotationPoints.isEmpty()) {
            return entryHeadingRad;
        }

        if (rotationPoints.size() == 1) {
            RotationPoint targetPoint = rotationPoints.get(0);

            if (t >= targetPoint.getT()) {
                return targetPoint.getHeadingRad();
            }

            if (targetPoint.getT() < 1e-6) {
                return targetPoint.getHeadingRad();
            }

            double localPct = t / targetPoint.getT();
            return PilotGeometry.lerpHeading(entryHeadingRad, targetPoint.getHeadingRad(), localPct);
        }

        if (t <= rotationPoints.get(0).getT()) {
            RotationPoint first = rotationPoints.get(0);
            if (first.getT() < 1e-6) {
                return first.getHeadingRad();
            }
            return PilotGeometry.lerpHeading(entryHeadingRad, first.getHeadingRad(), t / first.getT());
        }

        if (t >= rotationPoints.get(rotationPoints.size() - 1).getT()) {
            return rotationPoints.get(rotationPoints.size() - 1).getHeadingRad();
        }

        for (int i = 0; i < rotationPoints.size() - 1; i++) {
            RotationPoint p1 = rotationPoints.get(i);
            RotationPoint p2 = rotationPoints.get(i + 1);

            if (t >= p1.getT() && t <= p2.getT()) {
                double segmentSpan = p2.getT() - p1.getT();

                if (segmentSpan < 1e-6) {
                    return p2.getHeadingRad();
                }

                double localPct = (t - p1.getT()) / segmentSpan;
                return PilotGeometry.lerpHeading(p1.getHeadingRad(), p2.getHeadingRad(), localPct);
            }
        }

        return rotationPoints.get(rotationPoints.size() - 1).getHeadingRad();
    }

    public static double getRotationPower(double currentHeadingRad, double targetHeadingRad, double kP, double kF) {
        double errorRadians = MathUtils.angleNormDeltaRad(targetHeadingRad - currentHeadingRad);
        double power = errorRadians * kP;
        if (Math.abs(power) > 1e-6) {
            return power + (Math.signum(power) * kF);
        }
        return power;
    }

    public static boolean isPathFinished(double closestT, Vector2d robotPos, BezierCurve curve, double distanceThreshold) {
        return closestT >= 0.99 && MathUtils.vecDist(robotPos, curve.getEnd()) < distanceThreshold;
    }

    public static Vector2d getCentripetalCompensation(BezierCurve curve, double t, double robotSpeed, double centripetalGain, double epsilon) {
        double tA = Math.max(0.0, t - epsilon);
        double tC = Math.min(1.0, t + epsilon);

        Vector2d A = curve.getPoint(tA);
        Vector2d B = curve.getPoint(t);
        Vector2d C = curve.getPoint(tC);

        double ab = MathUtils.vecDist(A, B);
        double bc = MathUtils.vecDist(B, C);
        double ca = MathUtils.vecDist(C, A);

        double cross = (B.x - A.x) * (C.y - A.y) - (B.y - A.y) * (C.x - A.x);
        double twoArea = Math.abs(cross);

        if (twoArea < 1e-3) return new Vector2d(0, 0);

        double radius = (ab * bc * ca) / (2.0 * twoArea);
        double centripetalAccel = (robotSpeed * robotSpeed) / radius;

        double ax = A.x, ay = A.y;
        double bx = B.x, by = B.y;
        double cx = C.x, cy = C.y;

        double D = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));
        double ux = ((ax * ax + ay * ay) * (by - cy)
                + (bx * bx + by * by) * (cy - ay)
                + (cx * cx + cy * cy) * (ay - by)) / D;
        double uy = ((ax * ax + ay * ay) * (cx - bx)
                + (bx * bx + by * by) * (ax - cx)
                + (cx * cx + cy * cy) * (bx - ax)) / D;

        Vector2d toCenter = new Vector2d(ux - bx, uy - by);
        double toCenterNorm = toCenter.norm();

        if (toCenterNorm < 1e-9) return new Vector2d(0, 0);

        Vector2d inwardUnit = toCenter.times(1.0 / toCenterNorm);
        return inwardUnit.times(centripetalAccel * centripetalGain);
    }

    public record CentripetalInfo(Vector2d circleCenter, double circleRadius, Vector2d compensation) {}

    public static CentripetalInfo getCentripetalCompensationDebug(
            BezierCurve curve, double t, double robotSpeed, double centripetalGain, double epsilon, double dtSeconds) {
        double tA = Math.max(0.0, t - epsilon);
        double tC = Math.min(1.0, t + epsilon);

        Vector2d A = curve.getPoint(tA);
        Vector2d B = curve.getPoint(t);
        Vector2d C = curve.getPoint(tC);

        double ab = MathUtils.vecDist(A, B);
        double bc = MathUtils.vecDist(B, C);
        double ca = MathUtils.vecDist(C, A);

        double cross = (B.x - A.x) * (C.y - A.y) - (B.y - A.y) * (C.x - A.x);
        double twoArea = Math.abs(cross);

        if (twoArea < 1e-3) {
            return new CentripetalInfo(null, 0, new Vector2d(0, 0));
        }

        double radius = (ab * bc * ca) / (2.0 * twoArea);
        double centripetalAccel = (robotSpeed * robotSpeed) / radius;
        double deltaVelocity = centripetalAccel * dtSeconds;

        double ax = A.x, ay = A.y;
        double bx = B.x, by = B.y;
        double cx = C.x, cy = C.y;

        double D = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));
        double ux = ((ax * ax + ay * ay) * (by - cy)
                + (bx * bx + by * by) * (cy - ay)
                + (cx * cx + cy * cy) * (ay - by)) / D;
        double uy = ((ax * ax + ay * ay) * (cx - bx)
                + (bx * bx + by * by) * (ax - cx)
                + (cx * cx + cy * cy) * (bx - ax)) / D;

        Vector2d toCenter = new Vector2d(ux - bx, uy - by);
        double toCenterNorm = toCenter.norm();

        if (toCenterNorm < 1e-9) {
            return new CentripetalInfo(null, 0, new Vector2d(0, 0));
        }

        Vector2d inwardUnit = toCenter.times(1.0 / toCenterNorm);
        Vector2d compensation = inwardUnit.times(deltaVelocity * centripetalGain);
        return new CentripetalInfo(B.plus(toCenter), radius, compensation);
    }

    private static double dot(Vector2d a, Vector2d b) {
        return a.x * b.x + a.y * b.y;
    }
}