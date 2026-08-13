package org.firstinspires.ftc.teamcode.utils.bezierCurveDrive.tolerance;

import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.utils.pilotAutoBuilder.helperClasses.PilotGeometry;

import java.util.ArrayList;

public class CircleTolerance implements Tolerance {
    public static class DefaultParams {
        public double distTol = 3.0;
        public double headingTolDeg = 4.0;
        public int numToleranceCorners = 16;
    }
    public static DefaultParams defaultParams = new DefaultParams();
    private final double distTol;
    private final double headingTolRad;

    public CircleTolerance(double distTol, double headingTolDeg) {
        this.distTol = distTol;
        this.headingTolRad = PilotGeometry.fromDegrees(headingTolDeg);
    }

    public CircleTolerance() {
        this(defaultParams.distTol, defaultParams.headingTolDeg);
    }

    @Override
    public boolean inPositionTolerance(Vector2d positionError) {
        return positionError.dot(positionError) <= distTol * distTol;
    }

    @Override
    public boolean inHeadingTolerance(double headingErrorRad) {
        return Math.abs(headingErrorRad) < headingTolRad;
    }

    @Override
    public double getPositionDampening(Vector2d positionError) {
        return Math.min(1, positionError.dot(positionError) / (distTol * distTol));
    }

    @Override
    public double getHeadingDampening(double headingErrorRad) {
        return Math.min(1, Math.abs(headingErrorRad) / headingTolRad);
    }

    @Override
    public ArrayList<Vector2d> getToleranceCorners(Vector2d waypointPosition) {
        double r = distTol * 0.5;
        ArrayList<Vector2d> edges = new ArrayList<>();
        double angleChange = 2 * Math.PI / defaultParams.numToleranceCorners;
        for (int i = 0; i < defaultParams.numToleranceCorners; i++) {
            double angle = i * angleChange;
            edges.add(new Vector2d(Math.cos(angle) * r, Math.sin(angle) * r).plus(waypointPosition));
        }
        return edges;
    }
}
