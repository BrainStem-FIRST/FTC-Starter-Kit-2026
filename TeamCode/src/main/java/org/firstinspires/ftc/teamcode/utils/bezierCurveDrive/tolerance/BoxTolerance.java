package org.firstinspires.ftc.teamcode.utils.bezierCurveDrive.tolerance;

import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.utils.pilotAutoBuilder.helperClasses.PilotGeometry;

import java.util.ArrayList;
import java.util.Arrays;

public class BoxTolerance implements Tolerance {
    public static class DefaultParams {
        public double xTol = 3.0;
        public double yTol = 3.0;
        public double headingTolDeg = 4.0;
    }
    public static DefaultParams defaultParams = new DefaultParams();
    private final double xTol, yTol;
    private final double headingTolRad;

    public BoxTolerance(double xTol, double yTol, double headingTolDeg) {
        this.xTol = xTol;
        this.yTol = yTol;
        this.headingTolRad = PilotGeometry.fromDegrees(headingTolDeg);
    }

    public BoxTolerance(double distTol, double headingTolDeg) {
        this(distTol, distTol, headingTolDeg);
    }

    public BoxTolerance() {
        this(defaultParams.xTol, defaultParams.yTol, defaultParams.headingTolDeg);
    }

    @Override
    public boolean inPositionTolerance(Vector2d positionError) {
        return Math.abs(positionError.x) <= xTol && Math.abs(positionError.y) <= yTol;
    }

    @Override
    public boolean inHeadingTolerance(double headingErrorRad) {
        return Math.abs(headingErrorRad) < headingTolRad;
    }

    @Override
    public double getPositionDampening(Vector2d positionError) {
        double radius = Math.min(xTol, yTol);
        return Math.min(1, positionError.dot(positionError) / (radius * radius));
    }

    @Override
    public double getHeadingDampening(double headingErrorRad) {
        return Math.min(1, Math.abs(headingErrorRad) / headingTolRad);
    }

    @Override
    public ArrayList<Vector2d> getToleranceCorners(Vector2d position) {
        double x = xTol * 0.5;
        double y = yTol * 0.5;
        return new ArrayList<>(Arrays.asList(
                new Vector2d(-x, -y).plus(position),
                new Vector2d(x, -y).plus(position),
                new Vector2d(x, y).plus(position),
                new Vector2d(-x, y).plus(position)
        ));
    }
}
