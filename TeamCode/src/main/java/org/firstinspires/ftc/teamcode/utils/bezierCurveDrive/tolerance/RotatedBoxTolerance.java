package org.firstinspires.ftc.teamcode.utils.bezierCurveDrive.tolerance;

import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.utils.pilotAutoBuilder.helperClasses.PilotGeometry;

import java.util.ArrayList;
import java.util.Arrays;

public class RotatedBoxTolerance implements Tolerance {
    private final double parallelTol, perpendicularTol;
    private final double headingTolRad;
    private final double axisAngleRad;

    public RotatedBoxTolerance(double parallelTol, double perpendicularTol, double axisAngleRad, double headingTolDeg) {
        this.parallelTol = parallelTol;
        this.perpendicularTol = perpendicularTol;
        this.axisAngleRad = axisAngleRad;
        this.headingTolRad = PilotGeometry.fromDegrees(headingTolDeg);
    }

    @Override
    public boolean inPositionTolerance(Vector2d positionError) {
        Vector2d rotatedErrors = PilotGeometry.rotate(positionError, -axisAngleRad);
        return Math.abs(rotatedErrors.x) <= parallelTol && Math.abs(rotatedErrors.y) <= perpendicularTol;
    }

    @Override
    public boolean inHeadingTolerance(double headingErrorRad) {
        return Math.abs(headingErrorRad) < headingTolRad;
    }

    @Override
    public double getPositionDampening(Vector2d positionError) {
        double radius = Math.min(parallelTol, perpendicularTol);
        return Math.min(1, positionError.dot(positionError) / (radius * radius));
    }

    @Override
    public double getHeadingDampening(double headingErrorRad) {
        return Math.min(1, Math.abs(headingErrorRad) / headingTolRad);
    }

    @Override
    public ArrayList<Vector2d> getToleranceCorners(Vector2d waypointPosition) {
        double x = parallelTol * 0.5;
        double y = perpendicularTol * 0.5;
        ArrayList<Vector2d> unRotated = new ArrayList<>(Arrays.asList(
                new Vector2d(-x, -y),
                new Vector2d(x, -y),
                new Vector2d(x, y),
                new Vector2d(-x, y)
        ));
        ArrayList<Vector2d> rotated = new ArrayList<>();
        for (Vector2d vec : unRotated) {
            rotated.add(PilotGeometry.rotate(vec, -axisAngleRad).plus(waypointPosition));
        }
        return rotated;
    }
}