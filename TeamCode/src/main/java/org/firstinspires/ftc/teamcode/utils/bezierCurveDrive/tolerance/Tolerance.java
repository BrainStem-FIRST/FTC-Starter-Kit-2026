package org.firstinspires.ftc.teamcode.utils.bezierCurveDrive.tolerance;

import com.acmerobotics.roadrunner.Vector2d;

import java.util.ArrayList;

public interface Tolerance {
    boolean inPositionTolerance(Vector2d positionError);
    boolean inHeadingTolerance(double headingErrorRad);
    double getPositionDampening(Vector2d positionError);
    double getHeadingDampening(double headingErrorRad);
    ArrayList<Vector2d> getToleranceCorners(Vector2d position);
}
