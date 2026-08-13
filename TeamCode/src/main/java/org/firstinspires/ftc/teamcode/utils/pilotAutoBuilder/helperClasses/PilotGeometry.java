package org.firstinspires.ftc.teamcode.utils.pilotAutoBuilder.helperClasses;

import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.utils.math.MathUtils;

/** Heading and vector helpers for Brainstem Pilot path math. */
public final class PilotGeometry {
    private PilotGeometry() {}

    public static double fromDegrees(double degrees) {
        return Math.toRadians(degrees);
    }

    public static double toDegrees(double radians) {
        return Math.toDegrees(radians);
    }

    public static double absHeadingError(double targetRad, double currentRad) {
        return Math.abs(MathUtils.angleNormDeltaRad(targetRad - currentRad));
    }

    public static double lerpHeading(double startRad, double endRad, double t) {
        double delta = MathUtils.angleNormDeltaRad(endRad - startRad);
        return MathUtils.angleNormRad(startRad + delta * t);
    }

    public static double negateHeading(double headingRad) {
        return MathUtils.angleNormRad(-headingRad);
    }

    public static double flipHeadingForRed(double headingRad) {
        return MathUtils.angleNormRad(headingRad + Math.PI);
    }

    public static Vector2d rotate(Vector2d vector, double angleRad) {
        double cos = Math.cos(angleRad);
        double sin = Math.sin(angleRad);
        return new Vector2d(
                vector.x * cos - vector.y * sin,
                vector.x * sin + vector.y * cos
        );
    }

    public static Vector2d fieldToRobot(Vector2d fieldVelocity, double robotHeadingRad) {
        double cos = Math.cos(robotHeadingRad);
        double sin = Math.sin(robotHeadingRad);
        return new Vector2d(
                fieldVelocity.x * cos + fieldVelocity.y * sin,
                -fieldVelocity.x * sin + fieldVelocity.y * cos
        );
    }
}
