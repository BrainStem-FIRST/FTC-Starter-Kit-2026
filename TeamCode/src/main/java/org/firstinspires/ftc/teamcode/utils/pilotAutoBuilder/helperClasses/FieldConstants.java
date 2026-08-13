package org.firstinspires.ftc.teamcode.utils.pilotAutoBuilder.helperClasses;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

/** Field coordinate transforms for Brainstem Pilot paths (blue-field authoring). */
public final class FieldConstants {
    private FieldConstants() {}

    /** Mirror across the field centerline (left/right start side). */
    public static Vector2d mirrorSide(Vector2d point) {
        return new Vector2d(-point.x, point.y);
    }

    public static Pose2d mirrorSide(Pose2d pose) {
        return new Pose2d(-pose.position.x, pose.position.y, -pose.heading.toDouble());
    }

    /** Mirror for the red alliance (180° about field origin). */
    public static Vector2d mirrorAlliance(Vector2d point) {
        return new Vector2d(-point.x, -point.y);
    }

    public static Pose2d mirrorAlliance(Pose2d pose) {
        return new Pose2d(-pose.position.x, -pose.position.y, pose.heading.toDouble() + Math.PI);
    }
}
