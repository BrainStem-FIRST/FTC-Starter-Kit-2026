package org.firstinspires.ftc.teamcode.utils.bezierCurveDrive.buildingBlocks;

/**
 * Associates a target heading with a position along a Bézier curve, expressed as a t-value in [0, 1].
 */
public class RotationPoint {

    private final double headingRad;
    private final double t;

    public RotationPoint(double headingRad, double t) {
        this.headingRad = headingRad;
        this.t = Math.max(0.0, Math.min(1.0, t));
    }

    public double getHeadingRad() {
        return headingRad;
    }

    public double getT() {
        return t;
    }
}
