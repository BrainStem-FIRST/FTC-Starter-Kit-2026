package org.firstinspires.ftc.teamcode.utils.bezierCurveDrive.follower;

import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.utils.bezierCurveDrive.buildingBlocks.BezierCurve;
import org.firstinspires.ftc.teamcode.utils.bezierCurveDrive.buildingBlocks.BezierParams;
import org.firstinspires.ftc.teamcode.utils.bezierCurveDrive.buildingBlocks.RotationPoint;

import java.util.ArrayList;
import java.util.List;

public class BezierPath {
    public final BezierCurve curve;
    public final BezierParams params;
    public final ArrayList<RotationPoint> rotationPoints;

    public List<SubsystemTriggerPoint> subsystemTriggers = new ArrayList<>();

    public static class SubsystemTriggerPoint {
        public final Action action;
        public final double arcLength;
        public boolean triggered = false;

        public SubsystemTriggerPoint(Action action, double arcLength) {
            this.action = action;
            this.arcLength = arcLength;
        }
    }

    public BezierPath(BezierCurve curve, BezierParams params, ArrayList<RotationPoint> rotationPoints) {
        this.curve = curve;
        this.params = params;
        this.rotationPoints = rotationPoints;
    }
}
