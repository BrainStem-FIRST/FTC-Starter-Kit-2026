package org.firstinspires.ftc.teamcode.utils.pilotAutoBuilder.autoReader;

import com.acmerobotics.roadrunner.Vector2d;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import org.firstinspires.ftc.teamcode.utils.bezierCurveDrive.buildingBlocks.BezierCurve;
import org.firstinspires.ftc.teamcode.utils.bezierCurveDrive.buildingBlocks.BezierParams;
import org.firstinspires.ftc.teamcode.utils.bezierCurveDrive.buildingBlocks.RotationPoint;
import org.firstinspires.ftc.teamcode.utils.bezierCurveDrive.follower.BezierPath;
import org.firstinspires.ftc.teamcode.utils.bezierCurveDrive.tolerance.CircleTolerance;
import org.firstinspires.ftc.teamcode.utils.math.MathUtils;
import org.firstinspires.ftc.teamcode.utils.pilotAutoBuilder.helperClasses.PilotCommands;
import org.firstinspires.ftc.teamcode.utils.pilotAutoBuilder.helperClasses.PilotGeometry;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class PathParser {
    private static final ObjectMapper m_objectMapper = new ObjectMapper();

    private static class GlobalRotation {
        final double distance;
        final double headingRad;

        GlobalRotation(double distance, double headingRad) {
            this.distance = distance;
            this.headingRad = headingRad;
        }
    }

    public static BezierPath[] parsePathFile(String pathId, BezierParams defaultParams) throws IOException {
        JsonNode root = m_objectMapper.readTree(PilotAssetLoader.readPathText(pathId));

        JsonNode constraintsNode = root.get("constraints");
        double maxLinearVelocity = defaultParams.maxLinearSpeed;
        if (constraintsNode != null) {
            maxLinearVelocity = constraintsNode.path("maxVel").asDouble(defaultParams.maxLinearSpeed);
        }

        JsonNode waypointsNode = root.get("waypoints");
        if (waypointsNode == null || !waypointsNode.isArray() || waypointsNode.size() < 2) {
            throw new IOException("Path layout sequence requires at least two valid anchor coordinates.");
        }

        int segmentCount = waypointsNode.size() - 1;
        List<BezierCurve> curves = new ArrayList<>();

        double[] segmentLengths = new double[segmentCount];
        double totalPathLength = 0.0;

        for (int i = 0; i < segmentCount; i++) {
            JsonNode wpStart = waypointsNode.get(i);
            JsonNode wpEnd = waypointsNode.get(i + 1);

            Vector2d startPoint = getTranslation(wpStart);
            Vector2d endPoint = getTranslation(wpEnd);

            Vector2d control1 = wpStart.has("nextControl") && !wpStart.get("nextControl").isNull()
                    ? getTranslation(wpStart.get("nextControl"))
                    : startPoint.plus(endPoint.minus(startPoint).times(0.333));

            Vector2d control2 = wpEnd.has("prevControl") && !wpEnd.get("prevControl").isNull()
                    ? getTranslation(wpEnd.get("prevControl"))
                    : startPoint.plus(endPoint.minus(startPoint).times(0.667));

            BezierCurve curve = new BezierCurve(startPoint, control1, control2, endPoint);
            curves.add(curve);

            double segLength = 0.0;
            Vector2d lastPoint = curve.getPoint(0.0);
            int samples = 50;
            for (int j = 1; j <= samples; j++) {
                Vector2d currentPoint = curve.getPoint((double) j / samples);
                segLength += MathUtils.vecDist(currentPoint, lastPoint);
                lastPoint = currentPoint;
            }
            segmentLengths[i] = segLength;
            totalPathLength += segLength;
        }

        List<GlobalRotation> globalRotations = new ArrayList<>();

        double startHeading = waypointsNode.get(0).path("rotation").asDouble(0.0);
        globalRotations.add(new GlobalRotation(0.0, PilotGeometry.fromDegrees(startHeading)));

        JsonNode rotationTargetsNode = root.get("rotationTargets");
        if (rotationTargetsNode != null && rotationTargetsNode.isArray()) {
            for (JsonNode rotTarget : rotationTargetsNode) {
                double headingDegrees = rotTarget.path("rotation").asDouble(0.0);
                double targetDist = rotTarget.path("arcLengthM").asDouble(-1.0);
                if (targetDist < 0) {
                    targetDist = rotTarget.path("progress").asDouble(0.0) * totalPathLength;
                }
                globalRotations.add(new GlobalRotation(targetDist, PilotGeometry.fromDegrees(headingDegrees)));
            }
        }

        double finalHeading = waypointsNode.get(waypointsNode.size() - 1).path("rotation").asDouble(0.0);
        globalRotations.add(new GlobalRotation(totalPathLength, PilotGeometry.fromDegrees(finalHeading)));

        globalRotations.sort((r1, r2) -> Double.compare(r1.distance, r2.distance));

        List<ArrayList<RotationPoint>> rotationPointsPerSegment = new ArrayList<>();
        for (int i = 0; i < segmentCount; i++) {
            rotationPointsPerSegment.add(new ArrayList<>());
        }

        double currentSegmentStartDist = 0.0;
        for (int i = 0; i < segmentCount; i++) {
            double currentSegmentEndDist = currentSegmentStartDist + segmentLengths[i];
            ArrayList<RotationPoint> segmentList = rotationPointsPerSegment.get(i);

            segmentList.add(new RotationPoint(sampleGlobalRotation(globalRotations, currentSegmentStartDist), 0.0));
            segmentList.add(new RotationPoint(sampleGlobalRotation(globalRotations, currentSegmentEndDist), 1.0));

            for (GlobalRotation gr : globalRotations) {
                if (gr.distance > currentSegmentStartDist + 1e-4 && gr.distance < currentSegmentEndDist - 1e-4) {
                    double localT = (gr.distance - currentSegmentStartDist) / segmentLengths[i];
                    segmentList.add(new RotationPoint(gr.headingRad, localT));
                }
            }

            segmentList.sort((p1, p2) -> Double.compare(p1.getT(), p2.getT()));
            currentSegmentStartDist = currentSegmentEndDist;
        }

        JsonNode waypointParamsNode = root.get("waypointParams");
        BezierParams[] segmentParams = new BezierParams[segmentCount];

        for (int i = 0; i < segmentCount; i++) {
            String targetWaypointKey = String.valueOf(i + 1);
            BezierParams bp = new BezierParams()
                    .setSpeedKp(defaultParams.speedKp)
                    .setSpeedKf(defaultParams.speedKf)
                    .setHeadingKp(defaultParams.headingKp)
                    .setHeadingKf(defaultParams.headingKf)
                    .setCorrectivePower(defaultParams.correctivePower)
                    .setMaxLinearSpeed(maxLinearVelocity)
                    .setMinLinearSpeed(defaultParams.minLinearSpeed)
                    .setMaxTurnPower(defaultParams.maxTurnPower)
                    .setMaxDrivePowerRampRate(defaultParams.maxDrivePowerRampRate)
                    .setMaxTurnPowerRampRate(defaultParams.maxTurnPowerRampRate)
                    .setMaxTime(defaultParams.maxTime)
                    .setTolerance(defaultParams.tolerance)
                    .setPassPosition(defaultParams.passPosition);

            if (waypointParamsNode != null && waypointParamsNode.has(targetWaypointKey)) {
                JsonNode params = waypointParamsNode.get(targetWaypointKey);
                double dist = defaultParams.tolerance instanceof CircleTolerance
                        ? CircleTolerance.defaultParams.distTol : 3.0;
                double headDeg = defaultParams.tolerance instanceof CircleTolerance
                        ? CircleTolerance.defaultParams.headingTolDeg : 4.0;
                boolean toleranceOverridden = false;

                if (params.has("distTol")) {
                    dist = params.get("distTol").asDouble();
                    toleranceOverridden = true;
                }
                if (params.has("headingTol")) {
                    headDeg = params.get("headingTol").asDouble();
                    toleranceOverridden = true;
                }
                if (toleranceOverridden) bp.setTolerance(new CircleTolerance(dist, headDeg));
                if (params.has("minLinearSpeed")) bp.setMinLinearSpeed(params.get("minLinearSpeed").asDouble());
                if (params.has("maxLinearSpeed")) bp.setMaxLinearSpeed(params.get("maxLinearSpeed").asDouble() * maxLinearVelocity);
                if (params.has("maxTurnPower")) bp.setMaxTurnPower(params.get("maxTurnPower").asDouble());
                if (params.has("maxTime")) bp.setMaxTime(params.get("maxTime").asDouble());
                if (params.has("passPosition")) bp.setPassPosition(params.get("passPosition").asBoolean());
            }
            segmentParams[i] = bp;
        }

        JsonNode triggersNode = root.get("subsystemTriggers");
        List<List<BezierPath.SubsystemTriggerPoint>> triggersPerSegment = new ArrayList<>();
        for (int i = 0; i < segmentCount; i++) triggersPerSegment.add(new ArrayList<>());

        if (triggersNode != null && triggersNode.isArray()) {
            double runningDist = 0.0;
            for (int i = 0; i < segmentCount; i++) {
                double segEnd = runningDist + segmentLengths[i];
                for (JsonNode trig : triggersNode) {
                    double trigDist = trig.path("arcLengthM").asDouble(-1.0);
                    if (trigDist < 0) {
                        trigDist = trig.path("progress").asDouble(0.0) * totalPathLength;
                    }
                    if (trigDist >= runningDist && trigDist < segEnd) {
                        String sysName = trig.path("subsystemName").asText("");
                        String cmdName = trig.path("commandName").asText("");
                        if (!sysName.isEmpty() && !cmdName.isEmpty()) {
                            triggersPerSegment.get(i).add(
                                    new BezierPath.SubsystemTriggerPoint(
                                            PilotCommands.getCommand(sysName, cmdName),
                                            trigDist
                                    )
                            );
                        }
                    }
                }
                runningDist = segEnd;
            }
        }

        BezierPath[] pathSegments = new BezierPath[segmentCount];
        for (int i = 0; i < segmentCount; i++) {
            pathSegments[i] = new BezierPath(curves.get(i), segmentParams[i], rotationPointsPerSegment.get(i));
            pathSegments[i].subsystemTriggers = triggersPerSegment.get(i);
        }

        return pathSegments;
    }

    private static double sampleGlobalRotation(List<GlobalRotation> timeline, double distance) {
        if (timeline.isEmpty()) return 0.0;
        if (distance <= timeline.get(0).distance) return timeline.get(0).headingRad;
        if (distance >= timeline.get(timeline.size() - 1).distance) {
            return timeline.get(timeline.size() - 1).headingRad;
        }

        for (int i = 0; i < timeline.size() - 1; i++) {
            GlobalRotation r1 = timeline.get(i);
            GlobalRotation r2 = timeline.get(i + 1);
            if (distance >= r1.distance && distance <= r2.distance) {
                double t = (distance - r1.distance) / (r2.distance - r1.distance);
                return PilotGeometry.lerpHeading(r1.headingRad, r2.headingRad, t);
            }
        }
        return timeline.get(timeline.size() - 1).headingRad;
    }

    private static Vector2d getTranslation(JsonNode node) {
        return new Vector2d(node.path("x").asDouble(0.0), node.path("y").asDouble(0.0));
    }
}
