package org.firstinspires.ftc.teamcode.utils.pilotAutoBuilder.autoReader;

import android.content.Context;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.fasterxml.jackson.databind.ObjectMapper;

import org.firstinspires.ftc.teamcode.utils.misc.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.misc.TelemetryLog;
import org.firstinspires.ftc.teamcode.utils.bezierCurveDrive.buildingBlocks.BezierParams;
import org.firstinspires.ftc.teamcode.utils.bezierCurveDrive.follower.BezierDrivePath;
import org.firstinspires.ftc.teamcode.utils.bezierCurveDrive.follower.BezierPath;
import org.firstinspires.ftc.teamcode.utils.pilotAutoBuilder.helperClasses.CommandOverride;
import org.firstinspires.ftc.teamcode.utils.pilotAutoBuilder.helperClasses.FieldConstants;
import org.firstinspires.ftc.teamcode.utils.pilotAutoBuilder.helperClasses.ParallelWhilePrimaryRuns;
import org.firstinspires.ftc.teamcode.utils.pilotAutoBuilder.helperClasses.PilotCommands;
import org.firstinspires.ftc.teamcode.utils.pilotAutoBuilder.helperClasses.SkeletonAuto;
import org.firstinspires.ftc.teamcode.utils.pilotAutoBuilder.helperClasses.SkeletonCommand;
import org.firstinspires.ftc.teamcode.utils.pilotAutoBuilder.helperClasses.TriggerWatcher;
import org.firstinspires.ftc.teamcode.utils.pilotAutoBuilder.helperClasses.VariantAuto;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

public class BrainstemPilot {

    private static final String TAG = "BrainstemPilot";

    private static final ObjectMapper m_objectMapper = new ObjectMapper();
    private static MecanumDrive m_drive;
    private static Alliance m_alliance;
    private static BezierParams m_defaultParams;

    private static final Map<String, List<BezierPath[]>> m_parsedAutosCache = new HashMap<>();
    private static final Map<String, Pose2d> m_startingPoseCache = new HashMap<>();

    public static final String PATH_CHOOSER_PREFIX = "path:";

    public static void prepareAssets(Context appContext, BezierParams defaultParams) {
        PilotAssetLoader.initialize(appContext);
        m_defaultParams = defaultParams;
    }

    public static void initialize(Context appContext, MecanumDrive drive, Alliance alliance, BezierParams defaultParams) {
        prepareAssets(appContext, defaultParams);
        m_drive = drive;
        m_alliance = alliance;
    }

    public static PilotAutoBuilder buildAuto(String variantAutoName) {
        if (m_drive == null || m_defaultParams == null || m_alliance == null) {
            TelemetryLog.critical(TAG, "BrainstemPilot must be initialized before constructing autonomous routes.");
            throw new IllegalStateException("BrainstemPilot must be initialized before constructing autonomous routes.");
        }
        return PilotAutoBuilder.forAuto(variantAutoName);
    }

    public static PilotAutoBuilder buildPath(String pathId) {
        if (m_drive == null || m_defaultParams == null || m_alliance == null) {
            TelemetryLog.critical(TAG, "BrainstemPilot must be initialized before constructing autonomous routes.");
            throw new IllegalStateException("BrainstemPilot must be initialized before constructing autonomous routes.");
        }
        return PilotAutoBuilder.forPath(pathId);
    }

    static Action buildPathInternal(String pathId) {
        try {
            BezierPath[] pathSegments = PathParser.parsePathFile(pathId, m_defaultParams);

            String cacheKey = pathCacheKey(pathId);
            List<BezierPath[]> cachedPaths = new ArrayList<>();
            cachedPaths.add(pathSegments);
            m_parsedAutosCache.put(cacheKey, cachedPaths);
            return buildPathAction(pathId, pathSegments);
        } catch (IOException e) {
            TelemetryLog.error(TAG, "Failed to load path: " + pathId, e);
            return new InstantAction(() -> {});
        }
    }

    public static String pathChooserValue(String pathId) {
        return PATH_CHOOSER_PREFIX + pathId;
    }

    public static boolean isPathChooserValue(String chooserValue) {
        return chooserValue != null && chooserValue.startsWith(PATH_CHOOSER_PREFIX);
    }

    public static Action buildFromChooser(String chooserValue) {
        if (isPathChooserValue(chooserValue)) {
            return buildPath(chooserValue.substring(PATH_CHOOSER_PREFIX.length())).build();
        }
        return buildAuto(chooserValue).build();
    }

    public static Optional<Pose2d> getStartingPose(String chooserValue, Alliance alliance) {
        if (m_defaultParams == null || chooserValue == null || alliance == null) {
            return Optional.empty();
        }

        String firstPathId = resolveFirstPathIdFromChooser(chooserValue);
        if (firstPathId == null) {
            return Optional.empty();
        }

        String cacheKey = chooserValue + "|" + alliance.name();
        Pose2d cached = m_startingPoseCache.get(cacheKey);
        if (cached != null) {
            return Optional.of(cached);
        }

        try {
            Pose2d pose = parseStartingPoseFromPath(firstPathId, alliance);
            m_startingPoseCache.put(cacheKey, pose);
            return Optional.of(pose);
        } catch (IOException e) {
            TelemetryLog.warn(TAG, "Failed to resolve starting pose for: " + chooserValue, e);
            return Optional.empty();
        }
    }

    /** Returns display name -> chooser value entries for all bundled path JSON files. */
    public static Map<String, String> getAvailablePathOptions() {
        Map<String, String> options = new LinkedHashMap<>();
        try {
            for (String pathId : PilotAssetLoader.listPathIds()) {
                String displayName = "Path - " + pathId.replace("_", " ");
                options.put(displayName, pathChooserValue(pathId));
            }
        } catch (IOException e) {
            TelemetryLog.warn(TAG, "Failed to list path assets.", e);
        }
        return options;
    }

    static Action buildAutoInternal(String variantAutoName) {
        try {
            VariantAuto variant = loadVariant(variantAutoName);
            if (variant == null) {
                return new InstantAction(() -> {});
            }

            SkeletonAuto skeleton = loadSkeleton(variant.skeletonId);
            if (skeleton == null) {
                return new InstantAction(() -> {});
            }

            Map<String, CommandOverride> overrideMap = buildOverrideMap(variant);

            List<Action> autoActionsSequence = new ArrayList<>();
            List<BezierPath[]> pathsToCache = new ArrayList<>();

            for (SkeletonCommand skCmd : skeleton.commands) {
                CommandOverride override = overrideMap.get(skCmd.id);
                if (override != null && override.skip) continue;

                if ("path".equalsIgnoreCase(skCmd.type)) {
                    String activePathId = (override != null && override.pathId != null)
                            ? override.pathId : skCmd.label;
                    if (activePathId == null || activePathId.isEmpty()) {
                        TelemetryLog.warn(TAG, "Path ID was empty for command ID: " + skCmd.id);
                        continue;
                    }
                    try {
                        BezierPath[] pathSegments = PathParser.parsePathFile(activePathId, m_defaultParams);

                        autoActionsSequence.add(buildPathAction(activePathId, pathSegments));
                        pathsToCache.add(pathSegments);
                    } catch (IOException e) {
                        TelemetryLog.error(TAG, "Skipping invalid path: " + activePathId, e);
                    }

                } else if ("wait".equalsIgnoreCase(skCmd.type)) {
                    if (skCmd.defaultWait > 0) {
                        autoActionsSequence.add(new SleepAction(skCmd.defaultWait));
                    }

                } else if ("subsystem".equalsIgnoreCase(skCmd.type)) {
                    if (skCmd.subsystemName != null && skCmd.commandName != null) {
                        autoActionsSequence.add(PilotCommands.getCommand(skCmd.subsystemName, skCmd.commandName));
                    } else {
                        TelemetryLog.warn(TAG, "Subsystem command missing name fields, id: " + skCmd.id);
                    }

                } else if ("parallel".equalsIgnoreCase(skCmd.type)) {
                    if (skCmd.parallelSubs != null && !skCmd.parallelSubs.isEmpty()) {
                        List<Action> parallelActions = new ArrayList<>();
                        for (SkeletonCommand sub : skCmd.parallelSubs) {
                            if (sub.subsystemName != null && sub.commandName != null) {
                                parallelActions.add(PilotCommands.getCommand(sub.subsystemName, sub.commandName));
                            }
                        }
                        if (!parallelActions.isEmpty()) {
                            autoActionsSequence.add(new ParallelAction(parallelActions.toArray(new Action[0])));
                        }
                    }
                }
            }

            if (!pathsToCache.isEmpty()) {
                m_parsedAutosCache.put(variantAutoName, pathsToCache);
            }

            return new SequentialAction(autoActionsSequence.toArray(new Action[0]));

        } catch (Exception e) {
            TelemetryLog.critical(TAG, "Engine failure loading routing profiles for: " + variantAutoName, e);
            return new InstantAction(() -> {});
        }
    }

    private static Action buildPathAction(String pathId, BezierPath[] pathSegments) {
        BezierDrivePath driveAction = new BezierDrivePath(pathId, m_drive, m_alliance, pathSegments);

        boolean hasTriggers = false;
        for (BezierPath segment : pathSegments) {
            if (!segment.subsystemTriggers.isEmpty()) {
                hasTriggers = true;
                break;
            }
        }

        return hasTriggers
                ? new ParallelWhilePrimaryRuns(driveAction, new TriggerWatcher(m_drive, pathSegments))
                : driveAction;
    }

    private static String pathCacheKey(String pathId) {
        return "path_" + pathId;
    }

    public static void draw(Canvas canvas, String variantAutoName) {
        drawCachedPaths(canvas, variantAutoName);
    }

    private static VariantAuto loadVariant(String variantAutoName) throws IOException {
        String json = PilotAssetLoader.readText(PilotAssetLoader.variantAssetRelativePath(variantAutoName));
        return m_objectMapper.readValue(json, VariantAuto.class);
    }

    private static SkeletonAuto loadSkeleton(String skeletonId) throws IOException {
        String json = PilotAssetLoader.readText(PilotAssetLoader.skeletonAssetRelativePath(skeletonId));
        return m_objectMapper.readValue(json, SkeletonAuto.class);
    }

    private static Map<String, CommandOverride> buildOverrideMap(VariantAuto variant) {
        Map<String, CommandOverride> overrideMap = new HashMap<>();
        if (variant.commandOverrides != null) {
            for (CommandOverride override : variant.commandOverrides) {
                overrideMap.put(override.cmdId, override);
            }
        }
        return overrideMap;
    }

    private static String resolveFirstPathId(SkeletonAuto skeleton, Map<String, CommandOverride> overrideMap) {
        for (SkeletonCommand skCmd : skeleton.commands) {
            CommandOverride override = overrideMap.get(skCmd.id);
            if (override != null && override.skip) {
                continue;
            }
            if ("path".equalsIgnoreCase(skCmd.type)) {
                String activePathId = (override != null && override.pathId != null)
                        ? override.pathId : skCmd.label;
                if (activePathId != null && !activePathId.isEmpty()) {
                    return activePathId;
                }
            }
        }
        return null;
    }

    private static String resolveFirstPathIdFromChooser(String chooserValue) {
        if (isPathChooserValue(chooserValue)) {
            return chooserValue.substring(PATH_CHOOSER_PREFIX.length());
        }

        try {
            VariantAuto variant = loadVariant(chooserValue);
            if (variant == null) {
                return null;
            }
            SkeletonAuto skeleton = loadSkeleton(variant.skeletonId);
            if (skeleton == null) {
                return null;
            }
            return resolveFirstPathId(skeleton, buildOverrideMap(variant));
        } catch (IOException e) {
            TelemetryLog.warn(TAG, "Failed to resolve first path for: " + chooserValue, e);
            return null;
        }
    }

    private static Pose2d parseStartingPoseFromPath(String pathId, Alliance alliance)
            throws IOException {
        BezierPath[] segments = PathParser.parsePathFile(pathId, m_defaultParams);
        if (segments.length == 0) {
            throw new IOException("Path has no segments: " + pathId);
        }

        BezierPath firstSegment = segments[0];
        double headingRad = firstSegment.rotationPoints.isEmpty()
                ? 0.0
                : firstSegment.rotationPoints.get(0).getHeadingRad();
        Pose2d bluePose = new Pose2d(firstSegment.curve.getStart(), headingRad);

        if (alliance == Alliance.RED) {
            return FieldConstants.mirrorAlliance(FieldConstants.mirrorSide(bluePose));
        }
        return bluePose;
    }

    public static void drawPath(Canvas canvas, String pathId) {
        if (canvas == null) return;

        List<BezierPath[]> completeSequence = m_parsedAutosCache.get(pathCacheKey(pathId));
        if (completeSequence == null) {
            return;
        }

        for (BezierPath[] segmentGroup : completeSequence) {
            for (BezierPath segment : segmentGroup) {
                segment.curve.draw(canvas, 20);
            }
        }
    }

    private static void drawCachedPaths(Canvas canvas, String cacheKey) {
        if (canvas == null) return;

        List<BezierPath[]> completeSequence = m_parsedAutosCache.get(cacheKey);
        if (completeSequence != null) {
            for (BezierPath[] segmentGroup : completeSequence) {
                for (BezierPath segment : segmentGroup) {
                    segment.curve.draw(canvas, 20);
                }
            }
        }
    }
}
