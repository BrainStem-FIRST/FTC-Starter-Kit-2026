package org.firstinspires.ftc.teamcode.utils.pilotAutoBuilder.autoReader;

import android.content.Context;
import android.content.res.AssetManager;

import org.firstinspires.ftc.teamcode.utils.misc.TelemetryLog;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

/** Loads Brainstem Pilot JSON assets bundled under {@code assets/brainstemPilotAuto/}. */
public final class PilotAssetLoader {
    private static final String TAG = "PilotAssetLoader";
    private static Context appContext;

    private PilotAssetLoader() {}

    public static void initialize(Context context) {
        appContext = context.getApplicationContext();
    }

    public static boolean isInitialized() {
        return appContext != null;
    }

    public static InputStream open(String relativePath) throws IOException {
        if (!isInitialized()) {
            TelemetryLog.critical(TAG, "PilotAssetLoader must be initialized with app context before loading assets.");
            throw new IllegalStateException("PilotAssetLoader must be initialized with app context before loading assets.");
        }
        return appContext.getAssets().open("brainstemPilotAuto/" + relativePath);
    }

    public static String readText(String relativePath) throws IOException {
        try (InputStream inputStream = open(relativePath)) {
            ByteArrayOutputStream buffer = new ByteArrayOutputStream();
            byte[] data = new byte[4096];
            int bytesRead;
            while ((bytesRead = inputStream.read(data)) != -1) {
                buffer.write(data, 0, bytesRead);
            }
            return buffer.toString(StandardCharsets.UTF_8.name());
        }
    }

    public static String readPathText(String pathId) throws IOException {
        String pathJson = "paths/" + pathId + ".path.json";
        try {
            return readText(pathJson);
        } catch (IOException firstFailure) {
            return readText("paths/" + pathId + ".json");
        }
    }

    public static File resolvePathFile(String pathId) {
        return new File("brainstemPilotAuto/paths/" + pathId + ".path.json");
    }

    public static String pathAssetRelativePath(String pathId) {
        return "paths/" + pathId + ".path.json";
    }

    public static String variantAssetRelativePath(String variantAutoName) {
        return "variants/" + variantAutoName + ".variant.json";
    }

    public static String skeletonAssetRelativePath(String skeletonId) {
        return "skeletons/" + skeletonId + ".skeleton.json";
    }

    public static List<String> listPathIds() throws IOException {
        return listJsonIds("paths", ".path.json", ".json");
    }

    private static List<String> listJsonIds(String folder, String... suffixes) throws IOException {
        if (!isInitialized()) {
            return List.of();
        }

        AssetManager assets = appContext.getAssets();
        String[] files = assets.list("brainstemPilotAuto/" + folder);
        if (files == null) {
            return List.of();
        }

        List<String> ids = new ArrayList<>();
        Arrays.sort(files, Comparator.naturalOrder());
        for (String fileName : files) {
            String pathId = stripJsonSuffix(fileName, suffixes);
            if (pathId != null) {
                ids.add(pathId);
            }
        }
        return ids;
    }

    private static String stripJsonSuffix(String fileName, String... suffixes) {
        for (String suffix : suffixes) {
            if (fileName.endsWith(suffix)) {
                return fileName.substring(0, fileName.length() - suffix.length());
            }
        }
        return null;
    }
}
