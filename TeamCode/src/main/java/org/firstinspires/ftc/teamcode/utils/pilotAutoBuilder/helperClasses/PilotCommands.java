package org.firstinspires.ftc.teamcode.utils.pilotAutoBuilder.helperClasses;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;

import org.firstinspires.ftc.teamcode.utils.misc.TelemetryLog;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

public class PilotCommands {
    private static final String TAG = "PilotCommands";
    private static final Map<String, Supplier<Action>> m_registry = new HashMap<>();

    public static void registerCommand(String subsystemName, String commandName, Supplier<Action> commandSupplier) {
        m_registry.put(subsystemName + ":" + commandName, commandSupplier);
    }

    public static Action getCommand(String subsystemName, String commandName) {
        String key = subsystemName + ":" + commandName;
        Supplier<Action> supplier = m_registry.get(key);
        if (supplier == null) {
            TelemetryLog.warn(TAG, "No registered command for: " + key);
            return new InstantAction(() -> {});
        }
        return supplier.get();
    }
}
