package org.firstinspires.ftc.teamcode.utils.pilotAutoBuilder.helperClasses;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

/** Runs a secondary action until the primary action finishes (WPILib deadline equivalent). */
public class ParallelWhilePrimaryRuns implements Action {
    private final Action primary;
    private final Action secondary;
    private boolean initialized;

    public ParallelWhilePrimaryRuns(Action primary, Action secondary) {
        this.primary = primary;
        this.secondary = secondary;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if (!initialized) {
            initialized = true;
        }

        boolean primaryRunning = primary.run(packet);
        secondary.run(packet);
        return primaryRunning;
    }
}
