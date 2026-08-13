package org.firstinspires.ftc.teamcode.utils.pilotAutoBuilder;

import org.firstinspires.ftc.teamcode.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.utils.pilotAutoBuilder.helperClasses.PilotCommands;

/** Registers robot subsystem actions available to Brainstem Pilot JSON autos. */
public final class PilotCommandRegistry {
    private PilotCommandRegistry() {}

    public static void registerAll(BrainSTEMRobot robot) {
        PilotCommands.registerCommand("Collector", "Intake On", () -> packet -> {
            // collector code would go here
            return false;
        });
    }
}