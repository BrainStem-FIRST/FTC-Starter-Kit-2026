package org.firstinspires.ftc.teamcode.brainstemPilotAuto;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;

import org.brainstemfirst.pilot.ftc.PilotOpMode;
import org.brainstemfirst.pilot.ftc.bezier.follower.BezierFollowerConfig;
import org.brainstemfirst.pilot.ftc.model.PilotAlliance;
import org.brainstemfirst.pilot.ftc.model.PilotDrive;

/**
 * TEAM-OWNED — Brainstem Pilot UI creates this file once and will not overwrite it.
 * Wire your robot, drive, and {@code PilotRegistry.addCommand} calls here.
 * Generated OpModes in {@code opmodeAutos/} extend this class.
 *
 * Road Runner feedforward ({@code kV}/{@code kS}/{@code kA}) lives on your
 * {@code MecanumDrive.PARAMS}, not here.
 */
public abstract class PilotAutoBase extends PilotOpMode {
    protected PilotAutoBase(String autoId) {
        super(autoId);
        configureFollower();
    }

    /** Bézier follower gains. FTC Dashboard can still override these at runtime. */
    private void configureFollower() {
        BezierFollowerConfig.useVelocityProfile = true;
        BezierFollowerConfig.velKv = 0.014;
        BezierFollowerConfig.velKs = 0.03;
        BezierFollowerConfig.velKp = 0.05;
        BezierFollowerConfig.crossTrackKp = 0.05;
        BezierFollowerConfig.speedkP = 0.05;
        BezierFollowerConfig.speedkF = 0.05;
        BezierFollowerConfig.speedkD = 0.0;
        BezierFollowerConfig.correctivePower = 0.7;
        BezierFollowerConfig.headingkP = 0.05;
        BezierFollowerConfig.headingkF = 0.05;
        BezierFollowerConfig.overrideCruiseVel = false;
        BezierFollowerConfig.cruiseVel = 30;
        BezierFollowerConfig.overrideProfileDecel = false;
        BezierFollowerConfig.profileDecel = 40;
    }

    @Override
    protected void setupRobot(PilotAlliance alliance, Pose2d startPose) {
        // Construct your robot and seed odometry to startPose.
    }

    @Override
    protected PilotDrive getDrive() {
        // Return your drivetrain. Road Runner MecanumDrive can implement PilotDrive.
        throw new IllegalStateException("Implement getDrive() in PilotAutoBase");
    }

    @Override
    protected void registerCommands() {
        // Names must match the editor: PilotRegistry.addCommand("Subsystem", "Command", () -> action);
    }

    @Override
    protected boolean updateRobot(TelemetryPacket packet) {
        // Subsystem loop + localizer update. Return true to keep running.
        return true;
    }

    @Override
    protected void drawRobot(Canvas canvas) {
    }
}
