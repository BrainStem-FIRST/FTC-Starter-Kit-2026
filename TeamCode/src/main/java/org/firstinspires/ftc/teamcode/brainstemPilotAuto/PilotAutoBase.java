package org.firstinspires.ftc.teamcode.brainstemPilotAuto;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;

import org.brainstemfirst.pilot.ftc.PilotOpMode;
import org.brainstemfirst.pilot.ftc.bezier.follower.BezierDrivePath;
import org.brainstemfirst.pilot.ftc.bezier.follower.BezierFollowerConfig;
import org.brainstemfirst.pilot.ftc.model.FieldConstants;
import org.firstinspires.ftc.teamcode.BrainSTEMRobot;

import com.acmerobotics.roadrunner.PoseVelocity2d;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * TEAM-OWNED — Brainstem Pilot UI creates this file once and will not overwrite it.
 * Wire your robot, drive, and {@code PilotRegistry.addCommand} calls here.
 * Generated OpModes in {@code opmodeAutos/} extend this class.
 */
public abstract class PilotAutoBase extends PilotOpMode {
    protected BrainSTEMRobot robot;

    protected PilotAutoBase(String autoId) {
        super(autoId);
        configureFollower();
    }

    /** Bézier follower gains. Read every loop — edit these values here. */
    private void configureFollower() {
        // Use velocity profile for smoother motion.
        BezierFollowerConfig.useVelocityProfile = true;
        BezierFollowerConfig.velKv = 0.014;
        BezierFollowerConfig.velKs = 0.03;
        BezierFollowerConfig.velKp = 0.05;

        // Set cruise velocity/profile decel while tuning.
        BezierFollowerConfig.overrideCruiseVel = false;
        BezierFollowerConfig.cruiseVel = 30;
        BezierFollowerConfig.overrideProfileDecel = false;
        BezierFollowerConfig.profileDecel = 40;

        // Cross-track error gains.
        BezierFollowerConfig.crossTrackKp = 0.05;
        BezierFollowerConfig.correctivePower = 0.7;

        // Heading error gains.
        BezierFollowerConfig.headingkP = 0.05;
        BezierFollowerConfig.headingkF = 0.05;

        // Speed error gains (ONLY used if useVelocityProfile is false)
        BezierFollowerConfig.speedkP = 0.05;
        BezierFollowerConfig.speedkF = 0.05;
        BezierFollowerConfig.speedkD = 0.0;
    }

    @Override
    protected void setupRobot(FieldConstants.Alliance alliance, Pose2d startPose) {
        robot = new BrainSTEMRobot(hardwareMap, telemetry, startPose);
        robot.drive.localizer.setPose(startPose);
    }

    @Override
    protected Supplier<Pose2d> pose() {
        return robot.drive::getPose;
    }

    @Override
    protected Supplier<PoseVelocity2d> lastVelRobot() {
        return robot.drive::lastVelRobot;
    }

    @Override
    protected Consumer<PoseVelocity2d> setDrivePowers() {
        return robot.drive::setDrivePowers;
    }

    @Override
    protected DoubleSupplier maxAngVel() {
        return robot.drive::maxAngVel;
    }

    @Override
    protected void registerCommands() {
        // Names must match the editor: PilotRegistry.addCommand("Subsystem", "Command", () -> action);
    }

    @Override
    protected boolean updateRobot(TelemetryPacket packet) {
        robot.update();

        // Add telemetry for the BezierDrivePath status.
        BezierDrivePath.Status s = BezierDrivePath.status();
        telemetry.addData("target X", s.targetPoint.x);
        telemetry.addData("target Y", s.targetPoint.y);
        telemetry.addData("heading rad", s.targetHeadingRad);
        telemetry.addData("remaining", s.remainingLength);
        telemetry.update();
        return true;
    }
}
