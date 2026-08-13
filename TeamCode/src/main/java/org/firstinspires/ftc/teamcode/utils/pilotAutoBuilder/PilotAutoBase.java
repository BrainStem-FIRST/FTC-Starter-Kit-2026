package org.firstinspires.ftc.teamcode.utils.pilotAutoBuilder;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.utils.misc.Alliance;
import org.firstinspires.ftc.teamcode.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.utils.misc.TelemetryLog;
import org.firstinspires.ftc.teamcode.utils.bezierCurveDrive.buildingBlocks.BezierParams;
import org.firstinspires.ftc.teamcode.utils.bezierCurveDrive.tolerance.CircleTolerance;
import org.firstinspires.ftc.teamcode.utils.pilotAutoBuilder.autoReader.BrainstemPilot;

/**
 * Base OpMode for Brainstem Pilot JSON-driven autos.
 * Subclasses are auto-generated under this package by Brainstem Pilot UI (FTC projects).
 */
@Config
public abstract class PilotAutoBase extends LinearOpMode {
    public static Alliance defaultAlliance = Alliance.BLUE;
    private final String variantAutoName;
    private Alliance alliance;
    private BezierParams defaultParams;
    private Action pilotAuto;
    private Pose2d startPose;

    protected BrainSTEMRobot robot;
    public static double speedkP = 0.15, speedkF = 0.01;
    public static double headingkP = 0.15, headingkF = 0.01;

    protected PilotAutoBase(String variantAutoName) {
        this.variantAutoName = variantAutoName;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(11);
        TelemetryLog.set(telemetry);

        defaultParams = createDefaultBezierParams();
        BrainstemPilot.prepareAssets(hardwareMap.appContext, defaultParams);
        alliance = defaultAlliance;
        applyAllianceConfiguration();

        while (!isStarted() && !isStopRequested()) {
            Alliance previousAlliance = alliance;

            if (gamepad1.xWasPressed()) alliance = Alliance.BLUE;
            if (gamepad1.bWasPressed()) alliance = Alliance.RED;
            if (alliance != previousAlliance) applyAllianceConfiguration();

            telemetry.addData("Variant", variantAutoName);
            telemetry.addData("Alliance", alliance);
            telemetry.addData("Start pose", startPose);
            telemetry.addLine("X = Blue | B = Red");
            telemetry.addLine("Ready — waiting for START");
            telemetry.update();
        }

        waitForStart();

        Actions.runBlocking(new ParallelAction(pilotAuto, this::runRobotUpdateLoop));
    }

    private void applyAllianceConfiguration() {
        startPose = BrainstemPilot.getStartingPose(variantAutoName, alliance)
                .orElse(new Pose2d(0, 0, 0));
        robot = new BrainSTEMRobot(hardwareMap, telemetry, startPose);
        PilotCommandRegistry.registerAll(robot);
        BrainstemPilot.initialize(hardwareMap.appContext, robot.drive, alliance, defaultParams);
        robot.drive.localizer.setPose(startPose);
        pilotAuto = BrainstemPilot.buildAuto(variantAutoName).build();
    }

    private BezierParams createDefaultBezierParams() {
        return new BezierParams()
            .setSpeedKp(speedkP)
            .setSpeedKf(speedkF)
            .setHeadingKp(headingkP)
            .setHeadingKf(headingkF)
            .setTolerance(new CircleTolerance(2, 5));
    }

    private boolean runRobotUpdateLoop(TelemetryPacket packet) {
        robot.update();
        BrainstemPilot.draw(packet.fieldOverlay(), variantAutoName);
        return true;
    }
}