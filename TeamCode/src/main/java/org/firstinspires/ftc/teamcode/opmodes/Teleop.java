package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BrainSTEMRobot;

@TeleOp(name="Teleop")
public class Teleop extends LinearOpMode {
    private final Pose2d initialPose = new Pose2d(0, 0, 0);
    private BrainSTEMRobot robot;

    @Override
    public void runOpMode() throws InterruptedException {

        TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        Telemetry dashboardAndDs = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(20);

        robot = new BrainSTEMRobot(hardwareMap, telemetry, initialPose);

        waitForStart();

        while(opModeIsActive()) {
            updateDrive();
            updateDriver1();
            updateDriver2();
            robot.update();
            panelsTelemetry.update(dashboardAndDs);
        }
    }

    private void updateDrive() {
        robot.drive.setDrivePowers(new PoseVelocity2d(
            new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x),
            -gamepad1.right_stick_x)
        );
    }

    private void updateDriver1() {}
    private void updateDriver2() {}
}