package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrive;
import com.acmerobotics.roadrunner.Pose2d;

public class BrainSTEMRobot {

    public MecanumDrive drive;

    public BrainSTEMRobot(HardwareMap hardwareMap, Telemetry telemetry, Pose2d startPose) {
        drive = new MecanumDrive(hardwareMap, startPose);
    }

    public void update() {
        drive.updateVoltageFiltering();
        drive.updatePoseEstimate();
    }
}