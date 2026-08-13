package org.firstinspires.ftc.teamcode.utils.misc;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;

import java.util.ArrayList;

@Config
public class TelemetryHelper {
    public static String[] colors = { "red", "green", "purple", "black", "gray" };
    public static Integer[] strokeWidths = { 1, 1, 1, 1, 1 };
    public static Integer[] radii = { 10, 6, 3, 1, 1 };
    public static int numPosesToShow = 3;
    public static double strokeAlpha = 1;
    public static double fieldRotation = 0;

    public static void sendRobotPoses(Pose2d ...poses) {
        TelemetryPacket packet = new TelemetryPacket();
        addRobotPoseToCanvas(packet.fieldOverlay(), poses);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
    public static void addRobotPoseToCanvas(Canvas fieldOverlay, Pose2d...poses) {
        fieldOverlay.setRotation(Math.toRadians(fieldRotation));

        for (int i=0; i<Math.min(numPosesToShow, poses.length); i++) {
            Pose2d pose = poses[i];
            fieldOverlay.setAlpha(strokeAlpha);
            fieldOverlay.setStrokeWidth(strokeWidths[i % strokeWidths.length]);
            fieldOverlay.setStroke(colors[i % colors.length]);
            double x = pose.position.x, y = pose.position.y, heading = pose.heading.toDouble();
            fieldOverlay.strokeCircle(x, y, radii[i]);
            fieldOverlay.strokeLine(x, y, x + radii[i] * Math.cos(heading), y + radii[i] * Math.sin(heading));
        }
    }
    public static void addRobotPoseToCanvas(Canvas fieldOverlay, ArrayList<Pose2d> poses) {
        fieldOverlay.setRotation(Math.toRadians(fieldRotation));

        for (int i=0; i<Math.min(numPosesToShow, poses.size()); i++) {
            Pose2d pose = poses.get(i);
            fieldOverlay.setAlpha(strokeAlpha);
            fieldOverlay.setStrokeWidth(strokeWidths[i % strokeWidths.length]);
            fieldOverlay.setStroke(colors[i % colors.length]);
            double x = pose.position.x, y = pose.position.y, heading = pose.heading.toDouble();
            fieldOverlay.strokeCircle(x, y, radii[i]);
            fieldOverlay.strokeLine(x, y, x + radii[i] * Math.cos(heading), y + radii[i] * Math.sin(heading));
        }
    }
}
