package org.firstinspires.ftc.teamcode.utils.misc;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public final class Drawing {

    public static double width = 13;
    public static double length = 13;
    private Drawing() {}

    public static void drawRobot(Canvas c, Pose2d t) {
        double halfW = width * 0.5;
        double halfL = length * 0.5;

        double x = t.position.x;
        double y = t.position.y;
        double a = t.heading.toDouble();

        double cos = Math.cos(a);
        double sin = Math.sin(a);

        // Robot-frame corners (forward = +x, left = +y)
        double[] rx = {  halfL,  halfL, -halfL, -halfL };
        double[] ry = {  halfW, -halfW, -halfW,  halfW };

        double[] xPoints = new double[4];
        double[] yPoints = new double[4];

        for (int i = 0; i < 4; i++) {
            xPoints[i] = x + rx[i] * cos - ry[i] * sin;
            yPoints[i] = y + rx[i] * sin + ry[i] * cos;
        }

        c.setStrokeWidth(1);
        c.strokePolygon(xPoints, yPoints);
        c.strokeLine(x, y, x + cos * halfL, y + sin * halfW);
    }

    public static void drawCirclePose(Canvas c, Pose2d t, double radius) {

        c.setStrokeWidth(1);
        c.strokeCircle(t.position.x, t.position.y, radius);

        Vector2d halfv = t.heading.vec().times(0.5 * radius);
        Vector2d p1 = t.position.plus(halfv);
        Vector2d p2 = p1.plus(halfv);
        c.strokeLine(p1.x, p1.y, p2.x, p2.y);
    }
}
