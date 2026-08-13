package org.firstinspires.ftc.teamcode.utils.math;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Arrays;

// helps for concise, easy printing to telemetry
public class MathUtils {
    // normalizes between [0, 2pi]
    public static double angleNormRad(double rad) {
        if(rad >= 0 && rad < Math.PI * 2)
            return rad;
        return (rad % (Math.PI * 2) + Math.PI * 2) % (Math.PI * 2);
    }
    // normalizes between (-pi, pi]
    public static double angleNormDeltaRad(double rad) {
        rad = angleNormRad(rad);
        if (rad > Math.PI)
            rad -= 2 * Math.PI;
        return rad;
    }
//    public static void main(String[] args) {
//        double step = Math.toRadians(40);
//
//        for (double i=0; i<2 * Math.PI; i+=step) {
//            for (double j=0; j<2 * Math.PI; j+=step) {
//                System.out.println(Math.toDegrees(i) + " - " + Math.toDegrees(j) + " = " + Math.toDegrees(angleRadDiff(i, j)));
//
//            }
//        }
//    }
    public static double averageAngle(double a1, double a2) {
        Vector2d v1 = new Vector2d(Math.cos(a1), Math.sin(a1));
        Vector2d v2 = new Vector2d(Math.cos(a2), Math.sin(a2));
        return vecAngle(v1.plus(v2));
    }
    public static double angleRadDiff(Vector2d v2, Vector2d v1) {
        double a2 = Math.atan2(v2.y, v2.x);
        double a1 = Math.atan2(v1.y, v1.x);
        return angleNormDeltaRad(a2 - a1);
    }
    public static double v1ToV2Angle(Vector2d v1, Vector2d v2) {
        Vector2d v1ToV2 = v2.minus(v1);
        return Math.atan2(v1ToV2.y, v1ToV2.x);
    }
    public static double vecAngle(Vector2d v) {
        return Math.atan2(v.y, v.x);
    }
    public static double vecMag(Vector2d v) {
        return Math.hypot(v.x, v.y);
    }
    public static double vecDist(Vector2d v1, Vector2d v2) { return vecMag(v2.minus(v1)); }
    public static Vector2d getAverage(ArrayList<Vector2d> vecs) {
        double totalX = 0;
        double totalY = 0;
        for (Vector2d vec : vecs) {
            totalX += vec.x;
            totalY += vec.y;
        }
        return new Vector2d(totalX / vecs.size(), totalY / vecs.size());
    }
    public static String format1(Number num) {
        return format(num, 1);
    }
    public static String format2(Number num) {
        return format(num, 2);
    }
    public static String format3(Number num) { return format(num, 3); }
    public static String format(Number num, int decimalPlaces) {
        StringBuilder decimals = new StringBuilder();
        for (int i=0; i<decimalPlaces; i++)
            decimals.append("#");
        DecimalFormat customDf = new DecimalFormat("#." + decimals);
        return customDf.format(num);
    }
    public static String format2(double[] nums) {
        StringBuilder total = new StringBuilder();
        for (double num : nums)
            total.append(format2(num)).append(", ");
        return total.substring(0, total.length() - 2);
    }
    public static String format3(double[] nums) {
        StringBuilder total = new StringBuilder();
        for (double num : nums)
            total.append(format3(num)).append(", ");
        return total.substring(0, total.length() - 2);
    }
    public static String formatRad2(double rad) {
        return format2(Math.toDegrees(rad));
    }
    public static String formatRad3(double rad) {
        return format3(Math.toDegrees(rad));
    }
    public static String formatRad4(double rad) {
        return format(Math.toDegrees(rad), 4);
    }

    public static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }
    // start, end, current
    public static double inverseLerp(double a, double b, double x) {
        return (x - a) / (b - a);
    }
    public static double covariance(double[] a, double[] b) {
        if(a.length != b.length)
            throw new IllegalArgumentException("lists are of length " + a.length + " & " + b.length + "respectively; they must match");
        double sum = 0;
        double meanA = mean(a);
        double meanB = mean(b);
        for(int i = 0; i < a.length; i++) {
            sum += (a[i] - meanA) * (b[i] - meanB);
        }
        return sum / (a.length - 1);
    }
    public static double mean(double[] l) {
        double sum = 0;
        for (double v : l) sum += v;
        return sum / l.length;
    }

    public static String formatPose1(Pose2d pose) {
        if (pose == null)
            return "null";
        return "(" + format1(pose.position.x) + ", " + format1(pose.position.y) + ", " + format1(Math.toDegrees(pose.heading.toDouble())) + ")";
    }
    public static String formatPose2(Pose2d pose) {
        if (pose == null)
            return "null";
        return "(" + format2(pose.position.x) + ", " + format2(pose.position.y) + ", " + format2(Math.toDegrees(pose.heading.toDouble())) + ")";
    }
    public static String formatPose3(Pose2d pose) {
        if (pose == null)
            return "null";
        return "(" + format3(pose.position.x) + ", " + format3(pose.position.y) + ", " + format3(Math.toDegrees(pose.heading.toDouble())) + ")";
    }
    public static String formatVec0(Vector2d v) {
        if (v == null)
            return "null";
        return "(" + format(v.x, 0) + ", " + format(v.y, 0) + ")";
    }
    public static String formatVec1(Vector2d v) {
        if (v == null)
            return "null";
        return "(" + format(v.x, 1) + ", " + format(v.y, 1) + ")";
    }
    public static String formatVec2(Vector2d v) {
        if (v == null)
            return "null";
        return "(" + format2(v.x) + ", " + format2(v.y) + ")";
    }
    public static String formatVec3(Vector2d vec) {
        if(vec == null)
            return "null";
        return "(" + format3(vec.x) + ", " + format3(vec.y) + ")";
    }
    public static Pose2d createPose(double[] pose) {
        if (pose.length != 3)
            throw new IllegalArgumentException("cannot call createPose on " + Arrays.toString(pose) + " - must contain EXACTLY 3 elements");
        return new Pose2d(pose[0], pose[1], Math.toRadians(pose[2]));
    }
    public static Pose2d createPose(double[] pose, double headingDeg) {
        return new Pose2d(pose[0], pose[1], Math.toRadians(headingDeg));
    }
    public static Pose2d createInvertedPose(double[] pose) {
        if(pose.length != 3)
            throw new IllegalArgumentException("cannot call createInvertedPose on " + Arrays.toString(pose) + " - must contain EXACTLY 3 elements");
        return new Pose2d(pose[0], -pose[1], Math.toRadians(-pose[2]));
    }
    public static Pose2d createInvertedPose(double[] pos, double headingDeg) {
        return new Pose2d(pos[0], -pos[1], Math.toRadians(-headingDeg));
    }
    public static Vector2d createVec(double[] vec) {
        if(vec.length != 2)
            throw new IllegalArgumentException("cannot call createVec on " + Arrays.toString(vec) + " - must contain EXACTLY 2 elements");
        return new Vector2d(vec[0], vec[1]);
    }
    public static Vector2d createInvertedVec(double[] vec) {
        if(vec.length != 2)
            throw new IllegalArgumentException("cannot call createInvertedVec on " + Arrays.toString(vec) + " - must contain EXACTLY 2 elements");
        return new Vector2d(vec[0], -vec[1]);
    }
}
