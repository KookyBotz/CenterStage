package org.firstinspires.ftc.teamcode.common.util;

public class MathUtils {
    public static double clamp(double num, double min, double max) {
        return Math.max(min, Math.min(num, max));
    }

    public static double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
}