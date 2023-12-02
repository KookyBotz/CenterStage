package org.firstinspires.ftc.teamcode.common.util;

public class InverseKinematics {
    public static double t_extension = 0.0;
    public static double t_angle = 0.0;

    public static void calculateTarget(double backdrop_distance, int target_backdrop_height) {
        double distance_sensor_input = 3;
        double backdrop_front = 1.41;
        double perp_backdrop_distance = 0.0;
        double height_backdrop_distance = 0.0;
        double ticks_per_inch = 26;

        double d_retracted = 11.7416379163791637916379;
        double d_extended = 33.44;

        double t_y = (7.25 + 22.5 * ((target_backdrop_height) / 10.0)) + perp_backdrop_distance * Math.sin(Math.PI / 6) + height_backdrop_distance * Math.sin(2 * Math.PI / 3) + (18/25.4);
        double t_x = -(((20.125 * t_y) / 30.0) - backdrop_front + distance_sensor_input + perp_backdrop_distance * Math.cos(Math.PI / 6) + height_backdrop_distance * Math.cos(2 * Math.PI / 3));

        double x_c = 2.48; // gear center_x
        double y_c = 3.43; // gear center_y
        double r = 1.279; // distance from gear center to center of the boxtube

        double dx = t_x - x_c;
        double dy = t_y - y_c;
        double len = Math.sqrt(dx * dx + dy * dy);

        double x_t = x_c + r * dy / len;
        double y_t = y_c - r * dx / len;

        double diff_y = t_y - x_t;
        double diff_x = t_x - y_t;

        t_angle = Math.atan2(diff_y, diff_x);
        t_extension = MathUtils.clamp(Math.hypot(diff_x, diff_y), d_retracted, d_extended); //TODO replace 500 with the max distance in inches // 25 ticks per inch, 20 inches

        t_extension -= d_retracted;
        t_extension *= ticks_per_inch;
    }
}
