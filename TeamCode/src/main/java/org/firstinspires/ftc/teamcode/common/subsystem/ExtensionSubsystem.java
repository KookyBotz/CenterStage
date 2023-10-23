package org.firstinspires.ftc.teamcode.common.subsystem;

import android.util.Pair;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.util.MathUtils;
import org.firstinspires.ftc.teamcode.common.util.ScoringHeights;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WSubsystem;

/**
 * Parts List:
 * <ul>
 *     <li>1x Motor Pitch</li>
 *     <li>1x Motor Extension</li>
 *     <li>1x Analog Encoder Pitch</li>
 * </ul>
 */
@Config
public class ExtensionSubsystem extends WSubsystem {

    private RobotHardware robot;
    private int backdropHeight = 0;
    private boolean scoring = false;
    private boolean updated = false;

    public double t_angle = 0.0;
    public double t_extension = 0.0;
    public double diff_y = 0.0;
    public double diff_x =0.0;

    public ExtensionSubsystem() {
        this.robot = RobotHardware.getInstance();
    }

    @Override
    public void periodic() {
        double liftTicks = robot.extensionEncoder.getPosition();
        robot.pitchActuator.updateFeedforward(liftTicks / 560.0);
//        robot.extensionActuator.setTargetPositionOffset((robot.pitchActuator.getPosition() / Math.PI) * 50);
        robot.extensionActuator.setOffset(-(robot.pitchActuator.getPosition() / Math.PI) * 50);

        if (this.scoring && !updated) {
            // extension range angle math or whatever for given backdrop height selection
            Pair currentHeight = ScoringHeights.HEIGHTS[backdropHeight];
            robot.pitchActuator.setMotionProfileTargetPosition(((Double) currentHeight.first).doubleValue());
            robot.extensionActuator.setMotionProfileTargetPosition(((Integer) currentHeight.second).doubleValue());
            updated = true;
        }

        double distance_sensor_input = 12.0;
        double backdrop_front = 1.41;
        double perp_backdrop_distance = 0.75;
        double height_backdrop_distance = 3.0;

        double t_y = (7.25 + 22.5 * ((backdropHeight - 1) / 10.0)) + perp_backdrop_distance * Math.sin(Math.PI / 6) + height_backdrop_distance * Math.sin(2 * Math.PI / 3);
        double t_x = ((20.125 * t_y) / 30.0) - backdrop_front + distance_sensor_input + perp_backdrop_distance * Math.cos(Math.PI / 6) + height_backdrop_distance * Math.cos(2 * Math.PI / 3);

        double x_c = 2.48; // gear center_x
        double y_c = 3.43; // gear center_y
        double r = 1.57; // distance from gear center to base of the extension

        // IVK
        double x_m = (x_c + t_x) / 2;
        double y_m = (y_c + t_y) / 2;
        double r_d = 0.5 * Math.sqrt(Math.pow(t_x - x_c, 2) + Math.pow(t_y - y_c, 2));

        // ABC
        double A = 1 + Math.pow((2 * x_m - 2 * x_c) / (2 * y_m - 2 * y_c), 2);
        double B = -2 * x_m + 2 * x_c * Math.pow((2 * x_m - 2 * x_c) / (2 * y_m - 2 * y_c), 2)
                + (-2 * (2 * x_m - 2 * x_c) * (x_m * x_m - x_c * x_c + y_m * y_m - y_c * y_c - r * r + r_d * r_d)) / Math.pow(2 * y_m - 2 * y_c, 2);
        double C = x_m * x_m + y_m * y_m - r_d * r_d
                + Math.pow(((x_m * x_m - x_c * x_c + y_m * y_m - y_c * y_c - r * r + r_d * r_d) / (2 * y_m - 2 * y_c)) - y_m, 2);
        double disc = B * B - 4 * A * C;

        double x1 = (-B + Math.sqrt(disc)) / (2 * A);
        double x2 = (-B - Math.sqrt(disc)) / (2 * A);

        double y1 = (r_d * r_d - r * r + x_m * x_m - x_c * x_c + y_m * y_m - y_c * y_c - x1 * (2 * x_m - 2 * x_c)) / (2 * y_m - 2 * y_c);
        double y2 = (r_d * r_d - r * r + x_m * x_m - x_c * x_c + y_m * y_m - y_c * y_c - x2 * (2 * x_m - 2 * x_c)) / (2 * y_m - 2 * y_c);

        // the x and y component of the base of the extension
        double x_result = y1 > 0 ? x1 : x2;
        double y_result = y1 > 0 ? y1 : y2;

        // using t_x and t_y, we can calculate the angle and the distance of the extension
        diff_y = t_y - y_result;
        diff_x = t_x - x_result;

        t_angle = Math.atan2(diff_y, diff_x);
        t_extension = MathUtils.clamp(Math.hypot(diff_x, diff_y), 0, 20); //TODO replace 500 with the max distance in inches // 25 ticks per inch, 20 inches

        robot.pitchActuator.periodic();
        robot.extensionActuator.periodic();
    }

    @Override
    public void read() {
        robot.pitchActuator.read();
        robot.extensionActuator.read();
    }

    @Override
    public void write() {
        robot.pitchActuator.write();
        robot.extensionActuator.write();
    }

    @Override
    public void reset() {

    }

    public double getBackdropHeight() {
        return backdropHeight;
    }

    public void incrementBackdropHeight(int amount) {
        this.backdropHeight = (int) MathUtils.clamp(getBackdropHeight() + amount, 1, 10);
        updated = false;
    }

    public void setScoring(boolean scoring) {
        this.scoring = scoring;
    }

    public boolean getScoring() {
        return this.scoring;
    }

    public void setUpdated(boolean updated) {
        this.updated = updated;
    }
}
