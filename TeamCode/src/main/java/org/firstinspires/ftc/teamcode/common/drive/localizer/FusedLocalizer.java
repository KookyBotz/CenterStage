package org.firstinspires.ftc.teamcode.common.drive.localizer;

import androidx.annotation.NonNull;

import com.ThermalEquilibrium.homeostasis.Filters.Estimators.Estimator;
import com.ThermalEquilibrium.homeostasis.Filters.Estimators.LowPassEstimator;
import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.Filter;
import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.LowPassFilter;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.hardware.Sensors;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.Locale;
import java.util.function.DoubleSupplier;

@Config
public class FusedLocalizer extends ThreeTrackingWheelLocalizer {

    private final RobotHardware robot = RobotHardware.getInstance();

    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.689;
    public static double GEAR_RATIO = 1;

    public static double TRACK_WIDTH =   10.68950;
    public static double FORWARD_OFFSET = 4.32780;

    public final DoubleSupplier positionLeft, positionRight, positionFront;

    public double distanceMeasurement;

    private final ElapsedTime IMUTimer;
    private final ElapsedTime DTimer;

    private static final Pose SENSOR_POSE = new Pose(2, 4.5, 0);

    public FusedLocalizer() {
        super(Arrays.asList(
                new Pose2d(0, TRACK_WIDTH / 2, 0), // left
                new Pose2d(0, -TRACK_WIDTH / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        positionLeft = () -> -robot.doubleSubscriber(Sensors.SensorType.POD_LEFT) / 1.0085106382978723404255319148936;
        positionRight = () -> -robot.doubleSubscriber(Sensors.SensorType.POD_RIGHT) / 1.0085106382978723404255319148936;
        positionFront = () -> -robot.doubleSubscriber(Sensors.SensorType.POD_FRONT) / 0.99157894736842105263157894736842;

        IMUTimer = new ElapsedTime();
        DTimer = new ElapsedTime();
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(positionLeft.getAsDouble()),
                encoderTicksToInches(positionRight.getAsDouble()),
                encoderTicksToInches(positionFront.getAsDouble())
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {

        return Arrays.asList(
                encoderTicksToInches(robot.podLeft.getCorrectedVelocity()),
                encoderTicksToInches(robot.podRight.getCorrectedVelocity()),
                encoderTicksToInches(robot.podFront.getCorrectedVelocity())
        );
    }

    public void periodic() {
        super.update();

        Pose robotPose = getPose();
        Pose2d velocity = getPoseVelocity();

        // dont use imu if turning fast
//        if (velocity == null || Math.abs(velocity.getHeading()) > Math.PI / 4) return;

        // imu shenanigans
        // throttle to save loop times
//        if (IMUTimer.milliseconds() > 250) {
//            IMUTimer.reset();
//            robot.readIMU();
//            setHeading(robot.getAngle());
//            robotPose = getPose();
//        }

        distanceMeasurement = 0;
        if (robotPose.x >= 0) {
            distanceMeasurement = 70.5 - calculateDistance(robot.rightDistSensor.getVoltage(), robotPose.heading);
        } else {
            distanceMeasurement = calculateDistance(robot.leftDistSensor.getVoltage(), robotPose.heading) - 70.5;
        }

        // obstructions can only make the sensor think we are closer to the wall
//        if (Math.abs(distanceMeasurement) < Math.abs(robotPose.x)
//                || Math.abs(distanceMeasurement) - Math.abs(robotPose.x) < 2) {
//            setLateral(distanceMeasurement);
//            DTimer.reset();
//        }
    }

    public static double calculateDistance(double v, double h) {
        double VOLTAGE_TO_INCHES = 73.529411;
        double dist = (v * VOLTAGE_TO_INCHES) + SENSOR_POSE.y;
        double cosd = Math.cos(h) * dist;
        double offset = Math.sin(h) * SENSOR_POSE.x;
        return cosd - offset;
    }

    public Pose getPose() {
        Pose2d pose = getPoseEstimate();
        return new Pose(-pose.getY(), pose.getX(), pose.getHeading());
    }

    public void setPose(Pose pose) {
        super.setPoseEstimate(new Pose2d(pose.y, -pose.x, pose.heading));
    }

    public void setLateral(double x) {
        System.out.println("x: " + x);
        Pose p = getPose();
        p.x = x;
        setPose(p);
    }

    public void setHeading(double t) {
        Pose p = getPose();
        p.heading = t;
        setPose(p);
    }
}