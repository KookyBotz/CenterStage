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

    public static double TRACK_WIDTH = 10.79960; //10.73495;
    public static double FORWARD_OFFSET = 4.49008; //4.69008; //4.65059

    public final DoubleSupplier positionLeft, positionRight, positionFront;

    private final ElapsedTime IMUTimer;

    private Pose offset = new Pose();

    private final double VOLTAGE_TO_INCHES = 73.529411;
    private final Pose SENSOR_POSE = new Pose(2, 4.5, 0);
    public final LinkedList<Double> distanceMeasurements = new LinkedList<>();

    public FusedLocalizer() {
        super(Arrays.asList(
                new Pose2d(0, TRACK_WIDTH / 2, 0), // left
                new Pose2d(0, -TRACK_WIDTH / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        positionLeft = () -> -robot.doubleSubscriber(Sensors.SensorType.POD_LEFT) / 0.99747368421052631578947368421053;
        positionRight = () -> -robot.doubleSubscriber(Sensors.SensorType.POD_RIGHT) / 0.99747368421052631578947368421053;
        positionFront = () -> -robot.doubleSubscriber(Sensors.SensorType.POD_FRONT) / 0.99157894736842105263157894736842;

        IMUTimer = new ElapsedTime();
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
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

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
        if (velocity == null || Math.abs(velocity.getHeading()) > Math.PI / 4) return;

        // imu shenanigans
        // throttle to save loop times
        if (IMUTimer.milliseconds() > 500) {
            IMUTimer.reset();
            robot.readIMU();
            setHeading(robot.getAngle());
            robotPose = getPose();
        }

        if (robotPose.y < 24 && robotPose.y > -4) distanceMeasurements.clear();
        if (Math.abs(robotPose.heading) > Math.PI / 6) distanceMeasurements.clear();
        if (Math.hypot(velocity.getX(), velocity.getY())> 24) distanceMeasurements.clear();

        double distSensorReading;
        if (robotPose.x >= 0) {
            distSensorReading = 70.5 - calculateDistance(robot.rightDistSensor.getVoltage(), robotPose.heading);
        } else {
            distSensorReading = calculateDistance(robot.leftDistSensor.getVoltage(), robotPose.heading) - 70.5;
        }

        // obstructions can only make the sensor think we are closer
        if (Math.abs(distSensorReading) < Math.abs(robotPose.x)
                || Math.abs(distSensorReading) - Math.abs(robotPose.x) < 2) {
            distanceMeasurements.add(distSensorReading);
        } else {
            distanceMeasurements.clear();
        }

        if (distanceMeasurements.size() >= 100) {
            setLateral(distanceMeasurements.stream()
                    .mapToDouble(Double::doubleValue)
                    .average()
                    .orElse(0));
            distanceMeasurements.clear();
        }

    }

    public double calculateDistance(double v, double h) {
        double dist = (v * VOLTAGE_TO_INCHES) + SENSOR_POSE.y;
        double cosd = Math.cos(h) * dist;
        double offset = Math.sin(h) * SENSOR_POSE.x;
        return cosd - offset;
    }

    public Pose getPose() {
        Pose2d pose = getPoseEstimate();
        return new Pose(-pose.getY(), pose.getX(), pose.getHeading()).add(offset);
    }

    public void setPose(Pose pose) {
        offset = offset.subt(getPose()).add(pose);
    }

    public void setLateral(double x) {
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