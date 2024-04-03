package org.firstinspires.ftc.teamcode.common.drive.localizer;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;

import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.hardware.Sensors;

import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleSupplier;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    ^
 *    |
 *    | ( x direction)
 *    |
 *    v
 *    <----( y direction )---->

 *        (forward)
 *    /--------------\
 *    |     ____     |
 *    |     ----     |    <- Perpendicular Wheel
 *    |           || |
 *    |           || |    <- Parallel Wheel
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
public class TwoWheelLocalizer extends TwoTrackingWheelLocalizer {
    private final RobotHardware robot = RobotHardware.getInstance();

    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.689;
    public static double GEAR_RATIO = 1;

    public static double TRACK_WIDTH =      10.79960; //10.73495;
    public static double FORWARD_OFFSET =    4.69008; //4.65059

    public final DoubleSupplier positionLeft, positionRight, positionFront, imuAngle;

    public TwoWheelLocalizer() {
        super(Arrays.asList(
                new Pose2d(0, 0, 0), // left + right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        positionLeft = () -> -robot.doubleSubscriber(Sensors.SensorType.POD_LEFT) / 0.99747368421052631578947368421053;
        positionRight = () -> -robot.doubleSubscriber(Sensors.SensorType.POD_RIGHT) / 0.99747368421052631578947368421053;
        positionFront = () -> robot.doubleSubscriber(Sensors.SensorType.POD_FRONT) / 0.99157894736842105263157894736842;
        imuAngle = robot::getAngle;
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                (encoderTicksToInches(positionLeft.getAsDouble()) + encoderTicksToInches(positionRight.getAsDouble())) / 2.0,
                encoderTicksToInches(positionFront.getAsDouble())
        );
    }

    @Override
    public double getHeading() {
        return imuAngle.getAsDouble();
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(0.0, 0.0);
    }

    public void periodic() {
        super.update();
    }

    public Pose getPose() {
        Pose2d pose = getPoseEstimate();
        return new Pose(-pose.getY(), pose.getX(), pose.getHeading());
    }

    public void setPose(Pose pose) {
        super.setPoseEstimate(new Pose2d(pose.y, -pose.x, pose.heading));
    }


}