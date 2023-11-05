package org.firstinspires.ftc.teamcode.common.drive.localizer;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;

import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleSupplier;

@Config
public class ThreeWheelLocalizer extends ThreeTrackingWheelLocalizer implements Localizer {

    private final RobotHardware robot;

    public static double TICKS_PER_REV = 4096;
    public static double WHEEL_RADIUS = 0.689;
    public static double GEAR_RATIO = 1;

    public static double TRACK_WIDTH = 10.787;
    public static double FORWARD_OFFSET = 4.252;

    public final DoubleSupplier positionLeft, positionRight, positionFront, imuAngle;

    public ThreeWheelLocalizer() {
        super(Arrays.asList(
                new Pose2d(0, TRACK_WIDTH / 2, 0), // left
                new Pose2d(0, -TRACK_WIDTH / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        robot = RobotHardware.getInstance();

        positionLeft = () -> robot.podLeft.getPosition();
        positionRight = () -> robot.podRight.getPosition();
        positionFront = () -> -robot.podFront.getPosition();
//        imuAngle = robot::getAngle;
        imuAngle = () -> 0.0;

//        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftEncoder"));
//        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightEncoder"));
//        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontEncoder"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
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
                encoderTicksToInches(robot.podLeft.getRawVelocity()),
                encoderTicksToInches(robot.podRight.getRawVelocity()),
                encoderTicksToInches(robot.podFront.getRawVelocity())
        );
    }

    @Override
    public void periodic() {
        super.update();
    }

    @Override
    public Pose getPos() {
        Pose2d pose = getPoseEstimate();
        return new Pose(pose.getX(), -pose.getY(), pose.getHeading());
    }

    @Override
    public void setPos(Pose pose) {

    }

    public double getHeading() {
        return 0.0;
    }

    public double getRawExternalHeading() {
        return 0;
    }

    public double getExternalHeadingVelocity() {
        return 0.0;
    }

    @Override
    public void setPoseEstimate(Pose2d pose) {
        super.setPoseEstimate(pose);
    }

    public Pose getNewPoseVelocity() {
        Pose2d a = super.getPoseVelocity();
        return new Pose(a.getX(), a.getY(), Math.toRadians(a.getHeading()));
    }
}