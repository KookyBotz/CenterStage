package org.firstinspires.ftc.teamcode.common.drive.drivetrain;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.drivebase.RobotDrive;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Vector2D;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.util.MathUtils;
import org.firstinspires.ftc.teamcode.common.util.wrappers.KSubsystem;

import java.util.Locale;

public class MecanumDrivetrain extends KSubsystem implements Drivetrain {
    private RobotHardware robot = RobotHardware.getInstance();

    public DcMotorEx frontLeft, frontRight, backLeft, backRight;

    double[] ws = new double[4];

    public MecanumDrivetrain(DcMotorEx frontLeft, DcMotorEx frontRight, DcMotorEx backLeft, DcMotorEx backRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
    }

    @Override
    public void set(Pose pose) {
        double strafeSpeed = MathUtils.clamp(pose.x, -1, 1);
        double forwardSpeed = MathUtils.clamp(pose.heading, -1, 1);
        double turnSpeed = MathUtils.clamp(pose.y, -1, 1);

        Vector2d input = new Vector2d(strafeSpeed, forwardSpeed);
        input = input.rotateBy(-0);

        double theta = input.angle();

        double[] wheelSpeeds = new double[4];
        wheelSpeeds[0] = Math.sin(theta + Math.PI / 4);
        wheelSpeeds[1] = Math.sin(theta - Math.PI / 4);
        wheelSpeeds[2] = Math.sin(theta - Math.PI / 4);
        wheelSpeeds[3] = Math.sin(theta + Math.PI / 4);

        normalize(wheelSpeeds, input.magnitude());

        wheelSpeeds[0] += turnSpeed;
        wheelSpeeds[1] -= turnSpeed;
        wheelSpeeds[2] += turnSpeed;
        wheelSpeeds[3] -= turnSpeed;

        normalize(wheelSpeeds);

        ws[0] = wheelSpeeds[0];
        ws[1] = wheelSpeeds[1];
        ws[2] = wheelSpeeds[2];
        ws[3] = wheelSpeeds[3];
    }

    @Override
    public void periodic() {
        // Nothing here
    }

    @Override
    public void read() {
        // Nothing here
    }

    @Override
    public void write() {
//        robot.dtFrontLeftMotor.setPower(ws[0]);
//        robot.dtFrontRightMotor.setPower(ws[1]);
//        robot.dtBackLeftMotor.setPower(ws[2]);
//        robot.dtBackRightMotor.setPower(ws[3]);

        frontLeft.setPower(ws[0]);
        frontRight.setPower(ws[1]);
        backLeft.setPower(ws[2]);
        backRight.setPower(ws[3]);
    }

    @Override
    public void reset() {

    }

    public String toString() {
//        return String.format(Locale.ENGLISH, "WS0: %f\nWS1: %f\nWS2: %f\nWS3: %f");
        return "WS0: "+ws[0] + "WS1: "+ws[1] +"WS2: "+ws[2] +"WS3: "+ws[3];

    }

    protected void normalize(double[] wheelSpeeds, double magnitude) {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);
        for (int i = 1; i < wheelSpeeds.length; i++) {
            double temp = Math.abs(wheelSpeeds[i]);
            if (maxMagnitude < temp) {
                maxMagnitude = temp;
            }
        }
        for (int i = 0; i < wheelSpeeds.length; i++) {
            wheelSpeeds[i] = (wheelSpeeds[i] / maxMagnitude) * magnitude;
        }

    }

    /**
     * Normalize the wheel speeds
     */
    protected void normalize(double[] wheelSpeeds) {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);
        for (int i = 1; i < wheelSpeeds.length; i++) {
            double temp = Math.abs(wheelSpeeds[i]);
            if (maxMagnitude < temp) {
                maxMagnitude = temp;
            }
        }
        if (maxMagnitude > 1) {
            for (int i = 0; i < wheelSpeeds.length; i++) {
                wheelSpeeds[i] = (wheelSpeeds[i] / maxMagnitude);
            }
        }

    }
}
