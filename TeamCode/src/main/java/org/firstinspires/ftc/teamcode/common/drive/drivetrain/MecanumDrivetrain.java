package org.firstinspires.ftc.teamcode.common.drive.drivetrain;

import com.arcrobotics.ftclib.drivebase.RobotDrive;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Vector2D;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WSubsystem;

import java.util.Arrays;

public class MecanumDrivetrain extends WSubsystem implements Drivetrain {
    private final RobotHardware robot = RobotHardware.getInstance();

    double[] pws = new double[4];
    double[] ws = new double[4];

    @Override
    public void set(Pose pose) {
        set(pose, 0);
    }

    public void set(double strafeSpeed, double forwardSpeed,
                    double turnSpeed, double gyroAngle) {

        Vector2D input = new Vector2D(strafeSpeed, forwardSpeed).rotate(-gyroAngle);

        strafeSpeed = Range.clip(input.x, -1, 1);
        forwardSpeed = Range.clip(input.y, -1, 1);
        turnSpeed = Range.clip(turnSpeed, -1, 1);

        double[] wheelSpeeds = new double[4];

        wheelSpeeds[RobotDrive.MotorType.kFrontLeft.value] = forwardSpeed + strafeSpeed + turnSpeed;
        wheelSpeeds[RobotDrive.MotorType.kFrontRight.value] = forwardSpeed - strafeSpeed - turnSpeed;
        wheelSpeeds[RobotDrive.MotorType.kBackLeft.value] = (forwardSpeed - strafeSpeed + turnSpeed);
        wheelSpeeds[RobotDrive.MotorType.kBackRight.value] = (forwardSpeed + strafeSpeed - turnSpeed);
        // 1.06, 1.04

        if (Globals.IS_AUTO) {
            // feedforward & voltage comp
            double correction = 12 / robot.getVoltage();
            for (int i = 0; i < wheelSpeeds.length; i++) {
                wheelSpeeds[i] = Math.abs(wheelSpeeds[i]) < 0.01 ?
                        wheelSpeeds[i] * correction :
                        (wheelSpeeds[i] + Math.signum(wheelSpeeds[i]) * 0.085) * correction;
            }

        }

        double max = 1;
        for (double wheelSpeed : wheelSpeeds) max = Math.max(max, Math.abs(wheelSpeed));


        if (max > 1) {
            wheelSpeeds[RobotDrive.MotorType.kFrontLeft.value] /= max;
            wheelSpeeds[RobotDrive.MotorType.kFrontRight.value] /= max;
            wheelSpeeds[RobotDrive.MotorType.kBackLeft.value] /= max;
            wheelSpeeds[RobotDrive.MotorType.kBackRight.value] /= max;
        }

        ws[0] = wheelSpeeds[0];
        ws[1] = wheelSpeeds[1];
        ws[2] = wheelSpeeds[2];
        ws[3] = wheelSpeeds[3];

    }

    public void set(Pose pose, double angle) {
        set(pose.x, pose.y, pose.heading, angle);
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
        if (Math.abs(ws[0] - pws[0]) > 0.005) {
            robot.dtFrontLeftMotor.setPower(ws[0]);
            pws[0] = ws[0];
        }
        if (Math.abs(ws[1] - pws[1]) > 0.005) {
            robot.dtFrontRightMotor.setPower(ws[1]);
            pws[1] = ws[1];
        }
        if (Math.abs(ws[2] - pws[2]) > 0.005) {
            robot.dtBackLeftMotor.setPower(ws[2]);
            pws[2] = ws[2];
        }
        if (Math.abs(ws[3] - pws[3]) > 0.005) {
            robot.dtBackRightMotor.setPower(ws[3]);
            pws[3] = ws[3];
        }
    }


    @Override
    public void reset() {

    }

    public String toString() {
        return "WS0: " + ws[0] + "WS1: " + ws[1] + "WS2: " + ws[2] + "WS3: " + ws[3];

    }
}
