package org.firstinspires.ftc.teamcode.common.drive.drivetrain;

import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Vector2D;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.util.wrappers.KSubsystem;

public class MecanumDrivetrain extends KSubsystem implements Drivetrain {
    private RobotHardware robot = RobotHardware.getInstance();

    double[] ws = new double[4];

    @Override
    public void set(Pose pose) {
        Vector2D input = new Vector2D(pose.x, pose.y).rotate(-robot.getAngle());
        double theta = input.angle();

        double[] wheelSpeeds = new double[4];
        ws[0] = Math.sin(theta + Math.PI / 4);
        ws[1] = Math.sin(theta - Math.PI / 4);
        ws[2] = Math.sin(theta - Math.PI / 4);
        ws[3] = Math.sin(theta + Math.PI / 4);

        double maxMagnitude = Math.abs(wheelSpeeds[0]);
        for (int i = 1; i < wheelSpeeds.length; i++) {
            double temp = Math.abs(wheelSpeeds[i]);
            if (maxMagnitude < temp) {
                maxMagnitude = temp;
            }
        }
        for (int i = 0; i < wheelSpeeds.length; i++) {
            wheelSpeeds[i] = (wheelSpeeds[i] / maxMagnitude) * input.magnitude();
        }

        ws[0] += pose.heading;
        ws[1] -= pose.heading;
        ws[2] += pose.heading;
        ws[3] -= pose.heading;

        maxMagnitude = Math.abs(wheelSpeeds[0]);
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
        robot.frontLeftMotor.setPower(ws[0]);
        robot.frontRightMotor.setPower(ws[1]);
        robot.backLeftMotor.setPower(ws[2]);
        robot.backRightMotor.setPower(ws[3]);
    }

    @Override
    public void reset() {

    }
}
