package org.firstinspires.ftc.teamcode.common.drive.drivetrain;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.util.wrappers.KSubsystem;

public class MecanumDrivetrain extends MecanumDrive {
    private RobotHardware robot = RobotHardware.getInstance();

    public MecanumDrivetrain() {
        super(frontLeft, frontRight, backLeft, backRight);
    }

    public void set(Pose pose) {
        super.driveFieldCentric(pose.x, pose.y, pose.heading, robot.getAngle());
    }
}
