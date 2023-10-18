package org.firstinspires.ftc.teamcode.common.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.profile.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.profile.ProfileState;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.util.wrappers.Actuator;
import org.firstinspires.ftc.teamcode.common.util.wrappers.KSubsystem;

/**
 * Parts List:
 * <ul>
 *     <li>1x Motor Pitch</li>
 *     <li>1x Motor Extension</li>
 *     <li>1x Analog Encoder Pitch</li>
 * </ul>
 */
@Config
public class ExtensionSubsystem extends KSubsystem {

    private RobotHardware robot;

    public static double P = 0.0;
    public static double I = 0.0;
    public static double D = 0.0;
    public static double F = 0.0;

    public static double targetPosition = 0.0;

    public ExtensionSubsystem() {
        this.robot = RobotHardware.getInstance();
    }

    @Override
    public void periodic() {
        robot.extensionPitchActuator.updatePID(P, I, D);
        robot.extensionPitchActuator.setFeedforward(Actuator.FeedforwardMode.ANGLE_BASED, F);
        robot.extensionPitchActuator.setTargetPosition(targetPosition);

        robot.extensionPitchActuator.periodic();
    }

    @Override
    public void read() {
        robot.extensionPitchActuator.read();
    }

    @Override
    public void write() {
        robot.extensionPitchActuator.write();
    }

    @Override
    public void reset() {

    }
}
