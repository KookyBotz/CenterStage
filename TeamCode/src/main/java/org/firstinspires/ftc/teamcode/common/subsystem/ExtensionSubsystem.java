package org.firstinspires.ftc.teamcode.common.subsystem;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
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

    public ExtensionSubsystem() {
        this.robot = RobotHardware.getInstance();
    }

    @Override
    public void periodic() {
        double liftTicks = robot.extensionEncoder.getPosition();
        robot.pitchActuator.updateFeedforward(liftTicks / 560.0);

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
}
