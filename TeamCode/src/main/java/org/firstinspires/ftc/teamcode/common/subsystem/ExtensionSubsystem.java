package org.firstinspires.ftc.teamcode.common.subsystem;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.util.MathUtils;
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

    private final RobotHardware robot;
    private int backdropHeight = 0;

    public ExtensionSubsystem() {
        this.robot = RobotHardware.getInstance();
    }

    @Override
    public void periodic() {
        double liftTicks = robot.extensionEncoder.getPosition();
        robot.armActuator.updateFeedforward(liftTicks / 560.0);
        robot.extensionActuator.setOffset(-(robot.armActuator.getPosition() / Math.PI) * 50);

        robot.armActuator.periodic();
        robot.extensionActuator.periodic();
    }

    @Override
    public void read() {
        robot.armActuator.read();
        robot.extensionActuator.read();
    }

    @Override
    public void write() {
        robot.armActuator.write();
        robot.extensionActuator.write();
    }

    @Override
    public void reset() {

    }

    public int getBackdropHeight() {
        return backdropHeight;
    }

    public void incrementBackdropHeight(int amount) {
        this.backdropHeight = (int) MathUtils.clamp(getBackdropHeight() + amount, 0, 11);
    }

    public void setBackdropHeight(int amount) {
        this.backdropHeight = (int) MathUtils.clamp(amount, 0, 11);
    }
}
