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
    private boolean scoring = false;
    private boolean updated = false;
    private boolean flip = false;

    public ExtensionSubsystem() {
        this.robot = RobotHardware.getInstance();
    }

    @Override
    public void periodic() {
        double liftTicks = robot.extensionEncoder.getPosition();
        robot.pitchActuator.updateFeedforward(liftTicks / 560.0);
        robot.extensionActuator.setOffset(-(robot.pitchActuator.getPosition() / Math.PI) * 50);

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

    public int getBackdropHeight() {
        return backdropHeight;
    }

    public void incrementBackdropHeight(int amount) {
        this.backdropHeight = (int) MathUtils.clamp(getBackdropHeight() + amount, 0, 11);
        updated = false;
    }

    public void setBackdropHeight(int amount) {
        this.backdropHeight = (int) MathUtils.clamp(amount, 0, 11);
        updated = false;
    }

    public void setScoring(boolean scoring) {
        this.scoring = scoring;
    }

    public boolean getScoring() {
        return this.scoring;
    }

    public void setUpdated(boolean updated) {
        this.updated = updated;
    }

    public void setFlip(boolean flip) {
        this.flip = flip;
    }
}
