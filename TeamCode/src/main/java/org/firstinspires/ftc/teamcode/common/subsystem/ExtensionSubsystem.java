package org.firstinspires.ftc.teamcode.common.subsystem;

import android.util.Pair;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.util.MathUtils;
import org.firstinspires.ftc.teamcode.common.util.ScoringHeights;
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
    private int backdropHeight = 1;
    private boolean scoring = false;
    private boolean updated = false;

    public ExtensionSubsystem() {
        this.robot = RobotHardware.getInstance();
    }

    @Override
    public void periodic() {
        double liftTicks = robot.extensionEncoder.getPosition();
        robot.pitchActuator.updateFeedforward(liftTicks / 560.0);
//        robot.extensionActuator.setTargetPositionOffset((robot.pitchActuator.getPosition() / Math.PI) * 50);
        robot.extensionActuator.setOffset(-(robot.pitchActuator.getPosition() / Math.PI) * 50);

        if (this.scoring && !updated) {
            // extension range angle math or whatever for given backdrop height selection
            Pair currentHeight = ScoringHeights.HEIGHTS[backdropHeight - 1];
            robot.pitchActuator.setMotionProfileTargetPosition(((Double) currentHeight.first).doubleValue());
            robot.extensionActuator.setMotionProfileTargetPosition(((Integer) currentHeight.second).doubleValue());
            updated = true;
        }

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

    public double getBackdropHeight() {
        return backdropHeight;
    }

    public void incrementBackdropHeight(int amount) {
        this.backdropHeight = (int) MathUtils.clamp(getBackdropHeight() + amount, 1, 12);
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
}
