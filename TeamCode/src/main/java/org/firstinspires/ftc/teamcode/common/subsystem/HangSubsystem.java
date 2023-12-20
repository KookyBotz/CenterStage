package org.firstinspires.ftc.teamcode.common.subsystem;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WSubsystem;

public class HangSubsystem extends WSubsystem {
    private final RobotHardware robot = RobotHardware.getInstance();

    private HangState hangState = HangState.DISABLED;

    public enum HangState {
        ACTIVE,
        DISABLED
    }

    public HangSubsystem() {
        if (robot.leftHang != null && robot.rightHang != null) this.updateState(HangState.DISABLED);
    }

    public void updateState(HangState state) {
        this.hangState = state;
    }

    @Override
    public void periodic() {

    }

    @Override
    public void read() {

    }

    @Override
    public void write() {

    }

    @Override
    public void reset() {
        this.updateState(HangState.DISABLED);
    }

    public HangState getHangState() {
        return this.hangState;
    }
}
