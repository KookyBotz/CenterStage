package org.firstinspires.ftc.teamcode.common.subsystem;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WSubsystem;

public class HangSubsystem extends WSubsystem {
    private final RobotHardware robot = RobotHardware.getInstance();

    private HangState hangState = HangState.DISABLED;

    public HangSubsystem() {
        if (robot.leftHang != null && robot.rightHang != null) this.updateState(HangState.DISABLED);
    }

    public void updateState(HangState state) {
        this.hangState = state;
    }

    @Override
    public void periodic() {
        switch (hangState) {
            case EXTENDING:
                setPower(1);
            case RETRACTING:
                setPower(-1);
            case DISABLED:
                setPower(0);
        }
    }

    @Override
    public void read() {

    }

    @Override
    public void write() {

    }

    public void setPower(double power) {
        robot.leftHang.setPower(power);
        robot.rightHang.setPower(power);
    }

    @Override
    public void reset() {
        this.updateState(HangState.DISABLED);
    }

    public HangState getHangState() {
        return this.hangState;
    }

    public enum HangState {
        EXTENDING,
        RETRACTING,
        DISABLED
    }
}
