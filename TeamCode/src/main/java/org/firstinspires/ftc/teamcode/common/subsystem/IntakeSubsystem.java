package org.firstinspires.ftc.teamcode.common.subsystem;

import org.firstinspires.ftc.teamcode.common.centerstage.ClawSide;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.util.MathUtils;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WSubsystem;
import org.jetbrains.annotations.NotNull;

public class IntakeSubsystem extends WSubsystem {

    private final RobotHardware robot;

    private PivotState pivotState;

    public ClawState leftClaw = ClawState.CLOSED;
    public ClawState rightClaw = ClawState.CLOSED;

    public enum ClawState {
        CLOSED,
        INTERMEDIATE,
        OPEN
    }

    public enum PivotState {
        FLAT,
        STORED,
        SCORING
    }

    public IntakeSubsystem() {
        this.robot = RobotHardware.getInstance();

        updateState(ClawState.CLOSED, ClawSide.BOTH);
    }

    public void updateState(@NotNull ClawState state, @NotNull ClawSide side) {
        double position = getClawStatePosition(state, side);
        switch(side) {
            case LEFT:
                robot.intakeClawLeftServo.setPosition(position);
                this.leftClaw = state;
                break;
            case RIGHT:
                robot.intakeClawRightServo.setPosition(position);
                this.rightClaw = state;
                break;
            case BOTH:
                robot.intakeClawLeftServo.setPosition(getClawStatePosition(state, ClawSide.LEFT));
                this.leftClaw = state;
                robot.intakeClawRightServo.setPosition(getClawStatePosition(state, ClawSide.RIGHT));
                this.rightClaw = state;
                break;
        }
    }

    public void updateState(@NotNull PivotState state) {
        this.pivotState = state;
    }

    @Override
    public void periodic() {
        if (pivotState == PivotState.SCORING) {
            double targetAngle = ((robot.armActuator.getPosition()) - ((((robot.armActuator.getPosition() < Math.PI / 2) ? 5 : 13) * Math.PI) / 18));
            robot.intakePivotActuator.setTargetPosition(MathUtils.clamp(MathUtils.map(targetAngle, 0, Math.PI / 2, 0.47, 0.96), 0.075, 0.96));
        }
    }

    @Override
    public void read() {

    }

    @Override
    public void write() {
        robot.intakePivotActuator.write();
    }

    @Override
    public void reset() {
        updateState(PivotState.STORED);
    }

    private double getClawStatePosition(ClawState state, ClawSide side) {
        switch (side) {
            case LEFT:
                switch (state) {
                    case CLOSED:
                        return 0.12;
                    case INTERMEDIATE:
                        return 0.16;
                    case OPEN:
                        return 0.39;
                    default:
                        return 0.0;
                }
            case RIGHT:
                switch (state) {
                    case CLOSED:
                        return 0.54;
                    case INTERMEDIATE:
                        return 0.58;
                    case OPEN:
                        return 0.89;
                    default:
                        return 0.0;
                }
            default:
                return 0.0;
        }
    }

    public ClawState getClawState(ClawSide side) {
        if (side == ClawSide.BOTH) return (robot.intake.rightClaw == (IntakeSubsystem.ClawState.CLOSED) || (robot.intake.leftClaw == IntakeSubsystem.ClawState.CLOSED)) ? ClawState.CLOSED : ClawState.OPEN;
        return (side == ClawSide.LEFT) ? leftClaw : rightClaw;
    }
}
