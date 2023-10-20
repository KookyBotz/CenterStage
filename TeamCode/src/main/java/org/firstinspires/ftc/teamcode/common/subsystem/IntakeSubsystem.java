package org.firstinspires.ftc.teamcode.common.subsystem;

import static org.firstinspires.ftc.teamcode.common.hardware.Globals.INTAKE_CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.common.hardware.Globals.INTAKE_CLAW_OPEN;

import org.firstinspires.ftc.teamcode.common.centerstage.ClawSide;
import org.firstinspires.ftc.teamcode.common.centerstage.Side;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WSubsystem;
import org.jetbrains.annotations.NotNull;

/**
 * Parts List:
 * <ul>
 *     <li>2x Micro+ (Left/Right)</li>
 *     <li>1x Micro+ Pivot</li>
 *     <li>1x Servo Pivot</li>
 *     <li>4x Digital Sensor(2 Left, 2 Right)</li>
 * </ul>
 */
public class IntakeSubsystem extends WSubsystem {

    private final RobotHardware robot;

    private ClawState clawState;
    private PivotState pivotState;

    private boolean pixelLeftTop,    pixelRightTop,
                    pixelLeftBottom, pixelRightBottom;

    public enum ClawState {
        CLOSED,
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
        this.clawState = state;
        switch(side) {
            case LEFT:
                robot.intakeClawLeftServo.setPosition(position);
                break;
            case RIGHT:
                robot.intakeClawRightServo.setPosition(position);
                break;
            case BOTH:
                position = getClawStatePosition(state, ClawSide.LEFT);
                robot.intakeClawLeftServo.setPosition(position);
                position = getClawStatePosition(state, ClawSide.RIGHT);
                robot.intakeClawRightServo.setPosition(position);
                break;
        }
    }

    public void updateState(@NotNull PivotState state) {
        this.pivotState = state;
    }

    @Override
    public void periodic() {

        // TODO: fix
        double armAngle = 0.0; // some radian boi
        double pivotTargetAngle = 0.0;

        // TODO NOT FINAL. just some shitty pseudocode
        if (pivotState == PivotState.FLAT) {
            if (armAngle <= Math.PI / 2) {
                // angle = 0 - armAngle
                pivotTargetAngle = -armAngle;
            } else {
                // angle = pi + (pi - armAngle)
                pivotTargetAngle = Math.PI + (Math.PI - armAngle);
            }
        } else if (pivotState == PivotState.STORED) {
            // angle = pi/2 - armAngle
            pivotTargetAngle = (Math.PI / 2) - armAngle;
        } else if (pivotState == PivotState.SCORING) {
            // angle = 2pi / 3 + (pi - armAngle)
            pivotTargetAngle = (2 * Math.PI / 3) + (Math.PI - armAngle);
        }
    }

    @Override
    public void read() {

//        this.clawLeftPosition = robot.intakeClawLeftServo.getPosition();
//        this.clawRightPosition = robot.intakeClawRightServo.getPosition();



//        this.pixelLeftTop = robot.intakeClawLeftTop.getState();
//        this.pixelLeftBottom = robot.intakeClawLeftBottom.getState();
//        this.pixelRightTop = robot.intakeClawRightTop.getState();
//        this.pixelRightBottom = robot.intakeClawRightBottom.getState();

        robot.intakePivotActuator.read();
    }

    @Override
    public void write() {
        //TODO: write to all servos here
//        setPivot(pivotTargetAngle);
        robot.intakePivotActuator.write();
    }

    @Override
    public void reset() {
        //TODO: Determine if anything needs to be put here
        updateState(PivotState.STORED);
    }

    //TODO: Configure this
    //TODO add implementation for angle writing to servos
    private void setPivot(double angle) {
//        robot.pivotLeftServo.setPosition(position);
//        robot.pivotRightServo.setPosition(position);
    }

    private double getClawStatePosition(ClawState state, ClawSide side) {
        switch (side) {
            case LEFT:
                switch (state) {
                    case CLOSED:
                        return 0.09;
                    case OPEN:
                        return 0.4;
                    default:
                        return 0.0;
                }
            case RIGHT:
                switch (state) {
                    case CLOSED:
                        return 0.51;
                    case OPEN:
                        return 0.9;
                    default:
                        return 0.0;
                }
            default:
                return 0.0;
        }
    }
}
