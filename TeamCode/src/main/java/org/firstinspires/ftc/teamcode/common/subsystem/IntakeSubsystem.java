package org.firstinspires.ftc.teamcode.common.subsystem;

import static org.firstinspires.ftc.teamcode.common.hardware.Globals.INTAKE_CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.common.hardware.Globals.INTAKE_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.common.hardware.Globals.INTAKE_PIVOT_FLAT;
import static org.firstinspires.ftc.teamcode.common.hardware.Globals.INTAKE_PIVOT_SCORING;
import static org.firstinspires.ftc.teamcode.common.hardware.Globals.INTAKE_PIVOT_STORED;


import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.common.centerstage.ClawSide;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.util.wrappers.KSubsystem;
import org.jetbrains.annotations.NotNull;

/**
 * 2x Micro+ (left/right)
 * 2x Micro+ (pivot)
 * 4x Digital (left-+/right-+)
 */
public class IntakeSubsystem extends KSubsystem {

    private RobotHardware robot;

    private ClawState clawState;
    private PivotState pivotState;

    private double pivotAngle = 0.0;
    private double pivotTargetAngle = 0.0;

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


    }

    public void updateState(@NotNull ClawState state, @NotNull ClawSide side) {
        double position = getClawStatePosition(state);
        this.clawState = state;
        switch(side) {
            case LEFT:
                robot.intakeClawLeftServo.setPosition(position);
                break;
            case RIGHT:
                robot.intakeClawRightServo.setPosition(position);
                break;
            case BOTH:
                robot.intakeClawLeftServo.setPosition(position);
                robot.intakeClawRightServo.setPosition(position);
                break;
        }
    }

    public void updateState(@NotNull PivotState state) {
        this.pivotState = state;
//        switch(state) {
//            case FLAT:
//                setPivot(INTAKE_PIVOT_FLAT);
//            case STORED:
//                setPivot(INTAKE_PIVOT_STORED);
//            case SCORING:
//                setPivot(INTAKE_PIVOT_SCORING);
//        }
    }

    @Override
    public void periodic() {

        // TODO: fix
        double armAngle = 0.0; // some radian boi

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
        // TODO: add the rest of the + servos
        pivotAngle = robot.intakePivotLeftServo.getPosition();
    }

    @Override
    public void write() {
        //TODO: write to all servos here
        setPivot(pivotTargetAngle);
    }

    @Override
    public void reset() {
        //TODO: Determine if anything needs to be put here
        updateState(PivotState.STORED);
    }


    // TODO: Configure this
    //TODO add implementation for angle writing to servos
    private void setPivot(double angle) {
//        robot.pivotLeftServo.setPosition(position);
//        robot.pivotRightServo.setPosition(position);
    }

    private double getClawStatePosition(ClawState state) {
        switch (state) {
            case CLOSED:
                return INTAKE_CLAW_CLOSED;
            case OPEN:
                return INTAKE_CLAW_OPEN;
            default:
                return 0.0;
        }
    }
}
