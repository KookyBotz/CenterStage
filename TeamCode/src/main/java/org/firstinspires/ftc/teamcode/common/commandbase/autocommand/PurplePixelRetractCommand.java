package org.firstinspires.ftc.teamcode.common.commandbase.autocommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.centerstage.ClawSide;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;

public class PurplePixelRetractCommand extends SequentialCommandGroup {
    public PurplePixelRetractCommand(RobotHardware robot, ClawSide clawSide) {
        super(
                new InstantCommand(() -> robot.armActuator.setMotionProfileTargetPosition(0.0)),
                new InstantCommand(() -> robot.extensionActuator.setMotionProfileTargetPosition(0)),
                new InstantCommand(() -> robot.intake.updateState(IntakeSubsystem.PivotState.STORED)),
                new WaitCommand(250),
                new ClawCommand(robot.intake, IntakeSubsystem.ClawState.CLOSED, clawSide)
        );
    }
}
