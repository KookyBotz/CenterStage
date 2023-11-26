package org.firstinspires.ftc.teamcode.common.commandbase.autocommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;

public class YellowPixelExtendCommand extends SequentialCommandGroup {
    public YellowPixelExtendCommand(RobotHardware robot, ExtensionSubsystem extension, IntakeSubsystem intake) {
        super(
                new InstantCommand(() -> extension.setScoring(true)),
                new InstantCommand(() -> extension.setFlip(false)),
                new InstantCommand(() -> intake.updateState(IntakeSubsystem.PivotState.SCORING)),
                new InstantCommand(() -> robot.pitchActuator.setMotionProfileTargetPosition(0.22)),
                new WaitCommand(350),
                new InstantCommand(() -> robot.extensionActuator.setMotionProfileTargetPosition(330)),
                new WaitUntilCommand(() -> robot.pitchActuator.hasReached() && robot.extensionActuator.hasReached())
        );
    }
}
