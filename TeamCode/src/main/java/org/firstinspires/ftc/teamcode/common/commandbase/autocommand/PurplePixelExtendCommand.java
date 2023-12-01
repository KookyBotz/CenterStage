package org.firstinspires.ftc.teamcode.common.commandbase.autocommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;

public class PurplePixelExtendCommand extends SequentialCommandGroup {
    public PurplePixelExtendCommand(RobotHardware robot, ExtensionSubsystem extension, IntakeSubsystem intake) {
        super(
                new InstantCommand(() -> robot.pitchActuator.setMotionProfileTargetPosition(3.3)),
                new WaitCommand(250),
                new InstantCommand(() -> intake.updateState(IntakeSubsystem.PivotState.FLAT)),
                new InstantCommand(() -> robot.intakePivotActuator.setTargetPosition(0.35)),
                new WaitUntilCommand(() -> robot.pitchActuator.hasReached() && robot.extensionActuator.hasReached())
        );
    }
}
