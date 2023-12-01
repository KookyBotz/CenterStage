package org.firstinspires.ftc.teamcode.common.commandbase.autocommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.centerstage.ClawSide;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;

public class YellowPixelRetractCommand extends SequentialCommandGroup {
    public YellowPixelRetractCommand(RobotHardware robot, ExtensionSubsystem extension, IntakeSubsystem intake) {
        super(
                new InstantCommand(() -> robot.extensionActuator.setMotionProfileTargetPosition(0)),
                new WaitCommand(50),
                new InstantCommand(() -> extension.setScoring(false)),
                new InstantCommand(() -> extension.setFlip(false)),
                new InstantCommand(() -> robot.intakePivotActuator.setTargetPosition(0.0475)),
                new InstantCommand(() -> intake.updateState(IntakeSubsystem.ClawState.CLOSED, ClawSide.RIGHT))
        );
    }
}
