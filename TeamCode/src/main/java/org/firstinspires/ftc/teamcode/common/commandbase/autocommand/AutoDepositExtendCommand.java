package org.firstinspires.ftc.teamcode.common.commandbase.autocommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;

public class AutoDepositExtendCommand extends SequentialCommandGroup {
    public AutoDepositExtendCommand(RobotHardware robot, ExtensionSubsystem extension, IntakeSubsystem intake, double length, double angle) {
        super(
                new WaitCommand(1650),
                new InstantCommand(() -> robot.pitchActuator.setMotionProfileTargetPosition(angle)),
                new WaitCommand(850),
                new InstantCommand(() -> robot.extensionActuator.setMotionProfileTargetPosition(length)),
                new InstantCommand(() -> robot.intakePivotActuator.setTargetPosition(0.36)),
                new WaitUntilCommand(()->robot.extensionActuator.hasReached())
        );
    }
}
