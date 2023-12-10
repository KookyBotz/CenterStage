package org.firstinspires.ftc.teamcode.common.commandbase.autocommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.centerstage.ClawSide;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;

public class AutoStackExtendCommand extends SequentialCommandGroup {
    public AutoStackExtendCommand(RobotHardware robot, ExtensionSubsystem extension, IntakeSubsystem intake, double length, double angle) {
        super(
                new InstantCommand(()->robot.armActuator.setMotionProfileTargetPosition(angle)),
                new InstantCommand(() -> robot.intakePivotActuator.setTargetPosition(0.55)),
                new WaitCommand(1000),
                new ClawCommand(intake, IntakeSubsystem.ClawState.OPEN, ClawSide.LEFT),
                new InstantCommand(()->robot.extensionActuator.setMotionProfileTargetPosition(length)),
                new WaitUntilCommand(()->robot.extensionActuator.hasReached() && robot.armActuator.hasReached())
        );
    }
}
