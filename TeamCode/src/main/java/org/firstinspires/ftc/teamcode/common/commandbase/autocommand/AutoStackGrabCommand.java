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

public class AutoStackGrabCommand extends SequentialCommandGroup {
    public AutoStackGrabCommand(RobotHardware robot, ExtensionSubsystem extension, IntakeSubsystem intake) {
        super(
                new ClawCommand(intake, IntakeSubsystem.ClawState.CLOSED, ClawSide.LEFT),
                new WaitCommand(200),
                new InstantCommand(()->robot.pitchActuator.setMotionProfileTargetPosition(3)),
                new WaitCommand(100),
                new InstantCommand(()->robot.extensionActuator.setMotionProfileTargetPosition(0))

        );
    }
}
