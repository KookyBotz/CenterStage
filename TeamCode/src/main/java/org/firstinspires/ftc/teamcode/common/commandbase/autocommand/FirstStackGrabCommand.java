package org.firstinspires.ftc.teamcode.common.commandbase.autocommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.centerstage.ClawSide;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ArmLiftCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ExtensionCommand;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;

public class FirstStackGrabCommand extends SequentialCommandGroup {
    public FirstStackGrabCommand() {
        super(
                new ExtensionCommand(550),
                new WaitCommand(500),
                new ClawCommand(IntakeSubsystem.ClawState.CLOSED, ClawSide.RIGHT),
                new WaitCommand(250),
                new ArmLiftCommand(0.63),
                new WaitCommand(100),
                new ExtensionCommand(0)
        );
    }
}
