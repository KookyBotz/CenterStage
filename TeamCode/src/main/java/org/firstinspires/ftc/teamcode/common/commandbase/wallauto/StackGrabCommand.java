package org.firstinspires.ftc.teamcode.common.commandbase.wallauto;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.centerstage.ClawSide;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ExtensionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.PivotStateCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.vision.Location;

public class StackGrabCommand extends SequentialCommandGroup {
    public StackGrabCommand() {
        super(
                new ExtensionCommand(375),
                new WaitCommand(350),
                new ClawCommand(IntakeSubsystem.ClawState.CLOSED, Globals.ALLIANCE == Location.RED ? ClawSide.LEFT : ClawSide.RIGHT),
                new WaitCommand(250),
                new PivotStateCommand(IntakeSubsystem.PivotState.STORED),
                new ExtensionCommand(0),
                new ClawCommand(IntakeSubsystem.ClawState.CLOSED, ClawSide.BOTH)
        );
    }
}
