package org.firstinspires.ftc.teamcode.common.commandbase.wallauto;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.centerstage.ClawSide;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ArmFloatCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ArmLiftCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ExtensionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.PivotCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.PivotStateCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.vision.Location;

public class StackGrabCommand extends SequentialCommandGroup {
    public StackGrabCommand(double height, double angle) {
        super(
                new ArmLiftCommand(height),
                new ArmFloatCommand(true),
                new PivotStateCommand(IntakeSubsystem.PivotState.FLAT),
                new PivotCommand(angle),
                new WaitCommand(250),
                new ClawCommand(IntakeSubsystem.ClawState.AUTO, Globals.ALLIANCE == Location.RED ? ClawSide.RIGHT : ClawSide.LEFT),
                new ClawCommand(IntakeSubsystem.ClawState.OPEN, Globals.ALLIANCE == Location.RED ? ClawSide.LEFT : ClawSide.RIGHT),
                new WaitCommand(250),
                new ExtensionCommand(375),
                new WaitCommand(750),
                new ClawCommand(IntakeSubsystem.ClawState.CLOSED, Globals.ALLIANCE == Location.RED ? ClawSide.LEFT : ClawSide.RIGHT),
                new WaitCommand(250),
                new PivotStateCommand(IntakeSubsystem.PivotState.STORED),
                new ExtensionCommand(0),
                new ClawCommand(IntakeSubsystem.ClawState.CLOSED, ClawSide.BOTH)
        );
    }
}
