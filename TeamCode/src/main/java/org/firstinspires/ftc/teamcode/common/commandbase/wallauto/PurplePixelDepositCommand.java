package org.firstinspires.ftc.teamcode.common.commandbase.wallauto;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.centerstage.ClawSide;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ArmFloatCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ExtensionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.PivotStateCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.vision.Location;

public class PurplePixelDepositCommand extends SequentialCommandGroup {
    public PurplePixelDepositCommand() {
        super(
                new ClawCommand(IntakeSubsystem.ClawState.OPEN, Globals.ALLIANCE == Location.BLUE ? ClawSide.LEFT : ClawSide.RIGHT),
                new WaitCommand(50),
                new ArmFloatCommand(false),
                new ExtensionCommand(0),
                new ArmCommand(0),
                new WaitCommand(100),
                new ClawCommand(IntakeSubsystem.ClawState.CLOSED, ClawSide.BOTH),
                new WaitCommand(100),
                new PivotStateCommand(IntakeSubsystem.PivotState.STORED)
        );
    }
}
