package org.firstinspires.ftc.teamcode.common.commandbase.preloadautocommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.centerstage.ClawSide;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ExtensionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.PivotCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.PivotStateCommand;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;

public class YellowPixelRetractCommand extends SequentialCommandGroup {
    public YellowPixelRetractCommand() {
        super(
                new ExtensionCommand(0),
                new WaitCommand(50),
                new PivotStateCommand(IntakeSubsystem.PivotState.STORED),
                new ClawCommand(IntakeSubsystem.ClawState.CLOSED, ClawSide.BOTH)
        );
    }
}