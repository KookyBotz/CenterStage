package org.firstinspires.ftc.teamcode.common.commandbase.autocommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.centerstage.ClawSide;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ArmFloatCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ArmLiftCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ExtensionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.PivotCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.PivotStateCommand;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;

public class FirstDepositCommand extends SequentialCommandGroup {
    public FirstDepositCommand() {
        super(
                new WaitCommand(1250),
                new ArmCommand(3.05),
                new ArmFloatCommand(false),
                new ArmLiftCommand(0.3),
                new WaitCommand(500),
                new ExtensionCommand(535),
                new PivotCommand(0.75),
                new WaitCommand(750),
                new ClawCommand(IntakeSubsystem.ClawState.INTERMEDIATE, ClawSide.BOTH),
                new WaitCommand(250),
                new ClawCommand(IntakeSubsystem.ClawState.OPEN, ClawSide.BOTH),
                new WaitCommand(100),
                new ExtensionCommand(0),
                new WaitCommand(150),
                new ArmCommand(0.2),
                new ArmLiftCommand(0.73),
                new ClawCommand(IntakeSubsystem.ClawState.CLOSED, ClawSide.BOTH),
                new PivotStateCommand(IntakeSubsystem.PivotState.FLAT),
                new PivotCommand(0.46)
        );
    }
}
