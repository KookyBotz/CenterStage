package org.firstinspires.ftc.teamcode.common.commandbase.teleopcommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.centerstage.ClawSide;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ArmFloatCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ExtensionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.PivotCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.PivotStateCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;

public class IntakeRetractCommand extends SequentialCommandGroup {
    public IntakeRetractCommand() {
        super(
                new InstantCommand(Globals::stopIntaking),
                new ClawCommand(IntakeSubsystem.ClawState.CLOSED, ClawSide.BOTH),
                new WaitCommand(250),
                new ArmFloatCommand(false),
                new ArmCommand(-0.035),
                new ExtensionCommand(0),
                new PivotStateCommand(IntakeSubsystem.PivotState.STORED),
                new PivotCommand(0)
        );
    }
}
