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

public class IntakeExtendCommand extends SequentialCommandGroup {
    public IntakeExtendCommand(int target) {
        super(
                new InstantCommand(Globals::startIntaking),
                new ExtensionCommand(target),
                new PivotStateCommand(IntakeSubsystem.PivotState.FLAT),
                new PivotCommand(target > 150 ? 0.52 : 0.53),
                new WaitCommand(250),
                new ArmFloatCommand(true),
                new ClawCommand(IntakeSubsystem.ClawState.OPEN, ClawSide.BOTH)
        );
    }
}
