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
                new PivotStateCommand(IntakeSubsystem.PivotState.FLAT),
                new ExtensionCommand(target),
                new PivotCommand(target < 200? 0.535:0.545),
                new WaitCommand(100),
                new ArmFloatCommand(true),
                new ClawCommand(IntakeSubsystem.ClawState.OPEN, ClawSide.BOTH)
        );
    }
}
