package org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;

public class PivotCommand extends InstantCommand {
    public PivotCommand(IntakeSubsystem intake, IntakeSubsystem.PivotState state) {
        super(
                () -> intake.updateState(state)
        );
    }
}
