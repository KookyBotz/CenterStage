package org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.centerstage.ClawSide;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;

public class ClawCommand extends InstantCommand {
    public ClawCommand(IntakeSubsystem intake, IntakeSubsystem.ClawState state, ClawSide side) {
        super(
                () -> intake.updateState(state, side)
        );
    }
}
