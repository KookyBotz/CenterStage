package org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;

public class PivotStateCommand extends InstantCommand {
    public PivotStateCommand(IntakeSubsystem.PivotState state) {
        super(
                () -> RobotHardware.getInstance().intake.updateState(state)
        );
    }
}
