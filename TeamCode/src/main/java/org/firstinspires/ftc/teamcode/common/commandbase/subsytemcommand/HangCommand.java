package org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.HangSubsystem;

public class HangCommand extends InstantCommand {
    public HangCommand(HangSubsystem.HangState state) {
        super(
                () -> RobotHardware.getInstance().hang.updateState(state)
        );
    }
}
