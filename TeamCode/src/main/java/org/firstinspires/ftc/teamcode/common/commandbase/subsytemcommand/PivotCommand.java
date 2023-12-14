package org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class PivotCommand extends InstantCommand {
    public PivotCommand(double position) {
        super(
                () -> RobotHardware.getInstance().intakePivotActuator.setTargetPosition(position)
        );
    }
}
