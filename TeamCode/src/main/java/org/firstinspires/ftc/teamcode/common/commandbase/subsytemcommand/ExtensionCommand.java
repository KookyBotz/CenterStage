package org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class ExtensionCommand extends InstantCommand {
    public ExtensionCommand(int target) {
        super(
                () -> RobotHardware.getInstance().extensionActuator.setTargetPosition(target)
        );
    }
}
