package org.firstinspires.ftc.teamcode.common.commandbase.teleopcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class SetHeightCommand extends InstantCommand {
    public SetHeightCommand(int height) {
        super(
                () -> RobotHardware.getInstance().extension.setBackdropHeight(height)
        );
    }
}
