package org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class ArmFloatCommand extends InstantCommand {
    public ArmFloatCommand(boolean f) {
        super(() -> RobotHardware.getInstance().armActuator.setFloat(f));
    }
}
