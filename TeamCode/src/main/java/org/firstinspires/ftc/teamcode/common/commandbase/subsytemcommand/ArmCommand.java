package org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class ArmCommand extends InstantCommand {
    public ArmCommand(double target) {
        super(
                () -> RobotHardware.getInstance().armActuator.setTargetPosition(target)
        );
    }
}
