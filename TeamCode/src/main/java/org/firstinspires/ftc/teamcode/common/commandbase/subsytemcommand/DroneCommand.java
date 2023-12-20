package org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.DroneSubsystem;

public class DroneCommand extends InstantCommand {
    public DroneCommand(DroneSubsystem.DroneState state) {
        super(
                () -> RobotHardware.getInstance().drone.updateState(state)
        );
    }
}
