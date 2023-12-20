package org.firstinspires.ftc.teamcode.common.commandbase.teleopcommand;

import com.arcrobotics.ftclib.command.ConditionalCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.DroneCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.HangCommand;
import org.firstinspires.ftc.teamcode.common.subsystem.DroneSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystem.HangSubsystem;

public class EndgameStateCommand extends ConditionalCommand {
    public EndgameStateCommand(boolean enabled) {
        super(
                new HangCommand(HangSubsystem.HangState.ACTIVE)
                        .alongWith(new DroneCommand(DroneSubsystem.DroneState.ARMED)),
                new HangCommand(HangSubsystem.HangState.DISABLED)
                        .alongWith(new DroneCommand(DroneSubsystem.DroneState.STORED)),
                () -> enabled
        );
    }
}
