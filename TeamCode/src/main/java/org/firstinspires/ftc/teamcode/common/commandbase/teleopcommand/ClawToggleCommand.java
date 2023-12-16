package org.firstinspires.ftc.teamcode.common.commandbase.teleopcommand;

import com.arcrobotics.ftclib.command.ConditionalCommand;

import org.firstinspires.ftc.teamcode.common.centerstage.ClawSide;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;

public class ClawToggleCommand extends ConditionalCommand {
    public ClawToggleCommand(RobotHardware robot, ClawSide clawSide) {
        super(
                new ClawCommand(IntakeSubsystem.ClawState.INTERMEDIATE, clawSide),
                new ClawCommand(IntakeSubsystem.ClawState.OPEN, clawSide),
                () -> (robot.intake.getClawState(clawSide) == (IntakeSubsystem.ClawState.CLOSED))
        );
    }
}
