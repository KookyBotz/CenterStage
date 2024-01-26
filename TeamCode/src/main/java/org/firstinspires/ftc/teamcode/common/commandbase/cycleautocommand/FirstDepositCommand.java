package org.firstinspires.ftc.teamcode.common.commandbase.cycleautocommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.centerstage.ClawSide;
import org.firstinspires.ftc.teamcode.common.centerstage.ScoringPositions;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ArmLiftCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ExtensionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.PivotCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.PivotStateCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;

public class FirstDepositCommand extends SequentialCommandGroup {
    public FirstDepositCommand() {
        super(
                new ClawCommand(IntakeSubsystem.ClawState.INTERMEDIATE, ScoringPositions.YELLOW_PIXEL_POSITIONS[Globals.getTargetIndex()].getClawSide()),
                new WaitCommand(250),
                new ClawCommand(IntakeSubsystem.ClawState.OPEN, ScoringPositions.YELLOW_PIXEL_POSITIONS[Globals.getTargetIndex()].getClawSide()),
                new WaitCommand(250),
                new ExtensionCommand(75),
                new WaitCommand(50),
                new ArmCommand(0.2),
                new ArmLiftCommand(0.735),
                new ClawCommand(IntakeSubsystem.ClawState.CLOSED, ClawSide.BOTH),
                new PivotStateCommand(IntakeSubsystem.PivotState.STORED)
        );
    }
}
