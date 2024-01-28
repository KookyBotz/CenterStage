package org.firstinspires.ftc.teamcode.common.commandbase.cycleautocommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.centerstage.ScoringPositions;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ArmFloatCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ArmLiftCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ExtensionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.PivotStateCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;

public class FirstDepositExtendCommand extends SequentialCommandGroup {
    public FirstDepositExtendCommand() {
        super(
//                new WaitCommand(700),
                new ArmCommand(ScoringPositions.YELLOW_PIXEL_POSITIONS[Globals.getTargetIndex()].getTargetArmAngle()),
                new ArmFloatCommand(false),
                new ArmLiftCommand(0.3),
                new WaitCommand(200),
                new PivotStateCommand(IntakeSubsystem.PivotState.SCORING),
                new WaitCommand(300),
                new ExtensionCommand(ScoringPositions.YELLOW_PIXEL_POSITIONS[Globals.getTargetIndex()].getTargetExtension()),
                new WaitCommand(750)
        );
    }
}
