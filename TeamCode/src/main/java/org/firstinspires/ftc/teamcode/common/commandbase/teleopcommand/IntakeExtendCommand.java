package org.firstinspires.ftc.teamcode.common.commandbase.teleopcommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.centerstage.ClawSide;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ExtensionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.PivotCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.PivotStateCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;

public class IntakeExtendCommand extends SequentialCommandGroup {
    public IntakeExtendCommand(int target) {
        super(
                new InstantCommand(() -> Globals.retract()),
                new ArmCommand(-0.03),
                new ExtensionCommand(target),
                new PivotStateCommand(IntakeSubsystem.PivotState.FLAT),
                new PivotCommand(0.46),
                new WaitCommand(250),
                new ClawCommand(IntakeSubsystem.ClawState.OPEN, ClawSide.BOTH)
        );
    }
}
