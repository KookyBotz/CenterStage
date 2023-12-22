package org.firstinspires.ftc.teamcode.common.commandbase.autocommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.centerstage.ClawSide;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ArmFloatCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ArmLiftCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.PivotCommand;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;

public class PurplePixelDepositCommand extends SequentialCommandGroup {
    public PurplePixelDepositCommand() {
        super(
                new ClawCommand(IntakeSubsystem.ClawState.OPEN, ClawSide.RIGHT),
                new WaitCommand(150),
                new ArmCommand(0.2),
                new WaitCommand(500),
                new ArmLiftCommand(0.675),
                new WaitCommand(500),
                new ArmFloatCommand(true),
                new PivotCommand(0.49)
        );
    }
}
