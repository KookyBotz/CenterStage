package org.firstinspires.ftc.teamcode.common.commandbase.preloadautocommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.PivotCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.PivotStateCommand;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;

public class PurplePixelExtendCommand extends SequentialCommandGroup {
    public PurplePixelExtendCommand() {
        super(
                new ArmCommand(3.15),
                new WaitCommand(250),
                new PivotStateCommand(IntakeSubsystem.PivotState.FLAT),
                new PivotCommand(0.35),
                new WaitCommand(1000)
        );
    }
}
