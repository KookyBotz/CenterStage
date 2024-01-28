package org.firstinspires.ftc.teamcode.common.commandbase.cycleautocommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ExtensionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.PivotCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.PivotStateCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.vision.Location;

public class PurplePixelExtendCommand extends SequentialCommandGroup {
    public PurplePixelExtendCommand() {
        super(
                new ArmCommand(-0.04),
                new ExtensionCommand(Globals.ROUTE == Location.STAGEDOOR ? 300 : 150),
                new PivotStateCommand(IntakeSubsystem.PivotState.FLAT),
                new PivotCommand(0.48)
        );
    }
}
