package org.firstinspires.ftc.teamcode.common.commandbase.wallauto;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.centerstage.ClawSide;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ArmFloatCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ArmLiftCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ExtensionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.PivotStateCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.vision.Location;

public class YellowPixelExtendCommand extends SequentialCommandGroup {
    public YellowPixelExtendCommand() {
        super(
                new WaitCommand(200),
                new ArmCommand(2.92),
                new WaitCommand(250),
                new PivotStateCommand(IntakeSubsystem.PivotState.SCORING),
                new WaitCommand(250),
                new ExtensionCommand(265),
                new ArmLiftCommand(0.71),
                new WaitCommand(750)
        );
    }
}
