package org.firstinspires.ftc.teamcode.common.commandbase.cycleautocommand;

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
import org.firstinspires.ftc.teamcode.common.vision.Location;

public class PurplePixelDepositCommand extends SequentialCommandGroup {
    public PurplePixelDepositCommand() {
        super(
                new ClawCommand(IntakeSubsystem.ClawState.OPEN, Globals.ALLIANCE == Location.BLUE ? ClawSide.RIGHT : ClawSide.LEFT),
                new WaitCommand(25),
                new ExtensionCommand(0),
                new ArmCommand(0.2),
                new PivotStateCommand(IntakeSubsystem.PivotState.STORED),

                new WaitCommand(250)
        );
    }
}
