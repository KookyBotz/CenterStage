package org.firstinspires.ftc.teamcode.common.commandbase.cycleautocommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.centerstage.ClawSide;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ArmLiftCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ExtensionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.PivotCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.PivotStateCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.vision.Location;

public class FirstStackGrabCommand extends SequentialCommandGroup {
    public FirstStackGrabCommand() {
        super(
                new PivotStateCommand(IntakeSubsystem.PivotState.FLAT),
                new PivotCommand(0.51),
                new ExtensionCommand(550),
                new WaitCommand(500),
                new ClawCommand(IntakeSubsystem.ClawState.CLOSED, Globals.ALLIANCE == Location.BLUE ? ClawSide.RIGHT : ClawSide.LEFT),
                new WaitCommand(250),
                new ArmLiftCommand(0.63),
                new PivotStateCommand(IntakeSubsystem.PivotState.STORED),
                new ExtensionCommand(0)
        );
    }
}
