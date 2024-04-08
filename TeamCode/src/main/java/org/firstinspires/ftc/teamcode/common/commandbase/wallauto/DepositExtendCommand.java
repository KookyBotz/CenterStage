package org.firstinspires.ftc.teamcode.common.commandbase.wallauto;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ArmFloatCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ExtensionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.PivotCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.PivotStateCommand;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;

public class DepositExtendCommand extends SequentialCommandGroup {
    public DepositExtendCommand(double angle, double pivot){
        super(
                new WaitUntilCommand(()-> RobotHardware.getInstance().localizer.getPose().y < -24),
                new ArmCommand(angle),
                new ArmFloatCommand(false),
                new WaitCommand(250),
                new PivotStateCommand(IntakeSubsystem.PivotState.FLAT),
                new PivotCommand(pivot),
                new ExtensionCommand(150),
                new WaitCommand(500)
        );
    }
}
