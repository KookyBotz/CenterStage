package org.firstinspires.ftc.teamcode.common.commandbase.cycleautocommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ArmFloatCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ArmLiftCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.PivotCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.PivotStateCommand;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;

public class FirstStackSetupCommand extends SequentialCommandGroup {
    public FirstStackSetupCommand() {
        super(
                new ArmLiftCommand(0.7),
                new WaitCommand(400),
                new ArmFloatCommand(true),
//                new PivotStateCommand(IntakeSubsystem.PivotState.FLAT),
                new PivotCommand(0.25)
//                new PivotCommand(0.51)
        );
    }
}
