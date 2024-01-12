package org.firstinspires.ftc.teamcode.common.commandbase.autocommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ArmFloatCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ArmLiftCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.PivotCommand;

public class FirstStackSetupCommand extends SequentialCommandGroup {
    public FirstStackSetupCommand() {
        super(
                new ArmLiftCommand(0.68),
                new WaitCommand(400),
                new ArmFloatCommand(true),
                new PivotCommand(0.51)
        );
    }
}
