package org.firstinspires.ftc.teamcode.common.commandbase.teleopcommand;

import com.arcrobotics.ftclib.command.ConditionalCommand;
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
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.hardware.Sensors;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;

public class DepositRetractionCommand extends ConditionalCommand {
    public DepositRetractionCommand() {
        super(
                new SequentialCommandGroup(
                        new ExtensionCommand(0),
                        new WaitCommand((long) RobotHardware.getInstance().extensionEncoder.getPosition()),
                        new ArmCommand(0),
                        new InstantCommand(Globals::stopScoring),
                        new ClawCommand(IntakeSubsystem.ClawState.CLOSED, ClawSide.BOTH),
                        new PivotStateCommand(IntakeSubsystem.PivotState.STORED)
//                        new PivotCommand(0)
                ),
                new InstantCommand(),
                () -> Globals.IS_SCORING
        );
    }

}
