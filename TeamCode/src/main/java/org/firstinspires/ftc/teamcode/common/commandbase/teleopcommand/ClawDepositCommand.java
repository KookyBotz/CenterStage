package org.firstinspires.ftc.teamcode.common.commandbase.teleopcommand;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.centerstage.ClawSide;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ExtensionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.PivotStateCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.util.InverseKinematics;

public class ClawDepositCommand extends SequentialCommandGroup {
    public ClawDepositCommand() {
        super(
                new ConditionalCommand(
                        new ClawCommand(IntakeSubsystem.ClawState.INTERMEDIATE, ClawSide.BOTH),
                        new ClawCommand(IntakeSubsystem.ClawState.OPEN, ClawSide.BOTH),
                        () -> (RobotHardware.getInstance().intake.rightClaw == (IntakeSubsystem.ClawState.CLOSED) || (RobotHardware.getInstance().intake.leftClaw == IntakeSubsystem.ClawState.CLOSED))
                )
        );
    }
}
