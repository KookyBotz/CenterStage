package org.firstinspires.ftc.teamcode.common.commandbase.teleopcommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ExtensionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.PivotStateCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.util.InverseKinematics;

public class DepositExtendCommand extends SequentialCommandGroup {
    public DepositExtendCommand() {
        super(
                new InstantCommand(() -> Globals.startScoring()),
                new InstantCommand(() -> InverseKinematics.calculateTarget(3, RobotHardware.getInstance().extension.getBackdropHeight())),
                new ArmCommand(InverseKinematics.t_angle),
                new WaitCommand(200),
                new PivotStateCommand(IntakeSubsystem.PivotState.SCORING),
                new WaitCommand(400),
                new ExtensionCommand((int) InverseKinematics.t_extension)
        );
    }
}
