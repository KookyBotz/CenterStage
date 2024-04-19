package org.firstinspires.ftc.teamcode.common.commandbase.wallauto;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.centerstage.ClawSide;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ArmFloatCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ArmLiftCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.PivotCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.PivotStateCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.vision.Location;

public class StackSetupCommand extends SequentialCommandGroup {
    public StackSetupCommand(double height, double angle) {
        super(
                new WaitUntilCommand(() -> RobotHardware.getInstance().localizer.getPose().y > 24),
                new ArmCommand(0.2),
                new ArmLiftCommand(height),
                new WaitCommand(250),
                new ArmFloatCommand(true),
                new PivotStateCommand(IntakeSubsystem.PivotState.FLAT),
                new PivotCommand(angle),
                new ClawCommand(IntakeSubsystem.ClawState.AUTO, Globals.ALLIANCE == Location.RED ? ClawSide.RIGHT : ClawSide.LEFT),
                new ClawCommand(IntakeSubsystem.ClawState.OPEN, Globals.ALLIANCE == Location.RED ? ClawSide.LEFT : ClawSide.RIGHT),
                new WaitCommand(100)
        );
    }
}
