
package org.firstinspires.ftc.teamcode.common.commandbase.teleopcommand;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.HangSubsystem;

public class ActuateHangCommand extends ConditionalCommand {
    public ActuateHangCommand(double value) {
        super(
                new SequentialCommandGroup(
                        new InstantCommand(() -> RobotHardware.getInstance().leftHang.setPower(value)),
                        new InstantCommand(() -> RobotHardware.getInstance().rightHang.setPower(value))
                ),
                new WaitCommand(0),
                () -> RobotHardware.getInstance().hang.getHangState() == HangSubsystem.HangState.EXTENDING
        );
    }
}
