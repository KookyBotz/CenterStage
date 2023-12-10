package org.firstinspires.ftc.teamcode.common.commandbase.autocommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class YellowPixelExtendCommand extends SequentialCommandGroup {

    public YellowPixelExtendCommand(RobotHardware robot) {
        super(
                new InstantCommand(() -> robot.intakePivotActuator.setTargetPosition(0.35)),
                new InstantCommand(() -> robot.armActuator.setMotionProfileTargetPosition(0.22)),
                new WaitCommand(350),
                new InstantCommand(() -> robot.extensionActuator.setMotionProfileTargetPosition(420)),
                new WaitUntilCommand(() -> robot.armActuator.hasReached() && robot.extensionActuator.hasReached())
        );
    }
}
