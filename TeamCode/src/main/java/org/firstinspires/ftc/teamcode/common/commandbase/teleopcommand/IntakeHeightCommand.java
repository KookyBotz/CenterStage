package org.firstinspires.ftc.teamcode.common.commandbase.teleopcommand;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ArmFloatCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ArmLiftCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class IntakeHeightCommand extends SequentialCommandGroup {

    public IntakeHeightCommand(RobotHardware robot, int increment) {
        super(
                new InstantCommand(() -> robot.extension.incrementStackHeight(increment)),
                new ConditionalCommand(
                        new InstantCommand(() -> robot.armLiftServo.setPosition(robot.extension.getStackHeight())),
                        new InstantCommand(),
                        () -> Globals.IS_INTAKING
                )
        );
    }
}
