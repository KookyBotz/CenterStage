package org.firstinspires.ftc.teamcode.common.commandbase.teleopcommand;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ScoreCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.util.InverseKinematics;

public class HeightChangeCommand extends SequentialCommandGroup {
    public HeightChangeCommand(RobotHardware robot, int increment) {
        super(
                new InstantCommand(() -> robot.extension.incrementBackdropHeight(increment)),
                new InstantCommand(() -> InverseKinematics.calculateTarget(3, robot.extension.getBackdropHeight())),
                new ConditionalCommand(
                        new ScoreCommand(robot, 3, robot.extension.getBackdropHeight()),
                        new InstantCommand(),
                        () -> Globals.IS_SCORING
                )
        );
    }
}
