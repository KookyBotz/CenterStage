package org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.util.InverseKinematics;

public class ScoreCommand extends SequentialCommandGroup {
    public ScoreCommand(RobotHardware robot, double distance, int height) {
        super(
                new InstantCommand(() -> robot.armActuator.setTargetPosition(InverseKinematics.t_angle)),
                new InstantCommand(() -> robot.extensionActuator.setTargetPosition(InverseKinematics.t_extension))
        );
    }
}
