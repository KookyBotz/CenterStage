package org.firstinspires.ftc.teamcode.common.commandbase.cycleautocommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.commandbase.drivecommand.PositionCommand;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.vision.StackPipeline;

public class StackRelocalizationCommand extends PositionCommand {
    public StackRelocalizationCommand(StackPipeline pipeline, Pose targetPose) {
        super(
                new Pose(targetPose.x + pipeline.getErrorCorrection(),targetPose.y,targetPose.heading)
        );
    }
}
