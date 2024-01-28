package org.firstinspires.ftc.teamcode.common.commandbase.cycleautocommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.drivecommand.PositionCommand;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.vision.StackPipeline;

import java.util.function.DoubleSupplier;

public class StackRelocalizeCommand extends SequentialCommandGroup {
    public StackRelocalizeCommand(StackPipeline stackPipeline, Pose targetPose) {
        super (
//                new WaitCommand(50), // TODO: lower this. due to processing times with the pipeline, giving this a lenient time period
                new InstantCommand(() -> {
                    double correction = stackPipeline.getStrafeCorrection();
                    System.out.println("CORRECTION342    " + correction);
                    Pose currentPose = RobotHardware.getInstance().localizer.getPose();
                    System.out.println("CORRECTION342    " + currentPose);
                    if (Math.abs(correction) > 0.5) RobotHardware.getInstance().localizer.setPose(new Pose(currentPose.x + correction, currentPose.y, currentPose.heading));

                    System.out.println("CORRECTION342    " + RobotHardware.getInstance().localizer.getPose());
                }),
//                new WaitCommand(100),
                new PositionCommand(targetPose)
        );
    }
}
