package org.firstinspires.ftc.teamcode.common.commandbase.cycleautocommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.commandbase.drivecommand.PositionCommand;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.vision.Location;
import org.firstinspires.ftc.teamcode.common.vision.PreloadDetectionPipeline;

public class PreloadDetectionCommand extends SequentialCommandGroup {

    public PreloadDetectionCommand(Pose targetPose) {
        super(
                new InstantCommand(() -> {
                    Pose targetPose2 = targetPose;
                    System.out.println(targetPose2);
                    Location preloadLocation = RobotHardware.getInstance().preloadDetectionPipeline.getPreloadedZone();
                    System.out.println(preloadLocation);
                    if (preloadLocation == Location.LEFT) {
                        System.out.println("HERE3");
                        targetPose2.x -= 4;
                    }

                    System.out.println(targetPose2);
                    System.out.println("HERE HERE");
                    System.out.println(preloadLocation);

                }),
                new PositionCommand((RobotHardware.getInstance().preloadDetectionPipeline.getPreloadedZone() == Location.LEFT) ? targetPose.add(new Pose(-4, 0, 0)) : targetPose)
        );
    }
}
