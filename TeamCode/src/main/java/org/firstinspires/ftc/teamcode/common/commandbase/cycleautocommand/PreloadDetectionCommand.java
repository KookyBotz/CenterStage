package org.firstinspires.ftc.teamcode.common.commandbase.cycleautocommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.centerstage.ScoringPositions;
import org.firstinspires.ftc.teamcode.common.commandbase.drivecommand.PositionCommand;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.vision.Location;
import org.firstinspires.ftc.teamcode.common.vision.PreloadDetectionPipeline;

public class PreloadDetectionCommand extends SequentialCommandGroup {

    public PreloadDetectionCommand() {
        super(
                new InstantCommand(() -> {
                    System.out.println("SUMMARY");
                    System.out.println("RANDOMIZATION: " + Globals.RANDOMIZATION);
                    System.out.println("SIDE: " + Globals.SIDE);
                    System.out.println("PRELOAD: " + Globals.PRELOAD);
                }),
                new PositionCommand(ScoringPositions.YELLOW_PIXEL_POSITIONS[Globals.getTargetIndex()].getTargetPose())
        );
    }
}
