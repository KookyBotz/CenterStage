package org.firstinspires.ftc.teamcode.common.commandbase.autocommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class RelocalizeCommand extends SequentialCommandGroup {
    public RelocalizeCommand() {
        super(
                new WaitCommand(50),
                new InstantCommand(() -> {
                    Pose atag = RobotHardware.getInstance().getAprilTagPosition();
                    if (atag != null) RobotHardware.getInstance().localizer.setPose(atag);
                }),
                new WaitCommand(50)
        );
    }
}
