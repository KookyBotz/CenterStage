package org.firstinspires.ftc.teamcode.common.commandbase.autocommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class RelocalizeCommand extends InstantCommand {
    public RelocalizeCommand() {
        super(
                () -> {
                    Pose atag = RobotHardware.getInstance().getAprilTagPosition();
                    if (atag != null) RobotHardware.getInstance().localizer.setPose(atag);
                }
        );
    }
}
