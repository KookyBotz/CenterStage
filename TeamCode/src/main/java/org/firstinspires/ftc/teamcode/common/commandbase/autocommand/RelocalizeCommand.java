package org.firstinspires.ftc.teamcode.common.commandbase.autocommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class RelocalizeCommand extends SequentialCommandGroup {
    private Pose a, b, c;

    public RelocalizeCommand() {
        super.addCommands(
                // don't ask
                new WaitCommand(50),
                new InstantCommand(() -> {
                    a = RobotHardware.getInstance().getAprilTagPosition();
                }),
                new WaitCommand(50),
                new InstantCommand(() -> {
                    b = RobotHardware.getInstance().getAprilTagPosition();
                }),
                new WaitCommand(50),
                new InstantCommand(() -> {
                    c = RobotHardware.getInstance().getAprilTagPosition();
                    if (avg(a, b, c) != null) RobotHardware.getInstance().localizer.setPose(avg(a, b, c));
                }),
                new WaitCommand(50)
        );
    }

    public Pose avg(Pose... poses) {
        Pose pose = new Pose();
        int i = 0;
        for (Pose p : poses) {
            if (p != null) {
                pose = pose.add(p);
                i++;
            }
        }
        return i > 0 ? pose.divide(new Pose(i, i, i)) : null;
    }
}
