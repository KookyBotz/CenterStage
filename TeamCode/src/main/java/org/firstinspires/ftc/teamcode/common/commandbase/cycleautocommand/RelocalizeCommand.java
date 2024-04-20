package org.firstinspires.ftc.teamcode.common.commandbase.cycleautocommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class RelocalizeCommand extends SequentialCommandGroup {
    private final RobotHardware robot = RobotHardware.getInstance();
    private Pose a, b, c;

    public RelocalizeCommand() {
        super.addCommands(
                // don't ask
                new WaitCommand(50),
                new InstantCommand(() -> a = robot.getAprilTagPosition()),
                new WaitCommand(50),
                new InstantCommand(() -> b = robot.getAprilTagPosition()),
                new WaitCommand(50),
                new InstantCommand(() -> {
                    c = robot.getAprilTagPosition();

                    Pose avg = avg(a, b, c);
                    if (avg != null) {
                        robot.localizer.setPose(avg);
                        robot.readIMU();
                        double imuAngle = robot.getAngle();

                        robot.imuOffset += AngleUnit.normalizeRadians(imuAngle - avg.heading);

                        System.out.println("atag" + robot.localizer.getPose());
                    }
                }),
                new WaitCommand(50)
        );
    }

    public Pose avg(Pose... poses) {
        Pose pose = new Pose();
        int i = 0;
        for (Pose p : poses) {
            if (p != null && Math.abs(AngleUnit.normalizeRadians(robot.localizer.getPose().heading - p.heading)) < 0.2) {
                pose = pose.add(p);
                i++;
            }
        }
        return i > 0 ? pose.divide(new Pose(i, i, i)) : null;
    }
}
