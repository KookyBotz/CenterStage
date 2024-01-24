package org.firstinspires.ftc.teamcode.common.commandbase.cycleautocommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

public class PreloadDetectionCommand extends SequentialCommandGroup {
    public PreloadDetectionCommand() {
        super(
                new InstantCommand(() -> {

                })
        );
    }
}
