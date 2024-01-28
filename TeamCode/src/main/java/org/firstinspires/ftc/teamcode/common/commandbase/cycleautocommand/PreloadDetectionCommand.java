package org.firstinspires.ftc.teamcode.common.commandbase.cycleautocommand;

import org.firstinspires.ftc.teamcode.common.centerstage.ScoringPositions;
import org.firstinspires.ftc.teamcode.common.commandbase.drivecommand.PositionCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class PreloadDetectionCommand extends PositionCommand {
    private boolean flag = false;
    public PreloadDetectionCommand() {
        // default
        super(ScoringPositions.YELLOW_PIXEL_POSITIONS[Globals.getTargetIndex()].getTargetPose());
    }

    @Override
    public void execute(){
        super.execute();
        if(!flag) {
            // updated
            super.targetPose = ScoringPositions.YELLOW_PIXEL_POSITIONS[Globals.getTargetIndex()].getTargetPose();
            flag = true;
            System.out.println("HERE123 " + super.targetPose);

        }
        System.out.println("HERE125 " + RobotHardware.getInstance().localizer.getPose());

    }
}
