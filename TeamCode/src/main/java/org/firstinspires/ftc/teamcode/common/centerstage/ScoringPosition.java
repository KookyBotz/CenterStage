package org.firstinspires.ftc.teamcode.common.centerstage;

import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Pose;

public class ScoringPosition {
    private Pose targetPose;
    private int targetExtension;
    private double targetArmAngle;
    private ClawSide clawSide;

    public ScoringPosition(Pose targetPose, int targetExtension, double targetArmAngle, ClawSide clawSide) {
        this.targetPose = targetPose;
        this.targetExtension = targetExtension;
        this.targetArmAngle = targetArmAngle;
        this.clawSide = clawSide;
    }

    public Pose getTargetPose() {
        return this.targetPose;
    }

    public int getTargetExtension() {
        return this.targetExtension;
    }

    public double getTargetArmAngle() {
        return this.targetArmAngle;
    }

    public ClawSide getClawSide() {
        return this.clawSide;
    }
}
