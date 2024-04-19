package org.firstinspires.ftc.teamcode.common.centerstage;


import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Pose;

public class ScoringPositions {

    private static final int extension = 365;
    private static final double armAngle = 2.98;
    public static final ScoringPosition[] YELLOW_PIXEL_POSITIONS = {
            new ScoringPosition(new Pose(42.5, -35.25, 0), extension, armAngle, ClawSide.LEFT),  // BLUE LEFT LEFT
            new ScoringPosition(new Pose(38.75, -35.25, 0), extension, armAngle, ClawSide.BOTH),  // BLUE LEFT RIGHT
            new ScoringPosition(new Pose(36.5, -35.25, 0), extension, armAngle, ClawSide.LEFT),  // BLUE CENTER LEFT
            new ScoringPosition(new Pose(32.75, -35.25, 0), extension, armAngle, ClawSide.BOTH),  // BLUE CENTER RIGHT
            new ScoringPosition(new Pose(30.5, -35.25, 0), extension, armAngle, ClawSide.LEFT),  // BLUE RIGHT LEFT
            new ScoringPosition(new Pose(26.75, -35.25, 0), extension, armAngle, ClawSide.LEFT),  // BLUE RIGHT RIGHT
            new ScoringPosition(new Pose(-25.25, -35.75, 0), extension, armAngle, ClawSide.LEFT), // RED LEFT LEFT
            new ScoringPosition(new Pose(-29.5, -35.75, 0), extension, armAngle, ClawSide.LEFT), // RED LEFT RIGHT
            new ScoringPosition(new Pose(-31.25, -35.75, 0), extension, armAngle, ClawSide.BOTH), // RED CENTER LEFT
            new ScoringPosition(new Pose(-35.5, -35.75, 0), extension, armAngle, ClawSide.LEFT), // RED CENTER RIGHT
            new ScoringPosition(new Pose(-37.25, -35.75, 0), extension, armAngle, ClawSide.BOTH), // RED RIGHT LEFT
            new ScoringPosition(new Pose(-41.5, -35.75, 0), extension, armAngle, ClawSide.LEFT), // RED RIGHT RIGHT
    };

    public final ScoringPosition[] WHITE_PIXEL_POSITIONS = {

    };
}
