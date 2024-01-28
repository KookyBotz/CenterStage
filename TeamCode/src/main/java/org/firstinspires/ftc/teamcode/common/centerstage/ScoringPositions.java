package org.firstinspires.ftc.teamcode.common.centerstage;


import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Pose;

public class ScoringPositions {

    private static int extension = 365;
    private static double armAngle = 2.98;
    public static final ScoringPosition[] YELLOW_PIXEL_POSITIONS = {
            new ScoringPosition(new Pose(42.75, -35.25, 0), extension, armAngle, ClawSide.LEFT), // BLUE LEFT LEFT
            new ScoringPosition(new Pose(38.75, -35.25, 0), extension, armAngle, ClawSide.BOTH), // BLUE LEFT RIGHT
            new ScoringPosition(new Pose(36.25, -35.25, 0), extension, armAngle, ClawSide.LEFT), // BLUE CENTER LEFT
            new ScoringPosition(new Pose(32.25, -35.25, 0), extension, armAngle, ClawSide.BOTH), // BLUE CENTER RIGHT
            new ScoringPosition(new Pose(30.75, -35.25, 0), extension, armAngle, ClawSide.LEFT), // BLUE RIGHT LEFT
            new ScoringPosition(new Pose(26.75, -35.25, 0), extension, armAngle, ClawSide.LEFT), // BLUE RIGHT RIGHT
            new ScoringPosition(new Pose(-42.75, -35.25, 0), extension, armAngle, ClawSide.LEFT), // RED LEFT LEFT
            new ScoringPosition(new Pose(-38.75, -35.25, 0), extension, armAngle, ClawSide.BOTH), // RED LEFT RIGHT
            new ScoringPosition(new Pose(-36.25, -35.25, 0), extension, armAngle, ClawSide.LEFT), // RED CENTER LEFT
            new ScoringPosition(new Pose(-32.25, -35.25, 0), extension, armAngle, ClawSide.BOTH), // RED CENTER RIGHT
            new ScoringPosition(new Pose(-30.75, -35.25, 0), extension, armAngle, ClawSide.LEFT), // RED RIGHT LEFT
            new ScoringPosition(new Pose(-26.75, -35.25, 0), extension, armAngle, ClawSide.LEFT)  // RED RIGHT RIGHT
    };

    public final ScoringPosition[] WHITE_PIXEL_POSITIONS = {

    };
}
