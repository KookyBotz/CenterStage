package org.firstinspires.ftc.teamcode.common.centerstage;


import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Pose;

public class ScoringPositions {

    private static int extension = 510;
    private static double armAngle = 3.05;
    public static final ScoringPosition[] YELLOW_PIXEL_POSITIONS = {
            new ScoringPosition(new Pose(37.75, -29, 0), extension, armAngle, ClawSide.LEFT), // BLUE LEFT LEFT
            new ScoringPosition(new Pose(34.75, -29, 0), extension, armAngle, ClawSide.BOTH), // BLUE LEFT RIGHT
            new ScoringPosition(new Pose(31.75, -29, 0), extension, armAngle, ClawSide.LEFT), // BLUE CENTER LEFT
            new ScoringPosition(null, 0, 0, ClawSide.LEFT), // BLUE CENTER RIGHT
            new ScoringPosition(new Pose(28.75, -29, 0), extension, armAngle, ClawSide.LEFT), // BLUE RIGHT LEFT
            new ScoringPosition(new Pose(24.75, -29, 0), extension, armAngle, ClawSide.LEFT)  // BLUE RIGHT RIGHT
    };

    public final ScoringPosition[] WHITE_PIXEL_POSITIONS = {

    };
}
