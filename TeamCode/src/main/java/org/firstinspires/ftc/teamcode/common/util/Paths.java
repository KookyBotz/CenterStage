package org.firstinspires.ftc.teamcode.common.util;

import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Vector2D;
import org.firstinspires.ftc.teamcode.common.drive.pathing.path.HermitePath;

public class Paths {
    public static HermitePath PRELOAD_SCORE_RIGHT = new HermitePath()
            .addPose(136.5, 60, new Vector2D(50, 0))
            .addPose(124, 65, Vector2D.fromHeadingAndMagnitude(0.5, 100))
            .offsetX(-136.5)
            .offsetY(-60)
            .negateX()
            .construct();

    // todo add
    public static HermitePath PRELOAD_TO_BACKDROP = new HermitePath()
            .addPose(124, 65, Vector2D.fromHeadingAndMagnitude(0.5, 100))
            .addPose(108, 48, new Vector2D(0, 400))
            .addPose(108, 36, new Vector2D(0, 200))
            .offsetX(-136.5)
            .offsetY(-60)
            .negateX()
            .construct();

    public static HermitePath DIRECT_YELLOW_TO_BACKDROP = new HermitePath()
            .addPose(136.5, 60, new Vector2D(400, 0))
            .addPose(108, 30, new Vector2D(0, 500))
            .addPose(108, 24, new Vector2D(0, 200))
            .offsetX(-136.5)
            .offsetY(-60)
            .negateX()
            .construct();
}
