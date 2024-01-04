package org.firstinspires.ftc.teamcode.common.drive.localizer;

import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Pose;

public class AprilTagLocalizer {
    public static final double Y_OFFSET = 6.65;
    public static final double X_OFFSET = -2.82;

    public static final Pose BACKDROP_POSITION = new Pose(35.5, -61.25, 0);

    public static Pose convert(Pose pipeline) {
        pipeline.x += X_OFFSET;
        pipeline.y += Y_OFFSET;

        // calibrated
        pipeline.y -= ((pipeline.y - 6) * 0.0333);

        return pipeline.add(BACKDROP_POSITION);
    }
}
