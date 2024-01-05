package org.firstinspires.ftc.teamcode.common.drive.localizer;

import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Pose;

public class AprilTagLocalizer {
    public static final double Y_OFFSET = 6.65;
    public static final double X_OFFSET = -2.82;

    public static final Pose BLUE_BACKDROP_POSITION = new Pose(35.5, -61.25, 0);
    public static final Pose RED_BACKDROP_POSITION = new Pose(-35.5, -61.25, 0);

    public static Pose convertBlueBackdropPoseToGlobal(Pose pipeline) {
        pipeline.x += X_OFFSET;
        pipeline.y += Y_OFFSET;

        // calibrated
        pipeline.y -= ((pipeline.y - 6) * 0.0333);

        return pipeline.add(BLUE_BACKDROP_POSITION);
    }

    public static Pose convertRedBackdropPoseToGlobal(Pose pipeline) {
        pipeline.x += X_OFFSET;
        pipeline.y += Y_OFFSET;

        // calibrated
        pipeline.y -= ((pipeline.y - 6) * 0.0333);

        return pipeline.add(RED_BACKDROP_POSITION);
    }
}
