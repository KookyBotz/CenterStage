package org.firstinspires.ftc.teamcode.common.drive.localizer;

import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Point;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Pose;

public class AprilTagConstants {
    public static final double Y_OFFSET = 6.3;
    public static final double X_OFFSET = -2.3875 + 0.25;
    private static final Pose CAMERA_POSE= new Pose(X_OFFSET, Y_OFFSET, 0);

    public static final Pose BLUE_BACKDROP_POSITION = new Pose(35.5, -61.25, 0);
    public static final Pose RED_BACKDROP_POSITION = new Pose(-35.5, -61.25, 0);

    public static Pose convertBlueBackdropPoseToGlobal(Pose pipeline) {
        Pose offset = new Pose();

        offset.x = CAMERA_POSE.x * Math.cos(pipeline.heading) - CAMERA_POSE.y * Math.sin(pipeline.heading);
        offset.y = CAMERA_POSE.x * Math.sin(pipeline.heading) + CAMERA_POSE.y * Math.cos(pipeline.heading);

        pipeline.x += offset.x;
        pipeline.y += offset.y;

        return pipeline.add(BLUE_BACKDROP_POSITION);
    }

    public static Pose convertRedBackdropPoseToGlobal(Pose pipeline) {
        Pose offset = new Pose();

        offset.x = CAMERA_POSE.x * Math.cos(pipeline.heading) - CAMERA_POSE.y * Math.sin(pipeline.heading);
        offset.y = CAMERA_POSE.x * Math.sin(pipeline.heading) + CAMERA_POSE.y * Math.cos(pipeline.heading);

        pipeline.x += offset.x;
        pipeline.y += offset.y;

        return pipeline.add(RED_BACKDROP_POSITION);
    }
}
