package org.firstinspires.ftc.teamcode.common.drive.pathing.geometry;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.util.Locale;

public class Pose extends Point {

    public double heading;

    public Pose(double x, double y, double heading) {
        super(x, y);
        this.heading = AngleUnit.normalizeRadians(heading);
    }

    public Pose(Point p, double heading) {
        this(p.x, p.y, heading);
    }

    public Pose(Vector2D vec, double heading) {
        this(vec.x, vec.y, heading);
    }

    public Pose() {
        this(0, 0, 0);
    }

    public Pose(AprilTagPoseFtc ftcPose) {
        this.heading = Math.toRadians(-ftcPose.yaw);
        this.x = ftcPose.x * Math.cos(heading) - ftcPose.y * Math.sin(heading);
        this.y = ftcPose.x * Math.sin(heading) + ftcPose.y * Math.cos(heading);
    }

    public void set(Pose other) {
        this.x = other.x;
        this.y = other.y;
        this.heading = other.heading;
    }

    public Pose add(Pose other) {
        return new Pose(x + other.x, y + other.y, heading + other.heading);
    }

    public Pose subtract(Pose other) {
        return new Pose(this.x - other.x, this.y - other.y, AngleUnit.normalizeRadians(this.heading - other.heading));
    }

    public Pose divide(Pose other) {
        return new Pose(this.x / other.x, this.y / other.y, this.heading / other.heading);
    }

    public Pose scale(double scalar){
        return new Pose(this.x * scalar, this.y * scalar, this.heading * scalar);
    }

    public Pose subt(Pose other) {
        return new Pose(x - other.x, y - other.y, heading - other.heading);
    }

    public Vector2D toVec2D() {
        return new Vector2D(x, y);
    }

    @Override
    public String toString() {
        return String.format(Locale.ENGLISH, "%.2f %.2f %.3f", x, y, heading);
    }
}