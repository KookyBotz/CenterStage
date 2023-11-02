package org.firstinspires.ftc.teamcode.common.drive.pathing.path;

import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.HermitePose;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Spline;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Vector2D;

import java.util.ArrayList;

public class HermitePath {
    public HermiteInterpolator interpolator;
    private final ArrayList<HermitePose> controlPoses = new ArrayList<>();

    public HermitePath() {
        interpolator = new HermiteInterpolator();
    }

    public HermitePath addPose(double x, double y, Vector2D tangent) {
        this.addPose(new HermitePose(x, y, tangent));
        return this;
    }

    public HermitePath addPose(double x, double y, double h, double m) {
        return this.addPose(new HermitePose(x, y, Vector2D.fromHeadingAndMagnitude(h, m)));
    }

    public HermitePath addPose(double x, double y, double h) {
        return this.addPose(new HermitePose(x, y, Vector2D.fromHeadingAndMagnitude(h, 1)));
    }

    public HermitePath addPose(HermitePose pose) {
        this.controlPoses.add(pose);
        return this;
    }

    public HermitePath flip() {
        for (HermitePose pose : controlPoses) {
            double x = pose.x;
            pose.x = pose.y;
            pose.y = x;

            double v_x = pose.tangent.x;
            pose.tangent.x = pose.tangent.y;
            pose.tangent.y = v_x;
        }
        return this;
    }

    public HermitePath negateX() {
        for (HermitePose pose : controlPoses) {
            pose.x *= -1;
        }
        return this;
    }

    public HermitePath negateY() {
        for (HermitePose pose : controlPoses) {
            pose.y *= -1;
        }
        return this;
    }

    public HermitePath offsetX(double x) {
        for (HermitePose pose : controlPoses) {
            pose.x += x;
        }
        return this;
    }

    public HermitePath offsetY(double y) {
        for (HermitePose pose : controlPoses) {
            pose.y += y;
        }
        return this;
    }

    public HermitePath construct() {
        if (controlPoses.size() <= 1)
            throw new IllegalStateException("Need a minimum of two control poses.");
        interpolator = new HermiteInterpolator();
        interpolator.setControlPoses(controlPoses);
        return this;
    }

    public Pose get(double t, int n) {
        if (t <= 0) {
            return startPose();
        } else if (t >= controlPoses.size()) {
            return endPose();
        } else {
            return interpolator.get(t, n);
        }
    }

    public double getHeading(double t) {
        return interpolator.getHeading(t);
    }

    public double curvature(double t) {
        return interpolator.curvature(t);
    }

    public Spline getSpline(double t) {
        return interpolator.getSpline(t);
    }

    public ArrayList<Spline> getSplines() {
        return interpolator.getSplines();
    }

    public int length() {
        return controlPoses.size() - 1;
    }

    public Pose startPose() {
        return controlPoses.get(0).pose();
    }

    public Pose endPose() {
        return controlPoses.get(length()).pose();
    }
}