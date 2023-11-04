package org.firstinspires.ftc.teamcode.common.drive.pathing.path;

import org.firstinspires.ftc.teamcode.common.drive.drivetrain.MecanumDriveConstants;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Spline;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Vector2D;
import org.firstinspires.ftc.teamcode.common.util.MathUtils;

import java.util.ArrayList;

public class GVFPathFollower {
    private final HermitePath path;

    private final double MAX_VELOCITY = MecanumDriveConstants.MAX_LINEAR_SPEED - 20; /* Inches per second */
    private final double MAX_ACCEL = MecanumDriveConstants.MAX_LINEAR_ACCELERATION; /* Inches per second squared */
    private final double MAX_DECEL = MecanumDriveConstants.MAX_LINEAR_ACCELERATION; /* Inches per second squared */
    private final double FINISH_TOLERANCE = 1.5; /* Finishing Error */
    private final double FINISH_HEADING_TOLERANCE = 0.25;
    private double lastVelocity = 1e-7;

    private final double DECEL_PERIOD_DIST = (Math.pow(MAX_VELOCITY, 2)) / (2 * MAX_DECEL);

    private final double kN;
    private final double kS;
    private final double kC;
    private Pose currentPose;
    public static double nearestT = 0.0;
    public static Pose nearestPose = new Pose(0, 0, 0);
    public static Pose bestPos = new Pose(0, 0, 0);

    private final double TOLERANCE = 1e-6;
    private final int MAX_ITERATIONS = 10;

    public GVFPathFollower(HermitePath path, final Pose initialPose, double kN, double kS, double kC) {
        this.path = path;
        this.currentPose = initialPose;
        this.kN = kN;
        this.kS = kS;
        this.kC = kC;
    }

    public double projectPos(Spline s, Pose pos) {
        double curr = 0.5;
        double length = 0;

        for (double t = 0.01; t <= 1; t += 0.01) {
            Pose prev = s.calculate(t - 0.01, 0);
            Pose now = s.calculate(t, 0);
            length += prev.toVec2D().subt(now.toVec2D()).magnitude();
        }

        for (int i = 0; i < 250; i++) {
            Pose p = s.calculate(curr, 0);
            Pose deriv = s.calculate(curr, 1);

            double ds = pos.toVec2D().subt(p.toVec2D()).dot(deriv.toVec2D());

            ds = ds / deriv.toVec2D().dot(deriv.toVec2D());

            if (MathUtils.epsilonEquals(ds, 0)) {
                break;
            }

            curr += (ds / length);

            if (curr < 0) {
                //curr = 0;
            }
            if (curr > 1) {
                //curr = 1;
            }
        }

        return (Math.max(0, Math.min(curr, 1)));
    }

    public double projectPosNew(Pose position) {
        double minDist = Double.MAX_VALUE;
        bestPos = new Pose(0.0, 0.0, 0.0);
        double projectPos = 0;

        ArrayList<Spline> splines = path.getSplines();
        for (int i = 0; i < splines.size(); i++) {
            double d = projectPos(splines.get(i), position);
            Pose pos = splines.get(i).calculate(d, 0);

            if (pos.toVec2D().subt(position.toVec2D()).magnitude() < minDist) {
                minDist = pos.toVec2D().subt(position.toVec2D()).magnitude();
                bestPos = pos;
                projectPos = d + i;
            }
        }

        nearestPose = bestPos;

        return projectPos;
    }

    public Pose calculateGVF() {
        double startTime = System.nanoTime();
        nearestT = projectPosNew(currentPose);
        System.out.println("T " + nearestT);
        if (nearestT < 1e-2) {
            nearestT = 1e-7;
        }

        Pose tang = path.get(nearestT, 1);
        Vector2D tangent = tang.toVec2D().unit();
        Vector2D normal = tangent.rotate(Math.PI / 2);
        Pose nearestPose = path.get(nearestT, 0);

        double heading = tang.toVec2D().angle();

        Vector2D displacement = nearestPose.subt(currentPose).toVec2D();
        double error = displacement.magnitude() * Math.signum((displacement.cross(tangent)));

        double vMax = MAX_VELOCITY;

        Vector2D gvf = (tangent.subt(normal.mult(kN).mult(error))).unit();

        double decel_disp = currentPose.subt(path.endPose()).toVec2D().magnitude();
        if (decel_disp < DECEL_PERIOD_DIST) {
            vMax = vMax * (decel_disp / DECEL_PERIOD_DIST);
        }

        double curvature = path.curvature(nearestT);
        if (curvature != 0) {
            vMax = Math.min(Math.sqrt(MAX_ACCEL / (curvature * kC)), vMax);
        }

        double alpha = 0.9;
        vMax = alpha * lastVelocity + (1 - alpha) * vMax;

        gvf = gvf.mult(vMax).mult(kS);

        if (nearestT >= path.length() - 1e-2) {
            gvf = displacement.unit().project(gvf);
        }

//        lastVelocity = vMax;
        return new Pose(gvf, heading);
    }

    public double errorMap(double error) {
        return Math.max(error, 1);
    }

    public void setCurrentPose(Pose currentPose) {
        this.currentPose = currentPose;
    }

    public boolean isFinished() {
        return currentPose.toVec2D().subt(path.endPose().toVec2D()).magnitude() < FINISH_TOLERANCE /*&& Math.abs(currentPose.heading -  path.endPose().heading) < FINISH_HEADING_TOLERANCE*/;
    }

    public void resetV() {
        lastVelocity = 0.0;
    }


}