package org.firstinspires.ftc.teamcode.common.drive.pathing.path;

import com.arcrobotics.ftclib.util.MathUtils;

import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Spline;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Vector2D;

public class GVFPathFollower {
    private HermitePath path;

    private final double MAX_VELOCITY = 180; /* Inches per second */
    private final double MAX_ACCEL = 480; /* Inches per second squared */
    private final double MAX_DECEL = 720; /* Inches per second squared */
    private final double FINISH_TOLERANCE = 0.1; /* Finishing Error */
    private double lastVelocity = 0.5;

    private final double DECEL_PERIOD_DIST = (Math.pow(MAX_VELOCITY, 2)) / (2 * MAX_DECEL);

    private double kN;
    private double kS;
    private double kC;
    private Pose currentPose;
    public static double nearestT = 0.0;
    private boolean initialSearch = false;

    public GVFPathFollower(HermitePath path, final Pose initialPose, double kN, double kS, double kC) {
        this.path = path;
        this.currentPose = initialPose;
        this.kN = kN;
        this.kS = kS;
        this.kC = kC;
    }

    private double gradientOf(Spline s, double t, Pose targetPose) {
        double dx = s.getX().calculate(t, 1);
        double dy = s.getY().calculate(t, 1);
        double fx = s.getX().calculate(t, 0) - targetPose.x;
        double fy = s.getY().calculate(t, 0) - targetPose.y;
        return 2 * (dx * fx + dy * fy);
    }

    private Vector2D minimumT(double t, Pose targetPose) {
        final double TOLERANCE = 1e-6;
        final int MAX_ITERATIONS = 10;

        double currentT = t;
        for (int i = 0; i < MAX_ITERATIONS; i++) {
            Spline s = path.getSpline(currentT);
            double gradient = gradientOf(s, currentT, targetPose);
            double hessian = 2 * (Math.pow(s.getX().calculate(currentT, 1), 2) + Math.pow(s.getY().calculate(currentT, 1), 2));
            double newT = currentT - (gradient / hessian);

            if (Math.abs(newT - currentT) < TOLERANCE) {
                break;
            }
            currentT = newT;
        }

        double distance = (path.get(currentT, 0).subt(targetPose)).toVec2D().magnitude();
        return new Vector2D(currentT, distance);
    }

    public double getNearestT() {

        if (!initialSearch) {
            Vector2D nearestSplineDist = new Vector2D(0, Integer.MAX_VALUE);

            for (int i = 1; i < path.length() - 1; i++) {
                System.out.println("CURRENT ITER:" + i);
                Vector2D currentIterPair = minimumT(i, currentPose);
                if (currentIterPair.y < nearestSplineDist.y) {
                    nearestSplineDist = currentIterPair;
                }
            }
            initialSearch = !initialSearch;
            nearestT = nearestSplineDist.x;
        } else {
            nearestT = minimumT(nearestT, currentPose).x;
        }

        return MathUtils.clamp(nearestT, 0.0, path.length() - 1);
    }

    public Pose calculateGVF() {
        double startTime = System.nanoTime();
        nearestT = getNearestT();
        System.out.println("TIME:" + (System.nanoTime() - startTime) / 1000000000);
        if (nearestT < 1e-2) {
            nearestT = 1e-2;
        }

        Vector2D tangent = path.get(nearestT, 1).toVec2D().unit();
        Vector2D normal = tangent.rotate(Math.PI / 2);
        Pose nearestPose = path.get(nearestT, 0);

        double heading = nearestPose.heading;

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

        lastVelocity = vMax;
        return new Pose(gvf, heading);
    }

    public double errorMap(double error) {
        return Math.max(error, 1);
    }

    public void setCurrentPose(Pose currentPose) {
        this.currentPose = currentPose;
    }

    public boolean isFinished() {
        return currentPose.toVec2D().subt(path.endPose().toVec2D()).magnitude() < FINISH_TOLERANCE;
    }

    public void resetV() {
        lastVelocity = 0.0;
    }


}
