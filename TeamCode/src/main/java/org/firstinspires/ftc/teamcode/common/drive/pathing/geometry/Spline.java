package org.firstinspires.ftc.teamcode.common.drive.pathing.geometry;

public class Spline {
    private final Polynomial x;
    private final Polynomial y;

    public Spline(Polynomial x, Polynomial y) {
        this.x = x;
        this.y = y;
    }

    public Pose calculate(double t, int n) {
        return new Pose(x.calculate(t, n), y.calculate(t, n), 0.0);
    }

    public double curvature(double t) {
        double x_prime = x.calculate(t, 1);
        double y_prime = y.calculate(t, 1);

        double kNum = Math.abs((y.calculate(t, 2) * x_prime) - y_prime * x.calculate(t, 2));
        double kDom = Math.pow((x_prime * x_prime) + (y_prime * y_prime), 1.5);

        return kNum / kDom;
    }

    public double getHeading(double t) {
        return Math.atan2(y.calculate(t, 1), x.calculate(t, 1));
    }

    public Polynomial getX() {
        return x;
    }

    public Polynomial getY() {
        return y;
    }
}