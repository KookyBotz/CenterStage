package org.firstinspires.ftc.teamcode.common.drive.pathing.geometry;

public class Vector2D {
    public double x,y;

    public Vector2D(double x, double y){
        this.x = x;
        this.y = y;
    }

    public static Vector2D fromHeadingAndMagnitude(double h, double m){
        return new Vector2D(Math.cos(h) * m, Math.sin(h) * m);
    }

    public double angle() {
        return Math.atan2(y, x);
    }

    public Vector2D deadzoneX(double val){
        if(Math.abs(x) < val) return new Vector2D(0, y);
        return this;
    }

    public Vector2D mult(double scalar) {
        return new Vector2D(x * scalar, y * scalar);
    }

    public Vector2D divide(double scalar) {
        return new Vector2D(x / scalar, y / scalar);
    }

    public Vector2D subt(Vector2D other) {
        return new Vector2D(x - other.x, y - other.y);
    }

    public double dot(Vector2D other) {
        return x * other.x + y * other.y;
    }

    public double magnitude() {
        return Math.hypot(x, y);
    }

    public Vector2D unit() {
        return this.divide(magnitude());
    }

    public Vector2D rotate(double angle) {
        return new Vector2D(
                x * Math.cos(angle) - y * Math.sin(angle),
                x * Math.sin(angle) + y * Math.cos(angle));
    }

    public double cross(Vector2D other) {
        return x * other.y - y * other.x;
    }

    public Vector2D project(Vector2D other) {
        double magnitude = other.magnitude();
        double angle = angle();
        return new Vector2D(magnitude * Math.cos(angle), magnitude * Math.sin(angle));
    }

    public double magnitudeSquared() {
        return x * x + y * y;
    }

    @Override
    public String toString() {
        return String.format("{%.2f, %.2f}", x, y);
    }
}