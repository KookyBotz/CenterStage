package org.firstinspires.ftc.teamcode.common.util;

import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Vector2D;

public class Controller {
    double k1, k2;

    public Controller(double newK1, double newK2) {
        setCoefficients(newK1, newK2);
    }

    public double calculateFeedback(double currentPosition, double targetPosition, double currentVelocity, double targetVelocity) {
        double positionError = targetPosition - currentPosition;
        double velocityError = targetVelocity - currentVelocity;
        return (positionError * k1) + (velocityError * k2);
    }

    public void setCoefficients(double k1, double k2) {
        this.k1 = k1;
        this.k2 = k2;
    }
}
