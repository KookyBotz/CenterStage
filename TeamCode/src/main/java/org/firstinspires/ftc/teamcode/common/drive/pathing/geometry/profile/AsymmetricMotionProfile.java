package org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.profile;

import androidx.annotation.NonNull;

public class AsymmetricMotionProfile {
    public double initialPosition;
    public double finalPosition;
    public double distance;
    public double t1, t2, t3;
    public double totalTime;
    public double t1_stop_position;
    public double max_velocity;
    public double t2_stop_position;
    public boolean flipped = false;
    public double originalPos = 0;

    public ProfileState state = new ProfileState();
    public ProfileConstraints constraints;

    public AsymmetricMotionProfile(double initialPosition, double finalPosition, ProfileConstraints constraints) {
        if (finalPosition < initialPosition) {
            flipped = true;
            this.originalPos = initialPosition;
            double temp = initialPosition;
            initialPosition = finalPosition;
            finalPosition = temp;
        }
        this.initialPosition = initialPosition;
        this.finalPosition = finalPosition;
        this.distance = finalPosition - initialPosition;
        this.constraints = constraints;

        t1 = constraints.velo / constraints.accel;
        t3 = constraints.velo / constraints.decel;
        t2 = Math.abs(distance) / constraints.velo - (t1 + t3) / 2;

        if (t2 < 0) {
            this.t2 = 0;

            double a = (constraints.accel / 2) * (1 - constraints.accel / -constraints.decel);
            double c = -distance;

            t1 = Math.sqrt(-4 * a * c) / (2 * a);
            t3 = -(constraints.accel * t1) / -constraints.decel;
            t1_stop_position = (constraints.accel * Math.pow(t1, 2)) / 2;

            max_velocity = constraints.accel * t1;

            t2_stop_position = t1_stop_position;
        } else {
            max_velocity = constraints.velo;
            t1_stop_position = (constraints.velo * t1) / 2;
            t2_stop_position = t1_stop_position + t2 * max_velocity;
        }

        totalTime = t1 + t2 + t3;
    }

    public ProfileState calculate(final double time) {
        double position, velocity, acceleration, stage_time;
        if (time <= t1) {
            stage_time = time;
            acceleration = constraints.accel;
            velocity = acceleration * stage_time;
            position = velocity * stage_time / 2;
        } else if (time <= t1 + t2) {
            stage_time = time - t1;
            acceleration = 0;
            velocity = constraints.velo;
            position = t1_stop_position + stage_time * velocity;
        } else if (time <= totalTime) {
            stage_time = time - t1 - t2;
            acceleration = -constraints.decel;
            velocity = max_velocity - stage_time * constraints.decel;
            position = t2_stop_position + (max_velocity + velocity) / 2 * stage_time;
        } else {
            acceleration = 0;
            velocity = 0;
            position = finalPosition;
        }

        // lol
        if (time <= totalTime) {
            if (flipped) {
                state.x = originalPos - position;
            } else {
                state.x = initialPosition + position;
            }
        } else {
            if (flipped) {
                state.x = initialPosition;
            } else {
                state.x = originalPos + position;
            }
        }
        state.v = velocity;
        state.a = acceleration;
        return this.state;
    }

    @NonNull
    @Override
    public String toString() {
        return String.format("%f, %f, %f", state.x, state.v, state.a);
    }
}
