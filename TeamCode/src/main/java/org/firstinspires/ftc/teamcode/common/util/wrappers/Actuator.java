package org.firstinspires.ftc.teamcode.common.util.wrappers;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.profile.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.profile.ProfileConstraints;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.profile.ProfileState;
import org.firstinspires.ftc.teamcode.common.hardware.AbsoluteAnalogEncoder;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

public class Actuator {
    private final Map<String, HardwareDevice> devices = new ConcurrentHashMap<>();
    private AsymmetricMotionProfile profile;
    private ProfileConstraints constraints;
    private ProfileState state;
    private PIDController controller;
    private ElapsedTime timer;

    private double position = 0.0;
    private double targetPosition = 0.0;
    private double power = 0.0;
    private double tolerance = 0.0;
    private double feedforward = 0.0;

    private boolean reached = false;
    private boolean usingFeedforward = false;

    /**
     * Actuator constructor with varargs HardwareDevice parameter
     *
     * @param devices
     */
    public Actuator(HardwareDevice... devices) {
        for (HardwareDevice device : devices) {
            this.devices.put(device.getDeviceName(), device);
        }
    }

    /**
     * Reads every given hardware device containing either AnalogEncoder or Encoder object.
     * Will then store this, and terminate the loop, because there is only a need for one value in
     * a given actuation group.
     */
    public void read() {
        for (HardwareDevice device : devices.values()) {
            if (device instanceof AbsoluteAnalogEncoder) {
                this.position = ((AbsoluteAnalogEncoder) device).getCurrentPosition();
                return;
            } else if (device instanceof Motor.Encoder) {
                this.position = ((DcMotor) device).getCurrentPosition();
                return;
            }
        }


    }

    /**
     * Performs arithmetic with the AsymmetricMotionProfile and PIDController.
     * Stores a boolean representing whether or not the actuator group is within
     * some tolerance given by a specified value.
     */
    public void periodic() {
        if (profile != null) {
            this.state = profile.calculate(timer.time());
            this.targetPosition = state.x;
        }

        if (controller != null) {
            this.power = controller.calculate(position, targetPosition);
        }

        this.reached = Math.abs(targetPosition - position) < tolerance;
    }

    /**
     * For cohesiveness in our program's sequence, writes back all of the new
     * values calculated and saved. Runs different methods based on the given actuation group.
     */
    public void write() {
        for (HardwareDevice device : devices.values()) {
            if (device instanceof DcMotor) {
                ((DcMotor) device).setPower(power);
            } else if (device instanceof Servo) {
                ((Servo) device).setPosition(targetPosition);
            }
        }
    }

    /**
     * Will set the target position for the actuator group. In the case of a motion profile
     * being used, the profile will be reset and created again with the new target position.
     * Otherwise, a PIDController will be used in the periodic() method above.
     *
     * @param targetPosition
     */
    public void setTargetPosition(double targetPosition) {
        if (this.profile != null) {
            this.setMotionProfile(new AsymmetricMotionProfile(position, targetPosition, this.profile.constraints));
            this.timer.reset();
        } else {
            this.targetPosition = targetPosition;
        }
    }

    /**
     * Gets the value read by the actuation group.
     *
     * @return double
     */
    public double getPosition() {
        return position;
    }

    /**
     * Gets the current target position for the actuation group.
     * @return double
     */
    public double getTargetPosition() {
        return targetPosition;
    }

    /**
     * Gets any given HardwareDevice within this actuator group.
     *
     * @param deviceName The given HardwareMap name specified in the config,
     *                   or specified at object creation.
     * @return HardwareDevice object
     */
    public HardwareDevice getDevice(String deviceName) {
        return this.devices.get(deviceName);
    }

    /**
     * Returns a list of all given HardwareDevices.
     *
     * @return
     */
    public List<HardwareDevice> getDevices() {
        return new ArrayList<>(devices.values());
    }

    /**
     * Returns whether or not the given actuation group is within error
     * tolerance of the final position.
     *
     * @return
     */
    public boolean hasReached() {
        return this.reached;
    }

    /**
     * Saves the value passed in for the new motion profile.
     *
     * @param profile The new asymmetrical motion profile
     * @return
     */
    public Actuator setMotionProfile(AsymmetricMotionProfile profile) {
        this.profile = profile;
        return this;
    }

    /**
     * Saves the value passed in for the new PID controller.
     *
     * @param controller
     * @return
     */
    public Actuator setPIDController(PIDController controller) {
        this.controller = controller;
        return this;
    }

    /**
     * Creates a new PIDController object based on the coefficients passed in.
     *
     * @param p Proportional constant
     * @param i Integral Constant
     * @param d Derivative Constant
     * @return
     */
    public Actuator setPID(double p, double i, double d) {
        if (controller == null) {
            this.controller = new PIDController(p, i, d);
        } else {
            this.controller.setPID(p, i, d);
        }

        return this;
    }

    /**
     * Sets the allowed error tolerance for the actuation group to be considered
     * "close enough" within the target position.
     *
     * @param tolerance
     * @return
     */
    public Actuator setErrorTolerance(double tolerance) {
        this.tolerance = tolerance;
        return this;
    }
}
