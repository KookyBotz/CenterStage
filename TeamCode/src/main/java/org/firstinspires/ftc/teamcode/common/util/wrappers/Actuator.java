package org.firstinspires.ftc.teamcode.common.util.wrappers;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
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
    private Map<String, HardwareDevice> devices = new ConcurrentHashMap<>();
    private AsymmetricMotionProfile profile;
    private ProfileConstraints constraints;
    private ProfileState state;
    private PIDController controller;
    private ElapsedTime timer;

    private double position = 0.0;
    private double targetPosition = 0.0;
    private double power = 0.0;
    private double tolerance = 0.0;
    private boolean reached = false;

    public Actuator(PIDController controller, HardwareDevice... devices) {
        this.controller = controller;

        for (HardwareDevice device : devices) {
            this.devices.put(device.getDeviceName(), device);
        }
    }

    public Actuator(HardwareDevice... devices) {
        for (HardwareDevice device : devices) {
            this.devices.put(device.getDeviceName(), device);
        }
    }

    public void read() {
        for (HardwareDevice device : devices.values()) {
            if (device instanceof AbsoluteAnalogEncoder) {
                this.position = ((AbsoluteAnalogEncoder) device).getCurrentPosition();
                break;
            } else if (device instanceof Motor.Encoder) {
                this.position = ((DcMotor) device).getCurrentPosition();
                break;
            }
        }
    }

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

    public void write() {
        for (HardwareDevice device : devices.values()) {
            if (device instanceof DcMotor) {
                ((DcMotor) device).setPower(power);
            } else if (device instanceof Servo) {
                ((Servo) device).setPosition(targetPosition);
            }
        }
    }

    public void setTargetPosition(double targetPosition) {
        if (this.profile != null) {
            this.setMotionProfile(new AsymmetricMotionProfile(position, targetPosition, constraints));
            this.timer.reset();
        } else {
            this.targetPosition = targetPosition;
        }
    }

    public void setMotionProfile(AsymmetricMotionProfile profile) {
        this.profile = profile;
    }

    public void setPIDController(PIDController controller) {
        this.controller = controller;
    }

    public void setPID(double p, double i, double d) {
        this.controller.setPID(p, i, d);
    }

    public void setErrorTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    public double getPosition() {
        return position;
    }

    public HardwareDevice getDevice(String deviceName) {
        return this.devices.get(deviceName);
    }

    public List<HardwareDevice> getDevices() {
        return new ArrayList<>(devices.values());
    }

    public boolean hasReached() {
        return this.reached;
    }
}
