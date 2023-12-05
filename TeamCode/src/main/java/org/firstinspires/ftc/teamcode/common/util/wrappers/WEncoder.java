package org.firstinspires.ftc.teamcode.common.util.wrappers;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareDevice;

public class WEncoder implements HardwareDevice {
    public Motor.Encoder encoder;

    public WEncoder(Motor.Encoder encoder) {
        this.encoder = encoder;
    }

    @Override
    public Manufacturer getManufacturer() {
        return null;
    }

    @Override
    public String getDeviceName() {
        // Note: only one encoder should ever be in an actuator group at a time,
        // So a preset name works fine.
        return "WEncoder";
    }

    @Override
    public String getConnectionInfo() {
        return null;
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }

    @Override
    public void close() {

    }

    public double getPosition() {
        return this.encoder.getPosition();
    }

    public double getRawVelocity() {
        return this.encoder.getRawVelocity();
    }
}
