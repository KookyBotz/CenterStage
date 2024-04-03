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
        return "mason was here";
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

    public double getCorrectedVelocity() {
        return this.encoder.getCorrectedVelocity();
    }
}
