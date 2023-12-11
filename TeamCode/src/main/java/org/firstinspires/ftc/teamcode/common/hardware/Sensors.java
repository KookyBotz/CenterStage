package org.firstinspires.ftc.teamcode.common.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.HashMap;

public class Sensors {
    private HashMap<SensorType, Object> values;
    private HardwareMap hardwareMap;
    private RobotHardware robot = RobotHardware.getInstance();

    public enum SensorType {
        EXTENSION_ENCODER,
        ARM_ENCODER,
        POD_LEFT,
        POD_FRONT,
        POD_RIGHT
    }

    public Sensors(HardwareMap hardwareMap) {
        this.values = new HashMap<>();
        this.hardwareMap = hardwareMap;

        values.put(SensorType.EXTENSION_ENCODER, 0);
        values.put(SensorType.ARM_ENCODER, 0.0);
        values.put(SensorType.POD_LEFT, 0.0);
        values.put(SensorType.POD_FRONT, 0.0);
        values.put(SensorType.POD_RIGHT, 0.0);
    }

    public void read() {
        // manually go through each device and run getPosition or whatever, and then store it
        // then go into each subsystem and notify each one of a position change, send it over so it then saves it there?

        values.put(SensorType.EXTENSION_ENCODER, robot.extensionEncoder.getPosition());
        values.put(SensorType.ARM_ENCODER, robot.armPitchEncoder.getCurrentPosition());
        if (Globals.IS_AUTO) {
            values.put(SensorType.POD_LEFT, robot.podLeft.getPosition());
            values.put(SensorType.POD_FRONT, robot.podFront.getPosition());
            values.put(SensorType.POD_RIGHT, robot.podRight.getPosition());
        }

        if (Globals.IS_USING_IMU); // read imu here

    }

    public double doubleSubscriber(SensorType topic) {
        return (double) values.getOrDefault(topic, 0.0);
    }

    public int intSubscriber(SensorType topic) {
        return (int) values.getOrDefault(topic, 0);
    }

    public boolean boolSubscriber(SensorType topic) {
        return (boolean) values.getOrDefault(topic, 0);
    }
}
