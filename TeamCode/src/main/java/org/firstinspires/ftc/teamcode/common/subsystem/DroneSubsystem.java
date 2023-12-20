package org.firstinspires.ftc.teamcode.common.subsystem;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WSubsystem;

@Config
public class DroneSubsystem extends WSubsystem {

    private final RobotHardware robot = RobotHardware.getInstance();

    private DroneState droneState;

    public static double STORED_ACTUATOR = 0.0;
    public static double STORED_TRIGGER = 0.0;

    public static double ARMED_ACTUATOR = 0.0;
    public static double ARMED_TRIGGER = 0.0;

    public static double FIRED_ACTUATOR = 0.0;
    public static double FIRED_TRIGGER = 0.0;

    public enum DroneState {
        STORED,
        ARMED,
        FIRED
    }

    public DroneSubsystem() {
        if (robot.droneTrigger != null && robot.droneActuator != null) this.updateState(DroneState.STORED);
    }

    public void updateState(DroneState state) {
        switch (state) {
            case STORED:
                robot.droneActuator.setPosition(STORED_ACTUATOR);
                robot.droneTrigger.setPosition(STORED_TRIGGER);
                break;
            case ARMED:
                robot.droneActuator.setPosition(ARMED_ACTUATOR);
                robot.droneTrigger.setPosition(ARMED_TRIGGER);
                break;
            case FIRED:
                robot.droneTrigger.setPosition(FIRED_TRIGGER);
                break;
        }
    }

    @Override
    public void periodic() {
        
    }

    @Override
    public void read() {

    }

    @Override
    public void write() {

    }

    @Override
    public void reset() {
        this.updateState(DroneState.STORED);
    }

    public DroneState getDroneState() {
        return this.droneState;
    }
}
