package org.firstinspires.ftc.teamcode.common.subsystem;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WSubsystem;

@Config
public class DroneSubsystem extends WSubsystem {

    private final RobotHardware robot = RobotHardware.getInstance();

    private DroneState droneState;

    public static double ARMED_TRIGGER = 0.075;

    public static double FIRED_TRIGGER = 0.3 ;

    public enum DroneState {
        ARMED,
        FIRED
    }

    public DroneSubsystem() {
        if (robot.droneTrigger != null) this.updateState(DroneState.ARMED);
    }

    public void updateState(DroneState state) {
        switch (state) {
            case ARMED:
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
        this.updateState(DroneState.ARMED);
    }

    public DroneState getDroneState() {
        return this.droneState;
    }
}
