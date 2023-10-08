package org.firstinspires.ftc.teamcode.common.hardware;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.util.wrappers.AnalogServo;

import javax.annotation.Nonnegative;

public class RobotHardware {

    public MotorEx extensionMotor;
    public MotorEx extensionPitchMotor;

    public DcMotorEx frontLeftMotor;
    public DcMotorEx frontRightMotor;
    public DcMotorEx backLeftMotor;
    public DcMotorEx backRightMotor;

    public AbsoluteAnalogEncoder extensionPitchEncoder;

    // TODO: Configure hardwaremap
    // TODO: Configure invert positions
    public AnalogServo intakeClawLeftServo;
    public AnalogServo intakeClawRightServo;

    // TODO: Configure Hardwaremap
    // TODO: Configure invert positions
    public AnalogServo intakePivotLeftServo;
    public Servo intakePivotRightServo;

    public DigitalChannel intakeClawLeftBottom,  intakeClawLeftTop,
                          intakeClawRightBottom, intakeClawRightTop;

    //TODO: Add 4x wall distance sensors


    //TODO: Add 2x Cameras

    /**
     * Odometry pod encoders.
     */
    public Motor.Encoder parallelPodLeft;
    public Motor.Encoder parallelPodRight;
    public Motor.Encoder perpindicularPod;

    /**
     * HardwareMap storage.
     */
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    /**
     * Voltage timer and voltage value.
     */
    private ElapsedTime voltageTimer;
    private double voltage = 0.0;

    /**
     * Singleton variables.
     */
    private static RobotHardware instance = null;
    public boolean enabled;

    /**
     * Creating the singleton the first time, instantiating.
     */
    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        instance.enabled = true;
        return instance;
    }

    /**
     * Created at the start of every OpMode.
     *
     * @param hardwareMap The HardwareMap of the robot, storing all hardware devices
     * @param telemetry Saved for later in the event FTC Dashboard used
     */
    public void init(final HardwareMap hardwareMap, final Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    public void read() {}

    public void write() {}

    public void loop() {
        if (voltageTimer.seconds() > 5) {
            voltageTimer.reset();
            voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        }
    }

    public void reset() {

    }

    @Nonnegative
    public double getVoltage() {
        return voltage;
    }
}
