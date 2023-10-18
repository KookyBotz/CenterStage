package org.firstinspires.ftc.teamcode.common.hardware;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.outoftheboxrobotics.photoncore.hardware.PhotonLynxModule;
import com.outoftheboxrobotics.photoncore.hardware.i2c.imu.PhotonBNO055IMUNew;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.profile.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.profile.ProfileConstraints;
import org.firstinspires.ftc.teamcode.common.util.wrappers.Actuator;
import org.firstinspires.ftc.teamcode.common.util.wrappers.AnalogServo;
import org.firstinspires.ftc.teamcode.common.util.wrappers.KSubsystem;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import javax.annotation.Nonnegative;

public class RobotHardware {

    public MotorEx extensionMotor;
    public MotorEx extensionPitchMotor;
    public AbsoluteAnalogEncoder extensionPitchEncoder;
    public Actuator extensionPitchActuator;

    public DcMotorEx dtFrontLeftMotor;
    public DcMotorEx dtFrontRightMotor;
    public DcMotorEx dtBackLeftMotor;
    public DcMotorEx dtBackRightMotor;


    // TODO: Configure hardware map
    // TODO: Configure invert positions
    public AnalogServo intakeClawLeftServo;
    public AnalogServo intakeClawRightServo;

    // TODO: Configure hardware map
    // TODO: Configure invert positions
    public Servo intakePivotLeftServo;
    public Servo intakePivotRightServo;
    public AbsoluteAnalogEncoder intakePivotEncoder;
    public Actuator intakePivotActuator;

    public DigitalChannel intakeClawLeftBottom, intakeClawLeftTop,
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

    private final PhotonBNO055IMUNew imu = hardwareMap.get(PhotonBNO055IMUNew.class, "imu");
    public List<PhotonLynxModule> modules;

    private ArrayList<KSubsystem> subsystems;

    private double imuAngle;
    private double imuOffset;

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

        this.subsystems = new ArrayList<>();


        // TODO make sure all photon stuff is done
        for (PhotonLynxModule module : modules) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // TODO
        // Intake Pivot Actuator
        // Motion Profile
        // Error Tolerance
        this.intakePivotActuator = new Actuator(intakePivotLeftServo, intakePivotRightServo, intakePivotEncoder)
                .setMotionProfile(new AsymmetricMotionProfile(1, 1, new ProfileConstraints(1, 1, 1)))
                .setErrorTolerance(0.02);

        // TODO
        // Extension Pitch Actuator
        // PID
        // Motion Controller
        // Error Tolerance
        this.extensionPitchActuator = new Actuator((HardwareDevice) extensionPitchMotor, extensionPitchEncoder)
                .setPIDController(new PIDController(0, 0, 0))
//                .setMotionProfile(new AsymmetricMotionProfile(0, 1, new ProfileConstraints(1, 1, 1)))
                .setFeedforward(Actuator.FeedforwardMode.ANGLE_BASED, 0.0)
                .setErrorTolerance(20);
    }

    public void read() {
        for (KSubsystem subsystem : subsystems) {
            subsystem.read();
        }
    }

    public void write() {
        for (KSubsystem subsystem : subsystems) {
            subsystem.write();
        }
    }

    public void periodic() {
        if (voltageTimer.seconds() > 5) {
            voltageTimer.reset();
            voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        }

        for (KSubsystem subsystem : subsystems) {
            subsystem.periodic();
        }
    }

    public void reset() {
        for (KSubsystem subsystem : subsystems) {
            subsystem.reset();
        }
    }

    public void clearBulkCache() {
        for (PhotonLynxModule module : modules) {
            module.clearBulkCache();
        }
    }

    public void addSubsystem(KSubsystem... subsystems) {
        this.subsystems.addAll(Arrays.asList(subsystems));
    }

    @Nonnegative
    public double getVoltage() {
        return voltage;
    }

    @Nonnegative
    public double getAngle() {
        return imuAngle - imuOffset;
    }
}
