package org.firstinspires.ftc.teamcode.common.hardware;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.outoftheboxrobotics.photoncore.hardware.PhotonLynxModule;
import com.outoftheboxrobotics.photoncore.hardware.i2c.imu.PhotonBNO055IMUNew;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.profile.ProfileConstraints;
import org.firstinspires.ftc.teamcode.common.util.wrappers.KActuatorGroup;
import org.firstinspires.ftc.teamcode.common.util.wrappers.AnalogServo;
import org.firstinspires.ftc.teamcode.common.util.wrappers.KEncoder;
import org.firstinspires.ftc.teamcode.common.util.wrappers.KSubsystem;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import javax.annotation.Nonnegative;

public class RobotHardware {


    // TODO DONE:
    //drivetrain
    public DcMotorEx dtFrontLeftMotor;
    public DcMotorEx dtFrontRightMotor;
    public DcMotorEx dtBackLeftMotor;
    public DcMotorEx dtBackRightMotor;

    // extension
    public AbsoluteAnalogEncoder extensionPitchEncoder;
    public AnalogInput extensionPitchEnc;

    public DcMotorEx extensionMotor;
    public DcMotorEx armMotor;

    public KEncoder extensionEncoder;

    public KActuatorGroup pitchActuator;
    public KActuatorGroup extensionActuator;




    private YawPitchRollAngles angles;

    // OLD
//    public MotorEx extensionMotor;
//    public MotorEx extensionPitchMotor;
//    public AbsoluteAnalogEncoder extensionPitchEncoder;
//    public AnalogInput extensionPitchEnc;
//    public KActuatorGroup extensionPitchActuator;



    // TODO: Configure hardware map
    // TODO: Configure invert positions
    public AnalogServo intakeClawLeftServo;
    public AnalogServo intakeClawRightServo;

    // TODO: Configure hardware map
    // TODO: Configure invert positions
    public Servo intakePivotLeftServo;
    public Servo intakePivotRightServo;
    public AbsoluteAnalogEncoder intakePivotEncoder;
    public KActuatorGroup intakePivotActuator;

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

    private BNO055IMU imu;
    public List<LynxModule> modules;

    private ArrayList<KSubsystem> subsystems;

    private double imuAngle ;

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

        modules = hardwareMap.getAll(LynxModule.class);
        modules.get(0).setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        modules.get(1).setBulkCachingMode(LynxModule.BulkCachingMode.OFF);

        this.imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        // DRIVETRAIN
        this.dtBackLeftMotor = hardwareMap.get(DcMotorEx.class, "dtBackLeftMotor");
        dtBackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // motor1 = back left

        this.dtFrontLeftMotor = hardwareMap.get(DcMotorEx.class, "dtFrontLeftMotor");
        dtFrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // front left

        this.dtBackRightMotor = hardwareMap.get(DcMotorEx.class, "dtBackRightMotor");
        dtBackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dtBackRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // back right

        this.dtFrontRightMotor = hardwareMap.get(DcMotorEx.class, "dtFrontRightMotor");
        dtFrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dtFrontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // UWUXTENSION
        extensionMotor = hardwareMap.get(DcMotorEx.class, "extensionMotor");
        armMotor = hardwareMap.get(DcMotorEx.class, "extensionPitchMotor");
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        extensionEncoder = new KEncoder(new MotorEx(hardwareMap, "dtFrontLeftMotor").encoder);

        this.extensionPitchEnc = hardwareMap.get(AnalogInput.class, "extensionPitchEncoder");
        this.extensionPitchEncoder = new AbsoluteAnalogEncoder(extensionPitchEnc);
        extensionPitchEncoder.zero(2.086);
        extensionPitchEncoder.setInverted(true);
        extensionPitchEncoder.setWraparound(true);

        // TODO: add lift actuator here
        this.extensionActuator = new KActuatorGroup(extensionMotor, extensionEncoder)
                .setPIDController(new PIDController(0.016379, 0.0, 0.0254))
//                .setMotionProfile(new ProfileConstraints(0, 0, 0))
                .setFeedforward(KActuatorGroup.FeedforwardMode.CONSTANT, 0.0);

        pitchActuator = new KActuatorGroup(armMotor, extensionPitchEncoder)
                .setPIDController(new PIDController(1.3, 0, 0.035))
                .setMotionProfile(Math.PI / 2, new ProfileConstraints(4.7, 20, 7.5))
                .setFeedforward(KActuatorGroup.FeedforwardMode.ANGLE_BASED, 0.05, 0.13);
    }

    public void read() {
        imuAngle = imu.getAngularOrientation().firstAngle;
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
//        if (voltageTimer.seconds() > 5) {
//            voltageTimer.reset();
//            voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
//        }

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
//        for (LynxModule module : modules) {
//            module.clearBulkCache();
//        }
        modules.get(0).clearBulkCache();
    }

    public void addSubsystem(KSubsystem... subsystems) {
        this.subsystems.addAll(Arrays.asList(subsystems));
    }

    // TODO add offset
    // imuAngle - imuOffset;
    @Nonnegative
    public double getAngle() {
        return imuAngle;
    }
}
