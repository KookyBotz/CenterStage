package org.firstinspires.ftc.teamcode.common.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
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
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.profile.ProfileConstraints;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WActuatorGroup;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WEncoder;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WSubsystem;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WServo;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import javax.annotation.Nonnegative;

@Config
public class RobotHardware {

    public static double servoOffset = -0.13;

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

    public WEncoder extensionEncoder;

    public WActuatorGroup pitchActuator;
    public WActuatorGroup extensionActuator;
    public WActuatorGroup intakePivotActuator;

    // OLD
    public WServo intakeClawLeftServo;
    public WServo intakeClawRightServo;
    public AbsoluteAnalogEncoder intakeClawLeftEncoder;
    public AbsoluteAnalogEncoder intakeClawRightEncoder;
    public WServo intakePivotLeftServo;
    public WServo intakePivotRightServo;
    public AbsoluteAnalogEncoder intakePivotEncoder;

    public DigitalChannel intakeClawLeftBottom, intakeClawLeftTop,
            intakeClawRightBottom, intakeClawRightTop;

    //TODO: Add 4x wall distance sensors
    //TODO: Add 2x Cameras

    /**
     * Odometry pod encoders.
     */
    public WEncoder podLeft;
    public WEncoder podRight;
    public WEncoder podFront;

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

    private ArrayList<WSubsystem> subsystems;

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
        if (Globals.USING_DASHBOARD) {
            this.telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
        } else {
            this.telemetry = telemetry;
        }


        this.subsystems = new ArrayList<>();

        modules = hardwareMap.getAll(LynxModule.class);
        modules.get(0).setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        modules.get(1).setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

//        this.imu = hardwareMap.get(BNO055IMU.class, "imu");
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
//        imu.initialize(parameters);

        // DRIVETRAIN
        this.dtBackLeftMotor = hardwareMap.get(DcMotorEx.class, "dtBackLeftMotor");
        dtBackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.dtFrontLeftMotor = hardwareMap.get(DcMotorEx.class, "dtFrontLeftMotor");
        dtFrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.dtBackRightMotor = hardwareMap.get(DcMotorEx.class, "dtBackRightMotor");
        dtBackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dtBackRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.dtFrontRightMotor = hardwareMap.get(DcMotorEx.class, "dtFrontRightMotor");
        dtFrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dtFrontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // UWUXTENSION
        extensionMotor = hardwareMap.get(DcMotorEx.class, "extensionMotor");
        armMotor = hardwareMap.get(DcMotorEx.class, "extensionPitchMotor");
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        extensionEncoder = new WEncoder(new MotorEx(hardwareMap, "dtFrontLeftMotor").encoder);

        this.extensionPitchEnc = hardwareMap.get(AnalogInput.class, "extensionPitchEncoder");
        this.extensionPitchEncoder = new AbsoluteAnalogEncoder(extensionPitchEnc);
        extensionPitchEncoder.zero(2.086);
        extensionPitchEncoder.setInverted(true);
        extensionPitchEncoder.setWraparound(true);

        // TODO: tune extension, motion profile, feedforward, and error tolerance
        this.extensionActuator = new WActuatorGroup(extensionMotor, extensionEncoder)
                .setPIDController(new PIDController(0.02, 0.0, 0.001))
                .setMotionProfile(0, new ProfileConstraints(1000, 5000, 2000));

        this.pitchActuator = new WActuatorGroup(armMotor, extensionPitchEncoder)
                .setPIDController(new PIDController(1, 0, 0.05))
                .setMotionProfile(0, new ProfileConstraints(4, 20, 2.5))
                .setFeedforward(WActuatorGroup.FeedforwardMode.ANGLE_BASED, 0.0, 0.13)
                .setErrorTolerance(0.03);

        // INTAKE
        intakeClawLeftServo = new WServo(hardwareMap.get(Servo.class, "servo1"));
        intakeClawRightServo = new WServo(hardwareMap.get(Servo.class, "servo2"));
        intakeClawRightServo.setDirection(Servo.Direction.REVERSE);

        this.intakePivotLeftServo = new WServo(hardwareMap.get(Servo.class, "servo3"));
        this.intakePivotRightServo = new WServo(hardwareMap.get(Servo.class, "servo4"));
        intakePivotRightServo.setDirection(Servo.Direction.REVERSE);
        intakePivotRightServo.setOffset(servoOffset);

        this.intakePivotActuator = new WActuatorGroup(intakePivotLeftServo, intakePivotRightServo);

        this.podLeft = new WEncoder(new MotorEx(hardwareMap, "dtFrontRightMotor").encoder);
        this.podFront = new WEncoder(new MotorEx(hardwareMap, "dtBackRightMotor").encoder);
        this.podRight = new WEncoder(new MotorEx(hardwareMap, "dtBackLeftMotor").encoder);
    }

    public void read() {
//        imuAngle = imu.getAngularOrientation().firstAngle;
        for (WSubsystem subsystem : subsystems) {
            subsystem.read();
        }
    }

    public void write() {
        for (WSubsystem subsystem : subsystems) {
            subsystem.write();
        }
    }

    public void periodic() {
//        if (voltageTimer.seconds() > 5) {
//            voltageTimer.reset();
//            voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
//        }

        for (WSubsystem subsystem : subsystems) {
            subsystem.periodic();
        }
    }

    public void reset() {
        for (WSubsystem subsystem : subsystems) {
            subsystem.reset();
        }
    }

    public void clearBulkCache() {
        modules.get(0).clearBulkCache();
        modules.get(1).clearBulkCache();
    }

    public void addSubsystem(WSubsystem... subsystems) {
        this.subsystems.addAll(Arrays.asList(subsystems));
    }

    // TODO add offset
    // imuAngle - imuOffset;
    @Nonnegative
    public double getAngle() {
        return imuAngle;
    }
}
