package org.firstinspires.ftc.teamcode.common.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.drive.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.common.drive.localizer.ThreeWheelLocalizer;
import org.firstinspires.ftc.teamcode.common.subsystem.DroneSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystem.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystem.HangSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.util.InverseKinematics;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WActuatorGroup;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WEncoder;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WSubsystem;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WServo;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

import javax.annotation.Nonnegative;

@Config
public class RobotHardware {

    //drivetrain
    public DcMotorEx dtFrontLeftMotor;
    public DcMotorEx dtFrontRightMotor;
    public DcMotorEx dtBackLeftMotor;
    public DcMotorEx dtBackRightMotor;

    // extension
    public AbsoluteAnalogEncoder armPitchEncoder;
    public AnalogInput armPitchEnc;

    public DcMotorEx extensionMotor;
    public DcMotorEx armMotor;

    public WEncoder extensionEncoder;

    public WActuatorGroup armActuator;
    public WActuatorGroup extensionActuator;
    public WActuatorGroup intakePivotActuator;

    public WServo armLiftServo;

    public WServo intakeClawLeftServo;
    public WServo intakeClawRightServo;
    public WServo intakePivotLeftServo;
    public WServo intakePivotRightServo;

    public WEncoder podLeft;
    public WEncoder podRight;
    public WEncoder podFront;

    public WServo droneTrigger;

    public CRServoImplEx leftHang;
    public CRServoImplEx rightHang;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    private ElapsedTime voltageTimer = new ElapsedTime();
    private double voltage = 12.0;

    private static RobotHardware instance = null;
    public boolean enabled;

    private BNO055IMU imu;
    public List<LynxModule> modules;

    private double imuAngle;

    private ArrayList<WSubsystem> subsystems;

    public IntakeSubsystem intake;
    public ExtensionSubsystem extension;
    public MecanumDrivetrain drivetrain;
    public DroneSubsystem drone;
    public HangSubsystem hang;
    public ThreeWheelLocalizer localizer;

    public HashMap<Sensors.SensorType, Object> values;

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
     * @param telemetry   Saved for later in the event FTC Dashboard used
     */
    public void init(final HardwareMap hardwareMap, final Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.values = new HashMap<>();
        this.telemetry = (Globals.USING_DASHBOARD) ? new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry()) : telemetry;

        values.put(Sensors.SensorType.EXTENSION_ENCODER, 0);
        values.put(Sensors.SensorType.ARM_ENCODER, 0.0);
        values.put(Sensors.SensorType.POD_LEFT, 0.0);
        values.put(Sensors.SensorType.POD_FRONT, 0.0);
        values.put(Sensors.SensorType.POD_RIGHT, 0.0);

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

        this.armPitchEnc = hardwareMap.get(AnalogInput.class, "extensionPitchEncoder");
        this.armPitchEncoder = new AbsoluteAnalogEncoder(armPitchEnc);
        armPitchEncoder.zero(2.086);
        armPitchEncoder.setInverted(true);
        armPitchEncoder.setWraparound(true);

        this.extensionActuator = new WActuatorGroup(
                () -> intSubscriber(Sensors.SensorType.EXTENSION_ENCODER), extensionMotor)
                .setPIDController(new PIDController(0.008, 0.0, 0.0004))
                .setFeedforward(WActuatorGroup.FeedforwardMode.CONSTANT, 0.0)
//                .setMotionProfile(0, new ProfileConstraints(1000, 5000, 2000))
                .setErrorTolerance(20);

        this.armActuator = new WActuatorGroup(
                () -> doubleSubscriber(Sensors.SensorType.ARM_ENCODER), armMotor)
                .setPIDController(new PIDController(1.7500, 0, 0.05))
                .setFeedforward(WActuatorGroup.FeedforwardMode.CONSTANT, 0.0)
                .setErrorTolerance(0.03);

        armLiftServo = new WServo(hardwareMap.get(Servo.class, "lift"));

        // INTAKE
        intakeClawLeftServo = new WServo(hardwareMap.get(Servo.class, "servo1"));
        intakeClawRightServo = new WServo(hardwareMap.get(Servo.class, "servo2"));
        intakeClawRightServo.setDirection(Servo.Direction.REVERSE);

        this.intakePivotLeftServo = new WServo(hardwareMap.get(Servo.class, "servo3"));
        intakePivotLeftServo.setOffset(-0.03);
        this.intakePivotRightServo = new WServo(hardwareMap.get(Servo.class, "servo4"));
        intakePivotRightServo.setOffset(0.01);
        intakePivotRightServo.setDirection(Servo.Direction.REVERSE);

        this.intakePivotActuator = new WActuatorGroup(intakePivotLeftServo, intakePivotRightServo);

        this.podLeft = new WEncoder(new MotorEx(hardwareMap, "dtFrontRightMotor").encoder);
        this.podFront = new WEncoder(new MotorEx(hardwareMap, "dtBackRightMotor").encoder);
        this.podRight = new WEncoder(new MotorEx(hardwareMap, "dtBackLeftMotor").encoder);

        this.droneTrigger = new WServo(hardwareMap.get(Servo.class, "drone"));

        // TODO: Configure config names
//        this.leftHang = hardwareMap.get(CRServoImplEx.class, "leftHang");
//        this.leftHang.setPwmRange(new PwmControl.PwmRange(500, 2500));
//        this.rightHang = hardwareMap.get(CRServoImplEx.class, "rightHang");
//        this.rightHang.setPwmRange(new PwmControl.PwmRange(500, 2500));

        InverseKinematics.calculateTarget(3, 0);

        modules = hardwareMap.getAll(LynxModule.class);
        modules.get(0).setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        modules.get(1).setBulkCachingMode(LynxModule.BulkCachingMode.OFF);

        subsystems = new ArrayList<>();
        drivetrain = new MecanumDrivetrain();
        extension = new ExtensionSubsystem();
        intake = new IntakeSubsystem();
        if (Globals.IS_AUTO) localizer = new ThreeWheelLocalizer();
        else {
            drone = new DroneSubsystem();
            hang = new HangSubsystem();
        }

        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
    }

    public void read() {
        // Read all hardware devices here
        values.put(Sensors.SensorType.EXTENSION_ENCODER, extensionEncoder.getPosition());
        values.put(Sensors.SensorType.ARM_ENCODER, armPitchEncoder.getCurrentPosition());
        if (Globals.IS_AUTO) {
            values.put(Sensors.SensorType.POD_LEFT, podLeft.getPosition());
            values.put(Sensors.SensorType.POD_FRONT, podFront.getPosition());
            values.put(Sensors.SensorType.POD_RIGHT, podRight.getPosition());
        }

        if (Globals.IS_USING_IMU) ; // read imu here
    }

    public void write() {
        extension.write();
        intake.write();
        drivetrain.write();
    }

    public void periodic() {
        if (voltageTimer.seconds() > 5) {
            voltageTimer.reset();
            voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        }

        intake.periodic();
        extension.periodic();
        drivetrain.periodic();
        if (Globals.IS_AUTO) localizer.periodic();
    }

    public void reset() {
        for (WSubsystem subsystem : subsystems) {
            subsystem.reset();
        }
    }

    public void clearBulkCache() {
        modules.get(0).clearBulkCache();
//        modules.get(1).clearBulkCache();
    }

    public void addSubsystem(WSubsystem... subsystems) {
        this.subsystems.addAll(Arrays.asList(subsystems));
    }

    @Nonnegative
    public double getAngle() {
        return imuAngle;
    }

    public double getVoltage() {
        return voltage;
    }

    public double doubleSubscriber(Sensors.SensorType topic) {
        Object value = values.getOrDefault(topic, 0.0);
        if (value instanceof Integer) {
            return ((Integer) value).doubleValue();
        } else if (value instanceof Double) {
            return (Double) value;
        } else {
            throw new ClassCastException();
        }
    }

    public int intSubscriber(Sensors.SensorType topic) {
        Object value = values.getOrDefault(topic, 0);
        if (value instanceof Integer) {
            return (Integer) value;
        } else if (value instanceof Double) {
            return ((Double) value).intValue();
        } else {
            throw new ClassCastException();
        }
    }

    public boolean boolSubscriber(Sensors.SensorType topic) {
        return (boolean) values.getOrDefault(topic, 0);
    }
}
