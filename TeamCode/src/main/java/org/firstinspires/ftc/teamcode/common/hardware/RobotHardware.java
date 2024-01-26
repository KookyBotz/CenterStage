package org.firstinspires.ftc.teamcode.common.hardware;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.drive.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.common.drive.localizer.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.common.drive.localizer.ThreeWheelLocalizer;
import org.firstinspires.ftc.teamcode.common.drive.localizer.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.subsystem.DroneSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystem.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystem.HangSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.util.InverseKinematics;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WActuatorGroup;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WEncoder;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WServo;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WSubsystem;
import org.firstinspires.ftc.teamcode.common.vision.Location;
import org.firstinspires.ftc.teamcode.common.vision.PreloadDetectionPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

import javax.annotation.concurrent.GuardedBy;

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

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    private ElapsedTime voltageTimer = new ElapsedTime();
    private double voltage = 12.0;

    private static RobotHardware instance = null;
    private boolean enabled;

    public List<LynxModule> modules;
    public LynxModule CONTROL_HUB;


    private ArrayList<WSubsystem> subsystems;

    public IntakeSubsystem intake;
    public ExtensionSubsystem extension;
    public MecanumDrivetrain drivetrain;
    public DroneSubsystem drone;
    public HangSubsystem hang;

    public PreloadDetectionPipeline preloadDetectionPipeline;

    private final Object imuLock = new Object();
    @GuardedBy("imuLock")
    public BNO055IMU imu;
    private Thread imuThread;
    private double imuAngle = 0;
    private double imuOffset = 0;
    private double startOffset = 0;
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
     */
    public void init(final HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.values = new HashMap<>();

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
        this.intakePivotRightServo = new WServo(hardwareMap.get(Servo.class, "servo4"));
        intakePivotRightServo.setOffset(0.04);
//        intakePivotRightServo.setOffset(0.01);
        intakePivotRightServo.setDirection(Servo.Direction.REVERSE);

        this.intakePivotActuator = new WActuatorGroup(intakePivotLeftServo, intakePivotRightServo);
        intakePivotActuator.setOffset(-0.05);

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

        this.preloadDetectionPipeline = new PreloadDetectionPipeline();

        for (LynxModule m : modules) {
            m.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            if (m.isParent() && LynxConstants.isEmbeddedSerialNumber(m.getSerialNumber())) CONTROL_HUB = m;
        }


        subsystems = new ArrayList<>();
        drivetrain = new MecanumDrivetrain();
        extension = new ExtensionSubsystem();
        intake = new IntakeSubsystem();
        if (Globals.IS_AUTO) {
            localizer = new ThreeWheelLocalizer();

            startCamera();

//            synchronized (imuLock) {
//                imu = hardwareMap.get(BNO055IMU.class, "imu");
//                BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//                parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
//                imu.initialize(parameters);
//            }
        } else {
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
    }

    public void write() {
        extension.write();
        intake.write();
        drivetrain.write();
    }

    public void periodic() {
//        if (voltageTimer.seconds() > 5) {
//            voltageTimer.reset();
//            voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
//        }

        intake.periodic();
        extension.periodic();
        drivetrain.periodic();
        if (Globals.IS_AUTO) localizer.periodic();
    }

    public void startIMUThread(LinearOpMode opMode) {
        imuThread = new Thread(() -> {
            while (!opMode.isStopRequested()) {
                synchronized (imuLock) {
                    imuAngle = AngleUnit.normalizeRadians(imu.getAngularOrientation().firstAngle + startOffset);
                }
            }
        });
        imuThread.start();
    }

    public double getAngle() {
        return AngleUnit.normalizeRadians(imuAngle - imuOffset);
    }

    public void reset() {
        for (WSubsystem subsystem : subsystems) {
            subsystem.reset();
        }

        imuOffset = imuAngle;
    }

    public void setStartOffset(double off) {
        startOffset = off;
    }

    public void clearBulkCache() {
        CONTROL_HUB.clearBulkCache();
    }

    public void addSubsystem(WSubsystem... subsystems) {
        this.subsystems.addAll(Arrays.asList(subsystems));
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

    public Pose getAprilTagPosition() {
        if (aprilTag != null && localizer != null) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();

            List<Pose> backdropPositions = new ArrayList<>();
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    switch (detection.id) {
                        case 1:
                        case 4:
                            backdropPositions.add(new Pose(detection.ftcPose).add(new Pose(6, 0, 0)));
                            break;
                        case 2:
                        case 5:
                            backdropPositions.add(new Pose(detection.ftcPose));
                            break;
                        case 3:
                        case 6:
                            backdropPositions.add(new Pose(detection.ftcPose).subt(new Pose(6, 0, 0)));
                            break;
                        default:
                            break;
                    }
                }
            }

            Pose backdropPosition = backdropPositions.stream().reduce(Pose::add).orElse(new Pose());
            backdropPosition = backdropPosition.divide(new Pose(backdropPositions.size(), backdropPositions.size(), backdropPositions.size()));


            Pose globalTagPosition = localizer.getPose().x > 0 ?
                    AprilTagLocalizer.convertBlueBackdropPoseToGlobal(backdropPosition) :
                    AprilTagLocalizer.convertRedBackdropPoseToGlobal(backdropPosition);

            if (Double.isNaN(globalTagPosition.x) || Double.isNaN(globalTagPosition.y) || Double.isNaN(globalTagPosition.heading)) return null;

            return globalTagPosition;
        } else {
            return null;
        }
    }

    public List<AprilTagDetection> getAprilTagDetections() {
        if (aprilTag != null && localizer != null) return aprilTag.getDetections();
        System.out.println("Active");
        return null;
    }

    public void startCamera() {
        aprilTag = new AprilTagProcessor.Builder()
                // calibrated using 3DF Zephyr 7.021
                .setLensIntrinsics(549.651, 549.651, 317.108, 236.644)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessors(aprilTag, preloadDetectionPipeline)
                .enableLiveView(false)
                .build();
    }

    public VisionPortal.CameraState getCameraState() {
        if (visionPortal != null) return visionPortal.getCameraState();
        return null;
    }

    public void closeCamera() {
        if (visionPortal != null) visionPortal.close();
    }

    public void kill() {
        instance = null;
    }



    public int getTargetIndex() {
        int index = 0;


    }
}
