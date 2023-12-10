package org.firstinspires.ftc.teamcode.common.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.drive.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.common.drive.localizer.ThreeWheelLocalizer;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.profile.ProfileConstraints;
import org.firstinspires.ftc.teamcode.common.subsystem.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.util.InverseKinematics;
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

    public WServo intakeClawLeftServo;
    public WServo intakeClawRightServo;
    public WServo intakePivotLeftServo;
    public WServo intakePivotRightServo;

    public WEncoder podLeft;
    public WEncoder podRight;
    public WEncoder podFront;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    private ElapsedTime voltageTimer = new ElapsedTime();
    private double voltage = 12.0;

    private static RobotHardware instance = null;
    public boolean enabled;

    private BNO055IMU imu;
    public List<LynxModule> modules;

    private ArrayList<WSubsystem> subsystems;

    private double imuAngle ;

    private IntakeSubsystem intake;
    private ExtensionSubsystem extension;
    private MecanumDrivetrain drivetrain;
    private ThreeWheelLocalizer localizer;

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

        subsystems = new ArrayList<>();
        drivetrain = new MecanumDrivetrain();
        localizer = new ThreeWheelLocalizer();
        extension = new ExtensionSubsystem();
        intake = new IntakeSubsystem();

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

        this.extensionActuator = new WActuatorGroup(extensionMotor, extensionEncoder)
                .setPIDController(new PIDController(0.02, 0.0, 0.001))
                .setMotionProfile(0, new ProfileConstraints(1000, 5000, 2000))
                .setErrorTolerance(20);

        this.armActuator = new WActuatorGroup(armMotor, armPitchEncoder)
                .setPIDController(new PIDController(4, 0, 0.05))
                .setMotionProfile(0, new ProfileConstraints(6, 6, 5))
                .setFeedforward(WActuatorGroup.FeedforwardMode.ANGLE_BASED, 0.07, 0.2)
                .setErrorTolerance(0.03);

        // INTAKE
        intakeClawLeftServo = new WServo(hardwareMap.get(Servo.class, "servo1"));
        intakeClawRightServo = new WServo(hardwareMap.get(Servo.class, "servo2"));
        intakeClawRightServo.setDirection(Servo.Direction.REVERSE);

        this.intakePivotLeftServo = new WServo(hardwareMap.get(Servo.class, "servo3"));
        this.intakePivotRightServo = new WServo(hardwareMap.get(Servo.class, "servo4"));
        intakePivotRightServo.setDirection(Servo.Direction.REVERSE);
        intakePivotRightServo.setOffset(-0.13);

        this.intakePivotActuator = new WActuatorGroup(intakePivotLeftServo, intakePivotRightServo);

        this.podLeft = new WEncoder(new MotorEx(hardwareMap, "dtFrontRightMotor").encoder);
        this.podFront = new WEncoder(new MotorEx(hardwareMap, "dtBackRightMotor").encoder);
        this.podRight = new WEncoder(new MotorEx(hardwareMap, "dtBackLeftMotor").encoder);

        InverseKinematics.calculateTarget(3, 0);

        modules = hardwareMap.getAll(LynxModule.class);
        modules.get(0).setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        modules.get(1).setBulkCachingMode(LynxModule.BulkCachingMode.OFF);

        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
    }

    public void read() {
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
        if (voltageTimer.seconds() > 5) {
            voltageTimer.reset();
            voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        }

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

    @Nonnegative
    public double getAngle() {
        return imuAngle;
    }

    public double getVoltage() {
        return voltage;
    }

    public void log(String data) {
        telemetry.addLine(data);
    }

    public void log(String data, Object input) {
        telemetry.addData(data, input.toString());
    }
}
