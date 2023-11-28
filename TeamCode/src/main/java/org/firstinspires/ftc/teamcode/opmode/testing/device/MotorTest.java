package org.firstinspires.ftc.teamcode.opmode.testing.device;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.common.drive.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Pose;

import java.util.List;

@Config
@TeleOp(name = "DT-TEST3")
@Disabled
public class MotorTest extends OpMode {
    public DcMotorEx motor1;
    public DcMotorEx motor2;
    public DcMotorEx motor3;
    public DcMotorEx motor4;

    BNO055IMU imu;

    public MecanumDrivetrain dt;
    private YawPitchRollAngles angles;

    private List<LynxModule> modules;

    private double imuOffset = 0.0;
    private boolean start = false;

    @Override
    public void init() {
//        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        this.imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        modules = hardwareMap.getAll(LynxModule.class);
        modules.get(0).setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        modules.get(1).setBulkCachingMode(LynxModule.BulkCachingMode.OFF);

//        for(PhotonLynxModule module: modules) module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        this.motor1 = hardwareMap.get(DcMotorEx.class, "dtBackLeftMotor");
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // motor1 = back left

        this.motor2 = hardwareMap.get(DcMotorEx.class, "dtFrontLeftMotor");
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // front left

        this.motor3 = hardwareMap.get(DcMotorEx.class, "dtBackRightMotor");
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor3.setDirection(DcMotorSimple.Direction.REVERSE);
        // back right

        this.motor4 = hardwareMap.get(DcMotorEx.class, "dtFrontRightMotor");
        motor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor4.setDirection(DcMotorSimple.Direction.REVERSE);
        // front right

        telemetry.addLine("here1");
        telemetry.update();

        dt = new MecanumDrivetrain(motor2, motor3, motor1, motor4);

        telemetry.addLine("here");
        telemetry.update();
    }

    @Override
    public void loop() {
        modules.get(0).clearBulkCache();
//        modules.get(1).clearBulkCache();
//
//        imu.scheduleInterleavedCommand(new PhotonLynxGetBulkInputDataCommand(modules.get(0)));
//        angles = imu.getRobotYawPitchRollAngles();
//        modules.get(0).feedBulkData((LynxGetBulkInputDataResponse)imu.getResult());

        double imuAngle = imu.getAngularOrientation().firstAngle;
        if (!start) {
            imuOffset = imuAngle;
            start = true;
        }

        dt.set(new Pose(gamepad1.left_stick_x, -gamepad1.left_stick_y, joystickScalar(-gamepad1.left_trigger + gamepad1.right_trigger, 0.01)), imuAngle);
        dt.write();

        telemetry.addData("YAW: ", imuAngle);
        telemetry.addData("imu offset", imuOffset);
        telemetry.addData("angle: ", imuAngle);
        telemetry.addLine(gamepad1.left_stick_x + " " + gamepad1.left_stick_y + " " + gamepad1.right_stick_x);
        telemetry.update();

        modules.get(0).clearBulkCache();
//        modules.get(1).clearBulkCache();
    }

    private double joystickScalar(double num, double min) {
        return joystickScalar(num, min, 0.66, 4);
    }

    private double joystickScalar(double n, double m, double l, double a) {
        return Math.signum(n) * m
                + (1 - m) *
                (Math.abs(n) > l ?
                        Math.pow(Math.abs(n), Math.log(l / a) / Math.log(l)) * Math.signum(n) :
                        n / a);
    }
}
