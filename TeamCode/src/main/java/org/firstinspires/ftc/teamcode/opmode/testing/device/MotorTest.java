package org.firstinspires.ftc.teamcode.opmode.testing.device;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.Photon;
import com.outoftheboxrobotics.photoncore.hardware.PhotonLynxModule;
import com.outoftheboxrobotics.photoncore.hardware.i2c.imu.PhotonBNO055IMUNew;
import com.outoftheboxrobotics.photoncore.hardware.motor.commands.PhotonLynxGetBulkInputDataCommand;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.common.drive.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Pose;

import java.util.List;

@Config
@Photon
@TeleOp(name = "DT-TEST")
public class MotorTest extends OpMode {
    public DcMotorEx motor1;
    public DcMotorEx motor2;
    public DcMotorEx motor3;
    public DcMotorEx motor4;

    PhotonBNO055IMUNew imu;
    List<PhotonLynxModule> modules;

    public MecanumDrivetrain dt;

    private double imuOffset = 0.0;

    @Override
    public void init() {
//        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        this.imu = hardwareMap.get(PhotonBNO055IMUNew.class, "imu");
        modules = hardwareMap.getAll(PhotonLynxModule.class);

        for(PhotonLynxModule module: modules) module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        this.motor1 = hardwareMap.get(DcMotorEx.class, "motor1");
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // motor1 = back left

        this.motor2 = hardwareMap.get(DcMotorEx.class, "motor2");
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // front left

        this.motor3 = hardwareMap.get(DcMotorEx.class, "motor3");
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // back right

        this.motor4 = hardwareMap.get(DcMotorEx.class, "motor4");
        motor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // front right

        telemetry.addLine("here1");
        telemetry.update();

        dt = new MecanumDrivetrain(motor2, motor3, motor1, motor4);

//        imu = hardwareMap.get(PhotonBNO055IMUNew.class, "imu");

        telemetry.addLine("here");
        telemetry.update();
    }

    @Override
    public void loop() {
        modules.get(0).clearBulkCache();
        modules.get(1).clearBulkCache();

        imu.scheduleInterleavedCommand(new PhotonLynxGetBulkInputDataCommand(modules.get(0)));
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        modules.get(0).feedBulkData((LynxGetBulkInputDataResponse)imu.getResult());

//        if (gamepad1.a) {
//            motor1.setPower(0.2);
//            // back left
//        }
//
//        if (gamepad1.b) {
//            motor2.setPower(0.2);
//            // front left
//        }
//
//        if (gamepad1.x) {
//            motor3.setPower(0.2);
//            // back right
//        }
//
//        if (gamepad1.y) {
//            motor4.setPower(0.2);
//            // front right
//        }

        double yaw = angles.getYaw(AngleUnit.RADIANS);
        if (gamepad1.right_stick_button) {
            imuOffset = yaw + Math.PI;
        }
        double angle = yaw - imuOffset;

        dt.set(new Pose(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x));
        dt.write();

        telemetry.addData("YAW: ", angles.getYaw(AngleUnit.RADIANS));
//        telemetry.addData("PITCH: ", angles.getPitch(AngleUnit.RADIANS));
//        telemetry.addData("ROLL: ", angles.getRoll(AngleUnit.RADIANS));
        telemetry.addData("imu offset", imuOffset);
        telemetry.addData("angle: ", angle);
//        telemetry.addLine(dt.toString());
        telemetry.addLine(gamepad1.left_stick_x + " " + gamepad1.left_stick_y + " " + gamepad1.right_stick_x);
        telemetry.update();
    }
}
