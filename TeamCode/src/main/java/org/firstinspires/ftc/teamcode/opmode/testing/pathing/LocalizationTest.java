package org.firstinspires.ftc.teamcode.opmode.testing.pathing;

import android.util.Size;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.drive.localizer.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.util.MathUtils;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "LocalizationTest")
public class LocalizationTest extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    private double loopTime = 0.0;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        Globals.IS_AUTO = true;

        robot.init(hardwareMap, telemetry);

        robot.dtBackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        robot.dtFrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        robot.dtBackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.dtFrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.read();

        robot.localizer.setPose(new Pose(63.65, 39.35, Math.PI / 2));

        while (!isStarted()) {
            telemetry.addLine("auto in init");
            telemetry.update();
        }
    }

    @Override
    public void run() {

        robot.read();
        Pose drive = new Pose(gamepad1.left_stick_x, -gamepad1.left_stick_y, MathUtils.joystickScalar(-gamepad1.left_trigger + gamepad1.right_trigger, 0.01));
        robot.drivetrain.set(drive, 0);

        super.run();
        robot.localizer.periodic();
        robot.drivetrain.periodic();

        Pose currentPose = robot.localizer.getPose();
        Pose globalTagPosition = robot.getAprilTagPosition();

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        loopTime = loop;
        telemetry.addLine(globalTagPosition.toString());
        telemetry.addLine(currentPose.toString());
        telemetry.update();

        if (gamepad1.a) robot.localizer.setPose(globalTagPosition);

        robot.drivetrain.write();
        robot.clearBulkCache();

        if (isStopRequested()) robot.closeCamera();
    }
}
