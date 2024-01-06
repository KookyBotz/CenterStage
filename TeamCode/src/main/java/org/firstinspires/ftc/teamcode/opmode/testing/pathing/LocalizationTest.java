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

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private double loopTime = 0.0;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        Globals.IS_AUTO = true;
        Globals.IS_USING_IMU = false;

        robot.init(hardwareMap, telemetry);

        robot.dtBackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        robot.dtFrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        robot.dtBackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.dtFrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        robot.enabled = true;

        robot.read();

        aprilTag = new AprilTagProcessor.Builder()
                // calibrated using 3DF Zephyr 7.021
                .setLensIntrinsics(549.651, 549.651, 317.108, 236.644)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTag)
                .build();

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

        telemetry.addLine(currentPose.toString());
        telemetry.addLine(backdropPosition.toString());

        Pose globalTagPosition = currentPose.x > 0 ?
                AprilTagLocalizer.convertBlueBackdropPoseToGlobal(backdropPosition) :
                AprilTagLocalizer.convertRedBackdropPoseToGlobal(backdropPosition);

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        telemetry.addLine(drive.toString());
        loopTime = loop;
        telemetry.addLine(globalTagPosition.toString());
        telemetry.update();

        if (gamepad1.a) robot.localizer.setPose(globalTagPosition);

        robot.drivetrain.write();
        robot.clearBulkCache();
    }
}
