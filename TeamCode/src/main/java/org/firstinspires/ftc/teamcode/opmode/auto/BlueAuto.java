package org.firstinspires.ftc.teamcode.opmode.auto;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.centerstage.ClawSide;
import org.firstinspires.ftc.teamcode.common.centerstage.Side;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.FirstDepositCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.FirstStackGrabCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.FirstStackSetupCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.PurplePixelDepositCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.PurplePixelExtendCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.SecondDepositCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.SecondStackGrabCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.drivecommand.PositionCommand;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

@Config
@Autonomous(name = "Blue Auto")
public class BlueAuto extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();


    private double loopTime = 0.0;
    private final ElapsedTime timer = new ElapsedTime();
    private double endTime = 0;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        Globals.IS_AUTO = true;
        Globals.IS_USING_IMU = false;
        Globals.USING_DASHBOARD = true;
        Globals.COLOR = Side.BLUE;

        robot.init(hardwareMap, telemetry);
        robot.enabled = true;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.intake.updateState(IntakeSubsystem.ClawState.CLOSED, ClawSide.BOTH);

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


        while (!isStarted()) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();

            List<Pose> backdropPositions = new ArrayList<>();
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    switch (detection.id) {
                        case 1:
                            backdropPositions.add(new Pose(detection.ftcPose).add(new Pose(6, 0, 0)));
                            break;
                        case 2:
                            backdropPositions.add(new Pose(detection.ftcPose));
                            break;
                        case 3:
                            backdropPositions.add(new Pose(detection.ftcPose).subt(new Pose(6, 0, 0)));
                            break;
                        default:
                            break;
                    }
                }
            }

            Pose backdropPosition = backdropPositions.stream().reduce(Pose::add).orElse(new Pose());
            backdropPosition = backdropPosition.divide(new Pose(backdropPositions.size(), backdropPositions.size(), backdropPositions.size()));

            telemetry.addLine(backdropPositions.toString());
            telemetry.addLine(backdropPosition.toString());
            telemetry.update();
        }



        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(timer::reset),
                        // go to yellow pixel scoring pos
                        new PositionCommand(new Pose(25, 0, 0))
                                .alongWith(new PurplePixelExtendCommand()),

                        new PurplePixelDepositCommand(),

                        new PositionCommand(new Pose(25, -0.25, -Math.PI / 2))
                                .alongWith(new FirstStackSetupCommand()),

                        new FirstStackGrabCommand(),

                        new PositionCommand(new Pose(27, -68.25, -Math.PI / 2))
                                .alongWith(new FirstDepositCommand()),

                        new PositionCommand(new Pose(26, 0.6, -Math.PI / 2)),

                        new SecondStackGrabCommand(),

                        new PositionCommand(new Pose(27, -68.25, -Math.PI / 2))
                                .alongWith(new SecondDepositCommand()),

                        new InstantCommand(() -> endTime = timer.seconds())

//
//                        new PositionCommand(new Pose(27, -68, -Math.PI/2))
//                                .alongWith(new AutoDepositCommand())
                )
        );
    }

    @Override
    public void run() {
        robot.read();
        super.run();
        robot.periodic();

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        telemetry.addLine(robot.localizer.getPose().toString());
        telemetry.addData("Runtime: ", endTime == 0 ? timer.seconds() : endTime);
        loopTime = loop;
        telemetry.update();

        robot.write();
        robot.clearBulkCache();
    }
}
