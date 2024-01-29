package org.firstinspires.ftc.teamcode.opmode.auto;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.centerstage.ClawSide;
import org.firstinspires.ftc.teamcode.common.commandbase.drivecommand.PositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.preloadautocommand.PurplePixelExtendCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.preloadautocommand.PurplePixelRetractCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.preloadautocommand.YellowPixelExtendCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.preloadautocommand.YellowPixelRetractCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.PivotStateCommand;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.vision.Location;
import org.firstinspires.ftc.teamcode.common.vision.PropPipeline;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous(name = "🔴 Red Close Preload Auto")
public class RedClosePreloadAuto extends LinearOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();

    private double loopTime = 0.0;
    private final ElapsedTime timer = new ElapsedTime();
    private double endTime = 0;

    private PropPipeline propPipeline;
    private VisionPortal portal;
    private Location randomization;


    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().reset();

        Globals.IS_AUTO = true;
        Globals.ALLIANCE = Location.RED;
        Globals.SIDE = Location.CLOSE;

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);


        robot.init(hardwareMap);

        robot.intake.updateState(IntakeSubsystem.ClawState.CLOSED, ClawSide.BOTH);

        robot.localizer.setPose(new Pose());

        propPipeline = new PropPipeline();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .setCameraResolution(new Size(1920, 1080))
                .addProcessor(propPipeline)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        while (robot.getCameraState() != VisionPortal.CameraState.STREAMING && portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addLine("initializing... please wait");
            telemetry.update();
        }

        while (opModeInInit()) {
            telemetry.addLine("ready");
            telemetry.addData("position", propPipeline.getLocation());
            telemetry.update();
        }

        randomization = propPipeline.getLocation();
        portal.close();

        Pose yellowScorePos = new Pose();
        Pose purpleScorePos = new Pose();
        Pose parkPos = new Pose(31, 6, -3 * Math.PI / 2);


        // 0.3, 300

        switch (randomization) {
            case RIGHT:
                yellowScorePos = new Pose(24, 21.5, -1.52);
                purpleScorePos = new Pose(25, 26, -1.52);
                break;
            case CENTER:
                yellowScorePos = new Pose(24, 27.75, -1.52);
                purpleScorePos = new Pose(18, 36, -1.52);
                break;
            case LEFT:
                yellowScorePos = new Pose(24, 34.25, -1.52);
                purpleScorePos = new Pose(4, 25, -1.52);
                break;
            default:
                break;

        }


        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        // go to yellow pixel scoring pos
                        new PositionCommand(yellowScorePos)
                                .alongWith(new YellowPixelExtendCommand()),

                        // score yellow pixel
                        new ClawCommand(IntakeSubsystem.ClawState.INTERMEDIATE, ClawSide.LEFT),
                        new WaitCommand(200),

                        // retract
                        new YellowPixelRetractCommand(),

                        // go to purple pixel scoring pos
                        new PositionCommand(purpleScorePos)
                                .alongWith(new PurplePixelExtendCommand()),

                        // score purple pixel
                        new WaitCommand(500),
                        new ClawCommand(IntakeSubsystem.ClawState.OPEN, ClawSide.RIGHT),
                        new WaitCommand(350),

                        new PurplePixelRetractCommand(),

                        new PositionCommand(parkPos)
                                .alongWith(new WaitCommand(400).andThen(new PivotStateCommand(IntakeSubsystem.PivotState.STORED))),

                        new InstantCommand(robot::closeCamera)
                )
        );

        while (opModeIsActive() && !isStopRequested()) {
            CommandScheduler.getInstance().run();
            robot.clearBulkCache();
            robot.read();
            robot.periodic();
            robot.write();

            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            telemetry.addLine(robot.localizer.getPose().toString());
            telemetry.addData("Runtime: ", endTime == 0 ? timer.seconds() : endTime);
            telemetry.update();

            loopTime = loop;
        }

        robot.kill();
    }
}