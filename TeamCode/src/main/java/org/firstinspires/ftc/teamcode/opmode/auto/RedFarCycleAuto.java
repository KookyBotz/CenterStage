package org.firstinspires.ftc.teamcode.opmode.auto;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.centerstage.ClawSide;
import org.firstinspires.ftc.teamcode.common.commandbase.cycleautocommand.DepositRetractionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.cycleautocommand.FirstDepositCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.cycleautocommand.FirstDepositExtendCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.cycleautocommand.FirstStackGrabCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.cycleautocommand.FirstStackSetupCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.cycleautocommand.PreloadDetectionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.cycleautocommand.PurplePixelDepositCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.cycleautocommand.PurplePixelExtendCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.cycleautocommand.RelocalizeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.cycleautocommand.StackRelocalizeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.drivecommand.PositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.drivecommand.PurePursuitCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ArmFloatCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ArmLiftCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ExtensionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.PivotStateCommand;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.drive.pathing.purepursuit.PurePursuitPath;
import org.firstinspires.ftc.teamcode.common.drive.pathing.purepursuit.Waypoint;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.vision.Location;
import org.firstinspires.ftc.teamcode.common.vision.PropPipeline;
import org.firstinspires.ftc.teamcode.common.vision.StackPipeline;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Disabled
@Autonomous(name = "🔴 Red Far Cycle Auto")
public class RedFarCycleAuto extends LinearOpMode {
    private final RobotHardware robot = RobotHardware.getInstance();
    private final ElapsedTime timer = new ElapsedTime();
    private double loopTime = 0.0;
    private double endTime = 0;

    private PropPipeline propPipeline;
    private StackPipeline stackPipeline;
    private VisionPortal portal;
    private Location randomization;

    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().reset();

        Globals.IS_AUTO = true;
        Globals.ALLIANCE = Location.RED;
        Globals.SIDE = Location.FAR;
        Globals.ROUTE = Location.STAGEDOOR;

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        robot.init(hardwareMap);

        robot.intake.updateState(IntakeSubsystem.ClawState.CLOSED, ClawSide.BOTH);

        robot.localizer.setPose(new Pose(-63.65, 39.35, -Math.PI / 2));

        propPipeline = new PropPipeline();

        stackPipeline = new StackPipeline();

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .setCameraResolution(new Size(1920, 1080))
                .addProcessors(stackPipeline, propPipeline)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

//        portal.setProcessorEnabled(stackPipeline, true);

        FtcDashboard.getInstance().startCameraStream(stackPipeline, 0);

        while (robot.getCameraState() != VisionPortal.CameraState.STREAMING && portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addLine("initializing... please wait");
            telemetry.update();
        }

        double delay = 15;

        boolean pA = false;
        boolean pY = false;

        while (opModeInInit()) {
            telemetry.addLine("ready");
            telemetry.addData("position", propPipeline.getLocation());
            telemetry.addData("delay", delay);
            telemetry.update();

            boolean a = gamepad1.a;
            boolean y = gamepad1.y;

            if (a && !pA) delay -= 0.25;
            if (y && !pY) delay += 0.25;

            pA = a;
            pY = y;
        }

        randomization = propPipeline.getLocation();
        Globals.RANDOMIZATION = randomization;
        RobotHardware.getInstance().preloadDetectionPipeline.setTargetAprilTagID(randomization);

        Pose purplePixelPose;

        if (Globals.ROUTE == Location.STAGEDOOR) {
            switch (randomization) {
                case RIGHT:
                    purplePixelPose = new Pose(-49.25, 36.25, -2.16);
                    break;
                case LEFT:
                    purplePixelPose = new Pose(-53, 40, -1.17);
                    break;
                default:
                    purplePixelPose = new Pose(-43, 36.5, -1.88);
                    break;
            }
        } else {
            switch (randomization) {
                case RIGHT:
                    purplePixelPose = new Pose(-37.75, 25, -Math.PI / 2);
                    break;
                case LEFT:
                    purplePixelPose = new Pose(-37.75, 38.95, -0.95);
                    break;
                default:
                    purplePixelPose = new Pose(-37.75, 39.35, -Math.PI / 2);
                    break;
            }
        }


        double finalDelay = delay;
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(timer::reset),

                        new PositionCommand(purplePixelPose)
                                .alongWith(new PurplePixelExtendCommand()),

                        new PurplePixelDepositCommand(),


                        new ConditionalCommand(
                                new PositionCommand(new Pose(-15.5, 37.25, -Math.PI / 2)),
                                new PositionCommand(new Pose(-15.5, 39, -Math.PI / 2)),
                                () -> Globals.RANDOMIZATION == Location.LEFT)
                                .alongWith(new FirstStackSetupCommand()),

                        new PositionCommand(new Pose(-15.5, 39, 0)),

                        new FirstStackGrabCommand(),


                        new WaitUntilCommand(() -> timer.seconds() > finalDelay),


                        new InstantCommand(() -> robot.setProcessorEnabled(robot.preloadDetectionPipeline, true)),

                        new PurePursuitCommand(new PurePursuitPath(
                                new Waypoint(new Pose(-15.5, 39, 0), 15),
                                new Waypoint(new Pose(-14, -32, 0), 15)
                        )),

                        new PreloadDetectionCommand(),
                        new RelocalizeCommand(),
                        new WaitCommand(50),
                        new PreloadDetectionCommand(),

                        new InstantCommand(() -> robot.setProcessorEnabled(robot.preloadDetectionPipeline, false)),
                        new FirstDepositExtendCommand(),


                        new FirstDepositCommand(),
                        new ExtensionCommand(300),
                        new ClawCommand(IntakeSubsystem.ClawState.CLOSED, (Globals.ALLIANCE == Location.BLUE ? ClawSide.LEFT : ClawSide.RIGHT)),

                        new SequentialCommandGroup(
                                new WaitCommand(250),
                                new ArmCommand(2.82),
                                new ArmFloatCommand(false),
                                new ArmLiftCommand(0.3),
                                new WaitCommand(400),
                                new PivotStateCommand(IntakeSubsystem.PivotState.SCORING),
                                new ExtensionCommand(475),
                                new WaitCommand(250),
                                new ClawCommand(IntakeSubsystem.ClawState.OPEN, (Globals.ALLIANCE == Location.BLUE ? ClawSide.RIGHT : ClawSide.LEFT)),
                                new WaitCommand(250)

                        )
                                .alongWith(new PositionCommand(new Pose(-29, -35.25, 0))),
                        new DepositRetractionCommand(),

                        new PositionCommand(new Pose(-12, -50, 0))
                                .alongWith(new ExtensionCommand(0)),

                        new InstantCommand(() -> endTime = timer.seconds()),
                        new InstantCommand(robot::closeCamera),
                        new InstantCommand(this::requestOpModeStop)
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
//            telemetry.addData("CORRECTION", -stackPipeline.getStrafeCorrection());
            telemetry.addData("BACKDROP", RobotHardware.getInstance().preloadDetectionPipeline.getPreloadedZone());
            telemetry.addData("INDEX", Globals.getTargetIndex());
//            telemetry.addData("TARGET POSE", )
//            telemetry.addData("arm pos", robot.extensionActuator.getPosition());
//            telemetry.addLine("TAPE POSE (" + stackPipeline.getClosestTapeContour().x + " " + stackPipeline.getClosestTapeContour().y);
//            telemetry.addLine("PIXEL POSE (" + stackPipeline.getClosestPixelContour().x + " " + stackPipeline.getClosestPixelContour().y);
            telemetry.update();

            loopTime = loop;
        }

//        portal.setProcessorEnabled(stackPipeline, false);

        robot.kill();
    }
}
