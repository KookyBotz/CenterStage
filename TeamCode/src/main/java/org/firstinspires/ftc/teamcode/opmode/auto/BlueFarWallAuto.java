package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.common.drive.localizer.FusedLocalizer.calculateDistance;

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
import org.firstinspires.ftc.teamcode.common.commandbase.cycleautocommand.DepositRetractionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.cycleautocommand.FirstDepositCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.cycleautocommand.FirstDepositExtendCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.cycleautocommand.PreloadDetectionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.cycleautocommand.PurplePixelDepositCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.cycleautocommand.PurplePixelExtendCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.cycleautocommand.RelocalizeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.drivecommand.PositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.drivecommand.PurePursuitCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ArmFloatCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ArmLiftCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ExtensionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.PivotStateCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.wallauto.StackGrabCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.wallauto.StackSetupCommand;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Point;
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
@Autonomous(name = "🔵 Blue Far Wall Auto")
public class BlueFarWallAuto extends LinearOpMode {
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
        Globals.ALLIANCE = Location.BLUE;
        Globals.SIDE = Location.FAR;
        Globals.ROUTE = Location.STAGEDOOR;

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        robot.init(hardwareMap);

        robot.intake.updateState(IntakeSubsystem.ClawState.CLOSED, ClawSide.BOTH);

        robot.localizer.setPose(new Pose(63.65, 39.35, Math.PI / 2));
        robot.setIMUStartOffset(Math.PI / 2);

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

        double delay = 5;

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
//        randomization = Location.RIGHT;
        Globals.RANDOMIZATION = randomization;
        RobotHardware.getInstance().preloadDetectionPipeline.setTargetAprilTagID(randomization);

        Pose purplePixelPose;

        switch (randomization) {
            case LEFT:
                purplePixelPose = new Pose(49.25, 36.25, 2.16);
                break;
            case RIGHT:
                purplePixelPose = new Pose(52.5, 40, 1.17);
                break;
            default:
                purplePixelPose = new Pose(43.5, 36.5, 1.88);
                break;
        }

        Pose INTAKE = new Pose(44.75, 47.5, 0.25);
        Pose INTAKE_2 = new Pose(44.75, 48, 0.25);

        Pose INTERMEDIATE = new Pose(48, 40, 0);

        Pose DEPOSIT_1 = new Pose(42.5, -35.25, 0);
        Pose DEPOSIT_2 = new Pose(42.5, -35.25, 0);

        double finalDelay = delay;
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(timer::reset),

                        new PositionCommand(purplePixelPose)
                                .alongWith(new PurplePixelExtendCommand()),

                        new PurplePixelDepositCommand(),

                        new PositionCommand(INTAKE)
                                .alongWith(new StackSetupCommand(0.7, 0.53)),


                        new WaitCommand(520),
                        new InstantCommand(() -> robot.localizer.setLateral(getDistanceMeasurement())),
//                        new InstantCommand(() -> robot.localizer.setHeading(robot.getAngle())),

                        new WaitCommand(230),

                        new PositionCommand(INTAKE),


                        new StackGrabCommand(),

                        new InstantCommand(() -> robot.setProcessorEnabled(robot.preloadDetectionPipeline, true)),

                        new PurePursuitCommand(new PurePursuitPath(
                                new Waypoint(INTAKE, 15),
                                new Waypoint(new Point(60, 24), 15),
                                new Waypoint(new Pose(58, -24, 0), 15))
                        ),

                        new PreloadDetectionCommand(),
                        new RelocalizeCommand(),
                        new WaitCommand(50),
                        new PreloadDetectionCommand(),

                        new InstantCommand(() -> robot.setProcessorEnabled(robot.preloadDetectionPipeline, false)),

                        new FirstDepositExtendCommand(),
                        new FirstDepositCommand(),

                        new InstantCommand(() -> PositionCommand.DEAD_MS = 1000),
                        new SequentialCommandGroup(
                                new ExtensionCommand(300),
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
                                .alongWith(new WaitCommand(250).andThen(new PositionCommand(new Pose(40, -35.25, 0)))),
                        new DepositRetractionCommand(),

                        new ExtensionCommand(0),

                        new InstantCommand(() -> PositionCommand.DEAD_MS = 2520),


//                        new PurePursuitCommand(new PurePursuitPath(
//                                new Waypoint(new Pose(60, -12, 0), 15),
//                                new Waypoint(new Point(58, 32), 15),
//                                new Waypoint(INTERMEDIATE, 15)
//                        )),
//
//                        
//
//                        new WaitCommand(520),
//
//                        new InstantCommand(() -> robot.localizer.setLateral(getDistanceMeasurement())),
//                        new InstantCommand(()->robot.localizer.setHeading(robot.getAngle())),
//
//
//                        new WaitCommand(230),
//
//                        new InstantCommand(() -> System.out.println("heading correction: " + (INTAKE_2.heading - (robot.getAngle() - INTAKE_2.heading)))),
//                        new PositionCommand(INTAKE_2)
//                                .alongWith(new StackSetupCommand(0.74, 0.54)),
//
//                        
//
//                        new StackGrabCommand(),
//
//                        new PurePursuitCommand(new PurePursuitPath(
//                                new Waypoint(INTAKE_2, 15),
//                                new Waypoint(new Point(60, 28), 15),
//                                new Waypoint(new Pose(58, -18, 0), 15),
//                                new Waypoint(DEPOSIT_1, 15))
//                        ),
//
//                        new RelocalizeCommand(),
//
//                        new PositionCommand(DEPOSIT_1)
//                                .alongWith(new DepositExtendCommand(2.82, 0.755, 300)),
//
//                        new StackDepositCommand(465),
//
//                        // CYCLE TWO
//
//                        new PurePursuitCommand(new PurePursuitPath(
//                                new Waypoint(new Pose(60, -12, 0), 15),
//                                new Waypoint(new Point(58, 32), 15),
//                                new Waypoint(INTERMEDIATE, 15)
//                        )),
//
//                        
//
//
//                        new WaitCommand(520),
//
//                        new InstantCommand(() -> robot.localizer.setLateral(getDistanceMeasurement())),
//                        new InstantCommand(()->robot.localizer.setHeading(robot.getAngle())),
//
//
//                        new WaitCommand(230),
//
//                        new PositionCommand(INTAKE_2)
//                                .alongWith(new StackSetupCommand(0.3, 0.53)),
//
//                        
//
//
//                        new StackGrabCommand(),
//
//                        new PurePursuitCommand(new PurePursuitPath(
//                                new Waypoint(INTAKE_2, 15),
//                                new Waypoint(new Point(60, 28), 15),
//                                new Waypoint(new Pose(58, -18, 0), 15),
//                                new Waypoint(DEPOSIT_2, 15))
//                        ),
//
//                        new RelocalizeCommand(),
//
//                        new PositionCommand(DEPOSIT_2)
//                                .alongWith(new DepositExtendCommand(2.82, 0.755, 300)),
//
//                        new StackDepositCommand(475),

                        new PositionCommand(new Pose(48, -44, Math.PI / 4)),


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
            telemetry.addData("IMU", robot.getAngle());
//            telemetry.addData("TARGET POSE", )
//            telemetry.addData("arm pos", robot.extensionActuator.getPosition());
//            telemetry.addLine("TAPE POSE (" + stackPipeline.getClosestTapeContour().x + " " + stackPipeline.getClosestTapeContour().y);
//            telemetry.addLine("PIXEL POSE (" + stackPipeline.getClosestPixelContour().x + " " + stackPipeline.getClosestPixelContour().y);
            telemetry.update();

            loopTime = loop;

            if (gamepad1.b) robot.readIMU();
        }

//        portal.setProcessorEnabled(stackPipeline, false);

        robot.kill();
    }

    public double getDistanceMeasurement() {
        robot.readIMU();
        double heading = robot.getAngle();
        double voltage = robot.rightDistSensor.getVoltage();

        return 70.5 - calculateDistance(voltage, heading);
    }
}
