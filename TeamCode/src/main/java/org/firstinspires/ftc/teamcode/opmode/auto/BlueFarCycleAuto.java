package org.firstinspires.ftc.teamcode.opmode.auto;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.PrintCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.centerstage.ClawSide;
import org.firstinspires.ftc.teamcode.common.commandbase.cycleautocommand.FirstDepositCommand;
import org.firstinspires.ftc.teamcode.common.vision.StackPipeline;
import org.firstinspires.ftc.teamcode.common.vision.PropPipeline;
import org.firstinspires.ftc.teamcode.common.vision.Location;
import org.firstinspires.ftc.teamcode.common.commandbase.cycleautocommand.FirstDepositExtendCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.cycleautocommand.FirstStackGrabCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.cycleautocommand.FirstStackSetupCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.cycleautocommand.PurplePixelDepositCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.cycleautocommand.PurplePixelExtendCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.cycleautocommand.RelocalizeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.cycleautocommand.SecondDepositCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.cycleautocommand.SecondStackGrabCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.cycleautocommand.ThirdDepositCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.cycleautocommand.ThirdStackGrabCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.drivecommand.PositionCommand;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous(name = "🔵 Blue Far Cycle Auto")
public class BlueFarCycleAuto extends LinearOpMode {
    private final RobotHardware robot = RobotHardware.getInstance();

    private double loopTime = 0.0;
    private final ElapsedTime timer = new ElapsedTime();
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

        robot.init(hardwareMap);

        robot.intake.updateState(IntakeSubsystem.ClawState.CLOSED, ClawSide.BOTH);

        robot.localizer.setPose(new Pose(63.65, 39.35, Math.PI / 2));

        propPipeline = new PropPipeline();
        stackPipeline = new StackPipeline();

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .setCameraResolution(new Size(1920, 1080))
                .addProcessors(propPipeline, stackPipeline)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        portal.setProcessorEnabled(stackPipeline, false);

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
//        portal.close();
        portal.setProcessorEnabled(propPipeline, false);
        portal.setProcessorEnabled(stackPipeline, true);

        Pose purplePixelPose;
        Pose yellowPixelPose;

        switch (randomization) {
            case LEFT:
                purplePixelPose = new Pose(37.75, 25, Math.PI / 2);
                yellowPixelPose = new Pose(38.75, -29, 0);
                break;
            case RIGHT:
                purplePixelPose = new Pose(37.75, 39.35, 0.75);
                yellowPixelPose = new Pose(28.75, -29, 0);
                break;
            default:
                purplePixelPose = new Pose(37.75, 39.35, Math.PI / 2);
                yellowPixelPose = new Pose(31.75, -29, 0);
                break;
        }

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(timer::reset),

                        new PositionCommand(new Pose(37.75, 39.35, Math.PI / 2))
                                .alongWith(new PurplePixelExtendCommand(randomization)),

                        new PositionCommand(purplePixelPose),


                        new PurplePixelDepositCommand(),


                        new PositionCommand(new Pose(38, 39.25, -0.02))
                                .alongWith(new FirstStackSetupCommand()),

                        // // ERROR ADJUST
                        new WaitCommand(5000),
                        new PrintCommand("HERE " + stackPipeline.getStrafeCorrection()),
                        new WaitCommand(5000),
                        new PositionCommand(new Pose(38, 39.25, -0.02)),
                        new WaitCommand(5000),

                        new FirstStackGrabCommand(),


                        new PositionCommand(new Pose(35.75, -29, 0))
                                .andThen(new RelocalizeCommand())
                                .andThen(new PositionCommand(yellowPixelPose)
                                        .alongWith(new FirstDepositExtendCommand())),

                        new FirstDepositCommand(),

                        new PositionCommand(new Pose(35.75, -29, 0)),
                        new RelocalizeCommand(),

                        new PositionCommand(new Pose(38, 39, -0.02)),

                        // ERROR ADJUST
                        new WaitCommand(5000),
                        new PrintCommand("HERE " + stackPipeline.getStrafeCorrection()),
                        new WaitCommand(5000),
                        new PositionCommand(new Pose(38 + stackPipeline.getStrafeCorrection(), 39, -0.02)),
                        new WaitCommand(5000),

                        new SecondStackGrabCommand(),


                        new PositionCommand(new Pose(35.75, -31.5, 0))
                                .andThen(new RelocalizeCommand())
                                .alongWith(new SecondDepositCommand()),


                        new PositionCommand(new Pose(38, 39.5, -0.02)),

                        // ERROR ADJUST
                        new WaitCommand(5000),
                        new PrintCommand("HERE " + stackPipeline.getStrafeCorrection()),
                        new WaitCommand(5000),
                        new PositionCommand(new Pose(38, 39.5, -0.02)),
                        new WaitCommand(5000),

                        new ThirdStackGrabCommand(),


                        new PositionCommand(new Pose(35.75, -31.5, 0))
                                .andThen(new RelocalizeCommand())
                                .alongWith(new ThirdDepositCommand()),

                        new PositionCommand(new Pose(12, -54, 0)),

                        new InstantCommand(() -> endTime = timer.seconds()),
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

        portal.setProcessorEnabled(stackPipeline, false);

        robot.kill();
    }
}
