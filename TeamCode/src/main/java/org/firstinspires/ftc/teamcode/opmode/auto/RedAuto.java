package org.firstinspires.ftc.teamcode.opmode.auto;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.centerstage.ClawSide;
import org.firstinspires.ftc.teamcode.common.centerstage.PropPipeline;
import org.firstinspires.ftc.teamcode.common.centerstage.Side;
import org.firstinspires.ftc.teamcode.common.commandbase.auto.PositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.state.ClawCommand;
import org.firstinspires.ftc.teamcode.common.drive.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.common.drive.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.common.drive.localizer.ThreeWheelLocalizer;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WSubsystem;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous(name = "Red Auto")
public class RedAuto extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    private WSubsystem drivetrain;
    private ThreeWheelLocalizer localizer;
    private ExtensionSubsystem extension;
    private IntakeSubsystem intake;

    private PropPipeline propPipeline;
    private VisionPortal portal;


    private double loopTime = 0.0;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        Globals.IS_AUTO = true;
        Globals.IS_USING_IMU = false;
        Globals.USING_DASHBOARD = true;
        Globals.COLOR = Side.RED;

        robot.init(hardwareMap, telemetry);
        robot.enabled = true;
        drivetrain = new MecanumDrivetrain();
        localizer = new ThreeWheelLocalizer();
        extension = new ExtensionSubsystem();
        intake = new IntakeSubsystem();

        propPipeline = new PropPipeline();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .setCameraResolution(new Size(1920, 1080))
                .setCamera(BuiltinCameraDirection.BACK)
                .addProcessor(propPipeline)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        robot.addSubsystem(drivetrain, extension, intake);
        intake.updateState(IntakeSubsystem.ClawState.CLOSED, ClawSide.BOTH);

        robot.read();
        while (!isStarted()) {
            telemetry.addLine("auto in init");
            telemetry.addData("location", propPipeline.getLocation());
            telemetry.update();
        }

        localizer.setPoseEstimate(new Pose2d(0, 0, 0));

        Side side = propPipeline.getLocation();
        portal.close();

        Pose yellowScorePos = new Pose();
        Pose purpleScorePos = new Pose();
        Pose parkPos = new Pose();


        // 0.3, 300



        switch (side) {
            case RIGHT:
                yellowScorePos = new Pose(21, 26.25, -1.5);
                purpleScorePos = new Pose(28, 25, -1.5);
                parkPos = new Pose(50, 35, -3 * Math.PI / 2);
                break;
            case CENTER:
                yellowScorePos = new Pose(27, 26.25, -1.5);
                purpleScorePos = new Pose(36.5, 19.5, -1.5);
                parkPos = new Pose(49, 35, -3 * Math.PI / 2);
                break;
            case LEFT:
                yellowScorePos = new Pose(33, 26.25, -1.5 );
                purpleScorePos = new Pose(25, 4.5, -1.5);
                parkPos = new Pose(47.5, 35, -3 * Math.PI / 2);
                break;
            default:
                // your mom
                break;

        }

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        // scoring pos
                        new PositionCommand((Drivetrain) drivetrain, localizer, yellowScorePos),

                        // extend
                        new InstantCommand(() -> extension.setScoring(true)),
                        new InstantCommand(() -> extension.setFlip(false)),
                        new InstantCommand(() -> intake.updateState(IntakeSubsystem.PivotState.SCORING)),
                        new InstantCommand(() -> robot.pitchActuator.setMotionProfileTargetPosition(0.36)),
                        new InstantCommand(() -> robot.extensionActuator.setMotionProfileTargetPosition(315)),
                        new WaitCommand(750),
                        // open claw boi
                        new InstantCommand(() -> intake.updateState(IntakeSubsystem.ClawState.INTERMEDIATE, ClawSide.LEFT)),
                        new WaitCommand(200),

                        // retract
                        new InstantCommand(() -> robot.extensionActuator.setMotionProfileTargetPosition(0)),
                        new WaitCommand(50),
                        new InstantCommand(() -> extension.setScoring(false)),
                        new InstantCommand(() -> extension.setFlip(false)),
                        new InstantCommand(() -> robot.intakePivotActuator.setTargetPosition(0.0475)),
                        new InstantCommand(() -> intake.updateState(IntakeSubsystem.ClawState.CLOSED, ClawSide.LEFT)),
                        new InstantCommand(() -> robot.pitchActuator.setMotionProfileTargetPosition(Math.PI)),

                        new ParallelCommandGroup(
                                new PositionCommand((Drivetrain) drivetrain, localizer, purpleScorePos),
                                new SequentialCommandGroup(
                                        new WaitCommand(250),
                                        new InstantCommand(() -> intake.updateState(IntakeSubsystem.PivotState.FLAT)),
                                        new InstantCommand(() -> robot.intakePivotActuator.setTargetPosition(0.515))
                                )
                        ),

                        new WaitCommand(400),
                        new InstantCommand(() -> intake.updateState(IntakeSubsystem.ClawState.INTERMEDIATE, ClawSide.RIGHT)),
                        new WaitCommand(300),

                        new InstantCommand(() -> robot.pitchActuator.setMotionProfileTargetPosition(0.0)),
                        new InstantCommand(() -> robot.extensionActuator.setMotionProfileTargetPosition(0)),
                        new ClawCommand(intake, IntakeSubsystem.ClawState.CLOSED, ClawSide.RIGHT),
                        new InstantCommand(() -> intake.updateState(IntakeSubsystem.PivotState.STORED)),

                        new PositionCommand((Drivetrain) drivetrain, localizer, parkPos)
                                .alongWith(new WaitCommand(200).andThen(new InstantCommand(() -> robot.intakePivotActuator.setTargetPosition(0.0475))))
                )
        );
    }

    @Override
    public void run() {
        robot.read();

        super.run();
        robot.periodic();
        localizer.periodic();

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        loopTime = loop;
        telemetry.update();

        robot.write();
        robot.clearBulkCache();
    }
}
