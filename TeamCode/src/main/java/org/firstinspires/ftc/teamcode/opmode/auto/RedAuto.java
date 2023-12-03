package org.firstinspires.ftc.teamcode.opmode.auto;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.centerstage.ClawSide;
import org.firstinspires.ftc.teamcode.common.centerstage.PropPipeline;
import org.firstinspires.ftc.teamcode.common.centerstage.Side;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.PurplePixelExtendCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.PurplePixelRetractCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.YellowPixelExtendCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.YellowPixelRetractCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.drivecommand.PositionCommand;
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
//                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.addSubsystem(drivetrain, extension, intake);
        intake.updateState(IntakeSubsystem.ClawState.CLOSED, ClawSide.BOTH);

        robot.read();
        while (!isStarted()) {
            telemetry.addLine("auto in init");
            telemetry.addData("POS", propPipeline.getLocation());
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
                yellowScorePos = new Pose(21.5, 23.25, -1.52);
                purpleScorePos = new Pose(26, 25, -1.52);
                parkPos = new Pose(6, 31, -3 * Math.PI / 2);
                break;
            case CENTER:
                yellowScorePos = new Pose(27.75, 23.25, -1.52);
                purpleScorePos = new Pose(36, 18, -1.52);
                parkPos = new Pose(6, 31, -3 * Math.PI / 2);
                break;
            case LEFT:
                yellowScorePos = new Pose(34.25, 23.25, -1.52);
                purpleScorePos = new Pose(25, 4.5, -1.52);
                parkPos = new Pose(6, 31, -3 * Math.PI / 2);
                break;
            default:
                break;

        }


        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        // go to yellow pixel scoring pos
                        new PositionCommand((Drivetrain) drivetrain, localizer, yellowScorePos)
                                .alongWith(new YellowPixelExtendCommand(robot, extension, intake)),

                        // score yellow pixel
                        new InstantCommand(() -> intake.updateState(IntakeSubsystem.ClawState.INTERMEDIATE, ClawSide.LEFT)),
                        new WaitCommand(200),

                        // retract
                        new YellowPixelRetractCommand(robot, extension, intake, ClawSide.LEFT),

                        // go to purple pixel scoring pos
                        new PositionCommand((Drivetrain) drivetrain, localizer, purpleScorePos)
                                .alongWith(new PurplePixelExtendCommand(robot, extension, intake)),

                        // score purple pixel
                        new WaitCommand(500),
                        new InstantCommand(() -> intake.updateState(IntakeSubsystem.ClawState.OPEN, ClawSide.RIGHT)),
                        new WaitCommand(350),

                        new PurplePixelRetractCommand(robot, extension, intake, ClawSide.RIGHT),

                        new PositionCommand((Drivetrain) drivetrain, localizer, parkPos)
                                .alongWith(new WaitCommand(400).andThen(new InstantCommand(() -> robot.intakePivotActuator.setTargetPosition(0.0475))))
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
        telemetry.addLine(localizer.getPos().toString());
        loopTime = loop;
        telemetry.update();

        robot.write();
        robot.clearBulkCache();
    }
}
