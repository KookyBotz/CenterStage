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
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.centerstage.ClawSide;
import org.firstinspires.ftc.teamcode.common.centerstage.PropPipeline;
import org.firstinspires.ftc.teamcode.common.centerstage.Side;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.AutoDepositExtendCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.AutoDepositRetractCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.AutoStackExtendCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.AutoStackGrabCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.PurplePixelExtendCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.PurplePixelRetractCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.YellowPixelExtendCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.YellowPixelRetractCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.drivecommand.PositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.drivecommand.PurePursuitCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.common.drive.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.common.drive.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.common.drive.localizer.ThreeWheelLocalizer;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.drive.pathing.purepursuit.PurePursuitPath;
import org.firstinspires.ftc.teamcode.common.drive.pathing.purepursuit.Waypoint;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WSubsystem;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous(name = "Blue Auto")
public class BlueAuto extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    private WSubsystem drivetrain;
    private ThreeWheelLocalizer localizer;
    private ExtensionSubsystem extension;
    private IntakeSubsystem intake;

    private PropPipeline propPipeline;
    private VisionPortal portal;


    private double loopTime = 0.0;

    private Pose[] DEPOSIT_POSITIONS = new Pose[]{
            new Pose(30, -22, 1.52),
            new Pose(30, -22, 1.52)
    };

    private Pose[] INTERMEDIATE_POSES = new Pose[]{
            new Pose(48, -10, 1.5),
            new Pose(48, 15, 1.5),
    };

    private Pose[] INTAKE_POSITIONS = new Pose[]{
            new Pose(40.75, 63, 1.51),
            new Pose(40.75, 43.5, 1.51)
    };

    private double[] PITCH_INTAKE_POSITIONS = new double[]{
            3.3,
            3.24
    };

    private final double LIFT_INTAKE_POSITION = 100;
    private final double LIFT_DEPOSIT_POSITION = 555;
    private final double ARM_DEPOSIT_POSITION = 0.34;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        Globals.IS_AUTO = true;
        Globals.IS_USING_IMU = false;
        Globals.USING_DASHBOARD = true;
        Globals.COLOR = Side.BLUE;

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

//        Side side = propPipeline.getLocation();
        Side side = Side.LEFT;
        portal.close();

        Pose yellowScorePos = new Pose();
        Pose purpleScorePos = new Pose();
        Pose parkPos = new Pose();


        // 0.3, 300

        switch (side) {
            case LEFT:
                yellowScorePos = new Pose(21.5, -22.5, 1.52);
                purpleScorePos = new Pose(27, -25, 1.52);
                parkPos = new Pose(6, -31, 3 * Math.PI / 2);
                break;
            case CENTER:
                yellowScorePos = new Pose(28, -22.5, 1.52);
                purpleScorePos = new Pose(36, -18, 1.52);
                parkPos = new Pose(5, -31, 3 * Math.PI / 2);
                break;
            case RIGHT:
                yellowScorePos = new Pose(35, -22.5, 1.52);
                purpleScorePos = new Pose(26.5, -4.5, 1.52);
                parkPos = new Pose(2, -31, 3 * Math.PI / 2);
                break;
            default:
                // your mom
                break;

        }

        PurePursuitPath intake1 = new PurePursuitPath(
                new Waypoint(purpleScorePos, 20),
                new Waypoint(INTERMEDIATE_POSES[0], 20),
                new Waypoint(INTAKE_POSITIONS[0], 20)
        );

        PurePursuitPath deposit1 = new PurePursuitPath(
                new Waypoint(INTAKE_POSITIONS[0], 20),
                new Waypoint(INTERMEDIATE_POSES[1], 20),
                new Waypoint(DEPOSIT_POSITIONS[0], 20)
        );


        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        // go to yellow pixel scoring pos
                        new PositionCommand((Drivetrain) drivetrain, localizer, yellowScorePos)
                                .alongWith(new YellowPixelExtendCommand(robot, extension, intake)),

                        // score yellow pixel
                        new InstantCommand(() -> intake.updateState(IntakeSubsystem.ClawState.INTERMEDIATE, ClawSide.RIGHT)),
                        new WaitCommand(200),

                        // retract
                        new YellowPixelRetractCommand(robot, extension, intake),

                        // go to purple pixel scoring pos
                        new PositionCommand((Drivetrain) drivetrain, localizer, purpleScorePos)
                                .alongWith(new PurplePixelExtendCommand(robot, extension, intake)),

                        // score purple pixel
                        new WaitCommand(500),
                        new InstantCommand(() -> intake.updateState(IntakeSubsystem.ClawState.OPEN, ClawSide.LEFT)),
                        new WaitCommand(350),

                        new PurplePixelRetractCommand(robot, extension, intake),

                        new PurePursuitCommand((Drivetrain) drivetrain, localizer, intake1)
                                .alongWith(new AutoStackExtendCommand(robot, extension, intake, LIFT_INTAKE_POSITION, PITCH_INTAKE_POSITIONS[0])),

                        new AutoStackGrabCommand(robot, extension, intake),

                        new PurePursuitCommand((Drivetrain) drivetrain, localizer, deposit1)
                                .alongWith(new AutoDepositExtendCommand(robot, extension, intake, LIFT_DEPOSIT_POSITION, ARM_DEPOSIT_POSITION)),

                        new WaitCommand(500),
                        new ClawCommand(intake, IntakeSubsystem.ClawState.OPEN, ClawSide.LEFT),
                        new WaitCommand(500),

                        new AutoDepositRetractCommand(robot, extension, intake)


//
//                        new PositionCommand((Drivetrain) drivetrain, localizer, parkPos)
//                                .alongWith(new WaitCommand(400).andThen(new InstantCommand(() -> robot.intakePivotActuator.setTargetPosition(0.0475))))
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
