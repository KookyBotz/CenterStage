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
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Point;
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
@Autonomous(name = "Blue Cycle Auto")
public class BlueCycleAuto extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    private WSubsystem drivetrain;
    private ThreeWheelLocalizer localizer;
    private ExtensionSubsystem extension;
    private IntakeSubsystem intake;

    private double loopTime = 0.0;

    private Pose[] DEPOSIT_POSITIONS = new Pose[]{
            new Pose(32.5, -22, 1.5),
            new Pose(33, -22, 1.5)
    };

    private Pose[] INTERMEDIATE_POSES = new Pose[]{
            new Pose(47, 0, 1.5),
            new Pose(47, 15, 1.5),
    };

    private Pose[] INTAKE_POSITIONS = new Pose[]{
            new Pose(43.5, 47, 1.5),
            new Pose(43.5, 47, 1.5)
    };

    private double[] PITCH_INTAKE_POSITIONS = new double[]{
            3.2075,
            3.24
    };

    private final double LIFT_INTAKE_POSITION = 560;


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

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.addSubsystem(drivetrain, extension, intake);
        intake.updateState(IntakeSubsystem.ClawState.CLOSED, ClawSide.BOTH);

        robot.read();
        while (!isStarted()) {
            telemetry.addLine("auto in init");
            telemetry.update();
        }

        localizer.setPoseEstimate(new Pose2d(0, 0, 0));

        Side side = Side.LEFT;

        Pose yellowScorePos = new Pose();
        Pose purpleScorePos = new Pose();
        Pose parkPos = new Pose();


        switch (side) {
            case LEFT:
                yellowScorePos = new Pose(21, -26.65, 1.5);
                purpleScorePos = new Pose(30, -24.75, 1.5);
                parkPos = new Pose(6, -31, 3 * Math.PI / 2);
                break;
            case CENTER:
                yellowScorePos = new Pose(27, -26.5, 1.5);
                purpleScorePos = new Pose(35, -18, 1.5);
                parkPos = new Pose(5, -31, 3 * Math.PI / 2);
                break;
            case RIGHT:
                yellowScorePos = new Pose(33.5, -26.25, 1.5);
                purpleScorePos = new Pose(24, -5.5, 1.5);
                parkPos = new Pose(2, -31, 3 * Math.PI / 2);
                break;
            default:
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

        PurePursuitPath intake2 = new PurePursuitPath(
                new Waypoint(DEPOSIT_POSITIONS[0], 20),
                new Waypoint(INTERMEDIATE_POSES[0], 20),
                new Waypoint(INTAKE_POSITIONS[1], 20)
        );

        PurePursuitPath deposit2 = new PurePursuitPath(
                new Waypoint(INTAKE_POSITIONS[1], 20),
                new Waypoint(INTERMEDIATE_POSES[1], 20),
                new Waypoint(DEPOSIT_POSITIONS[1], 20)
        );

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        // go to yellow pixel scoring pos
                        new PositionCommand((Drivetrain) drivetrain, localizer, yellowScorePos)
                                .alongWith(new YellowPixelExtendCommand(robot, extension, intake)),

                        // score yellow pixel
                        new InstantCommand(() -> intake.updateState(IntakeSubsystem.ClawState.INTERMEDIATE, ClawSide.LEFT)),
                        new WaitCommand(500),

                        // retract
                        new YellowPixelRetractCommand(robot, extension, intake),

                        // go to purple pixel scoring pos
                        new PositionCommand((Drivetrain) drivetrain, localizer, purpleScorePos)
                                .alongWith(new PurplePixelExtendCommand(robot, extension, intake)),

                        // score purple pixel
                        new WaitCommand(500),
                        new InstantCommand(() -> intake.updateState(IntakeSubsystem.ClawState.INTERMEDIATE, ClawSide.RIGHT)),
                        new WaitCommand(500),

                        // retract
                        new PurplePixelRetractCommand(robot, extension, intake),

                        // first cycle intake position
                        new PurePursuitCommand((Drivetrain) drivetrain, localizer, intake1)
                                .alongWith(new AutoStackExtendCommand(robot, extension, intake, LIFT_INTAKE_POSITION, PITCH_INTAKE_POSITIONS[0])),

                        new AutoStackGrabCommand(robot, extension, intake),
                        new PurePursuitCommand((Drivetrain) drivetrain, localizer, deposit1)
//                        new WaitCommand(500),
//                        new PurePursuitCommand((Drivetrain) drivetrain, localizer, intake2),
//                        new WaitCommand(500),
//                        new PurePursuitCommand((Drivetrain) drivetrain, localizer, deposit2)
                )
        );
    }

    @Override
    public void run() {
        robot.clearBulkCache();
        robot.read();

        CommandScheduler.getInstance().run();
        robot.periodic();
        localizer.periodic();

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        telemetry.addData("pose", localizer.getPos());
        telemetry.addData("voltage", robot.getVoltage());
        loopTime = loop;
        telemetry.update();

        robot.write();
    }
}
