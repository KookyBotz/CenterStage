package org.firstinspires.ftc.teamcode.opmode.auto;

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

import org.firstinspires.ftc.teamcode.common.centerstage.ClawSide;
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

@Config
@Autonomous(name = "LeftSideAuto")
public class LeftSideAuto extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    private WSubsystem drivetrain;
    private ThreeWheelLocalizer localizer;
    private ExtensionSubsystem extension;
    private IntakeSubsystem intake;

    private double loopTime = 0.0;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        Globals.IS_AUTO = true;
        Globals.IS_USING_IMU = false;
        Globals.USING_DASHBOARD = true;

        robot.init(hardwareMap, telemetry);
        robot.enabled = true;
        drivetrain = new MecanumDrivetrain();
        localizer = new ThreeWheelLocalizer();
        extension = new ExtensionSubsystem();
        intake = new IntakeSubsystem();

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        robot.addSubsystem(drivetrain, extension, intake);
        intake.updateState(IntakeSubsystem.ClawState.CLOSED, ClawSide.BOTH);

        robot.read();
        while (!isStarted()) {
            telemetry.addLine("auto in init");
            telemetry.update();
        }

        localizer.setPoseEstimate(new Pose2d(0, 0, 0));

        // asdasdasdasd
        Side side = Side.LEFT;

        Pose yellowScorePos = new Pose();
        Pose purpleScorePos = new Pose();
        Pose parkPos = new Pose(50, -35, 3 * Math.PI / 2);
        double purpleExtensionPos = 0;
        double purpleAngle = Math.PI;

        Pose cyclePose = new Pose(45, 62, 1.5);
        double extensionPos = 114;
        double extensionAngle = 3.26;

        // 0.3, 300

        switch (side) {
            case LEFT:
                yellowScorePos = new Pose(21, -26, 1.5);
                purpleScorePos = new Pose(27, -24, 1.5);
                break;
            case CENTER:
                yellowScorePos = new Pose(27, -26, 1.5);
                purpleScorePos = new Pose(36, -18, 1.5);
                break;
            case RIGHT:
                yellowScorePos = new Pose(33, -26, 1.5);
                purpleScorePos = new Pose(27, -4, 1.5);
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
                        new InstantCommand(() -> robot.pitchActuator.setMotionProfileTargetPosition(0.34)),
                        new InstantCommand(() -> robot.extensionActuator.setMotionProfileTargetPosition(320)),
                        new WaitCommand(750),
                        // open claw boi
                        new InstantCommand(() -> intake.updateState(IntakeSubsystem.ClawState.INTERMEDIATE, ClawSide.RIGHT)),
                        new WaitCommand(500),
                        new InstantCommand(() -> robot.extensionActuator.setMotionProfileTargetPosition(0)),
                        new WaitCommand(250),

                        // retract
                        new InstantCommand(() -> robot.pitchActuator.setMotionProfileTargetPosition(0)),
                        new WaitCommand(250),
                        new InstantCommand(() -> intake.updateState(IntakeSubsystem.PivotState.STORED)),
                        new InstantCommand(() -> robot.intakePivotActuator.setTargetPosition(0.0475)),
                        new InstantCommand(() -> intake.updateState(IntakeSubsystem.ClawState.CLOSED, ClawSide.RIGHT)),

                        // go to position
                        new WaitCommand(500),

                        new ParallelCommandGroup(
                                new PositionCommand((Drivetrain) drivetrain, localizer, purpleScorePos),
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> robot.pitchActuator.setMotionProfileTargetPosition(Math.PI)),
                                        new InstantCommand(() -> robot.extensionActuator.setMotionProfileTargetPosition(0)),
                                        new WaitCommand(250),
                                        new InstantCommand(() -> intake.updateState(IntakeSubsystem.PivotState.FLAT)),
                                        new InstantCommand(() -> robot.intakePivotActuator.setTargetPosition(0.515))
                                )
                        ),

                        new WaitCommand(750),
                        new InstantCommand(() -> intake.updateState(IntakeSubsystem.ClawState.OPEN, ClawSide.LEFT)),
                        new WaitCommand(500),

                        new InstantCommand(() -> extension.setScoring(false)),
                        new InstantCommand(() -> extension.setFlip(false)),
                        new InstantCommand(() -> robot.pitchActuator.setMotionProfileTargetPosition(0.0)),
                        new InstantCommand(() -> robot.extensionActuator.setMotionProfileTargetPosition(0)),
                        new WaitCommand(250),
                        new ClawCommand(intake, IntakeSubsystem.ClawState.CLOSED, ClawSide.BOTH),
                        new InstantCommand(() -> intake.updateState(IntakeSubsystem.PivotState.STORED)),
                        new InstantCommand(() -> robot.intakePivotActuator.setTargetPosition(0.0475)),

                        new PositionCommand((Drivetrain) drivetrain, localizer, parkPos)
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
        telemetry.addData("voltage", robot.getVoltage());
        loopTime = loop;
        telemetry.update();

        robot.write();
        robot.clearBulkCache();
    }
}
