package org.firstinspires.ftc.teamcode.opmode.auto;

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

import org.firstinspires.ftc.teamcode.common.centerstage.ClawSide;
import org.firstinspires.ftc.teamcode.common.commandbase.auto.GVFCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.state.ClawCommand;
import org.firstinspires.ftc.teamcode.common.drive.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.common.drive.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.common.drive.localizer.ThreeWheelLocalizer;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Vector2D;
import org.firstinspires.ftc.teamcode.common.drive.pathing.path.HermitePath;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.util.Paths;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WSubsystem;

@Config
@Autonomous(name = "LeftSideAuto2")
public class LeftSideAuto extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    private WSubsystem drivetrain;
    private ThreeWheelLocalizer localizer;
    private ExtensionSubsystem extension;
    private IntakeSubsystem intake;

    // path that goes forward and to the left
    private final HermitePath trajectory = new HermitePath()
            .addPose(0, 0, new Vector2D(0, 100))
            .addPose(0, 20, new Vector2D(0, 500))
            .addPose(20, 40, new Vector2D(500, 0))
            .addPose(40, 40, new Vector2D(100, 0))
            .flip()
            .construct();

    private final HermitePath trajectory2 = new HermitePath()
            .addPose(136.5, 60, new Vector2D(50, 0))
            .addPose(124, 65, Vector2D.fromHeadingAndMagnitude(0.5, 100))
            .offsetX(-136.5)
            .offsetY(-60)
            .negateX()
            .construct();

    private double loopTime = 0.0;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        Globals.IS_AUTO = true;
        Globals.IS_USING_IMU = true;
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

//        CommandScheduler.getInstance().schedule(
//                new SequentialCommandGroup(
//                    new GVFCommand((Drivetrain) drivetrain, localizer, trajectory)
//                    new WaitCommand(4000),
//                    new SequentialCommandGroup(
//                            new InstantCommand(() -> extension.setScoring(false)),
//                            new InstantCommand(() -> extension.setFlip(false)),
//                            new InstantCommand(() -> robot.pitchActuator.setMotionProfileTargetPosition(-0.025)),
//                            new InstantCommand(() -> robot.extensionActuator.setMotionProfileTargetPosition(200)),
//                            new InstantCommand(() -> intake.updateState(IntakeSubsystem.PivotState.FLAT)),
//                            new InstantCommand(() -> robot.intakePivotActuator.setTargetPosition(0.515)),
//                            new WaitCommand(200)
//                    ),
//                    new WaitCommand(4000),
//                    new ClawCommand(intake, IntakeSubsystem.ClawState.OPEN, ClawSide.LEFT),
//                    new WaitCommand(4000),
//                    new SequentialCommandGroup(
//                            new InstantCommand(() -> robot.pitchActuator.setMotionProfileTargetPosition(0.0)),
//                            new InstantCommand(() -> robot.extensionActuator.setMotionProfileTargetPosition(0)),
//                            new InstantCommand(() -> intake.updateState(IntakeSubsystem.ClawState.CLOSED, ClawSide.LEFT)),
//                            new WaitCommand(250),
//                            new InstantCommand(() -> intake.updateState(IntakeSubsystem.PivotState.STORED)),
//                            new InstantCommand(() -> robot.intakePivotActuator.setTargetPosition(0.0475))
//                    )
//                )
//        );

        // extend preload + start driving at the same time
        // once position is reached, open claw
        // retract + bring pivot + arm upwards, will allow time for the claw to close
        // path over to the desired preload spot
        // extend to score at the preload spot
        // retract and then park
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        // extend, keep arm slightly off ground to help not hit tape
                        new InstantCommand(() -> extension.setScoring(false)),
                        new InstantCommand(() -> extension.setFlip(false)),
                        new InstantCommand(() -> robot.pitchActuator.setMotionProfileTargetPosition(0.05)),
                        new InstantCommand(() -> robot.extensionActuator.setMotionProfileTargetPosition(220)),
                        new InstantCommand(() -> intake.updateState(IntakeSubsystem.PivotState.FLAT)),
                        new InstantCommand(() -> robot.intakePivotActuator.setTargetPosition(0.515)),

                        // drive
                        new GVFCommand((Drivetrain) drivetrain, localizer, Paths.PRELOAD_SCORE_RIGHT),
                        new WaitCommand(1000),

                        // wont trigger until bot reaches its position
                        // (when facing bot from the front, the left side of the claw is (LEFT) in code with the purple pixel)
                        new ClawCommand(intake, IntakeSubsystem.ClawState.OPEN, ClawSide.LEFT),

                        // delay, then retract
                        new WaitCommand(4000),
                        new InstantCommand(() -> robot.pitchActuator.setMotionProfileTargetPosition(0.05)),
                        new InstantCommand(() -> robot.extensionActuator.setMotionProfileTargetPosition(0)),
                        new WaitCommand(4000),
                        new InstantCommand(() -> intake.updateState(IntakeSubsystem.ClawState.CLOSED, ClawSide.LEFT)),
                        new WaitCommand(1000),
                        new InstantCommand(() -> intake.updateState(IntakeSubsystem.PivotState.STORED)),
                        new InstantCommand(() -> robot.intakePivotActuator.setTargetPosition(0.0475)),

                        new WaitCommand(4000),
                        // go to the next position
                        new GVFCommand((Drivetrain) drivetrain, localizer, Paths.PRELOAD_TO_BACKDROP),
//// 0.36, 190
                        // extend to score
                        new WaitCommand(1000),
                        new InstantCommand(() -> extension.setScoring(true)),
                        new InstantCommand(() -> robot.pitchActuator.setMotionProfileTargetPosition(0.36)),
                        new WaitCommand(200),
                        new InstantCommand(() -> robot.intakePivotActuator.setTargetPosition(0.24)),
                        new WaitCommand(400),
                        new InstantCommand(() -> robot.extensionActuator.setMotionProfileTargetPosition(454)),
//
                        // drop pixels
                        new WaitCommand(3000),
                        new InstantCommand(() -> intake.updateState(IntakeSubsystem.ClawState.INTERMEDIATE, ClawSide.RIGHT)),
                        new WaitCommand(2000),
                        new InstantCommand(() -> intake.updateState(IntakeSubsystem.ClawState.OPEN, ClawSide.RIGHT)),
//
                        // retract
                        new WaitCommand(1000),
                        new InstantCommand(() -> extension.setScoring(false)),
                        new InstantCommand(() -> extension.setFlip(false)),
                        new InstantCommand(() -> robot.pitchActuator.setMotionProfileTargetPosition(0.0)),
                        new InstantCommand(() -> robot.extensionActuator.setMotionProfileTargetPosition(0)),
                        new WaitCommand(250),
                        new ClawCommand(intake, IntakeSubsystem.ClawState.CLOSED, ClawSide.BOTH),
                        new InstantCommand(() -> intake.updateState(IntakeSubsystem.PivotState.STORED)),
                        new InstantCommand(() -> robot.intakePivotActuator.setTargetPosition(0.0475))

                        // figure out what to do from here
                )
        );

//        while (opModeIsActive()) {
//            robot.read();
//
//            super.run();
//            robot.periodic();
//            localizer.periodic();
//
//            telemetry.addData("nearestT", GVFPathFollower.nearestT);
//            telemetry.addData("targetVel", GVFCommand.gvf);
//            telemetry.addData("currentVel", localizer.getNewPoseVelocity());
//            telemetry.addData("currentPose", localizer.getPoseEstimate());
//            telemetry.addData("HEADING", localizer.getPos().heading);
//            telemetry.addData("powers2", GVFCommand.powers2);
//            telemetry.addData("delta", GVFCommand.hahaFunnyDelta);
//            telemetry.addData("funny", GVFCommand.funny);
//
//            telemetry.update();
//            CommandScheduler.getInstance().run();
//            robot.write();
//            robot.clearBulkCache();
//        }
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
