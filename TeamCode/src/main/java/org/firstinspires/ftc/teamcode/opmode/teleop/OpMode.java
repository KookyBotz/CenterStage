package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.centerstage.ClawSide;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ScoreCommand;
import org.firstinspires.ftc.teamcode.common.drive.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.util.InverseKinematics;
import org.firstinspires.ftc.teamcode.common.util.MathUtils;

@Config
@TeleOp(name = "teleop")
public class OpMode extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();

    private GamepadEx gamepadEx;
    private GamepadEx gamepadEx2;

    private double loopTime = 0.0;

    private final boolean rightStickGreat = false;
    private boolean lastRightStickGreat = false;

    private boolean lastJoystickUp = false;
    private boolean lastJoystickDown = false;

    public boolean aButton = true;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        Globals.IS_AUTO = false;
        Globals.IS_USING_IMU = true;
        Globals.USING_DASHBOARD = true;

        gamepadEx = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        robot.init(hardwareMap, telemetry);

        robot.intakePivotActuator.setTargetPosition(0);
        robot.intakePivotActuator.write();

        // G1 - Claw Control
        gamepadEx.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                        .whenPressed(new ConditionalCommand(
                                new ClawCommand(robot.intake, IntakeSubsystem.ClawState.INTERMEDIATE, ClawSide.LEFT),
                                new ClawCommand(robot.intake, IntakeSubsystem.ClawState.OPEN, ClawSide.LEFT),
                                () -> (robot.intake.leftClaw == (IntakeSubsystem.ClawState.CLOSED))
                        ));

        gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new ConditionalCommand(
                        new ClawCommand(robot.intake, IntakeSubsystem.ClawState.INTERMEDIATE, ClawSide.RIGHT),
                        new ClawCommand(robot.intake, IntakeSubsystem.ClawState.OPEN, ClawSide.RIGHT),
                        () -> (robot.intake.rightClaw == (IntakeSubsystem.ClawState.CLOSED))
                ));

        // G1 - Retract deposit
        gamepadEx.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> Globals.retract()),
                                        new InstantCommand(() -> robot.armActuator.setMotionProfileTargetPosition(-0.06)),
                                        new InstantCommand(() -> robot.extensionActuator.setMotionProfileTargetPosition(0)),
                                        new WaitCommand(250),
                                        new ClawCommand(robot.intake, IntakeSubsystem.ClawState.CLOSED, ClawSide.BOTH),
                                        new InstantCommand(() -> robot.intake.updateState(IntakeSubsystem.PivotState.STORED)),
                                        new InstantCommand(() -> robot.intakePivotActuator.setTargetPosition(0))
                                ),
                                new WaitCommand(1),
                                () -> Globals.IS_SCORING
                        )

                );

        // G1 - Claw control for scoring, retraction for when intaking
        gamepadEx.getGamepadButton(GamepadKeys.Button.A)
                        .whenPressed(
                                new ConditionalCommand(
                                        new ConditionalCommand(
                                                new ClawCommand(robot.intake, IntakeSubsystem.ClawState.INTERMEDIATE, ClawSide.BOTH),
                                                new ClawCommand(robot.intake, IntakeSubsystem.ClawState.OPEN, ClawSide.BOTH),
                                                () -> (robot.intake.rightClaw == (IntakeSubsystem.ClawState.CLOSED) || (robot.intake.leftClaw == IntakeSubsystem.ClawState.CLOSED))
                                        ),
                                        new SequentialCommandGroup(
                                                new InstantCommand(() -> aButton = true),
                                                new InstantCommand(() -> Globals.retract()),
                                                new ClawCommand(robot.intake, IntakeSubsystem.ClawState.CLOSED, ClawSide.BOTH),
                                                new WaitCommand(250),
                                                new InstantCommand(() -> robot.armActuator.setMotionProfileTargetPosition(-0.06)),
                                                new InstantCommand(() -> robot.extensionActuator.setMotionProfileTargetPosition(0)),
                                                new InstantCommand(() -> robot.intake.updateState(IntakeSubsystem.PivotState.STORED)),
                                                new InstantCommand(() -> robot.intakePivotActuator.setTargetPosition(0))),
                                        () -> Globals.IS_SCORING)
                                );

        // G2 - Intake Sequence
        gamepadEx2.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(

                        new SequentialCommandGroup(
                                new InstantCommand(() -> aButton = true),
                                new InstantCommand(() -> Globals.retract()),
                                new InstantCommand(() -> robot.armActuator.setMotionProfileTargetPosition(-0.06)),
                                new InstantCommand(() -> robot.extensionActuator.setMotionProfileTargetPosition(350)),
                                new InstantCommand(() -> robot.intake.updateState(IntakeSubsystem.PivotState.FLAT)),
                                new InstantCommand(() -> robot.intakePivotActuator.setTargetPosition(0.46)), // 0.515
                                new WaitCommand(250),
                                new ClawCommand(robot.intake, IntakeSubsystem.ClawState.OPEN, ClawSide.BOTH)
                        ));

        // G2 - Retract from Depositing
        gamepadEx2.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> Globals.retract()),
                                        new InstantCommand(() -> robot.armActuator.setMotionProfileTargetPosition(-0.06)),
                                        new InstantCommand(() -> robot.extensionActuator.setMotionProfileTargetPosition(0)),
                                        new WaitCommand(250),
                                        new ClawCommand(robot.intake, IntakeSubsystem.ClawState.CLOSED, ClawSide.BOTH),
                                        new InstantCommand(() -> robot.intake.updateState(IntakeSubsystem.PivotState.STORED)),
                                        new InstantCommand(() -> robot.intakePivotActuator.setTargetPosition(0))),
                                new WaitCommand(1),
                                () -> Globals.IS_SCORING)
                );

        // G2 - Begin Scoring Sequence
        gamepadEx2.getGamepadButton(GamepadKeys.Button.Y)
                        .whenPressed(new SequentialCommandGroup(
                                new InstantCommand(() -> Globals.startScoring()),
                                new InstantCommand(() -> robot.armActuator.setMotionProfileTargetPosition(InverseKinematics.t_angle)),
                                new WaitCommand(200),
                                new InstantCommand(() -> robot.intake.updateState(IntakeSubsystem.PivotState.SCORING)),
                                new WaitCommand(400),
                                new InstantCommand(() -> robot.extensionActuator.setMotionProfileTargetPosition(InverseKinematics.t_extension))
                        ));

        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                        .whenPressed(new InstantCommand(() -> robot.extension.setBackdropHeight(6)
                        ));

        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(() -> robot.extension.setBackdropHeight(0)
                ));

        robot.read();
        while (opModeInInit()) {
            telemetry.addLine("Robot Initialized.");
            telemetry.update();
        }
    }

    @Override
    public void run() {
        robot.read();

        robot.drivetrain.set(new Pose(gamepad1.left_stick_x, -gamepad1.left_stick_y, MathUtils.joystickScalar(-gamepad1.left_trigger + gamepad1.right_trigger, 0.01)), 0);

        boolean currentJoystickUp = gamepad2.right_stick_y < -0.5;
        boolean currentJoystickDown = gamepad2.right_stick_y > 0.5;
        if (currentJoystickUp && !lastJoystickUp) {
            // height go upp
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new InstantCommand(() -> robot.extension.incrementBackdropHeight(1)),
                            new InstantCommand(() -> InverseKinematics.calculateTarget(5, robot.extension.getBackdropHeight())),
                            new InstantCommand(() -> System.out.println(robot.extension.getBackdropHeight())),
                            new ConditionalCommand(
                                    new ScoreCommand(robot, 5, robot.extension.getBackdropHeight()),
                                    new WaitCommand(1),
                                    () -> Globals.IS_SCORING
                            )
            ));
        }

        if (currentJoystickDown && !lastJoystickDown) {
            // gheight go dwodn
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new InstantCommand(() -> robot.extension.incrementBackdropHeight(-1)),
                            new InstantCommand(() -> InverseKinematics.calculateTarget(5, robot.extension.getBackdropHeight())),
                            new ConditionalCommand(
                                    new ScoreCommand(robot, 3, robot.extension.getBackdropHeight()),
                                    new WaitCommand(1),
                                    () -> Globals.IS_SCORING
                            )
                    ));
        }
        lastJoystickUp = currentJoystickUp;
        lastJoystickDown = currentJoystickDown;

        super.run();
        robot.periodic();

        lastRightStickGreat = rightStickGreat;

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        loopTime = loop;
        telemetry.update();
        robot.write();
        robot.clearBulkCache();
    }
}
