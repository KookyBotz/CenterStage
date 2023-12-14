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
import org.firstinspires.ftc.teamcode.common.commandbase.teleopcommand.ClawToggleCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.teleopcommand.DepositExtendCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.teleopcommand.DepositRetractionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.teleopcommand.HeightChangeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.teleopcommand.IntakeExtendCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.teleopcommand.IntakeRetractCommand;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
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
    private boolean lastJoystickUp = false;
    private boolean lastJoystickDown = false;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        Globals.IS_AUTO = false;
        Globals.IS_USING_IMU = false;
        Globals.USING_DASHBOARD = false;

        gamepadEx = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        robot.init(hardwareMap, telemetry);

        robot.intakePivotActuator.setTargetPosition(0);
        robot.intakePivotActuator.write();

        // G1 - Claw Control
        gamepadEx.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new ClawToggleCommand(robot, ClawSide.LEFT));
        gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new ClawToggleCommand(robot, ClawSide.RIGHT));

        // G1 - Retract deposit
        gamepadEx.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new DepositRetractionCommand());

        // G1 - Claw control for scoring, retraction for when intaking
        gamepadEx.getGamepadButton(GamepadKeys.Button.A)
                        .whenPressed(
                                new ConditionalCommand(
                                        new ConditionalCommand(
                                                new ClawCommand(IntakeSubsystem.ClawState.INTERMEDIATE, ClawSide.BOTH),
                                                new ClawCommand(IntakeSubsystem.ClawState.OPEN, ClawSide.BOTH),
                                                () -> (robot.intake.rightClaw == (IntakeSubsystem.ClawState.CLOSED) || (robot.intake.leftClaw == IntakeSubsystem.ClawState.CLOSED))
                                        ),
                                        new IntakeRetractCommand(),
                                        () -> Globals.IS_SCORING)
                                );

        // G2 - Intake Sequence
        gamepadEx2.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new IntakeExtendCommand(350));

        // G2 - Retract from Depositing
        gamepadEx2.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new DepositRetractionCommand());

        // G2 - Begin Scoring Sequence
        gamepadEx2.getGamepadButton(GamepadKeys.Button.Y)
                        .whenPressed(new DepositExtendCommand());

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

        // Change Height
        boolean currentJoystickUp = gamepad2.right_stick_y < -0.5;
        boolean currentJoystickDown = gamepad2.right_stick_y > 0.5;
        if (currentJoystickDown && !lastJoystickDown) {
            CommandScheduler.getInstance().schedule(new HeightChangeCommand(robot, -1));
        } else if (currentJoystickUp && !lastJoystickUp) {
            CommandScheduler.getInstance().schedule(new HeightChangeCommand(robot, 1));
        }
        lastJoystickUp = currentJoystickUp;
        lastJoystickDown = currentJoystickDown;

        super.run();
        robot.periodic();

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        loopTime = loop;
        telemetry.update();
        robot.write();
        robot.clearBulkCache();
    }
}
