package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.centerstage.ClawSide;
import org.firstinspires.ftc.teamcode.common.commandbase.teleopcommand.ClawDepositCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.teleopcommand.ClawToggleCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.teleopcommand.DepositExtendCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.teleopcommand.DepositRetractionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.teleopcommand.HeightChangeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.teleopcommand.IntakeExtendCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.teleopcommand.IntakeRetractCommand;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.DroneSubsystem;
import org.firstinspires.ftc.teamcode.common.util.InverseKinematics;
import org.firstinspires.ftc.teamcode.common.util.MathUtils;
import org.firstinspires.ftc.teamcode.common.util.logging.CSVInterface;
import org.firstinspires.ftc.teamcode.common.util.logging.LogType;
import org.firstinspires.ftc.teamcode.common.util.logging.Logger;

@Config
@TeleOp(name = "Solo")
public class Solo extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    private GamepadEx gamepadEx;
    private GamepadEx gamepadEx2;

    private double loopTime = 0.0;
    private boolean lastJoystickUp = false;
    private boolean lastJoystickDown = false;

    private boolean extendIntake = true;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        Globals.IS_AUTO = false;
        Globals.stopIntaking();
        Globals.stopScoring();

        Logger.logData(LogType.ARM_POSITION, "DATA!");
        CSVInterface.log();

        gamepadEx = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        robot.init(hardwareMap);

        robot.intakePivotActuator.setTargetPosition(0.03);
        robot.intakePivotActuator.write();

        robot.drone.reset();

        // G1 - Claw Control
        gamepadEx.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new ClawToggleCommand(robot, ClawSide.LEFT));
        gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new ClawToggleCommand(robot, ClawSide.RIGHT));

        // G1 - Retract deposit
        gamepadEx.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(
                        new ConditionalCommand(
                                new DepositRetractionCommand(),
                                new InstantCommand(() -> CommandScheduler.getInstance().schedule(new DepositExtendCommand())),
                                () -> Globals.IS_SCORING
                        )
                );

        // G1 - Claw control for scoring, retraction for when intaking
        gamepadEx.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(
                        () -> CommandScheduler.getInstance().schedule(
                                new ConditionalCommand(
                                        new ClawDepositCommand(),
                                        new ConditionalCommand(
                                                new IntakeRetractCommand(),
                                                new IntakeExtendCommand(extendIntake ? 500 : 100),
                                                () -> Globals.IS_INTAKING
                                        ),
                                        () -> Globals.IS_SCORING
                                ))
                );

        gamepadEx.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(() -> CommandScheduler.getInstance().schedule(new InstantCommand(() -> extendIntake = !extendIntake)));

        robot.read();
        while (opModeInInit()) {
            telemetry.addLine("Robot Initialized.");
            telemetry.update();
        }
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        robot.clearBulkCache();
        robot.read();
        robot.periodic();
        robot.write();

        // G1 - Drivetrain Control
        robot.drivetrain.set(
                new Pose(
                        gamepad1.left_stick_x,
                        -gamepad1.left_stick_y,
                        MathUtils.joystickScalar(-gamepad1.left_trigger + gamepad1.right_trigger, 0.01)
                ), 0
        );

        boolean currentJoystickUp = gamepad1.right_stick_y < -0.5;
        boolean currentJoystickDown = gamepad1.right_stick_y > 0.5;

        if (currentJoystickDown && !lastJoystickDown) {
            CommandScheduler.getInstance().schedule(new HeightChangeCommand(robot, -1));
        } else if (currentJoystickUp && !lastJoystickUp) {
            CommandScheduler.getInstance().schedule(new HeightChangeCommand(robot, 1));
        }

        lastJoystickUp = currentJoystickUp;
        lastJoystickDown = currentJoystickDown;

        if (gamepad1.dpad_up && gamepad1.y) robot.drone.updateState(DroneSubsystem.DroneState.FIRED);

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        telemetry.addData("height", robot.extension.getBackdropHeight());
        loopTime = loop;
        telemetry.update();
    }


}
