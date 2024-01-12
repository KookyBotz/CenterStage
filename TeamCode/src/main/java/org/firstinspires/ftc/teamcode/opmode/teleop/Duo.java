package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.centerstage.ClawSide;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.PivotStateCommand;
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
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.util.InverseKinematics;
import org.firstinspires.ftc.teamcode.common.util.MathUtils;

@Config
@TeleOp(name = "Duo")
public class Duo extends CommandOpMode {

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
        Globals.USING_DASHBOARD = false;
        Globals.stopIntaking();
        Globals.stopScoring();

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

        gamepadEx.getGamepadButton(GamepadKeys.Button.Y )
                        .whenPressed(new SequentialCommandGroup(
                                new InstantCommand(Globals::startScoring),
                                new ArmCommand(2.94),
                                new PivotStateCommand(IntakeSubsystem.PivotState.SCORING)
                        )).whenReleased(new SequentialCommandGroup(
                                new DepositRetractionCommand()
                                )
                );

        gamepadEx.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(() -> CommandScheduler.getInstance().schedule(new InstantCommand(() -> extendIntake = !extendIntake)));

        gamepadEx2.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(() -> CommandScheduler.getInstance().schedule(new DepositExtendCommand()));

        gamepadEx2.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new DepositRetractionCommand());

        robot.read();
        while (opModeInInit()) {
            telemetry.addLine("Robot Initialized.");
            telemetry.update();
        }
    }

    @Override
    public void run() {
        robot.read();

        // G1 - Drivetrain Control
        robot.drivetrain.set(new Pose(gamepad1.left_stick_x, -gamepad1.left_stick_y, MathUtils.joystickScalar(-gamepad1.left_trigger + gamepad1.right_trigger, 0.01)), 0);

        boolean currentJoystickUp = gamepad1.right_stick_y < -0.5 || gamepad2.right_stick_y < -0.5;
        boolean currentJoystickDown = gamepad1.right_stick_y > 0.5 || gamepad2.right_stick_y > 0.5;

        if (currentJoystickDown && !lastJoystickDown) {
            CommandScheduler.getInstance().schedule(new HeightChangeCommand(robot, -1));
        } else if (currentJoystickUp && !lastJoystickUp) {
            CommandScheduler.getInstance().schedule(new HeightChangeCommand(robot, 1));
        }

        if (gamepad2.right_bumper && gamepad2.left_bumper) robot.drone.updateState(DroneSubsystem.DroneState.FIRED);

        lastJoystickUp = currentJoystickUp;
        lastJoystickDown = currentJoystickDown;

        super.run();
        robot.periodic();

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        telemetry.addData("currentAngle", robot.armActuator.getPosition());
        telemetry.addData("angle", InverseKinematics.t_angle);
        telemetry.addData("power", robot.armActuator.getPower());
        telemetry.addData("feedforward", robot.armActuator.getCurrentFeedforward());
        telemetry.addData("height", robot.extension.getBackdropHeight());
        loopTime = loop;
        telemetry.update();
        robot.write();
        robot.clearBulkCache();
    }
}
