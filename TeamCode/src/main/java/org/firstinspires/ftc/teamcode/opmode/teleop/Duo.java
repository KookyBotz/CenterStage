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
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.PivotStateCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.teleopcommand.ClawDepositCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.teleopcommand.ClawToggleCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.teleopcommand.DepositExtendCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.teleopcommand.DepositRetractionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.teleopcommand.HeightChangeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.teleopcommand.IntakeExtendCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.teleopcommand.IntakeHeightCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.teleopcommand.IntakeRetractCommand;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.DroneSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystem.HangSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.util.MathUtils;

@Config
@TeleOp(name = "Duo")
public class Duo extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    private GamepadEx gamepadEx;
    private GamepadEx gamepadEx2;

    private double loopTime = 0.0;
    private boolean lastJoystickUpRight = false;
    private boolean lastJoystickDownRight = false;
    //    private boolean lastJoystickitUpLeft = false;
//    private boolean lastJoystickDownLeft = false;
    private boolean extendIntake = true;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        Globals.IS_AUTO = false;
        Globals.stopIntaking();
        Globals.stopScoring();

        gamepadEx = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        robot.init(hardwareMap);

        robot.intakePivotActuator.setTargetPosition(0.08);
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
                                                new IntakeExtendCommand(extendIntake ? 400 : 125),
                                                () -> Globals.IS_INTAKING
                                        ),
                                        () -> Globals.IS_SCORING
                                ))
                );

        gamepadEx.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new SequentialCommandGroup(
                        new InstantCommand(Globals::startScoring),
                        new ArmCommand(2.94),
                        new PivotStateCommand(IntakeSubsystem.PivotState.SCORING)
                ));

        gamepadEx.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(() -> CommandScheduler.getInstance().schedule(new InstantCommand(() -> extendIntake = !extendIntake)
                        .alongWith(new ConditionalCommand(
                                new InstantCommand(() -> gamepad1.rumble(200))
                                        .andThen(new WaitCommand(300)
                                                .andThen(new InstantCommand(() -> gamepad1.rumble(200)))),
                                new InstantCommand(() -> gamepad1.rumble(200)),
                                () -> !extendIntake
                        ))));

        gamepadEx2.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(() -> CommandScheduler.getInstance().schedule(new DepositExtendCommand()
                        .alongWith(new InstantCommand(() -> gamepad2.rumble(200)))));

        gamepadEx2.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new DepositRetractionCommand()
                        .alongWith(new InstantCommand(() -> gamepad2.rumble(200))));

//        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_UP)
//                        .whenPressed(new IntakeHeightCommand(robot, 1));
//        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
//                .whenPressed(new IntakeHeightCommand(robot, -1));
//        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
//                .whenPressed(new IntakeHeightCommand(robot, 0));

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
        robot.drivetrain.set(new Pose(gamepad1.left_stick_x, -gamepad1.left_stick_y, MathUtils.joystickScalar(-gamepad1.left_trigger + gamepad1.right_trigger, 0.01)), 0);

        boolean currentJoystickUpRight = gamepad1.right_stick_y < -0.5 || gamepad2.right_stick_y < -0.5;
        boolean currentJoystickDownRight = gamepad1.right_stick_y > 0.5 || gamepad2.right_stick_y > 0.5;
//        boolean currentJoystickUpLeft = gamepad2.left_stick_y < -0.5;
//        boolean currentJoystickDownLeft = gamepad2.left_stick_y > 0.5;

        if (currentJoystickDownRight && !lastJoystickDownRight) {
            CommandScheduler.getInstance().schedule(new HeightChangeCommand(robot, -1));
        } else if (currentJoystickUpRight && !lastJoystickUpRight) {
            CommandScheduler.getInstance().schedule(new HeightChangeCommand(robot, 1));
        }

//        if (currentJoystickDownLeft && !lastJoystickDownLeft) {
//            CommandScheduler.getInstance().schedule(new IntakeHeightCommand(robot, -1));
//        } else if (currentJoystickUpLeft && !lastJoystickUpLeft) {
//            CommandScheduler.getInstance().schedule(new IntakeHeightCommand(robot, 1));
//        }

        robot.leftHang.setPower(gamepad2.left_stick_y);
        robot.rightHang.setPower(-gamepad2.left_stick_y);

        if (gamepad2.right_bumper && gamepad2.left_bumper) robot.drone.updateState(DroneSubsystem.DroneState.FIRED);

        if (Math.abs(gamepad2.right_trigger) > 0.5) robot.hang.updateState(HangSubsystem.HangState.EXTENDING);
        else if (Math.abs(gamepad2.left_trigger) > 0.5) robot.hang.updateState(HangSubsystem.HangState.RETRACTING);
        else robot.hang.updateState(HangSubsystem.HangState.DISABLED);

        lastJoystickUpRight = currentJoystickUpRight;
        lastJoystickDownRight = currentJoystickDownRight;
//        lastJoystickUpLeft = currentJoystickUpLeft;
//        lastJoystickDownLeft = currentJoystickDownLeft;

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
//        telemetry.addData("intake index", robot.extension.getStackHeightIndex());
//        telemetry.addData("intake height", robot.extension.getStackHeight());
        telemetry.addData("lift ticks", robot.extensionActuator.getPosition() / 26);
        loopTime = loop;
        telemetry.update();
    }
}
