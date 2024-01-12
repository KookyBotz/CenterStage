package org.firstinspires.ftc.teamcode.opmode.testing.device;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.ExtensionSubsystem;

@TeleOp(name = "ExtensionTest")
@Disabled
public class ExtensionTest extends CommandOpMode {

    private ElapsedTime timer;
    private double loopTime = 0.0;

    private final RobotHardware robot = RobotHardware.getInstance();
    private ExtensionSubsystem extension;

    public static double targetPosition = 0.0;

    private GamepadEx gamepadEx;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
        Globals.IS_AUTO = false;

        robot.init(hardwareMap);
        extension = new ExtensionSubsystem();
        robot.addSubsystem(extension);

        gamepadEx = new GamepadEx(gamepad1);

//        gamepadEx.getGamepadButton(GamepadKeys.Button.A)
//                .whenPressed(() -> schedule(new InstantCommand(() -> robot.extensionPitchActuator.setTargetPosition(targetPosition))));
    }

    @Override
    public void run() {
        robot.read();
        super.run();
        robot.write();

//        telemetry.addData("current position", extensionPitchActuator.getPosition());
//        telemetry.addData("target position", extensionPitchActuator.getTargetPosition());
//        telemetry.addData("reached", robot.extensionPitchActuator.hasReached());

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        loopTime = loop;
        telemetry.update();

        robot.clearBulkCache();
    }
}
