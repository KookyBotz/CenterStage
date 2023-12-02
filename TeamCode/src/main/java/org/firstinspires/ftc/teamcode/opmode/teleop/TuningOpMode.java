package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;

@Disabled
@Config
@TeleOp(name = "TuningOpMode")
public class TuningOpMode extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    private ExtensionSubsystem extension;
    private IntakeSubsystem intake;

    private GamepadEx gamepadEx;

    private double loopTime = 0.0;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
        Globals.IS_AUTO = false;
        Globals.IS_USING_IMU = true;

        gamepadEx = new GamepadEx(gamepad1);

        robot.init(hardwareMap, telemetry);
        extension = new ExtensionSubsystem();
        intake = new IntakeSubsystem();
        robot.addSubsystem(extension, intake);

        robot.read();
        while (opModeInInit()) {
            telemetry.addLine("Robot Initialized. Mason is very cool and he is the best perosn to ever exist in the owrld and java ois the owrst progmraming kanguage nad ih ate it so so os much LLL + Ratio + cope + cget out of my game L");
            telemetry.update();
        }
    }

    @Override
    public void run() {
        robot.clearBulkCache();
        robot.read();

        super.run();
        robot.periodic();

        telemetry.addData("armPosition", robot.pitchActuator.getPosition());
        telemetry.addData("extensionPosition", robot.extensionActuator.getPosition());
        telemetry.addData("pivotPosition", robot.intakePivotActuator.getPosition());

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        loopTime = loop;
        telemetry.update();
    }
}
