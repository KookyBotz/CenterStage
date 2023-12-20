package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.hardware.Sensors;
import org.firstinspires.ftc.teamcode.common.subsystem.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;

import java.util.function.DoubleSupplier;

//@Disabled
@Config
@TeleOp(name = "TuningOpMode")
public class TuningOpMode extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    private ExtensionSubsystem extension;
    private IntakeSubsystem intake;

    private GamepadEx gamepadEx, gamepadEx2;

    private double loopTime = 0.0;

    public static double kG = 0.0;

    private DoubleSupplier armPos = () -> robot.doubleSubscriber(Sensors.SensorType.ARM_ENCODER);
    private DoubleSupplier extensionPos = () -> robot.doubleSubscriber(Sensors.SensorType.EXTENSION_ENCODER);

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();


        Globals.IS_AUTO = false;
        Globals.IS_USING_IMU = false;
        Globals.USING_DASHBOARD = true;

        gamepadEx = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        robot.init(hardwareMap, telemetry);

        robot.read();
        while (opModeInInit()) {
            telemetry.addLine("Robot Initialized. Mason is very cool and he is the best perosn to ever exist in the owrld and java ois the owrst progmraming kanguage nad ih ate it so so os much LLL + Ratio + cope + cget out of my game L");
            telemetry.update();
        }
    }

    @Override
    public void run() {
        robot.read();

        super.run();
        robot.periodic();


        telemetry.addData("armPosition", armPos.getAsDouble());
        telemetry.addData("extensionPosition", extensionPos.getAsDouble() -(armPos.getAsDouble() / Math.PI) * 50);
        telemetry.addData("feedforward", robot.extension.feedforward);


        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        loopTime = loop;
        telemetry.update();
//        robot.write();
        robot.armMotor.setPower(robot.extension.feedforward);
        robot.clearBulkCache();
    }
}
