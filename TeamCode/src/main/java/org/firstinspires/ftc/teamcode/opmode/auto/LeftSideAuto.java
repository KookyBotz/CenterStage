package org.firstinspires.ftc.teamcode.opmode.auto;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.drive.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WSubsystem;

@Autonomous(name = "LeftSideAuto")
public class LeftSideAuto extends LinearOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    private WSubsystem drivetrain;
    private WSubsystem localizer;
    private WSubsystem extension;
    private WSubsystem intake;

    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();

        Globals.IS_AUTO = true;
        Globals.IS_USING_IMU = true;

        robot.init(hardwareMap, telemetry);
        drivetrain = new MecanumDrivetrain();
//        localizer = new ThreeWheelLocalizer();
        extension = new ExtensionSubsystem();
        intake = new IntakeSubsystem();

        robot.addSubsystem(drivetrain, localizer, extension, intake);

        robot.enabled = true;

        robot.read();
        while (!isStarted()) {
            telemetry.addLine("auto in init");
            telemetry.update();
        }

        // todo finish rest of impl
    }
}
